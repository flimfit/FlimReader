#include "AbstractFifoReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

using namespace std;

AbstractFifoReader::AbstractFifoReader() 
{
}


AbstractFifoReader::AbstractFifoReader(const std::string& filename) :
FlimReader(filename)
{
}

AbstractFifoReader::~AbstractFifoReader()
{
}

void AbstractFifoReader::readSettings()
{

   boost::filesystem::path filepath(filename);

   std::vector<boost::filesystem::path> metapath;

   const auto parent_path = filepath.parent_path();

   metapath.push_back(parent_path / "PicoquantLoaderSettings.info");
   metapath.push_back(parent_path / "FifoSettings.info");
   
   time_shifts_ps.resize(n_chan, 0.0);

   // Try load in shift settings
   for (auto& path : metapath)
   {
      if (boost::filesystem::exists(path))
      {
         boost::property_tree::ptree tree;
         boost::property_tree::read_info(path.string(), tree);

         for (int c = 0; c < n_chan; c++)
            time_shifts_ps[c] = tree.get<float>("shifts." + std::to_string(c), 0);

         sync.bidirectional = tree.get<bool>("sync.bidirectional", false);
         sync.phase = tree.get<double>("sync.phase", 0.0);
         sync.n_line = tree.get<int>("sync.n_line", 0);
         sync.n_x = tree.get<int>("sync.n_x", 0);
      }

   }
}


void AbstractFifoReader::determineDimensions()
{
   if (!has_sync_settings)
   {
      sync.n_x = n_x;
      sync.n_line = n_y;

      readSettings();

      event_reader->setToStart();

      auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);

      fifo_frame->loadNext();
      FifoProcessor processor(fifo_frame, sync);
      auto params = processor.determineSyncSettings(sync, n_chan);

      sync = params.sync;
      n_x = sync.n_x;
      n_y = sync.n_line;

      setNumChannels(params.n_chan);
      recommended_channels = params.recommended_channels;

      has_sync_settings = true;
   }
}

void AbstractFifoReader::setNumChannels(int n_chan_)
{
   double frame_time_ps = n_y * sync.count_per_line * macro_time_resolution_ps;

   n_chan = n_chan_;
   time_shifts_ps.resize(n_chan, 0);
   rate_estimator.assign(n_chan,
      CountRateEstimator(macro_time_resolution_ps, time_resolution_native_ps, frame_time_ps));
   count_rates.resize(n_chan);
   decay.assign(n_chan, std::vector<int>(timepoints.size()));
   next_decay.assign(n_chan, std::vector<int>(timepoints.size()));
   setUseAllChannels();
}


void AbstractFifoReader::initaliseTimepoints(int n_timebins_native, double time_resolution_native_ps_)
{
   time_resolution_native_ps = time_resolution_native_ps_;
   native_timepoints.resize(n_timebins_native);
   for (int i = 0; i < n_timebins_native; i++)
      native_timepoints[i] = time_resolution_native_ps * i;

   FlimReader::initaliseTimepoints();
}


void AbstractFifoReader::setTemporalDownsampling(int downsampling_)
{      
   int downsampling_factor = 1 << downsampling;
  
   double t_0 = 0;
   double t_step = time_resolution_native_ps * downsampling_factor;
  
   t_rep_resunit = (int)std::round(t_rep_ps / time_resolution_native_ps);

   size_t n_t_native = native_timepoints.size();
   size_t n_t = n_t_native >> downsampling_;

   if (n_t == 0)
      throw std::runtime_error("Invalid downsampling value");

   if (t_rep_ps != 0)
   {
      int n_t_rep = (int)floor(t_rep_ps / t_step);
      int n_t = std::min(n_t_rep, n_t);
   }

   downsampling = downsampling_;
   timepoints.resize(n_t);

   for (size_t i = 0; i < n_t; i++)
      timepoints[i] = native_timepoints[i << downsampling];

   time_shifts_resunit.clear();
   for(auto shift : time_shifts_ps)
      time_shifts_resunit.push_back((int) std::round(shift / time_resolution_native_ps));
};


template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   if (terminate)
      return;

   if (realignment_complete) // we are reprocessing
      computeIntensityNormalisation();
   if (realign_params.type == RealignmentType::None)
      intensity_normalisation = cv::Mat();

   assert(event_reader != nullptr);
   event_reader->setToStart();

   event_reader->setRetainData(retain_data);

   auto channels = validateChannels(channels_, n_chan_stride);

   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;

   size_t n_bin = timepoints.size();
   int n_x_binned = getNumX(); // with binning
   int n_y_binned = getNumY();
   int n_invalid = 0;
   int last_frame_written = -1;

   auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);
   fifo_frame->loadNext();

   ma_image.clear();

   int frame_idx = 0;
   int image_idx = 0;

   while (!fifo_frame->isEmpty())
   {
      FifoProcessor processor(fifo_frame, sync);

      int frame = frame_idx / n_z;
      int z = frame_idx % n_z;
      frame_idx++;

      bool use_frame;
      Photon p;
      while ((p = processor.getNextPhoton()))
      {
         if (p.image > image_idx)
         {
            std::fill_n(histogram, n_bin * n_chan * n_x_binned * n_y_binned, (T) 0);
            image_idx = p.image;
            for (auto& d : next_decay)
               std::fill(d.begin(), d.end(), 0);
         }

         p.z = z;

         if (terminate) break;

         if (frame > last_frame_written)
         {
            computeMeanArrivalImage(histogram);
            last_frame_written = frame;

            waitForFrameReady(frame);
            use_frame = frame_aligner->getRealignmentResult(frame).useFrame(realign_params);
         }

         if (!(p.valid && use_frame) || p.channel >= n_chan) continue;

         int mapped_channel = channel_map[p.channel];
         if (mapped_channel == -1) continue;

         if ((p.x >= n_x) || (p.x < 0) || (p.y >= n_y) || (p.y < 0))
            continue;

         T corrected_value = 1;
         if (!spectral_correction.empty())
            corrected_value = (T)spectral_correction[mapped_channel].at<float>((int)std::round(p.y), (int)std::round(p.x));

         frame_aligner->shiftPixel(frame, p.x, p.y, p.z);

         p.x /= spatial_binning;
         p.y /= spatial_binning;

         int64_t x = (int64_t)std::round(p.x);
         int64_t y = (int64_t)std::round(p.y);
         int64_t z = (int64_t)std::round(p.z);

         if ((x >= n_x_binned) || (x < 0) || (y >= n_y_binned) || (y < 0) || (z >= n_z) || (z < 0))
            continue;

         uint32_t bin = p.micro_time;
         if (t_rep_resunit > 0)
         {
            bin = (bin + time_shifts_resunit[p.channel]) % t_rep_resunit;
            bin = bin < 0 ? bin + t_rep_resunit : bin;
         }
         bin = bin >> downsampling;

         if (bin < n_bin)
         {
            size_t idx = (x + n_x_binned * y + n_x_binned * n_y_binned * z);
            histogram[bin + n_bin * (mapped_channel + n_chan_stride * idx)] += corrected_value;
            rate_estimator[p.channel].addEvent(p.macro_time, p.micro_time);
            next_decay[p.channel][bin]++;
         }
         else
         {
            n_invalid++;
         }
      }

      for (int i = 0; i < n_chan; i++)
      {
         count_rates[i] = rate_estimator[i].getCountRate();
         rate_estimator[i].setNewFrame();

         for (int i=0; i<decay.size(); i++)
         {
            std::copy(next_decay[i].begin(), next_decay[i].end(), decay[i].begin());
            //std::fill(next_decay[i].begin(), next_decay[i].end(), 0);
         }
      }

      fifo_frame->loadNext();
   }

   if (save_mean_arrival_images)
   {
      computeMeanArrivalImage(histogram);
      writeMultipageTiff("c:/users/cimlab/documents/test/ma-image.tif", ma_image);
   }
}

void AbstractFifoReader::readData(float* data, const std::vector<int>& channels, int n_chan_stride) { readData_(data, channels, n_chan_stride); };
void AbstractFifoReader::readData(double* data, const std::vector<int>& channels, int n_chan_stride) { readData_(data, channels, n_chan_stride); };
void AbstractFifoReader::readData(uint16_t* data, const std::vector<int>& channels, int n_chan_stride) { readData_(data, channels, n_chan_stride); };



void AbstractFifoReader::loadIntensityFramesImpl()
{
   int fb = realign_params.frame_binning;

   if (!frames.empty() && (fb == last_frame_binning))
      return;

   fb = last_frame_binning;

   assert(event_reader != nullptr);
   event_reader->setToStart();

   int n_invalid = 0;

   std::unique_lock<std::mutex> lk(frame_mutex);
   frames.clear();
   lk.unlock();

   auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);

   int idx = 0;
   int frame = 0;
   int z = 0;

   std::vector<int> dims = { n_z, n_y, n_x };
   int cur_frame_idx = 0;
   cv::Mat cur_frame = cv::Mat(dims, CV_32F, cv::Scalar(0));

   while (event_reader->hasMoreData())
   {
      if (terminate) break;

      fifo_frame->loadNext();
      FifoProcessor processor(fifo_frame, sync);

      Photon p;
      while ((p = processor.getNextPhoton()))
      {
         if ((p.x < n_x) && (p.x >= 0) && (p.y < n_y) && (p.y >= 0) && use_channel[p.channel])
            cur_frame.at<float>(z, (int)p.y, (int)p.x)++;
      }

      idx++;
      frame = idx / n_z;
      z = idx % n_z;

      if (frame > cur_frame_idx)
      {
         setIntensityFrame(cur_frame_idx, cur_frame);

         cur_frame.setTo(0);
         cur_frame_idx = frame;
      }
   }

   // last frame
   if (z > 0)
      setIntensityFrame(cur_frame_idx, cur_frame);

   if (terminate)
      frames.clear();
}


void AbstractFifoReader::stopReading()
{
   terminate = true;
   realign_cv.notify_all();
}
