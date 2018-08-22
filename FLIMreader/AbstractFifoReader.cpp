#include "AbstractFifoReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

using namespace std;

AbstractFifoReader::AbstractFifoReader(const std::string& filename) :
FlimReader(filename)
{
   readSettings();
}

AbstractFifoReader::~AbstractFifoReader()
{
   terminate = true;
   if (realignment_thread.joinable())
      realignment_thread.join();
}

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

   auto channels = validateChannels(channels_, n_chan_stride);

   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;

   int n_bin = (int)timepoints.size();
   int n_x_binned = n_x / spatial_binning;
   int n_y_binned = n_y / spatial_binning;
   int n_invalid = 0;
   int last_frame_written = 0;

   auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);

   if (sync.has_initial_frame_marker)
      fifo_frame->loadNext();

   cv::Mat pos(3, 1, CV_64F, cv::Scalar(0));
   cv::Mat tr_pos(3, 1, CV_64F, cv::Scalar(0));

   double* p_pos = pos.ptr<double>();
   double* p_tr_pos = pos.ptr<double>();

   cv::Mat affine;
   cv::Point2d shift;

   ma_image.clear();

   int frame_idx = 0;

   while (event_reader->hasMoreData() && !terminate)
   {

      fifo_frame->loadNext();
      FifoProcessor processor(markers, sync);
      processor.setFrame(fifo_frame);

      int frame = frame_idx / n_z;
      int z = frame_idx % n_z;
      frame_idx++;

      Photon p;
      while ((p = processor.getNextPhoton()))
      {
         if (frame > last_frame_written)
         {
            computeMeanArrivalImage(histogram);
            last_frame_written = frame;
         }

         if (!useFrame(p.frame)) continue;
         if (!p.valid || p.channel > n_chan) continue;

         int mapped_channel = channel_map[p.channel];
         if (mapped_channel == -1) continue;

         p.z = z;

         if (frame_aligner != nullptr && frame_aligner->frameValid(frame))
         {
            // Check that we have realigned this frame
            if (!frame_aligner->frameReady(frame))
            {
               std::unique_lock<std::mutex> lk(realign_mutex);
               realign_cv.wait(lk, [this, frame] {
                  return (frame_aligner->frameReady(frame) || terminate);
               });
               lk.unlock();
            }

            if (terminate) break;

            double correlation = frame_aligner->getFrameCorrelation(frame);
            double coverage = frame_aligner->getFrameCoverage(frame);
            if (correlation < realign_params.correlation_threshold || coverage < realign_params.coverage_threshold)
               continue;

            frame_aligner->shiftPixel(frame, p.x, p.y, p.z);
         }

         p.x /= spatial_binning;
         p.y /= spatial_binning;

         int64_t x = (int64_t)std::round(p.x);
         int64_t y = (int64_t)std::round(p.y);
         int64_t z = (int64_t)std::round(p.z);

         int bin = p.bin;
         if (t_rep_resunit > 0)
         {
            bin = (bin + time_shifts_resunit[p.channel]) % t_rep_resunit;
            bin = bin < 0 ? bin + t_rep_resunit : bin;
         }
         bin = bin >> downsampling;

         if ((bin < n_bin) && (x < n_x_binned) && (x >= 0)
            && (y < n_y_binned) && (y >= 0)
            && (z < n_z) && (z >= 0))
         {
            size_t idx = (x + n_x_binned * y + n_x_binned * n_y_binned * z);
            histogram[bin + n_bin * (mapped_channel + n_chan_stride * idx)]++;
         }
         else
         {
            n_invalid++;
         }
      }
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



void AbstractFifoReader::readSettings()
{
   using namespace boost;
   
   filesystem::path filepath(filename);
   auto metapath = filepath.parent_path();
   metapath /= "PicoquantLoaderSettings.info";
   
   time_shifts_ps.resize(4, 0.0);

   // Try load in shift settings
   if (filesystem::exists(metapath))
   {
      property_tree::ptree tree;
      property_tree::read_info(metapath.string(), tree);
      
      time_shifts_ps[0] = tree.get<float>("shifts.1", 0);
      time_shifts_ps[1] = tree.get<float>("shifts.2", 0);
      time_shifts_ps[2] = tree.get<float>("shifts.3", 0);
      time_shifts_ps[3] = tree.get<float>("shifts.4", 0);
   }
}

void AbstractFifoReader::determineDwellTime()
{
   using namespace boost::accumulators;

   assert(event_reader != nullptr);
   
   event_reader->setToStart();

   uint64_t frame_start = 0;
   uint64_t sync_start_count = 0;
   int n_averaged = 0;
   int n_line = 0;
   int n_frame = 0;

   accumulator_set<uint64_t, stats<tag::median > > sync_count_per_line_acc;
   accumulator_set<uint64_t, stats<tag::median > > sync_count_interline_acc;

   bool line_active = false;
   do
   {
      FifoEvent p = event_reader->getEvent();

      if (!p.valid)
         continue;

      if ((markers.FrameMarker > 0) && (p.mark & markers.FrameMarker))
      {
         if (n_line > 0)
         {
            if (n_frame == 0)
               frame_start = p.macro_time;
            else
               sync.counts_interframe = (double)(p.macro_time - frame_start);
            n_frame++; // count full frames (i.e. ignore first start, if it's there)
            line_active = false;
         }
         else
         {
            sync.has_initial_frame_marker = true;
         }

      }

      if ((p.mark & markers.LineEndMarker) && line_active)
      {
         if (p.macro_time >= sync_start_count) // not sure why this is sometimes violated
         {
            uint64_t diff = p.macro_time - sync_start_count;
            sync_count_per_line_acc(diff);

            n_averaged++;
         }

         line_active = false;
      }
      else if (p.mark & markers.LineStartMarker)
      {
         if ((p.macro_time >= sync_start_count) && n_frame == 0)
         {
            if (n_line > 0)
            {
               uint64_t diff = p.macro_time - sync_start_count;
               sync_count_interline_acc(diff);
            }

            n_line++;
         }
         sync_start_count = p.macro_time;
         line_active = true;
      }

      // if we don't have frame markers break after 512 lines (speed considerations)
      if (markers.FrameMarker == 0x0 && n_line >= n_y)
         break;

   } while (event_reader->hasMoreData() && (n_frame == 0));
   
   if (markers.FrameMarker == 0x0)
      n_frame = 1;

   sync.count_per_line = median(sync_count_per_line_acc);
   sync.counts_interline = median(sync_count_interline_acc);

   if (line_averaging > 1)
   {
      double factor = static_cast<double>(line_averaging) / (line_averaging + 1);
      sync.count_per_line *= factor;
      sync.counts_interline *= factor;
   }

   if (n_line == 0 || n_frame == 0)
      throw std::runtime_error("Error interpreting sync markers");

   if (n_y == 0)
   {
     n_y = n_line / line_averaging / n_frame;
	  if (n_x == 0)
         n_x = n_y;
   }
   else if (markers.FrameMarker != 0x0)
   {
      //assert(n_line == (n_y * n_frame));
   }

   sync.n_x = n_x;
   sync.n_line = n_y;

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

void AbstractFifoReader::loadIntensityFramesImpl()
{
   int fb = realign_params.frame_binning;

   assert(event_reader != nullptr);
   event_reader->setToStart();

   int n_invalid = 0;

   {
      std::lock_guard<std::mutex> lk(frame_mutex);
//      if (!frames.empty() && (frames[0].size() != cv::Size(n_x, n_y))) // TODO
         frames.clear();
   }

   if (frames.empty())
   {
      auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);

      if (sync.has_initial_frame_marker)
         fifo_frame->loadNext();

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
         FifoProcessor processor(markers, sync);
         processor.setFrame(fifo_frame);

         Photon p;
         while (( p = processor.getNextPhoton() ))
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
   }


   fb = last_frame_binning;

   if (terminate)
      frames.clear();

}

