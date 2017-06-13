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
   assert(event_reader != nullptr);
   
   event_reader->setToStart();
   
   uint64_t frame_start = 0;
   uint64_t sync_count_accum = 0;
   uint64_t sync_start_count = 0;
   double sync_count_per_line = 0;
   double sync_count_interline = 0;
   int n_averaged = 0;
   int n_line = 0;
   int n_frame = 0;

   std::vector<uint64_t> sync_counts;
   if (n_y > 0)
      sync_counts.reserve(n_y);

   bool line_active = false;
   do
   {
      TcspcEvent p = event_reader->getEvent();

      sync_count_accum += p.macro_time_offset;
      uint64_t macro_time = sync_count_accum + p.macro_time;

      if (!p.valid)
         continue;

      if ((p.mark & markers.FrameMarker) && n_line > 0)
      {
         if (n_frame == 0)
            frame_start = macro_time;
         else
            sync.counts_interframe = (double) (macro_time - frame_start);
         n_frame++; // count full frames (i.e. ignore first start, if it's there)

      }

      if ((p.mark & markers.LineEndMarker) && line_active)
      {
         if (macro_time >= sync_start_count) // not sure why this is sometimes violated
         {
            uint64_t diff = macro_time - sync_start_count;
            sync_count_per_line += diff;

            sync_counts.push_back(diff);

            n_averaged++;
         }

         line_active = false;
      }
      
      if (p.mark & markers.LineStartMarker)
      {
         if (n_line > 0)
         {
            uint64_t diff = macro_time - sync_start_count;
            sync_count_interline += (macro_time - sync_start_count);
         }

         n_line++;
         sync_start_count = macro_time;
         line_active = true;
      }

      // if we don't have frame markers break after 512 lines (speed considerations)
      if (markers.FrameMarker == 0x0 && n_line >= n_y)
         break;

   } while (event_reader->hasMoreData() && (n_frame < 2));
   
   sync_count_per_line /= n_averaged;
   sync_count_interline /= (n_averaged-1);

   if (line_averaging > 1)
       sync_count_per_line *= static_cast<double>(line_averaging) / (line_averaging+1);

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

   sync.count_per_line = sync_count_per_line;
   sync.counts_interline = sync_count_interline;
   sync.n_x = n_x;
   
}

void AbstractFifoReader::setTemporalResolution(int temporal_resolution__)
{
   // These should be set before calling this function
   assert(n_timebins_native > 0);
   assert(time_resolution_native_ps > 0);

   int native_resolution = (int) ceil(log2(n_timebins_native));
   
   temporal_resolution = std::min(native_resolution, temporal_resolution__);
   temporal_resolution = std::max(0, temporal_resolution);
      
   downsampling = (native_resolution - temporal_resolution);
   int downsampling_factor = 1 << downsampling;
  
   double t_0 = 0;
   double t_step = time_resolution_native_ps * downsampling_factor;
  
   t_rep_resunit = (int)std::round(t_rep_ps / time_resolution_native_ps);

   int n_t_native = 1 << temporal_resolution;
   int n_t = n_t_native;

   if (t_rep_ps != 0)
   {
      int n_t_rep = (int)floor(t_rep_ps / t_step);
      int n_t = std::min(n_t_rep, n_t);
   }


   timepoints_.resize(n_t);

   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;

   time_shifts_resunit.clear();
   for(auto shift : time_shifts_ps)
      time_shifts_resunit.push_back((int) std::round(shift / time_resolution_native_ps));
};

void AbstractFifoReader::alignFrames()
{
   if (!realign_params.use_realignment())
      return;

   if (frame_aligner == nullptr || frame_aligner->getType() != realign_params.type)
      frame_aligner = AbstractFrameAligner::createFrameAligner(realign_params);

   getIntensityFrames();

   if (frames.size() == 0)
      return;

   ImageScanParameters image_params(sync.count_per_line, sync.counts_interline, sync.counts_interframe, n_x, n_y, sync.bi_directional);

   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames((int)frames.size());

   int max_idx = reference_index;
   frame_aligner->setReference(max_idx, frames[max_idx]);

   realignment = std::vector<RealignmentResult>(frames.size());
   realignment_complete = false;

   intensity_normalisation = cv::Mat(frames[0].size(), CV_16U, cv::Scalar(1));


   if (realignment_thread.joinable())
      realignment_thread.join();

   realignment_thread = std::thread(&AbstractFifoReader::alignFramesImpl, this);
}

void AbstractFifoReader::alignFramesImpl()
{
   #pragma omp parallel for schedule(dynamic,1)
   for (int i = 0; i < frames.size(); i++)
   {
      if (terminate) break;
      realignment[i] = frame_aligner->addFrame(i, frames[i]);

      {
         std::lock_guard<std::mutex> lk(realign_mutex);
         realignment[i].done = true;

         if ((realignment[i].correlation >= realign_params.correlation_threshold) &&
            (realignment[i].coverage >= realign_params.coverage_threshold))
               intensity_normalisation += realignment[i].mask;
      }

      std::cout << "*";
      realign_cv.notify_all();
   }

   realignment_complete = true;
   realign_cv.notify_all();
      realignment[i].done = true;
      realign_cv.notify_one();
   }
}

void AbstractFifoReader::getIntensityFrames()
{
   int sb = realign_params.spatial_binning;
   int fb = realign_params.frame_binning;

   assert(event_reader != nullptr);
   event_reader->setToStart();

   int n_x_binned = n_x / sb;
   int n_y_binned = n_y / sb;
   int n_invalid = 0;

   if (!frames.empty() && (frames[1].size() != cv::Size(n_x_binned, n_y_binned)))
      frames.clear();

   if (frames.empty())
   {
      FifoProcessor processor(markers, sync);
      while (event_reader->hasMoreData())
      {
         TcspcEvent e = event_reader->getEvent();
         Photon p = processor.addEvent(e);

#ifdef _DEBUG
         if (p.frame > 10)
            break;
#endif

         if (p.valid)
         {
            p.x /= sb;
            p.y /= sb;
            p.frame /= fb;

            while (p.frame >= frames.size())
               frames.push_back(cv::Mat(n_y_binned, n_x_binned, CV_32F, cv::Scalar(0)));

            if ((p.x < n_x_binned) && (p.x >= 0) && (p.y < n_y_binned) && (p.y >= 0))
               frames[p.frame].at<float>((int)p.y, (int)p.x)++;
         }
      }
   }
}


void AbstractFifoReader::computeIntensityNormalisation()
{
   if (!realignment.empty())
   {
      // Get intensity
      cv::Mat intensity(realignment[0].frame.size(), CV_16U, cv::Scalar(1));
      for (int i = 0; i < realignment.size(); i++)
      {
         if ((realignment[i].correlation >= realign_params.correlation_threshold) &&
             (realignment[i].coverage >= realign_params.coverage_threshold) &&
              realignment[i].done &&
             (!realignment[i].mask.empty()))
            intensity += realignment[i].mask;
      }
      intensity_normalisation = intensity;
   }
}