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
FLIMReader(filename)
{
   readSettings();
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

      if (p.mark & markers.FrameMarker)
         n_frame++;

      if (p.mark > 0 && p.mark < 0x40)
         n_frame = n_frame;

      if (n_frame > 0 || markers.FrameMarker == 0x0)
      {
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
         else if (p.mark & markers.LineStartMarker)
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
      }

      // if we don't have frame markers break after 512 lines (speed considerations)
      if (markers.FrameMarker == 0x0 && n_line >= n_y)
         break;

   } while (event_reader->hasMoreData() && (n_frame < 2 || line_active));
   
   sync_count_per_line /= n_averaged;
   sync_count_interline /= (n_averaged-1);

   if (line_averaging > 1)
       sync_count_per_line *= static_cast<double>(line_averaging) / (line_averaging+1);
   
   if (n_y == 0)
   {
      n_y = n_line / line_averaging;
	  if (n_x == 0)
         n_x = n_line / line_averaging;
   }
   else if (markers.FrameMarker != 0x0)
   {
      assert(n_y == n_line);
   }

   sync.count_per_line = sync_count_per_line;
   sync.counts_interline = sync_count_interline;
   sync.n_x = n_x;
   
}

void AbstractFifoReader::setTemporalResolution(int temporal_resolution__)
{
   int native_resolution = (int) ceil(log2(n_timebins_native));
   
   temporal_resolution = std::min(native_resolution, temporal_resolution__);
   temporal_resolution = std::max(0, temporal_resolution);
      
   downsampling = (native_resolution - temporal_resolution);
   int downsampling_factor = 1 << downsampling;
  
   double t_0 = 0;
   double t_step = time_resolution_native_ps * downsampling_factor;
  
   t_rep_resunit = (int)std::round(t_rep_ps / time_resolution_native_ps);

   int n_t_rep = floor(t_rep_ps / t_step);
   int n_t_native = 1 << temporal_resolution;
   int n_t = std::min(n_t_rep, n_t_native);

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
               frames.push_back(cv::Mat(n_x_binned, n_y_binned, CV_32F, cv::Scalar(0)));

            if ((p.x < n_x_binned) && (p.x >= 0) && (p.y < n_y_binned) && (p.y >= 0))
               frames[p.frame].at<float>((int)p.y, (int)p.x)++;
         }
      }
   }

   if (frames.size() == 0)
      return;

   double max_m = 0;
   int max_idx = 0;
   for (int i = 0; i < frames.size(); i++)
   {
      double new_v = cv::mean(frames[i])(0);
      if (new_v > max_m)
      {
         max_m = new_v;
         max_idx = i;
      }
   }

   ImageScanParameters image_params(sync.count_per_line, sync.counts_interline, n_x, n_y, sync.bi_directional);

   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames((int) frames.size());
   
   max_idx = 0;
   frame_aligner->setReference(max_idx, frames[max_idx]);

   realignment.clear();
   realignment.resize(frames.size());

   #pragma omp parallel for schedule(dynamic)
   for (int i = 0; i < frames.size(); i++)
   {
      if (terminate) continue;
      realignment[i] = frame_aligner->addFrame(i, frames[i]);
   }

   //frame_aligner->reprocess();
}