#include "AbstractFifoReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <opencv2/imgproc.hpp>

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
   int n_averaged = 0;
   int n_frame = 0;
   int n_line = 0;
   bool line_started = false;
   do
   {
      TcspcEvent p = event_reader->getEvent();

      sync_count_accum += p.macro_time_offset;
      uint64_t macro_time = sync_count_accum + p.macro_time;

      if (!p.valid)
         continue;

      if (p.mark & markers.FrameMarker)
      {
         n_frame++;
      }
      if (n_frame > 0)
      {
         if ((p.mark & markers.LineEndMarker) && line_started)
         {
            uint64_t diff = macro_time - sync_start_count;
            sync_count_per_line += (macro_time - sync_start_count);
            n_averaged++;
            line_started = false;
         }
         if (p.mark & markers.LineStartMarker)
         {
            n_line++;
            sync_start_count = macro_time;
            line_started = true;
         }

      }
      if (n_frame >= 2)
         break;
   } while (event_reader->hasMoreData());
   
   sync_count_per_line /= n_averaged;
   
   if (line_averaging > 1)
       sync_count_per_line *= static_cast<double>(line_averaging) / (line_averaging+1);
   
   if (n_y == 0)
   {
      n_y = n_line / line_averaging;
	  if (n_x == 0)
         n_x = n_line / line_averaging;
   }
   else
      assert(n_y == n_line);

   sync.count_per_line = sync_count_per_line;
   sync.n_x = n_x;
}

void AbstractFifoReader::setTemporalResolution(int temporal_resolution__)
{
   int native_resolution = ceil(log2(n_timebins_native));
   
   temporal_resolution = std::min(native_resolution, temporal_resolution__);
   temporal_resolution = std::max(0, temporal_resolution);
   
   int n_t = 1 << temporal_resolution;
   timepoints_.resize(n_t);
   
   downsampling = (native_resolution - temporal_resolution);
   int downsampling_factor = 1 << downsampling;
   
   double t_0 = 0;
   double t_step = time_resolution_native_ps * downsampling_factor;
   
   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;
   
   t_rep_resunit = (int) std::round(t_rep_ps / time_resolution_native_ps);
   
   time_shifts_resunit.clear();
   for(auto shift : time_shifts_ps)
      time_shifts_resunit.push_back((int) std::round(shift / time_resolution_native_ps));
};

void AbstractFifoReader::alignFrames()
{

   int sb = 4;
   int fb = 4;

   assert(event_reader != nullptr);
   event_reader->setToStart();

   int n_x_binned = n_x / sb;
   int n_y_binned = n_y / sb;
   int n_invalid = 0;

   FifoProcessor processor(markers, sync);

   std::vector<cv::Mat> frames;

   while (event_reader->hasMoreData())
   {
      TcspcEvent e = event_reader->getEvent();
      Photon p = processor.addEvent(e);

      if (p.valid)
      {
         p.x /= sb;
         p.y /= sb;
         p.frame /= fb;

         while (p.frame >= frames.size())
            frames.push_back(cv::Mat(n_x_binned, n_y_binned, CV_32F, cv::Scalar(0)));

         if ((p.x < n_x_binned) && (p.x >= 0) && (p.y < n_y_binned) && (p.y >= 0))
            frames[p.frame].at<float>(p.y, p.x)++;
      }
   }


   cv::Mat window;
   cv::createHanningWindow(window, cv::Size(n_x_binned, n_y_binned), CV_32F);

   frame_transform.clear();

   for (int f = 0; f<fb; f++)
      frame_transform.push_back(Transform());
  
   cv::Point2f centre_binned(n_x_binned / 2.0, n_y_binned / 2.0);
   cv::Point2f centre(n_x / 2.0, n_y / 2.0);

   cv::Mat log_polar0, log_polari, rotatedi;
   cv::logPolar(frames[0], log_polar0, centre, 1.0, CV_WARP_FILL_OUTLIERS);

   bool use_rotation = true;

   for (int i = 1; i < frames.size(); i++)
   {
      Transform transform;

      if (use_rotation)
      {
         cv::logPolar(frames[i], log_polari, centre, 1.0, CV_WARP_FILL_OUTLIERS);
         auto p = cv::phaseCorrelate(log_polar0, log_polari, window);
         double rotation = p.y * 90.0 / n_y_binned - 45.0;

         cv::Mat t = cv::getRotationMatrix2D(centre_binned, rotation, 1);
         cv::warpAffine(frames[i], rotatedi, t, frames[i].size());

         transform.affine = cv::getRotationMatrix2D(centre, rotation, 1);
      }
      else
      {
         rotatedi = frames[i];
      }

      auto p = cv::phaseCorrelate(frames[0], rotatedi, window);
     
      transform.shift = p * sb;

      for(int f=0; f<fb; f++)
         frame_transform.push_back(transform);
   }

}
/*
{
   vector<int> channels(n_chan);
   for (int i = 0; i < n_chan; i++)
      channels[i] = i;
   
   int sb = getSpatialBinning();

   setSpatialBinning(16);

   cv::Mat f1(n_x / spatial_binning, n_y / spatial_binning, CV_32F, cv::Scalar(0.0));
   cv::Mat f2(n_x / spatial_binning, n_y / spatial_binning, CV_32F, cv::Scalar(0.0));
   cv::Mat window, d;


   frame_shifts.clear();
   frame_shifts.push_back(cv::Point2d(0, 0));

   readData_(f1.ptr<float>(), channels, n_chan);
   cv::createHanningWindow(window, f1.size(), CV_32F);
   while (event_reader->hasMoreData())
   {
      readData_(f2.ptr<float>(), channels, 1);

      
      auto p = cv::phaseCorrelate(f1, f2, window);
      p *= spatial_binning;
      frame_shifts.push_back(p);
      
      //cv::Mat t = f1;
      //f2 = f1;
      //f1 = f2;
   }

   setSpatialBinning(sb);
   event_reader->setToStart();

}    
*/