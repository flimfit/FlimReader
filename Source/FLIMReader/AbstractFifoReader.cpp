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
   sync_count_per_line = 0;
   int n_averaged = 0;
   int n_line = 0;
   int n_frame = 0;

   bool line_started = false;
   do
   {
      TcspcEvent p = event_reader->getEvent();

      sync_count_accum += p.macro_time_offset;
      uint64_t macro_time = sync_count_accum + p.macro_time;

      if (!p.valid)
         continue;

      if (p.mark & markers.FrameMarker)
         n_frame++;

      if (n_frame > 0 || markers.FrameMarker == 0x0)
      {
         if ((p.mark & markers.LineEndMarker) && line_started)
         {
            uint64_t diff = macro_time - sync_start_count;
            sync_count_per_line += (macro_time - sync_start_count);
            n_averaged++;
            line_started = false;
         }
         else if (p.mark & markers.LineStartMarker)
         {
            n_line++;
            sync_start_count = macro_time;
            line_started = true;
         }
      }

      // if we don't have frame markers break after 512 lines (speed considerations)
      if (markers.FrameMarker == 0x0 && n_line >= n_y)
         break;

   } while (event_reader->hasMoreData() && n_frame < 2);
   
   sync_count_per_line /= n_averaged;
   
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

