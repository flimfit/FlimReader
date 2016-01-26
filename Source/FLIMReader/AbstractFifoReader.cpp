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
   assert(measurement_mode == 3);
   assert(event_reader != nullptr);
   
   event_reader->setToStart();
   
   double sync_count_accum = 0;
   double sync_start_count = 0;
   sync_count_per_line = 0;
   int n_averaged = 0;
   int n_frame = 0;
   int n_line = 0;
   bool line_started = false;
   do
   {
      PicoquantT3Event p = event_reader->getEvent();
      
      if (p.special)
      {
         if (p.dtime == 0)
            sync_count_accum += 0xFFFF;
         else
         {
            int marker = p.dtime;
            if (marker & 4)
            {
               n_frame++;
               if (n_frame >= 2)
                  break;
            }
            if (n_frame > 0)
            {
               if ((marker & 2) && line_started)
               {
                  double diff = sync_count_accum + p.nsync - sync_start_count;
                  sync_count_per_line += (sync_count_accum + p.nsync - sync_start_count);
                  n_averaged++;
                  line_started = false;
               }
               if (marker & 1)
               {
                  n_line++;
                  sync_start_count = sync_count_accum + p.nsync;
                  line_started = true;
               }
               
            }
         }
      }
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
   
}

void AbstractFifoReader::setTemporalResolution(int temporal_resolution)
{
   // Some formats give in resolution ns, some in s. Thanks Picoquant...
   // Convert both to picoseconds
   double time_resolution = resolution * ((resolution < 1e-9) ? 1e12 : 1e3);
   int native_resolution = 14 - static_cast<int>(log2(std::round(time_resolution)));
   
   
   temporal_resolution_ = std::min(native_resolution, temporal_resolution);
   temporal_resolution_ = std::max(0, temporal_resolution_);
   temporal_resolution = temporal_resolution_;
   
   int n_t = 1 << temporal_resolution_;
   timepoints_.resize(n_t);
   
   downsampling = (native_resolution - temporal_resolution_);
   int downsampling_factor = 1 << downsampling;
   
   double t_0 = 0;
   double t_step = time_resolution * downsampling_factor;
   
   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;
   
   t_rep_resunit = (int) std::round(t_rep_ps / time_resolution);
   
   time_shifts_resunit.clear();
   for(auto shift : time_shifts_ps)
      time_shifts_resunit.push_back((int) std::round(shift / time_resolution));
};

int AbstractFifoReader::numChannels()
{
   return routing_channels;
}


