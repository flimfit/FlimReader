#include "AbstractPicoquantReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

using namespace std;

AbstractPicoquantReader::AbstractPicoquantReader(const std::string& filename) :
FLIMReader(filename)
{
   readSettings();
}

void AbstractPicoquantReader::readSettings()
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
      
      time_shifts_ps[0] = tree.get<double>("shifts.1", 0);
      time_shifts_ps[1] = tree.get<double>("shifts.2", 0);
      time_shifts_ps[2] = tree.get<double>("shifts.3", 0);
      time_shifts_ps[3] = tree.get<double>("shifts.4", 0);
   }
}

void AbstractPicoquantReader::determineDwellTime()
{
   assert(measurement_mode == 3);
   
   ifstream fs(filename, ifstream::in | ifstream::binary);
   fs.seekg(data_position, ios_base::beg);
   
   double sync_count_accum = 0;
   double sync_start_count = 0;
   sync_count_per_line = 0;
   int n_averaged = 0;
   int n_frame = 0;
   int n_line = 0;
   bool line_started = false;
   do
   {
      uint32_t evt;
      READ(fs, evt);
      PicoquantT3Event p(evt);
      
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
   } while (!fs.eof());
   
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

void AbstractPicoquantReader::setTemporalResolution(int temporal_resolution)
{
   // Some formats give in resolution ns, some in s. Thanks Picoquant...
   // Convert both to picoseconds
   double time_resolution = resolution * ((resolution < 1e-9) ? 1e12 : 1e3);
   int native_resolution = 14 - log2(std::round(time_resolution));
   
   
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
   
   t_rep_resunit = std::round(t_rep_ps / time_resolution);
   
   time_shifts_resunit.clear();
   for(auto shift : time_shifts_ps)
      time_shifts_resunit.push_back(std::round(shift / time_resolution));
};

int AbstractPicoquantReader::numChannels()
{
   return routing_channels;
}


