#include "AbstractPicoquantReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

using namespace std;

AbstractPicoquantReader::AbstractPicoquantReader(const std::string& filename) :
FLIMReader(filename)
{
   
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
   
   if (n_y == 0)
   {
      n_y = n_line;
      if (n_x == 0)
         n_x = n_line;
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
   double t_step = time_resolution * downsampling_factor; // convert ns->ps
   
   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;
};

int AbstractPicoquantReader::numChannels()
{
   return routing_channels;
}


