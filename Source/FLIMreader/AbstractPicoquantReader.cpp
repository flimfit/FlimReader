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
   
   long sync_count_accum = 0;
   long sync_start_count = 0;
   sync_count_per_line = 0;
   int n_averaged = 0;
   int n_frame = 0;
   int n_line = 0;
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
            if (marker & 1)
            {
               if (n_frame == 1)
                  n_line++;
               sync_start_count = sync_count_accum + p.nsync;
            }
            if ((marker & 2) && (sync_start_count > 0))
            {
               sync_count_per_line += sync_count_accum + p.nsync - sync_start_count;
               n_averaged++;
            }
            if (marker & 4)
            {
               n_frame++;
               if (n_frame >= 2)
                  break;
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
   double time_resolution = resolution * 1000;
   int native_resolution = 14 - log2(time_resolution);
   
   
   temporal_resolution_ = std::min(native_resolution, temporal_resolution);
   temporal_resolution_ = std::max(0, temporal_resolution_);
   temporal_resolution = temporal_resolution_;
   
   int n_t = 1 << temporal_resolution_;
   timepoints_.resize(n_t);
   
   downsampling = (native_resolution - temporal_resolution_);
   int downsampling_factor = 1 << downsampling;
   
   double t_0 = 0;
   double t_step = resolution * downsampling_factor * 1e3; // convert ns->ps
   
   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;
};

int AbstractPicoquantReader::numChannels()
{
   return routing_channels;
}


