#pragma once
#include "FLIMReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>

class PicoquantT3Event
{
public:
   PicoquantT3Event(uint32_t evt)
   {
      nsync = evt & 0xFFFF; evt >>= 16;
      dtime = evt & 0xFFF; evt >>= 12;
      channel = evt & 0xF;
      
      special = (channel == 15);
   }
   
   bool special;
   int channel;
   int dtime;
   int nsync;
};


class AbstractPicoquantReader : public FLIMReader
{
public:
   
   AbstractPicoquantReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   void setTemporalResolution(int temporal_resolution);
   
   int numChannels();
   
protected:
   
   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   
   void determineDwellTime();
   
   long long data_position = 0;
   double sync_count_per_line;
   int downsampling;
   
   // Required Picoquant information
   int routing_channels;
   int measurement_mode;
   int n_records;
   float resolution;
};



template<typename T>
void AbstractPicoquantReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   using namespace std;
   
   auto channels = validateChannels(channels_, n_chan_stride);
   
   assert(measurement_mode == 3);
   
   // Determine channel mapping
   std::vector<int> channel_map(routing_channels, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;
   
   int n_bin = 1 << temporal_resolution_;
   
   ifstream fs(filename, ifstream::in | ifstream::binary);
   
   fs.seekg(0, ios_base::end);
   int end_pos = fs.tellg();

   int n_records_true = (end_pos - data_position) / 4;
   
   
   fs.seekg(data_position, ios_base::beg);
   
   long sync_count_accum = 0;
   int cur_line = 0;
   bool frame_started = 0;
   bool line_valid = false;
   bool auto_off = false;
   int sync_start = 0;
   
   
   vector<uint32_t> records(n_records_true);
   fs.read(reinterpret_cast<char*>(records.data()), n_records_true*sizeof(uint32_t));
   
   int n_x_binned = n_x / spatial_binning_;
   
   int n_frame = 0;
   
   for (int i = 0; i < n_records_true; i++)
   {
      PicoquantT3Event p(records[i]);
      
      int cur_sync = p.nsync + sync_count_accum;
      
      if (auto_off && ((cur_sync - sync_start) > sync_count_per_line))
      {
         auto_off = false;
         line_valid = false;
         cur_line++;
      }
      
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
               frame_started = true;
               cur_line = 0;
            }
            
            if (frame_started)
            {
               if (marker & 3)
               {
                  line_valid = true;
                  sync_start = cur_sync;
                  auto_off = true;
               }
               else if (marker & 1)
               {
                  line_valid = true;
                  sync_start = cur_sync;
               }
               else if (marker & 2)
               {
                  line_valid = false;
                  double dsync = cur_sync - sync_start;
                  cur_line++;
               }
            }
            
         }
      }
      else if (line_valid)
      {
         int mapped_channel = channel_map[p.channel-1];
         if (mapped_channel > -1)
         {
            double dsync = cur_sync - sync_start;
            double cur_loc = (cur_sync - sync_start) / sync_count_per_line * n_x_binned;
            int cur_px = static_cast<int>(cur_loc);
            
            int bin = p.dtime >> downsampling;
            int x = cur_px;
            int y = cur_line / spatial_binning_;
            
            assert(x < n_x_binned);
            assert(y < n_x_binned);
            
            if (bin < n_bin)
               histogram[bin + n_bin * (mapped_channel + n_chan_stride * (x + n_x_binned * y))]++;
         }
      }
   }
}