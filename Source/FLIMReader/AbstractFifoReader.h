#pragma once
#include "FLIMReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>

class Markers
{
public:
   uint8_t PhotonMarker = 0x0;
   uint8_t LineStartMarker = 0x0;
   uint8_t LineEndMarker = 0x0;
   uint8_t FrameMarker = 0x0;
   uint8_t Invalid = 0x80;
};


class TcspcEvent
{
public:
   uint64_t macro_time = 0;
   uint32_t micro_time = 0;
   uint8_t channel = 0;
   uint8_t mark = 0;
   uint64_t macro_time_offset = 0;
   bool valid = true;
};

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, int data_position) :
   data_position(data_position)
   {
      fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
      setToStart();
   }
   
   void setToStart()
   {
      fs.clear();
      fs.seekg(data_position, std::ios_base::beg);
   }

   virtual bool hasMoreData()
   {
      return !fs.eof();
   }

   virtual TcspcEvent getEvent() = 0;
   
protected:
   
   std::ifstream fs;
   int data_position;
};

class AbstractFifoReader : public FLIMReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   void setTemporalResolution(int temporal_resolution);
      
protected:
   
   void readSettings();
   
   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   
   void determineDwellTime();
   
   double sync_count_per_line;
   double sync_offset = 0;
   double first_line_sync_offset = 0;
   double scan_active_fraction = 1;
   
   int line_averaging = 1;
   int downsampling;
   
   std::vector<float> time_shifts_ps;
   
   // Required Picoquant information
   int measurement_mode = 0;
   double time_resolution_native_ps;
   float t_rep_ps;
   int n_timebins_native;

   bool bi_directional = false;
   int cur_direction = 1;

   std::unique_ptr<AbstractEventReader> event_reader = nullptr;
   Markers markers;
   
private:
   
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;
};



template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   using namespace std;

   auto channels = validateChannels(channels_, n_chan_stride);
   
   assert(event_reader != nullptr);
   
   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;
   
   int n_bin = 1 << temporal_resolution;
   
   event_reader->setToStart();
   
   long long sync_count_accum = 0;
   int cur_line = 0;
   bool frame_started = markers.FrameMarker == 0x0;
   bool line_valid = false;
   long long sync_start = 0;
      
   int n_x_binned = n_x / spatial_binning;
   int n_y_binned = n_y / spatial_binning;
   
   int n_frame = 0;
   int n_invalid = 0;

   auto incrementFrame = [&]() {
      n_frame++;
      frame_started = true;
      cur_line = -1;
      cur_direction = 1;
   };
   
   while (event_reader->hasMoreData())
   {
      TcspcEvent p = event_reader->getEvent();

      sync_count_accum += p.macro_time_offset;
      long long cur_sync = p.macro_time + sync_count_accum;

      if (p.valid)
      {
         if (p.mark & markers.FrameMarker)
            incrementFrame();

         if (frame_started)
         {
            if ((p.mark & markers.LineEndMarker) && line_valid)
            {
               line_valid = false;
            }
            else if (p.mark & markers.LineStartMarker)
            {
               line_valid = true;
               sync_start = cur_sync;
               cur_line++;

               if ((cur_line >= n_y) && markers.FrameMarker == 0x0)
                  incrementFrame();

               if (bi_directional) 
                  cur_direction *= -1;
            }
         }

         if ((p.mark == markers.PhotonMarker) && line_valid && frame_started)
         {
            int mapped_channel = channel_map[p.channel];
            if (mapped_channel > -1)
            {
               double cur_loc = ((cur_sync - sync_start) / sync_count_per_line - sync_offset) * (n_x_binned);

               int cur_px = static_cast<int>(cur_loc);

               if (cur_direction == -1)
                  cur_px = n_x_binned - 1 - cur_px;

               int dtime = (p.micro_time + time_shifts_resunit[p.channel]) % t_rep_resunit;
               dtime = dtime < 0 ? dtime + t_rep_resunit : dtime;
               int bin = dtime >> downsampling;

               int x = cur_px;
               int y = cur_line / (spatial_binning * line_averaging);

               if ((bin < n_bin) && (x < n_x_binned) && (x >= 0) && (y < n_y_binned) && (y >= 0))
                  histogram[bin + n_bin * (mapped_channel + n_chan_stride * (x + n_x_binned * y))]++;
               else
                  n_invalid++;
            }
         }
      }
   }
   
   std::cout << "Num frames: " << n_frame << "\n";
   std::cout << "Num invalid: " << n_invalid << "\n";

}