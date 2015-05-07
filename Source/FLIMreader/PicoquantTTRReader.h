#pragma once

#include "FLIMReader.h"

#include <fstream>
#include <cstdint>
#include <cassert>
#include <cmath>

struct PicoquantHardwareInfo
{
   char ident[16];
   char version[8];
   int32_t serial;
   int32_t sync_divider;
   float resolution;
};

struct PicoquantTTRInfo
{
   char ident[16];
   char format_version[6];
   char creator_name[18];
   char creator_version[12];
   char file_time[18];
   char comment[256];

   int32_t n_curves;
   int32_t bits_per_record;
   int32_t routing_channels;
   int32_t n_boards;
   int32_t active_curve;
   int32_t measurement_mode;
   int32_t sub_mode;
   int32_t range_no;
   int32_t offset;
   int32_t acq_time;
   int32_t stop_at;
   int32_t stop_on_overflow;
   int32_t restart;

   int32_t repeat_mode;
   int32_t repeats_per_curve;
   int32_t repeat_time;
   int32_t repeat_wait_time;
   char script_name[20];

   int32_t input0_countrate;
   int32_t input1_countrate;
   int32_t stop_after;
   int32_t stop_reason;
   int32_t n_records;
   int32_t spec_header_length;

   int32_t dimensions;
   int32_t imaging_ident;
   int32_t n_x;
   int32_t n_y;
};

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

class PicoquantTTTRReader : public FLIMReader
{
public:

   PicoquantTTTRReader(const std::string& filename);

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };

   void setTemporalResolution(int temporal_resolution);

   int numChannels();
   int dataSizePerChannel();



protected:

   void readHeader();

   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);

   void determineDwellTime();

   PicoquantTTRInfo info;
   PicoquantHardwareInfo hw_info;
   long long data_position;
   double sync_count_per_line;
   int downsampling;
};


template<typename T>
void PicoquantTTTRReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   using namespace std;

   auto channels = validateChannels(channels_, n_chan_stride);

   assert(info.measurement_mode == 3);

   // Determine channel mapping
   std::vector<int> channel_map(info.routing_channels, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;

   int n_bin = 1 << temporal_resolution_;

   ifstream fs(filename, ifstream::in | ifstream::binary);
   fs.seekg(data_position, ios_base::cur);

   long sync_count_accum = 0;
   int cur_line = 0;
   bool frame_started = 0;
   bool line_valid = false;
   int sync_start = 0;

   vector<uint32_t> records(info.n_records);
   fs.read(reinterpret_cast<char*>(records.data()), info.n_records*sizeof(uint32_t));

   int max_t = 0;

   for (int i = 0; i < info.n_records; i++)
   {
      PicoquantT3Event p(records[i]);

      int cur_sync = p.nsync + sync_count_accum;

      if (p.special)
      {
         if (p.dtime == 0)
            sync_count_accum += 0xFFFF;
         else
         {
            int marker = p.dtime;

            if (marker == 4)
            {
               frame_started = true;
               cur_line = 0;
            }

            if ((marker == 1) && frame_started)
            {
               line_valid = true;
               sync_start = cur_sync;
            }

            if (marker == 2)
            {
               line_valid = false;
               cur_line++;
            }

         }
      }
      else if (line_valid)
      {
         int mapped_channel = channel_map[p.channel-1];
         if (mapped_channel > -1)
         {
            double cur_loc = (cur_sync - sync_start) / sync_count_per_line * info.n_x;
            int cur_px = static_cast<int>(cur_loc);

            if (p.dtime > max_t)
               max_t = p.dtime;

            int bin = p.dtime >> downsampling;
            if (bin < n_bin)
               histogram[bin + n_bin * (mapped_channel + n_chan_stride * (cur_px + info.n_x * cur_line))]++;
         }
      }
   }
}