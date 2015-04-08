#include "PicoquantTTRReader.h"
#include <cassert>
#include <algorithm>

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

void PicoquantTTTRReader::ReadHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);

   assert(fs.is_open());

   READ(fs, info.ident);
   READ(fs, info.format_version);
   READ(fs, info.creator_name);
   READ(fs, info.creator_version);
   READ(fs, info.file_time);
   fs.ignore(2);
   READ(fs, info.comment);

   READ(fs, info.n_curves);
   READ(fs, info.bits_per_record);
   READ(fs, info.routing_channels);
   READ(fs, info.n_boards);
   READ(fs, info.active_curve);
   READ(fs, info.measurement_mode);
   READ(fs, info.sub_mode);
   READ(fs, info.range_no);
   READ(fs, info.offset);
   READ(fs, info.acq_time);
   READ(fs, info.stop_at);
   READ(fs, info.stop_on_overflow);
   READ(fs, info.restart);

   fs.ignore(20); // display settings
   fs.ignore(64); // curve display settings
   fs.ignore(3 * 3 * 4); // param settings

   READ(fs, info.repeat_mode);
   READ(fs, info.repeats_per_curve);
   READ(fs, info.repeat_time);
   READ(fs, info.repeat_wait_time);
   READ(fs, info.script_name);

   READ(fs, hw_info.ident);
   READ(fs, hw_info.version);
   READ(fs, hw_info.serial);
   READ(fs, hw_info.sync_divider);
   fs.ignore(4 * 4); // cfd settings
   READ(fs, hw_info.resolution);
   fs.ignore(26 * 4); // router settings

   fs.ignore(12); // reserved

   READ(fs, info.input0_countrate);
   READ(fs, info.input1_countrate);
   READ(fs, info.stop_after);
   READ(fs, info.stop_reason);
   READ(fs, info.n_records);
   READ(fs, info.spec_header_length);

   std::streampos p = fs.tellg();

   if (info.spec_header_length > 2)
   {
      READ(fs, info.dimensions);
      READ(fs, info.imaging_ident);

      switch (info.imaging_ident)
      {
      case 1: // PI E710
         fs.ignore(6 * 4);
         READ(fs, info.n_x);
         READ(fs, info.n_y);
         fs.ignore(5 * 4);
         break;
      case 4: // KDT180-100-lm
         fs.ignore(6 * 4);
         READ(fs, info.n_x);
         READ(fs, info.n_y);
         fs.ignore(4);
         break;
      case 3: // LSM
         //fs.ignore(4 * 4);
         int32_t frame, linestart, linestop, pattern;
         READ(fs, frame);
         READ(fs, linestart);
         READ(fs, linestop);
         READ(fs, pattern);
         
         READ(fs, info.n_x);
         READ(fs, info.n_y);
         break;
      }
   }
   else
   {
      info.dimensions = 0;
      info.imaging_ident = 0;
      info.n_x = 1;
      info.n_y = 1;
   }

   fs.seekg(p);
   fs.seekg(info.spec_header_length * 4, ios_base::cur);

   data_position = fs.tellg();


   assert(strcmp(info.ident, "PicoHarp 300") == 0);
}

void PicoquantTTTRReader::DetermineDwellTime()
{
   assert(info.measurement_mode == 3);

   ifstream fs(filename, ifstream::in | ifstream::binary);
   fs.seekg(data_position, ios_base::cur);

   long sync_count_accum = 0;
   long sync_start_count = 0;
   sync_count_per_line = 0;
   int n_averaged = 0;
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
            if (marker == 1)
            {
               sync_start_count = sync_count_accum + p.nsync;
            }
            if (marker == 2 && (sync_start_count > 0))
            {
               sync_count_per_line += sync_count_accum + p.nsync - sync_start_count;
               n_averaged++;
            }
         }
      }
   } while (!fs.eof() && (n_averaged < info.n_y));

   sync_count_per_line /= n_averaged;
}

void PicoquantTTTRReader::SetTemporalResolution(int temporal_resolution_)
{
   temporal_resolution_ = std::min(10, temporal_resolution_);
   temporal_resolution_ = std::max(0, temporal_resolution_);
   temporal_resolution = temporal_resolution_;

   int n_t = 1 << temporal_resolution;
   timepoints.resize(n_t);

   int downsampling_factor = 1 << (10 - temporal_resolution);

   double t_0 = 0;
   double t_step = hw_info.resolution * downsampling_factor;

   for (int i = 0; i < n_t; i++)
      timepoints[i] = t_0 + i * t_step;
};

int PicoquantTTTRReader::GetNumberOfChannels() 
{ 
   return info.routing_channels; 
}

int PicoquantTTTRReader::GetDataSizePerChannel()
{
   int n_bin = 1 << temporal_resolution;
   return n_bin * info.n_x * info.n_y;
}

void PicoquantTTTRReader::ReadData(const std::vector<int>& channels, float* histogram)
{
   assert(info.measurement_mode == 3);

   // Determine channel mapping
   size_t n_channel = channels.size();
   std::vector<int> channel_map(info.routing_channels, -1);   
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;

   int downsampling = 10 - temporal_resolution;
   int n_bin = 1 << temporal_resolution;
   
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
         int mapped_channel = channel_map[p.channel];
         if (mapped_channel > -1)
         {
            int cur_px = round((cur_sync-sync_start) / sync_count_per_line * info.n_x);

            if (p.dtime > max_t)
               max_t = p.dtime;

            int bin = p.dtime >> downsampling;
            if (bin < n_bin)
               histogram[bin + n_bin * (mapped_channel + n_channel * (cur_px + info.n_x * cur_line))]++;
         }
      }
   }

   max_t = max_t;
}