#include "PicoquantTTRReader.h"
#include <cassert>
#include <algorithm>

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

PicoquantTTTRReader::PicoquantTTTRReader(const std::string& filename) :
FLIMReader(filename)
{
   readHeader();
   setTemporalResolution(8);
   determineDwellTime();

   n_x = info.n_x;
   n_y = info.n_y;
}

void PicoquantTTTRReader::readHeader()
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

void PicoquantTTTRReader::determineDwellTime()
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

void PicoquantTTTRReader::setTemporalResolution(int temporal_resolution)
{
   temporal_resolution = std::min(10, temporal_resolution);
   temporal_resolution = std::max(0, temporal_resolution);
   temporal_resolution_ = temporal_resolution;

   int n_t = 1 << temporal_resolution_;
   timepoints_.resize(n_t);

   int downsampling_factor = 1 << (10 - temporal_resolution_);

   double t_0 = 0;
   double t_step = hw_info.resolution * downsampling_factor;

   for (int i = 0; i < n_t; i++)
      timepoints_[i] = t_0 + i * t_step;
};

int PicoquantTTTRReader::numChannels() 
{ 
   return info.routing_channels; 
}

int PicoquantTTTRReader::dataSizePerChannel()
{
   int n_bin = 1 << temporal_resolution_;
   return n_bin * info.n_x * info.n_y;
}
