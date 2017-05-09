#include "PicoquantTTRReader.h"
#include <cassert>
#include <algorithm>
#include <string>
#include <cmath>

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

PicoquantTTTRReader::PicoquantTTTRReader(const std::string& filename) :
AbstractFifoReader(filename)
{
   readHeader();
   
   n_chan = info.routing_channels;
   measurement_mode = info.measurement_mode;
   //n_records = info.n_records;
   time_resolution_native_ps = hw_info.resolution * 1e3;
   n_x = info.n_x;
   n_y = info.n_y;
   t_rep_ps = 1e12f / info.input0_countrate; // rep time in picoseconds
      
   int n_bits = 14 - info.range_no;
   n_timebins_native = 1 << n_bits;

   int n_timebins_useful = (int) ceil(t_rep_ps / time_resolution_native_ps); // how many timebins are actually useful?

   n_timebins_native = min(n_timebins_native, n_timebins_useful);

   assert(measurement_mode == 3);

   markers.FrameMarker = 0x4;
   markers.LineEndMarker = 0x2;
   markers.LineStartMarker = 0x1;

   event_reader = std::unique_ptr<AbstractEventReader>(new PicoquantEventReader(filename, data_position));

   setTemporalResolution(14);
   determineDwellTime();
}

void PicoquantTTTRReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);

   if(!fs.is_open())
      throw std::runtime_error("Could not open file");
   
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
   
   assert(std::string("PicoHarp 300").compare(info.ident) == 0);
}

