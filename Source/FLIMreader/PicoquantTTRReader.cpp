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
   time_resolution_native_ps = hw_info.resolution * 1e3;
   n_x = info.n_x;
   n_y = info.n_y;
   rep_rate_hz = info.input0_countrate;
   t_rep_ps = 1e12f / info.input0_countrate; // rep time in picoseconds
      
   int n_bits = 14 - info.range_no;
   n_timebins_native = 1 << n_bits;

   int n_timebins_useful = (int) ceil(t_rep_ps / time_resolution_native_ps); // how many timebins are actually useful?

   n_timebins_native = min(n_timebins_native, n_timebins_useful);

   if (measurement_mode != 3)
      throw std::runtime_error("Picoquant data should be recorded in TTTR Mode 3 for FLIM analysis");

   PicoquantRecordType rec_type = (std::string("PicoHarp 300").compare(info.ident) == 0) ? PicoHarp_T3 : HydraHarpV2_T3;

   event_reader = std::unique_ptr<AbstractEventReader>(new PicoquantEventReader(filename, data_position, rec_type));

   setTemporalResolution(14);
   determineDwellTime();
}

void PicoquantTTTRReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);

   if (!fs.is_open())
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

   fs.ignore(20); // display settings+
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
         markers.FrameMarker = 0x4;
         markers.LineEndMarker = 0x2;
         markers.LineStartMarker = 0x1;
         markers.PixelMarker = 0x0; // no pixel marker
         break;
      case 4: // KDT180-100-lm
         fs.ignore(6 * 4);
         READ(fs, info.n_x);
         READ(fs, info.n_y);
         fs.ignore(4);
         markers.FrameMarker = 0x4;
         markers.LineEndMarker = 0x2;
         markers.LineStartMarker = 0x1;
         markers.PixelMarker = 0x0; // no pixel marker
         break;
      case 3: // LSM
         //fs.ignore(4 * 4);
         int32_t frame, linestart, linestop, pattern;
         READ(fs, frame);
         READ(fs, linestart);
         READ(fs, linestop);
         READ(fs, pattern);
         markers.FrameMarker = 1 << (frame-1);
         markers.LineStartMarker = 1 << (linestart-1);
         markers.LineEndMarker = 1 << (linestop-1);
         markers.PixelMarker = 0x0; // no pixel marker
         sync.bi_directional = (pattern == 1);
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


   typedef std::string s;
   tags[s(s("ident"))] = s(info.ident);
   tags[s("format_version")] = s(info.format_version);
   tags[s("creator_name")] = s(info.creator_name);
   tags[s("creator_version")] = s(info.creator_version);
   tags[s("file_time")] = s(info.file_time);
   tags[s("comment")] = s(info.comment);

   tags[s("n_curves")] = info.n_curves;
   tags[s("bits_per_record")] = info.bits_per_record;
   tags[s("routing_channels")] = info.routing_channels;
   tags[s("n_boards")] = info.n_boards;
   tags[s("active_curve")] = info.active_curve;
   tags[s("measurement_mode")] = info.measurement_mode;
   tags[s("sub_mode")] = info.sub_mode;
   tags[s("range_no")] = info.range_no;
   tags[s("offset")] = info.offset;
   tags[s("acq_time")] = info.acq_time;
   tags[s("stop_at")] = info.stop_at;
   tags[s("stop_on_overflow")] = info.stop_on_overflow;
   tags[s("restart")] = info.restart;
   tags[s("repeat_mode")] = info.repeat_mode;
   tags[s("repeats_per_curve")] = info.repeats_per_curve;
   tags[s("repeat_time")] = info.repeat_time;
   tags[s("repeat_wait_time")] = info.repeat_wait_time;
   tags[s("script_name")] = s(info.script_name);

   tags[s("ident")] = s(hw_info.ident);
   tags[s("version")] = s(hw_info.version);
   tags[s("serial")] = hw_info.serial;
   tags[s("sync_divider")] = hw_info.sync_divider;
   tags[s("resolution")] = hw_info.resolution;
   tags[s("input0_countrate")] = info.input0_countrate;
   tags[s("input1_countrate")] = info.input1_countrate;
   tags[s("stop_after")] = info.stop_after;
   tags[s("stop_reason")] = info.stop_reason;
   tags[s("n_records")] = info.n_records;

}

