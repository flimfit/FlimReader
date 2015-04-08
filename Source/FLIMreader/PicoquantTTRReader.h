#pragma once

#include "FLIMreader.h"

#include <fstream>
#include <stdint.h>

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

   PicoquantTTTRReader(const std::string& filename) :
      FLIMReader(filename)
   {
      ReadHeader();
      SetTemporalResolution(8);
      DetermineDwellTime();

      n_x = info.n_x;
      n_y = info.n_y;
   }

   void SetTemporalResolution(int temporal_resolution);

   void ReadHeader();

   int GetNumberOfChannels();
   std::vector<double> GetTimePoints();


   int GetDataSizePerChannel();
   void ReadData(const std::vector<int>& channels, float* data);

protected:

   void DetermineDwellTime();

   PicoquantTTRInfo info;
   PicoquantHardwareInfo hw_info;
   long long data_position;
   double sync_count_per_line;
};