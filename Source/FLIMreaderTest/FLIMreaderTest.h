#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <stdint.h>

class FLIMReader
{
public:
   
   FLIMReader(const std::string& filename)
   {
   }

   virtual void ReadHeader() {};

   virtual int GetNumberOfChannels() = 0;
   virtual std::vector<double> GetTimePoints() = 0;

   virtual void GetData(double* data) = 0;

protected:
   std::string filename;
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
};

class PicoquantTTTRReader : FLIMReader
{
public:
   void ReadHeader();

protected:

   PicoquantTTRInfo info;
};