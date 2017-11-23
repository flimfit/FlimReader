#include "PicoquantPTUReader.h"
#include "PicoquantHelper.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

PicoquantPTUReader::PicoquantPTUReader(const std::string& filename) :
AbstractFifoReader(filename)
{
   readHeader();

   n_timebins_native = 1 << 14;
   n_timebins_native /= temporal_binning;

   int n_timebins_useful = (int) ceil(t_rep_ps / time_resolution_native_ps); // how many timebins are actually useful?
   n_timebins_native = min(n_timebins_native, n_timebins_useful);

   // Default if markers aren't specified in data
   if (markers.LineEndMarker == 0x00 && markers.LineStartMarker == 0x00)
   {
      markers.FrameMarker = 0x4;
      markers.LineEndMarker = 0x2;
      markers.LineStartMarker = 0x1;
      markers.PixelMarker = 0x0; // no pixel marker
   }

   setTemporalResolution(14);
   assert(measurement_mode == 3);

   event_reader = std::shared_ptr<AbstractEventReader>(new PicoquantEventReader(filename, data_position, rec_type));
      
   determineDwellTime();
}

void PicoquantPTUReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);
   
   char magic[8];
   char version[8];
   std::vector<char> data;

   fs.read(magic, sizeof(magic));
   
   if (std::string("PQTTTR") != magic)
      throw runtime_error("Wrong magic string, this is not a PTU file");

   n_chan = 1;
   markers.FrameMarker = 0; // No frame marker by default

   
   fs.read(version, sizeof(version));

   TagHead tag_head;
   do
   {
      fs.read(reinterpret_cast<char*>(&tag_head), sizeof(tag_head));
      
      cout << "\n" << tag_head.Ident << " : ";

      switch (tag_head.Typ)
      {
         case tyEmpty8:
            cout << "<empty Tag>";
            break;
            
         case tyBool8:
            cout << (tag_head.TagValue != 0);
            if (strcmp(tag_head.Ident, ImgHdr_BiDirect) == 0)
               sync.bi_directional = tag_head.TagValue != 0;
            break;
            
         case tyInt8:
            cout << tag_head.TagValue;
            if (strcmp(tag_head.Ident, TTTRTagTTTRRecType) == 0)
               rec_type = (PicoquantRecordType) tag_head.TagValue;
            if (strcmp(tag_head.Ident, Measurement_Mode)==0) // measurement mode
               measurement_mode = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, HWRouter_Channels)==0)
               n_chan = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, HW_InpChannels) == 0)
               n_chan = ((int)tag_head.TagValue) - 1; // start is included in input count
            if (strcmp(tag_head.Ident, Line_Averaging)==0)
               line_averaging = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, TTResult_SyncRate) == 0)
            {
               rep_rate_hz = (double) tag_head.TagValue;
               t_rep_ps = 1e12 / tag_head.TagValue;
            }
            if (strcmp(tag_head.Ident, MeasDesc_BinningFactor)==0)
               temporal_binning = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, ImgHdr_LineStart)==0)
               markers.LineStartMarker = 1 << (tag_head.TagValue - 1);
            if (strcmp(tag_head.Ident, ImgHdr_LineStop) == 0)
               markers.LineEndMarker = 1 << (tag_head.TagValue - 1);
            if (strcmp(tag_head.Ident, ImgHdr_Frame) == 0)
               markers.FrameMarker = 1 << (tag_head.TagValue - 1);
            if (strcmp(tag_head.Ident, ImgHdr_PixX) == 0)
               n_x = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, ImgHdr_PixY) == 0)
            {
               n_y = (int) tag_head.TagValue;
               sync.n_line = n_y;
            }
            break;
            
         case tyBitSet64:
            cout << tag_head.TagValue;
            break;
         
         case tyColor8:
            cout << tag_head.TagValue;
            break;
         
         case tyFloat8:
            cout << *(double*)&(tag_head.TagValue);
            if (strcmp(tag_head.Ident, TTTRTagRes)==0) // Resolution for TCSPC-Decay
               time_resolution_native_ps = *(double*)&(tag_head.TagValue) * 1e12;
            break;
         
         case tyTDateTime:
            break;
         
         case tyAnsiString:
         case tyFloat8Array:
         case tyWideString:
         case tyBinaryBlob:
            data.resize(tag_head.TagValue);
            fs.read(data.data(), tag_head.TagValue);
            cout << std::string(data.data());
            break;
         
         default:
            throw runtime_error("Illegal Type identifier found! Broken file?");
      }
              
   } while (strncmp(tag_head.Ident, FileTagEnd, sizeof(FileTagEnd)));
   
   data_position = fs.tellg();
}




PicoquantEventReader::PicoquantEventReader(const std::string& filename, std::streamoff data_position, PicoquantRecordType rec_type)
   : AbstractEventReader(filename, data_position, sizeof(uint32_t)), rec_type(rec_type)
{}

std::tuple<FifoEvent, uint64_t>  PicoquantEventReader::getRawEvent()
{
   uint32_t evt = getPacket<uint32_t>();

   switch (rec_type)
   {
   case PicoHarp_T3:
      return getPicoharpT3Event(evt);
   case HydraHarpV1_T3:
   case HydraHarpV2_T3:
   case TimeHarp260N_T3:
   case TimeHarp260P_T3:
      return getTimeharpT3Event(evt, rec_type);
   default:
      throw std::runtime_error("Unsupported record type");
   }
}

std::tuple<FifoEvent, uint64_t> PicoquantEventReader::getPicoharpT3Event(uint32_t evt)
{
   FifoEvent e;
   uint64_t macro_time_offset = 0;

   int nsync = evt & 0xFFFF; evt >>= 16;
   int dtime = evt & 0xFFF; evt >>= 12;
   e.channel = evt & 0xF;

   e.micro_time = dtime;
   e.macro_time = nsync;

   if (e.channel == 15) // special
   {
      if (dtime == 0) // macro time overflow
      {
         macro_time_offset = 0xFFFF;
         e.mark = 0x80;
         e.valid = false;
      }
      else
      {
         e.mark = dtime;
      }
   }

   e.channel--; // 1-indexing -> 0-indexing

   return std::tuple<FifoEvent, uint64_t>(e, macro_time_offset);
}

std::tuple<FifoEvent, uint64_t> PicoquantEventReader::getTimeharpT3Event(uint32_t evt, PicoquantRecordType rec_type)
{
   FifoEvent e;
   uint64_t macro_time_offset = 0;

   int nsync = evt & 0x3FF; evt >>= 10;
   int dtime = evt & 0x7FFF; evt >>= 15;
   e.channel = evt & 0x3F; evt >>= 6;
   bool special = evt & 0x1;

   e.micro_time = dtime;
   e.macro_time = nsync;

   if (special) // special
   {
      if (e.channel == 0x3F) // macro time overflow
      {
         int n_overflow = (rec_type == HydraHarpV1_T3) ? 1 : nsync;
         macro_time_offset = 0x3FF * n_overflow;
         e.mark = 0x80;
         e.valid = false;
      }
      else
      {
         e.mark = e.channel;
      }
   }

   return std::tuple<FifoEvent, uint64_t>(e, macro_time_offset);
}
