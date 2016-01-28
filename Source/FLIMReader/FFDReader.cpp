#include "FFDReader.h"

#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <functional>

using namespace std;


#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

FfdReader::FfdReader(const std::string& filename) :
   AbstractFifoReader(filename)
{
   readHeader();

   setTemporalResolution(8);

   markers.FrameMarker = 0x8;
   markers.LineEndMarker = 0x4;
   markers.LineStartMarker = 0x2;

   event_reader = std::make_unique<FfdEventReader>(filename, data_position, use_compression, message_size);


   determineDwellTime();
}

void FfdReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);
   uint32_t magic, version;

   READ(fs, magic);

   if (magic != 0xF1F0)
      throw runtime_error("Wrong magic string, this is not a valid FFD file");

   READ(fs, version);
   READ(fs, data_position);


   char tag_name[255];
   uint32_t tag_name_length, tag_data_length;
   uint16_t tag_type;

   auto isTag = [&](const char* t) { return strncmp(t, tag_name, tag_name_length-1) == 0; };

   do
   {
      READ(fs, tag_name_length);
      
      tag_name_length = std::min(tag_name_length, (uint32_t)255);
      fs.read(tag_name, tag_name_length);
      cout << "\n" << tag_name << " : ";

      READ(fs, tag_type);
      READ(fs, tag_data_length);

      vector<char> data(tag_data_length);
      char* data_ptr = data.data();
      fs.read(data_ptr, tag_data_length);

      if (tag_type == TagDouble)
      {
         double value = *(double*)data_ptr;
         std::cout << value;

         if (isTag("MicrotimeResolutionUnit_ps"))
            time_resolution_native_ps = value;
         else if (isTag("SyncRate_Hz"))
            t_rep_ps = 1e12 / value;

      }
      else if (tag_type == TagInt64)
      {
         int64_t value = *(int64_t*)data_ptr;
         std::cout << value;

         if (isTag("NumChannels"))
            n_chan = value;
         else if(isTag("NumTimeBins"))
            n_timebins_native = value;
      }
      else if (tag_type == TagUInt64)
      {
         uint64_t value = *(uint64_t*)data_ptr;
         std::cout << value;

         if (isTag("L4ZMessageSize"))
            message_size = value;

      }
      else if (tag_type == TagBool)
      {
         bool value = *(bool*)data_ptr;
         std::cout << value;

         if (isTag("L4ZCompression"))
            use_compression = value;

      }
      else if (tag_type == TagDate)
      {
         std::string value;
         value.resize(tag_data_length);
         memcpy(&value[0], data_ptr, tag_data_length);
         std::cout << value;
      }
      else if (tag_type == TagString)
      {
         std::string value;
         value.resize(tag_data_length);
         memcpy(&value[0], data_ptr, tag_data_length);
         std::cout << value;
      }
      /*
      case tyInt8:
         cout << (int)tag_head.TagValue;
         // get some Values we need to analyse records
         if (strcmp(tag_head.Ident, TTTRTagNumRecords) == 0) // Number of records
            n_records = tag_head.TagValue;
         if (strcmp(tag_head.Ident, Measurement_Mode) == 0) // measurement mode
            measurement_mode = (int)tag_head.TagValue;
         if (strcmp(tag_head.Ident, HWRouter_Channels) == 0)
            routing_channels = (int)tag_head.TagValue;
         if (strcmp(tag_head.Ident, Line_Averaging) == 0)
            line_averaging = (int)tag_head.TagValue;
         if (strcmp(tag_head.Ident, "TTResult_SyncRate") == 0)
            t_rep_ps = 1e12f / tag_head.TagValue;
         break;

      case tyBitSet64:
         cout << tag_head.TagValue;
         break;

      case tyColor8:
         cout << tag_head.TagValue;
         break;

      case tyFloat8:
         cout << *(double*)&(tag_head.TagValue);
         if (strcmp(tag_head.Ident, TTTRTagRes) == 0) // Resolution for TCSPC-Decay
            resolution = *(double*)&(tag_head.TagValue);
         //if (strcmp(tag_head.Ident, TTTRTagGlobRes)==0) // Global resolution for timetag
         //   GlobRes = *(double*)&(tag_head.TagValue); // in ns
         break;

      case tyTDateTime:
         break;

      case tyAnsiString:
      case tyFloat8Array:
      case tyWideString:
      case tyBinaryBlob:
         fs.seekg((long)tag_head.TagValue, ios_base::cur);
         break;

      default:
         throw runtime_error("Illegal Type identifier found! Broken file?");
      }
      */

   } while (tag_type != TagEndHeader);

   data_position = fs.tellg();
}