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


   setTemporalResolution(14);
   assert(measurement_mode == 3);

   markers.FrameMarker = 0x4;
   markers.LineEndMarker = 0x2;
   markers.LineStartMarker = 0x1;

   event_reader = std::unique_ptr<AbstractEventReader>(new PicoquantEventReader(filename, data_position));
      
   determineDwellTime();
}

void PicoquantPTUReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);
   
   char magic[8];
   char version[8];
   
   fs.read(magic, sizeof(magic));
   
   if (string("PQTTTR") != magic)
      throw runtime_error("Wrong magic string, this is not a PTU file");

   n_chan = 1;

   
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
            break;
            
         case tyInt8:
            cout << (tag_head.TagValue != 0);
            // get some Values we need to analyse records
//            if (strcmp(tag_head.Ident, TTTRTagNumRecords)==0) // Number of records
//               n_records = tag_head.TagValue;
            if (strcmp(tag_head.Ident, Measurement_Mode)==0) // measurement mode
               measurement_mode = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, HWRouter_Channels)==0)
               n_chan = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, Line_Averaging)==0)
               line_averaging = (int) tag_head.TagValue;
            if (strcmp(tag_head.Ident, TTResult_SyncRate)==0)
               t_rep_ps = 1e12f / tag_head.TagValue;
            if (strcmp(tag_head.Ident, MeasDesc_BinningFactor)==0)
               temporal_binning = (int) tag_head.TagValue;
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
            fs.seekg((long)tag_head.TagValue, ios_base::cur);
            break;
         
         default:
            throw runtime_error("Illegal Type identifier found! Broken file?");
      }
              
   } while (strncmp(tag_head.Ident, FileTagEnd, sizeof(FileTagEnd)));
   
   data_position = fs.tellg();
}