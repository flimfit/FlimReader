#pragma once

#include "AbstractFifoReader.h"

struct ffd_evt_v1
{
   uint64_t macro_time;
   uint16_t micro_time;
   uint8_t channel;
   uint8_t mark;
};


struct ffd_evt
{
   uint16_t macro_time;
   uint16_t micro_time;
};

class FfdEvent : public TcspcEvent
{
public:
   FfdEvent(ffd_evt_v1 evt)
   {
      micro_time = evt.micro_time;
      macro_time = evt.macro_time;
      channel = evt.channel;
      mark = evt.mark;
   }
   
   FfdEvent(ffd_evt evt)
   {
      macro_time = evt.macro_time;
      channel = evt.micro_time & 0xF;

      if (channel == 0xF)
      {
         if (evt.micro_time == 0xF)
         {
            macro_time_offset = ((macro_time == 0) ? 1 : macro_time) * 0xFFFF;
            valid = false;
         }
         else
         {
            mark = evt.micro_time >> 4;
         }
      }
      else
      {
         mark = 0;
         micro_time = evt.micro_time >> 4;
      }
   }
};


class FfdEventReader : public AbstractEventReader
{
public:

   FfdEventReader(const std::string& filename, int version, int data_position, bool use_compression = false, size_t max_message_size = 16 * 1024)
      : AbstractEventReader(filename, data_position),
        version(version),
        use_compression(use_compression),
        max_message_size(max_message_size)
   {
   }

   
   bool hasMoreData()
   {
      return !fs.eof() && !fs.fail();
   }



   TcspcEvent getEvent()
   {
      if (version == 1)
      {
         ffd_evt_v1 evt;
         fs.read(reinterpret_cast<char*>(&evt), sizeof(evt));
         return FfdEvent(evt);
      }
      else
      {
         ffd_evt evt;
         fs.read(reinterpret_cast<char*>(&evt), sizeof(evt));
         return FfdEvent(evt);
      }

      assert(fs.good());

   }

protected: 

   static const int buffer_size = 128 * 1024;
   size_t max_message_size;

   uint32_t version;
   uint64_t macro_time_offset = 0;
   bool finished_decoding = false;
   bool use_compression = false;
};


class FfdReader : public AbstractFifoReader
{
public:

   FfdReader(const std::string& filename);

protected:
   
   enum 
   {
      TagDouble = 0,
      TagUInt64 = 1,
      TagInt64 = 2,
      TagBool = 4,
      TagString = 5,
      TagDate = 6,
      TagEndHeader = 7
   } FlimMetadataTag;

   void readHeader();
   int data_position = 0;
   bool use_compression = false;
   size_t message_size = 16 * 1024;
   uint32_t version;

};
