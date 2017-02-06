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

   FfdEventReader(const std::string& filename, int version, std::streamoff data_position)
      : AbstractEventReader(filename, data_position, (version == 1) ? sizeof(ffd_evt_v1) : sizeof(ffd_evt) ),
        version(version)
   {
   }

   TcspcEvent getEvent()
   {
      if (version == 1)
      {
         ffd_evt_v1 evt = *reinterpret_cast<const ffd_evt_v1*>(getPacket());
         return FfdEvent(evt);
      }
      else
      {
         ffd_evt evt = *reinterpret_cast<const ffd_evt*>(getPacket());
         return FfdEvent(evt);
      }

      assert(fs.good());

   }

protected: 

   static const int buffer_size = 128 * 1024;
   uint32_t version;
   uint64_t macro_time_offset = 0;
   bool finished_decoding = false;
};


class FfdReader : public AbstractFifoReader
{
public:

   FfdReader(const std::string& filename);

protected:

   void readHeader();
   std::streamoff data_position = 0;
   uint32_t version;
};
