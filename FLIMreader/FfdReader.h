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


class FfdEventReader : public AbstractEventReader
{
public:

   FfdEventReader(const std::string& filename, int version, std::streamoff data_position)
      : AbstractEventReader(filename, data_position, (version == 1) ? sizeof(ffd_evt_v1) : sizeof(ffd_evt) ),
        version(version)
   {
   }

   std::tuple<TcspcEvent, uint64_t> getRawEvent()
   {
      if (version == 1)
      {
         ffd_evt_v1 evt = *reinterpret_cast<const ffd_evt_v1*>(getPacket());
         return getFfdEvent(evt);
      }
      else
      {
         ffd_evt evt = *reinterpret_cast<const ffd_evt*>(getPacket());
         return getFfdEvent(evt);
      }

      assert(fs.good());

   }

   std::tuple<TcspcEvent, uint64_t> getFfdEvent(ffd_evt_v1 evt)
   {
      TcspcEvent e;
      uint64_t macro_time_offset = 0;

      e.micro_time = evt.micro_time;
      e.macro_time = evt.macro_time;
      e.channel = evt.channel;
      e.mark = evt.mark;

      return std::tuple<TcspcEvent, uint64_t>(e, macro_time_offset);
   }

   std::tuple<TcspcEvent, uint64_t> getFfdEvent(ffd_evt evt)
   {
      TcspcEvent e;
      uint64_t macro_time_offset = 0;

      e.macro_time = evt.macro_time;
      e.channel = evt.micro_time & 0xF;

      if (e.channel == 0xF)
      {
         if (evt.micro_time == 0xF)
         {
            macro_time_offset = ((e.macro_time == 0) ? 1 : e.macro_time) * 0xFFFF;
            e.valid = false;
         }
         else
         {
            e.mark = evt.micro_time >> 4;
         }
      }
      else
      {
         e.mark = 0;
         e.micro_time = evt.micro_time >> 4;
      }

      return std::tuple<TcspcEvent, uint64_t>(e, macro_time_offset);
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
