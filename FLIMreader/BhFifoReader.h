#pragma once
#include "AbstractFifoReader.h"

class BhFifoReader : public AbstractFifoReader
{
public:

   BhFifoReader(const std::string& filename);

protected:

   void readHeader();
   std::streamoff data_position = 0;

};

class BhEvent : public TcspcEvent
{
public:
   
};


class BhEventReader : public AbstractEventReader
{
public:

   BhEventReader(const std::string& filename, std::streamoff data_position)
      : AbstractEventReader(filename, data_position, sizeof(uint32_t))
   {}

   std::tuple<TcspcEvent, uint64_t> getRawEvent()
   {
      uint32_t evt = *reinterpret_cast<const uint32_t*>(getPacket());
      return getBhEvent(evt);
   }

   std::tuple<TcspcEvent, uint64_t> getBhEvent(uint32_t evt0)
   {
      TcspcEvent e;
      uint64_t macro_time_offset = 0;

      uint32_t evt = evt0;
      e.macro_time = evt & 0xFFF; evt >>= 12;
      e.channel = evt & 0xF; evt >>= 4;
      e.micro_time = evt & 0xFFF; evt >>= 12;
      e.micro_time = 4095 - e.micro_time; // Reverse start-stop

      bool is_mark = (evt & 0x1) != 0;
      bool gap = (evt & 0x2) != 0;
      bool mtov = (evt & 0x4) != 0;
      bool invalid = (evt & 0x8) != 0;

      e.valid = true;

      if (gap)
         int a = 1;

      if (is_mark)
      {
         e.mark = e.channel;
      }
      if (mtov)
      {
         if (invalid && !is_mark)
         {
            e.valid = false;
            macro_time_offset = 0xFFF * (evt0 & 0xFFFFFFF);
         }
         else
         {
            macro_time_offset = 0xFFF;
         }
      }

      return std::tuple<TcspcEvent, uint64_t>(e, macro_time_offset);
   }
};