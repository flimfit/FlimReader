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
   BhEvent(uint32_t evt0)
   {
      uint32_t evt = evt0;
      macro_time = evt & 0xFFF; evt >>= 12;
      channel = evt & 0xF; evt >>= 4;
      micro_time = evt & 0xFFF; evt >>= 12;

      bool is_mark = (evt & 0x1) != 0;
      bool gap = (evt & 0x2) != 0;
      bool mtov = (evt & 0x4) != 0;
      bool invalid = (evt & 0x8) != 0;

      if (is_mark)
      {
         mark = channel;
         if (mark > 1)
            mark = mark;
      }

      valid = true;
      if (mtov)
      {
         if (invalid)
         {
            valid = false;
            macro_time_offset = 0xFFF;
         }
         else
         {
            macro_time_offset = 0xFFF * (evt0 & 0xFFFFFFFF);
         }
      }
   }
};


class BhEventReader : public AbstractEventReader
{
public:

   BhEventReader(const std::string& filename, std::streamoff data_position)
      : AbstractEventReader(filename, data_position, sizeof(uint32_t))
   {}

   TcspcEvent getEvent()
   {
      uint32_t evt = *reinterpret_cast<const uint32_t*>(getPacket());
      return BhEvent(evt);
   }
};