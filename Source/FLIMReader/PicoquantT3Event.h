#pragma once
#include "AbstractFifoReader.h"

class PicoquantT3Event : public TcspcEvent
{
public:
   PicoquantT3Event(uint32_t evt)
   {

      int nsync = evt & 0xFFFF; evt >>= 16;
      int dtime = evt & 0xFFF; evt >>= 12;
      channel = evt & 0xF;

      micro_time = dtime;
      macro_time = nsync;

      if (channel == 15) // special
      {
         if (dtime == 0) // macro time overflow
         {
            macro_time_offset = 0xFFFF;
            mark = 0x80;
         }
         else
         {
            mark = dtime;
         }

      }

      channel--; // 1-indexing -> 0-indexing
   }
};


class PicoquantEventReader : public AbstractEventReader
{
public:

   PicoquantEventReader(const std::string& filename, std::streamoff data_position)
      : AbstractEventReader(filename, data_position)
   {}

   TcspcEvent getEvent()
   {
      uint32_t evt;
      fs.read(reinterpret_cast<char *>(&evt), sizeof(evt));
      return PicoquantT3Event(evt);
   }
};
