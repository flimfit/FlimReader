#pragma once
#include "AbstractFifoReader.h"

enum PicoquantRecordType
{
   PicoHarp_T3 = 0x00010303,
   PicoHarp_T2 = 0x00010203,
   HydraHarpV1_T3 = 0x00010304,
   HydraHarpV1_T2 = 0x00010204,
   HydraHarpV2_T3 = 0x01010304,
   HydraHarpV2_T2 = 0x01010204,
   TimeHarp260N_T2 = 0x01010205,
   TimeHarp260N_T3 = 0x01010305,
   TimeHarp260P_T2 = 0x01010206,
   TimeHarp260P_T3 = 0x01010306
};

class PicoharpT3Event : public TcspcEvent
{
public:
   PicoharpT3Event(uint32_t evt)
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
            valid = false;
         }
         else
         {
            mark = dtime;
         }

      }

      channel--; // 1-indexing -> 0-indexing
   }
};

class TimeharpT3Event : public TcspcEvent
{
public:
   TimeharpT3Event(uint32_t evt, PicoquantRecordType rec_type)
   {

      int nsync = evt & 0x3FF; evt >>= 10;
      int dtime = evt & 0x7FFF; evt >>= 15;
      channel = evt & 0x3F; evt >>= 6;
      bool special = evt & 0x1;

      micro_time = dtime;
      macro_time = nsync;

      if (special) // special
      {
         if (channel == 0x3F) // macro time overflow
         {
            int n_overflow = (rec_type == HydraHarpV1_T3) ? 1 : nsync;
            macro_time_offset = 0x3FF * n_overflow;
            mark = 0x80;
            valid = false;
         }
         else
         {
            mark = channel;
         }

      }

      channel--; // 1-indexing -> 0-indexing
   }
};

class PicoquantEventReader : public AbstractEventReader
{
public:

   PicoquantEventReader(const std::string& filename, std::streamoff data_position, PicoquantRecordType rec_type)
      : AbstractEventReader(filename, data_position, sizeof(uint32_t)), rec_type(rec_type)
   {}

   TcspcEvent getEvent()
   {
      uint32_t evt = *reinterpret_cast<const uint32_t*>(getPacket());

      switch (rec_type)
      {
      case PicoHarp_T3:
         return PicoharpT3Event(evt);
      case HydraHarpV1_T3:
      case HydraHarpV2_T3:
      case TimeHarp260N_T3:
      case TimeHarp260P_T3:
         return TimeharpT3Event(evt, rec_type);
      }
   }

   PicoquantRecordType rec_type;
};
