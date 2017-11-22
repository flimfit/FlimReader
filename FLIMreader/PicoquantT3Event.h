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
   TimeHarp260N_T2 = 0x00010205,
   TimeHarp260N_T3 = 0x00010305,
   TimeHarp260P_T2 = 0x00010206,
   TimeHarp260P_T3 = 0x00010306
};

class PicoquantEventReader : public AbstractEventReader
{
public:

   PicoquantEventReader(const std::string& filename, std::streamoff data_position, PicoquantRecordType rec_type)
      : AbstractEventReader(filename, data_position, sizeof(uint32_t)), rec_type(rec_type)
   {}

   std::tuple<TcspcEvent, uint64_t>  getRawEvent()
   {
      uint32_t evt = *reinterpret_cast<const uint32_t*>(getPacket());

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

   std::tuple<TcspcEvent, uint64_t> getPicoharpT3Event(uint32_t evt)
   {
      TcspcEvent e;
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

      return std::tuple<TcspcEvent, uint64_t>(e, macro_time_offset);
   }

   std::tuple<TcspcEvent, uint64_t> getTimeharpT3Event(uint32_t evt, PicoquantRecordType rec_type)
   {
      TcspcEvent e;
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

      return std::tuple<TcspcEvent, uint64_t>(e, macro_time_offset);
   }

   PicoquantRecordType rec_type;
};
