#pragma once
#include "AbstractFifoReader.h"
#include "FileEventReader.h"

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

class PicoquantEventReader : public FileEventReader
{
public:

   PicoquantEventReader(const std::string& filename, std::streamoff data_position, PicoquantRecordType rec_type);

   std::tuple<FifoEvent, uint64_t> getRawEvent();
   std::tuple<FifoEvent, uint64_t> getPicoharpT3Event(uint32_t evt);
   std::tuple<FifoEvent, uint64_t> getTimeharpT3Event(uint32_t evt, PicoquantRecordType rec_type);

   PicoquantRecordType rec_type;
};
