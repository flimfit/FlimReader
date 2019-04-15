#pragma once

#include "AbstractFifoReader.h"
#include "FileEventReader.h"

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


class FfdEventReader : public FileEventReader
{
public:

   FfdEventReader(const std::string& filename, int version, std::streamoff data_position)
      : FileEventReader(filename, data_position, (version == 1) ? sizeof(ffd_evt_v1) : sizeof(ffd_evt) ),
        version(version)
   {
   }

   std::tuple<FifoEvent, uint64_t> getRawEvent();
   std::tuple<FifoEvent, uint64_t> getFfdEvent(ffd_evt_v1 evt);
   std::tuple<FifoEvent, uint64_t> getFfdEvent(ffd_evt evt);

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
   int n_timepoints_native = 0;
   double time_resolution_native_ps = 0;
   uint32_t version;
};
