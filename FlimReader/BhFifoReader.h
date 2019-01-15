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

class BhEvent : public FifoEvent
{
public:
   
};


class BhEventReader : public AbstractEventReader
{
public:

   BhEventReader(const std::string& filename, std::streamoff data_position)
      : AbstractEventReader(filename, data_position, sizeof(uint32_t))
   {}

   std::tuple<FifoEvent, uint64_t> getRawEvent();
   std::tuple<FifoEvent, uint64_t> getBhEvent(uint32_t evt0);
};