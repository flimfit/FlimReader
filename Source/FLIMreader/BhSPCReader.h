#pragma once
#include "AbstractFifoReader.h"

class BhSPCReader : public AbstractFifoReader
{
public:

   BhSPCReader(const std::string& filename);

protected:

   void readHeader();
   std::streamoff data_position = 0;

};

class BhEvent : public TcspcEvent
{
public:
   BhEvent(uint16_t evt1, uint32_t evt2)
   {
      
      int adc = evt1 & 0xFFF; evt1 >>= 12; 
      int invalid = evt1 & 0x1;
      int mtov = evt1 & 0x2;
      int gap = evt1 & 0x4;

    
   }
};

class BhEventReader : public AbstractEventReader
{
public:

   BhEventReader(const std::string& filename, std::streamoff data_position)
      : AbstractEventReader(filename, data_position)
   {}

   TcspcEvent getEvent()
   {
      uint16_t evt1;
      uint32_t evt2;
      fs.read(reinterpret_cast<char*>(&evt1), sizeof(evt1));
      fs.read(reinterpret_cast<char*>(&evt2), sizeof(evt2));
      return BhEvent(evt1, evt2);
   }
};

