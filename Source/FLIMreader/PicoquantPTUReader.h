#pragma once
#include "AbstractFifoReader.h"
#include "PicoquantT3Event.h"

class PicoquantPTUReader : public AbstractFifoReader
{
public:
   
   PicoquantPTUReader(const std::string& filename);
   
protected:
   
   void readHeader();
   int data_position = 0;
   int temporal_binning = 0;

};
