#pragma once
#include "AbstractFifoReader.h"
#include "PicoquantT3Event.h"

class PicoquantPTUReader : public AbstractFifoReader
{
public:
   
   PicoquantPTUReader(const std::string& filename);
   
protected:
   
   void readHeader();
   std::streamoff data_position = 0;
   int temporal_binning = 0;
   double time_resolution_native_ps = 0;
   PicoquantRecordType rec_type;
};
