#pragma once
#include "AbstractFifoReader.h"

class PicoquantPTUReader : public AbstractFifoReader
{
public:
   
   PicoquantPTUReader(const std::string& filename);
   
protected:
   
   void readHeader();
};
