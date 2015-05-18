#pragma once
#include "AbstractPicoquantReader.h"

class PicoquantPTUReader : public AbstractPicoquantReader
{
public:
   
   PicoquantPTUReader(const std::string& filename);
   
protected:
   
   void readHeader();
};
