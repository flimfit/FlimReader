#pragma once
#include "AbstractEventReader.h"

#include <fstream>
#include <iostream>

class FileEventReader : public AbstractEventReader
{
public:
   FileEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size);
   ~FileEventReader();
   double getProgress();
   bool hasMoreData();

protected:

   void read();

   std::ifstream fs;
   std::streamoff data_position;

   uint64_t length = 0;
   uint64_t n_packet = 0;

   std::thread reader_thread;
   bool terminate = false;
};