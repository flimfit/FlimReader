#pragma once
#include "AbstractEventReader.h"

#include <fstream>
#include <iostream>
#include <future>

class FileEventReader : public AbstractEventReader
{
public:
   FileEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size);
   ~FileEventReader();
   double getProgress();
   bool hasMoreData();

   void setToStart();

protected:

   void startReading();
   void readThread();

   std::ifstream fs;
   std::streamoff data_position;

   uint64_t length = 0;
   uint64_t n_packet = 0;

   std::future<void> reader_future;
   bool terminate = false;
};