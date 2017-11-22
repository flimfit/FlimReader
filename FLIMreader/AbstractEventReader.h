#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
 
#include "FifoProcessor.h"

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size);
   ~AbstractEventReader();
   
   void read();
   double getProgress();
   void setToStart();
   bool hasMoreData();

   TcspcEvent getEvent();
   const char* getPacket();
   
protected:
   
   virtual std::tuple<TcspcEvent, uint64_t> getRawEvent() = 0;

   std::ifstream fs;
   std::streamoff data_position;
   uint64_t packet_size;
   uint64_t length = 0;
   uint64_t n_packet = 0;
   std::vector<std::vector<char>> data;

   uint64_t block_size = 1024;

   uint64_t cur_pos = 0;
   uint64_t sync_count_accum = 0;

   std::thread reader_thread;
   std::mutex m;
   std::condition_variable cv;

   bool terminate = false;
};