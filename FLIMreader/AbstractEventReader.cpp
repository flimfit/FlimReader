#pragma once
#include "AbstractEventReader.h"

AbstractEventReader::AbstractEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size) :
data_position(data_position), packet_size(packet_size)
{
   fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
   
   fs.seekg(0, fs.end);
   length = fs.tellg() - data_position;
   n_packet = length / packet_size;
   uint64_t n_block = n_packet / block_size + 1;
   data.reserve(n_block);
   
   reader_thread = std::thread(&AbstractEventReader::read, this);
   
   setToStart();
}

AbstractEventReader::~AbstractEventReader()
{
   terminate = true;
   if (reader_thread.joinable())
      reader_thread.join();
}


void AbstractEventReader::read()
{
   uint64_t sz = block_size * packet_size;
   
   fs.seekg(data_position);

   size_t n_read = 0;
   do
   {
      std::vector<char> block(sz);
      fs.read(&block[0], sz);
      n_read = fs.gcount();

      
      std::unique_lock<std::mutex> lk(m);
      data.push_back(block);
      cv.notify_one();
   } while (n_read > 0 && !terminate);
}

double AbstractEventReader::getProgress()
{
   return ((double) cur_pos) / n_packet;
}

void AbstractEventReader::setToStart()
{
   cur_pos = 0;
   sync_count_accum = 0;
}

bool AbstractEventReader::hasMoreData()
{
   return cur_pos < n_packet;
}

FifoEvent AbstractEventReader::getEvent() 
{
   // Get event and compute absolute macro time
   auto t = getRawEvent();
   FifoEvent event = std::get<0>(t);
   sync_count_accum += std::get<1>(t);
   event.macro_time += sync_count_accum;
   return event;
}
