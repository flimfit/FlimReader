#pragma once

#include <vector>
#include <list>
#include <string>
#include <cassert>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstdint>

class Markers
{
public:
   uint8_t PhotonMarker = 0x0;
   uint8_t PixelMarker = 0x0;
   uint8_t LineStartMarker = 0x0;
   uint8_t LineEndMarker = 0x0;
   uint8_t FrameMarker = 0x0;
   uint8_t ImageMarker = 0x0;
   uint8_t Invalid = 0x80;
};

class FifoEvent
{
public:
   uint64_t macro_time = 0;
   uint32_t micro_time = 0;
   uint8_t channel = 0;
   uint8_t mark = 0;
   uint8_t valid = true;
   uint8_t gap = false;
};

class SyncSettings
{
public:
   double count_per_line;
   double counts_interline;
   double counts_interframe;
   int n_x = 0;
   bool bidirectional = false;
   bool bidirectional_per_frame = false; // Leica software stores bidirectional data in alternative frames
   double phase = 0;

   int line_averaging = 1;
   int n_line = 0; // used if we don't have frame markers
};

class AbstractEventReader
{
public:
   AbstractEventReader(unsigned int packet_size);
   
   virtual double getProgress() = 0;
   virtual bool hasMoreData() = 0;

   void setToStart();

   void stop();
   void clear();

   FifoEvent getEvent();

   template<class T>
   T getPacket();
   
protected:

   virtual std::tuple<FifoEvent, uint64_t> getRawEvent() = 0;

   bool read_from_start = true;
   std::list<std::vector<char>> data;
   std::list<std::vector<char>>::iterator data_it;
   std::vector<char>::iterator block_it;
   size_t cur_pos = 0;

   uint64_t block_size = 1024;

   uint64_t packet_size;
   uint64_t sync_count_accum = 0;

   std::mutex m;
   std::condition_variable cv;
   bool terminate = false;
};

template<class T>
T AbstractEventReader::getPacket()
{
   if (read_from_start)
   {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [&]() { return !data.empty(); });
      data_it = data.begin();
      block_it = (*data_it).begin();
      read_from_start = false;
   }

   if (block_it == (*data_it).end())
   {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [&]() {
         auto it = data_it;
         return ((++it) != data.end()) || !hasMoreData();
      });
      data_it++;
      block_it = (*data_it).begin();
   }

   if (data_it == data.end()) 
      return T();

   T v = *reinterpret_cast<T*>(&(*block_it));
   block_it += packet_size;
   cur_pos++;

   //uint64_t block = cur_pos / block_size;
   //uint64_t packet = cur_pos % block_size;

   //std::unique_lock<std::mutex> lk(m);
   //cv.wait(lk, [&]() { return block < data.size(); });

   //cur_pos++;
   //T* ptr = reinterpret_cast<T*>(&(data[block][packet*packet_size]));
   //T v = *ptr;
   return v;
}
