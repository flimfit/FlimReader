#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <cstdint>

class Markers
{
public:
   uint8_t PhotonMarker = 0x0;
   uint8_t PixelMarker = 0x1;
   uint8_t LineStartMarker = 0x2;
   uint8_t LineEndMarker = 0x4;
   uint8_t FrameMarker = 0x8;
   uint8_t Invalid = 0x80;
};

class FifoEvent
{
public:
   uint64_t macro_time = 0;
   uint32_t micro_time = 0;
   uint8_t channel = 0;
   uint8_t mark = 0;
   bool valid = true;
   bool gap = false;
};

class SyncSettings
{
public:
   double count_per_line;
   double counts_interline;
   double counts_interframe;
   int n_x;
   bool bidirectional = false;
   double phase = 0;

   int n_line = -1; // used if we don't have frame markers
};

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size);
   ~AbstractEventReader();
   
   void read();
   double getProgress();
   void setToStart();
   bool hasMoreData();

   FifoEvent getEvent();

   template<class T>
   T getPacket();
   
protected:
   
   virtual std::tuple<FifoEvent, uint64_t> getRawEvent() = 0;

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

template<class T>
T AbstractEventReader::getPacket()
{
   uint64_t block = cur_pos / block_size;
   uint64_t packet = cur_pos % block_size;

   std::unique_lock<std::mutex> lk(m);
   cv.wait(lk, [&]() { return block < data.size(); });

   cur_pos++;
   T* ptr = reinterpret_cast<T*>(&(data[block][packet*packet_size]));
   T v = *ptr;
   return v;
}