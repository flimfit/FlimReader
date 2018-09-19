#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "AbstractEventReader.h"

class Photon {
public:
   Photon() :
      valid(false)
   {}

   Photon(int frame, int x, int y, int channel, int bin) :
      frame(frame), x(x), y(y), z(0), channel(channel), bin(bin), valid(true)
   {}

   explicit operator bool() const { return valid; }

   int frame;
   double x;
   double y;
   double z;
   int channel;
   int bin;
   bool valid;
};

class FifoFrame
{
public:
   FifoFrame(std::shared_ptr<AbstractEventReader> reader, Markers markers) :
      reader(reader), markers(markers), frame_number(-1)
   {
   }

   void initalise();
   void loadNext();

   std::vector<FifoEvent> events;
   std::vector<FifoEvent> marker_events;
   FifoEvent next_frame_event;
   FifoEvent frame_start_event;
   int frame_number;

protected:

   std::shared_ptr<AbstractEventReader> reader;
   Markers markers;
};


class FifoProcessor
{
public:
   FifoProcessor(Markers markers, SyncSettings sync);

   void setFrame(std::shared_ptr<FifoFrame> frame_);
   void determineLineStartTimes();
   Photon getNextPhoton();


protected:
   std::shared_ptr<FifoFrame> frame;
   Markers markers;
   SyncSettings sync;
   size_t idx;
   size_t line_idx;

   std::vector<uint64_t> real_line_time;

private:

   long long sync_start = 0;

   int cur_px = -1;
   int cur_line = -1;
   bool line_valid = false;
};