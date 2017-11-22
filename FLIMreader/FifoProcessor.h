#pragma once
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

class TcspcEvent
{
public:
   uint64_t macro_time = 0;
   uint32_t micro_time = 0;
   uint8_t channel = 0;
   uint8_t mark = 0;
   bool valid = true;
};

class SyncSettings
{
public:
   double count_per_line;
   double counts_interline;
   double counts_interframe;
   int n_x;
   bool bi_directional = false;
   double phase = 0;

   int n_line = -1; // used if we don't have frame markers
};

class Photon {
public:
   Photon() :
      valid(false)
   {}

   Photon(int frame, int x, int y, int channel, int bin) :
      frame(frame), x(x), y(y), z(0), channel(channel), bin(bin), valid(true)
   {}

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
   FifoFrame(std::shared_ptr<AbstractEventReader> reader)
      reader(reader)
   {

   }

   void loadNextFrame()
   {
      events.clear();
      marker_events.clear();
      
      while (event_reader->hasMoreData())
      {
         TcspcEvent p = event_reader->getEvent();

         if (p.valid && (p.mark & markers.FrameMarker))
         {
            last_frame_event = p;
            break;
         }   
         else
         {
            if (p.mark)
               marker_events.push_back(p);
            events.push_back(p);
         }
      }
   }

protected:
   std::vector<TcspcEvent> events;
   std::vector<TcspcEvent> marker_events;
   std::shared_ptr<AbstractEventReader> reader;
   TcspcEvent last_frame_event;
};


class FifoProcessor
{
public:

   FifoProcessor(Markers markers, SyncSettings sync);
   Photon addEvent(TcspcEvent p);

protected:

   void incrementFrame();

   Markers markers;
   SyncSettings sync;

   int frame_idx = -1;

   long long sync_count_accum = 0;
   long long sync_start = 0;

   int cur_px = -1;
   int cur_line = -1;
   int cur_direction = 1;
   bool line_valid = false;
   bool frame_started = false;
};