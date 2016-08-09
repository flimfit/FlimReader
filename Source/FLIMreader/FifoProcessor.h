#pragma once

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
   uint64_t macro_time_offset = 0;
   bool valid = true;
};

class SyncSettings
{
public:
   double count_per_line;
   int n_x;
   bool bi_directional = false;
};

class Photon {
public:
   Photon() :
      valid(false)
   {}

   Photon(int frame, int x, int y, int channel, int bin) :
      frame(frame), x(x), y(y), channel(channel), bin(bin), valid(true)
   {}

   int frame;
   int x;
   int y;
   int channel;
   int bin;
   bool valid;
};

class FifoProcessor
{
public:

   FifoProcessor(Markers markers, SyncSettings sync) :
      markers(markers), sync(sync)
   {

   }

   Photon addEvent(TcspcEvent p)
   {
      sync_count_accum += p.macro_time_offset;
      long long cur_sync = p.macro_time + sync_count_accum;

      if (p.valid)
      {
         if (p.mark & markers.FrameMarker)
         {
            frame_idx++;
            frame_started = true;
            cur_line = -1;
            cur_direction = 1;
         }
         if (frame_started)
         {
            if (p.mark & markers.LineEndMarker)
            {
               line_valid = false;
            }
            if (p.mark & markers.LineStartMarker)
            {
               line_valid = true;
               sync_start = cur_sync;
               cur_line++;

               if (bi_directional)
                  cur_direction *= -1;
            }
         }

         if ((p.mark == markers.PhotonMarker) && line_valid && frame_started)
         {
            double cur_loc = ((cur_sync - sync_start) / sync.count_per_line) * (sync.n_x);

            if (cur_direction == -1)
               cur_loc = sync.n_x - 1 - cur_loc;

            int cur_px = static_cast<int>(cur_loc);
            return Photon(frame_idx, cur_px, cur_line, p.channel, p.micro_time);
         }
      }

      return Photon();
   }

protected:

   Markers markers;
   SyncSettings sync;

   int frame_idx = -1;

   long long sync_count_accum = 0;
   long long sync_start = 0;

   int cur_line;
   int cur_direction = 1;
   bool bi_directional = false;
   bool line_valid = false;
   bool frame_started = false;
};