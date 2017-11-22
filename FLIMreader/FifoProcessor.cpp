#include "FifoProcessor.h"

FifoProcessor::FifoProcessor(Markers markers, SyncSettings sync) :
   markers(markers), sync(sync)
{
   incrementFrame(); // handle cases where we don't have a frame marker before the start of the data
}

Photon FifoProcessor::addEvent(TcspcEvent p)
{
   if (p.valid)
   {
      if (p.mark & markers.FrameMarker)
      {
         if (cur_line >= 0) // end of first frame (no initial marker)
            incrementFrame();
         else // start of first frame 
            frame_started = true;
      }
      if ((p.mark & markers.LineEndMarker) && line_valid == true)
      {
         line_valid = false;
      }
      if (p.mark & markers.LineStartMarker)
      {
         if (markers.FrameMarker == 0x0 && ((cur_line + 1) == sync.n_line || cur_line == -1))
            incrementFrame();

         line_valid = true;
         sync_start = p.macro_time;
         cur_line++;
         cur_px = -1;

         if (sync.bi_directional)
            cur_direction *= -1;
      }
      if (p.mark & markers.PixelMarker)
         cur_px++;

      if ((p.mark == markers.PhotonMarker) && line_valid)
      {
         double cur_loc = (markers.PixelMarker == 0) ?
            ((p.macro_time - sync_start) / sync.count_per_line) * (sync.n_x) :
            cur_px;

         if (cur_direction == -1)
            cur_loc = sync.n_x - 1 - cur_loc - sync.phase;

         return Photon(frame_idx, (int)cur_loc, cur_line, p.channel, p.micro_time);
      }
   }

   return Photon();
}

void FifoProcessor::incrementFrame()
{
   frame_idx++;
   frame_started = true;
   cur_line = -1;
   cur_direction = sync.bi_directional ? -1 : 1;
}
