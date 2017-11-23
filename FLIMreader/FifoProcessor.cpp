#include "FifoProcessor.h"

#include <tuple>
#include <vector>
#include <thread>

uint64_t interpolateTime(const std::vector<uint64_t>& x, const std::vector<uint64_t>& y, int begin, int end, int xi)
{
   double xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
   int n = end - begin;
   for (size_t i = begin; i < end; i++)
   {
      xSum += x[i];
      ySum += y[i];
      xxSum += x[i] * x[i];
      xySum += x[i] * y[i];
   }
   slope = (n * xySum - xSum * ySum) / (n * xxSum - xSum * xSum);
   intercept = (ySum - slope * xSum) / n;

   return static_cast<uint64_t>(intercept + slope * xi);
}



void FifoProcessor2::determineLineStartTimes()
{

   std::vector<uint64_t> line_start_time;
   std::vector<uint64_t> line_index;

   uint64_t frame_start_time = frame->frame_start_event.macro_time;
   for (auto& m : frame->marker_events)
      if (m.mark & markers.LineStartMarker)
      {
         line_start_time.push_back(m.macro_time);
         line_index.push_back(std::round((m.macro_time - frame_start_time) / sync.count_per_line));
      }

   int dline = 10;

   real_line_time.resize(sync.n_line);
   for (int i = 0; i < sync.n_line; i++)
   {
      int begin = i - dline;
      int end = i + dline;

      if (begin < 0)
      {
         end -= begin;
         begin = 0;
      }
      else if (end > line_start_time.size())
      {
         begin -= (end - line_start_time.size());
         end = line_start_time.size();
      }

      real_line_time[i] = interpolateTime(line_index, line_start_time, begin, end, i);
   }
}


void FifoFrame::loadNext()
{
   using namespace std::chrono_literals;

   std::cout << "Loading next frame\n";

   frame_start_event = next_frame_event;

   events.clear();
   marker_events.clear();

   events.push_back(frame_start_event);
   marker_events.push_back(frame_start_event);

   while (reader->hasMoreData())
   {
      FifoEvent p = reader->getEvent();
      if (p.valid)
      {
         events.push_back(p);

         if (p.mark & markers.FrameMarker)
         {
            next_frame_event = p;
            break;
         }
         else if (p.mark)
         {
            marker_events.push_back(p);
         }
      }
   }
}



FifoProcessor2::FifoProcessor2(Markers markers, SyncSettings sync) :
   markers(markers), sync(sync)
{
}


Photon FifoProcessor2::getNextPhoton()
{
   while(true)
   {
      if (idx == frame->events.size())
         return Photon();

      FifoEvent& p = frame->events[idx++];

      if ((p.mark & markers.LineEndMarker) && line_valid == true)
      {
         line_valid = false;
      }
      if (p.mark & markers.LineStartMarker)
      {
         line_valid = true;
         sync_start = p.macro_time;
         cur_line++;
         cur_px = -1;

         if (sync.bi_directional)
            cur_direction *= -1;
      }
      if (p.mark & markers.PixelMarker)
         cur_px++;

      int a = 1;

      if ((p.mark == markers.PhotonMarker) && line_valid)
      {
         double cur_loc = (markers.PixelMarker == 0) ?
            ((p.macro_time - sync_start) / sync.count_per_line) * (sync.n_x) :
            cur_px;

         if (cur_direction == -1)
            cur_loc = sync.n_x - 1 - cur_loc - sync.phase;
         
         while ((cur_line > 0) && (p.macro_time < real_line_time[cur_line]))
            cur_line--;

         while ((cur_line < (sync.n_line - 1)) && (p.macro_time > real_line_time[cur_line + 1]))
            cur_line++;

         return Photon(a, (int)cur_loc, cur_line, p.channel, p.micro_time);
      }
   }
}



FifoProcessor::FifoProcessor(Markers markers, SyncSettings sync) :
   markers(markers), sync(sync)
{
   incrementFrame(); // handle cases where we don't have a frame marker before the start of the y
}

Photon FifoProcessor::addEvent(FifoEvent p)
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
