#include "FifoProcessor.h"

#include <tuple>
#include <vector>
#include <thread>
#include <cmath>
#include <numeric>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>


uint64_t interpolateTime(const std::vector<uint64_t>& x, const std::vector<uint64_t>& y, size_t begin, size_t end, int xi)
{
   double xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
   size_t n = end - begin;
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

FifoProcessor::FifoProcessor(std::shared_ptr<FifoFrame> frame, SyncSettings sync) :
   frame(frame), sync(sync)
{
   markers = frame->getMarkers();

   //determineLineStartTimes();
   event_it = frame->events.begin();
}

void FifoProcessor::determineLineStartTimes()
{

   std::vector<uint64_t> line_start_time;
   std::vector<uint64_t> line_index;

   uint64_t frame_start_time = frame->frame_start_event.macro_time;
   for (auto& m : frame->marker_events)
      if (m.mark & markers.LineStartMarker)
      {
         line_start_time.push_back(m.macro_time);
         line_index.push_back((uint64_t) std::round((m.macro_time - frame_start_time) / sync.count_per_line));
      }

   if (line_index.empty())
      return;


   int dline = 10;

   real_line_time.resize(sync.n_line);
   for (int i = 0; i < sync.n_line; i++)
   {
      size_t begin = i - dline;
      size_t end = i + dline;

      if (begin < 0)
      {
         end -= begin;
         begin = 0;
      }
      else if (end > line_start_time.size())
      {
         begin -= (int) (end - line_start_time.size());
         end = line_start_time.size();
      }

      real_line_time[i] = interpolateTime(line_index, line_start_time, begin, end, i);
   }
}


Photon FifoProcessor::getNextPhoton()
{
   double sync_wraparound = 0.5 * (sync.counts_interline + sync.count_per_line);

   while(event_it != frame->events.end())
   {
      FifoEvent& p = *(event_it++);

      if ((p.mark & markers.LineEndMarker) && line_valid == true)
      {
         line_valid = false;
      }
      else if (p.mark & markers.LineStartMarker)
      {
         line_valid = true;
         sync_start = p.macro_time;
         cur_line++;
         cur_px = -1;
      }
      if (p.mark & markers.PixelMarker)
         cur_px++;

      if ((p.mark == markers.PhotonMarker))
      {
         double cur_loc = cur_px;
         int this_line = cur_line;
         
         if (!markers.PixelMarker)
         {
            double diff = p.macro_time - sync_start;
            
            // Wrap around extra photons
            if (diff > sync_wraparound)
            {
               diff -= sync.counts_interline;
               this_line++;
            }

            cur_loc = diff / sync.count_per_line * sync.n_x;
         }
         
         
         if ((sync.bidirectional && (this_line % 2 == 1)) ||
             (sync.bidirectional_per_frame && (frame->frame_number % 2 == 1)))
            cur_loc = sync.n_x - 1 - cur_loc - sync.phase;
         /*
         while ((cur_line > 0) && (p.macro_time < real_line_time[cur_line]))
            cur_line--;

         while ((cur_line < (sync.n_line - 1)) && (p.macro_time > real_line_time[cur_line + 1]))
            cur_line++;
            */
         return Photon{ 
            frame->image_number,
            frame->frame_number, 
            cur_loc, 
            (double) this_line, 
            0, // z
            p.channel, 
            p.micro_time,
            p.macro_time };
      }
   }

   return Photon();
}


FifoInferredParameters FifoProcessor::determineSyncSettings(SyncSettings sync, int n_chan)
{
   using namespace boost::accumulators;

   uint64_t frame_start = std::numeric_limits<uint64_t>::max();
   uint64_t sync_start_count = 0;
   int n_line = 0;
   int n_frame = 0;

   std::vector<size_t> channel_counts(128, 0);

   std::vector<uint64_t> sync_count_interline; sync_count_interline.reserve(4096);
   accumulator_set<uint64_t, stats<tag::median > > sync_count_per_line_acc;
   accumulator_set<uint64_t, stats<tag::median > > sync_count_interline_acc;

   int missed_start = 0;
   int missed_end = 0;

   bool line_active = false;


   for (auto& p : frame->events)
   {
      if (!p.valid)
         continue;

      if (!p.mark && p.channel < channel_counts.size())
         channel_counts[p.channel]++;

      if ((markers.FrameMarker > 0) && (p.mark & markers.FrameMarker))
      {
         if (n_line > 0)
         {
            if (frame_start == std::numeric_limits<uint64_t>::max())
            {
               frame_start = p.macro_time;
            }
            else
            {
               if (p.macro_time < frame_start)
                  throw std::runtime_error("Incorrect interframe counts");
               sync.counts_interframe = (double)(p.macro_time - frame_start);
            }
            n_frame++; // count full frames (i.e. ignore first start, if it's there)
            line_active = false;
         }
         else
         {
            frame_start = p.macro_time;
         }

      }

      if (p.mark & markers.LineEndMarker)
      {
         if ((markers.LineStartMarker != markers.LineEndMarker) && !line_active)
            missed_start++;

         if (line_active && (p.macro_time >= sync_start_count)) // not sure why this is sometimes violated
         {
            uint64_t diff = p.macro_time - sync_start_count;
            sync_count_per_line_acc((double)diff);
         }

         line_active = false;
      }

      if (p.mark & markers.LineStartMarker)
      {
         if ((markers.LineStartMarker != markers.LineEndMarker) && line_active)
            missed_end++;

         if (!line_active && (p.macro_time >= sync_start_count) && n_frame == 0)
         {
            if (n_line > 0)
            {
               uint64_t diff = p.macro_time - sync_start_count;
               sync_count_interline_acc((double)diff);
               sync_count_interline.push_back(diff);
            }

            n_line++;
            sync_start_count = p.macro_time;
         }

         line_active = true;
      }
   }

   if (markers.FrameMarker == 0x0)
      n_frame = 1;

   sync.count_per_line = median(sync_count_per_line_acc);
   sync.counts_interline = median(sync_count_interline_acc);

   if (sync.count_per_line > 0.5 * sync.counts_interline)
      sync.bidirectional = true;

   // Count number of lines, accounting for missing start/end markers
   int n_line_corrected = std::accumulate(sync_count_interline.begin(), sync_count_interline.end(), 1,
      [&](int n_line, uint64_t interline) { return n_line + (int)std::round(interline / sync.counts_interline); });


   if (sync.line_averaging > 1)
   {
      double factor = static_cast<double>(sync.line_averaging) / (sync.line_averaging + 1);
      sync.count_per_line *= factor;
      sync.counts_interline *= factor;
   }

   if (n_line == 0 || n_frame == 0)
      throw std::runtime_error("Error interpreting sync markers");

   if (sync.n_line == 0)
      sync.n_line = n_line_corrected / sync.line_averaging / n_frame;
   if (sync.n_x == 0)
      sync.n_x = sync.n_line;

   if (!isfinite(sync.counts_interframe))
      throw std::runtime_error("Incorrect interframe counts");

   int highest_chan = 0;
   std::vector<bool> recommended_channels(channel_counts.size());
   for (int i = 0; i < recommended_channels.size(); i++)
   {
      recommended_channels[i] = channel_counts[i] > 0;
      if (recommended_channels[i])
         highest_chan = i;
   }

   n_chan = std::max(n_chan, highest_chan + 1);
   recommended_channels.resize(n_chan);

   return FifoInferredParameters{
      sync,
      n_chan,
      recommended_channels
   };
}

/*
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

         if (sync.bidirectional)
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
   cur_direction = sync.bidirectional ? -1 : 1;
}

*/