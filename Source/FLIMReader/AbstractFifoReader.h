#pragma once
#include "FLIMReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>

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

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, int data_position) :
   data_position(data_position)
   {
      fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
      setToStart();
   }
   
   void setToStart()
   {
      fs.clear();
      fs.seekg(data_position, std::ios_base::beg);
   }

   virtual bool hasMoreData()
   {
      return !fs.eof();
   }

   virtual TcspcEvent getEvent() = 0;
   
protected:
   
   std::ifstream fs;
   int data_position;
};

class Transform
{
public:

   Transform()
   {
      affine = cv::Mat::eye(2, 3, CV_64F);
      shift.x = 0.0;
      shift.y = 0.0;

   }

   Transform(cv::Mat affine, cv::Point2d shift) :
      affine(affine), shift(shift)
   {
   }


   cv::Mat affine;
   cv::Point2d shift;
};

class AbstractFifoReader : public FLIMReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   void alignFrames();

   void setTemporalResolution(int temporal_resolution);
      
protected:
   
   void readSettings();
   
   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   
   void determineDwellTime();
      
   int line_averaging = 1;
   int downsampling;
   
   std::vector<float> time_shifts_ps;
   
   // Required Picoquant information
   int measurement_mode = 0;
   //long long n_records;
   double time_resolution_native_ps;
   float t_rep_ps;
   int n_timebins_native;

   SyncSettings sync;

   std::unique_ptr<AbstractEventReader> event_reader = nullptr;
   Markers markers;
   
private:
   
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;
   std::vector<Transform> frame_transform;
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




template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   assert(event_reader != nullptr);
   event_reader->setToStart();

   auto channels = validateChannels(channels_, n_chan_stride);

   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;
   
   int n_bin = 1 << temporal_resolution;
   int n_x_binned = n_x / spatial_binning;
   int n_y_binned = n_y / spatial_binning;
   int n_invalid = 0;

   FifoProcessor processor(markers, sync);

   cv::Mat pos(3, 1, CV_64F, cv::Scalar(0));
   cv::Mat tr_pos(3, 1, CV_64F, cv::Scalar(0));

   double* p_pos = pos.ptr<double>();
   double* p_tr_pos = pos.ptr<double>();

   while (event_reader->hasMoreData())
   {
      TcspcEvent e = event_reader->getEvent();
      Photon p = processor.addEvent(e);
   
      if (p.valid && p.channel < n_chan)
      {
         int mapped_channel = channel_map[p.channel];

         if (mapped_channel == -1)
            continue;


         if (p.frame < frame_transform.size())
         {
            Transform& tr = frame_transform[p.frame];

            p_pos[0] = p.x; p_pos[1] = p.y;
            tr_pos = tr.affine * pos;
            p.x = p_tr_pos[0] - tr.shift.x; 
            p.y = p_tr_pos[1] - tr.shift.y;
         }

         p.x /= spatial_binning;
         p.y /= spatial_binning;

         int bin = (p.bin + time_shifts_resunit[p.channel]) % t_rep_resunit;
         bin = bin < 0 ? bin + t_rep_resunit : bin;
         bin = bin >> downsampling;

         if ((bin < n_bin) && (p.x < n_x_binned) && (p.x >= 0) && (p.y < n_y_binned) && (p.y >= 0))
            histogram[bin + n_bin * (mapped_channel + n_chan_stride * (p.x + n_x_binned * p.y))]++;
         else
            n_invalid++;
      }

   }
//   std::cout << "Num frames: " << n_frame << "\n";
//   std::cout << "Num invalid: " << n_invalid << "\n";
}


/*
template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride, bool get_single_frame_intensity)
{
   using namespace std;

   auto channels = validateChannels(channels_, n_chan_stride);
   
   if (get_single_frame_intensity)
      n_chan_stride = 1;

   assert(event_reader != nullptr);

   if (!get_single_frame_intensity)
      event_reader->setToStart();
   
   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
   {
      channel_map[c] = idx;
      if (!get_single_frame_intensity)
         idx++;
   }
   
   int n_bin = get_single_frame_intensity ? 1 : 1 << temporal_resolution;
   
   long long sync_count_accum = 0;
   int cur_line = 0;
   bool frame_started = 0;
   bool line_valid = false;
   long long sync_start = 0;
      
   int n_x_binned = n_x / spatial_binning;
   int n_y_binned = n_y / spatial_binning;
   
   int n_frame = 0;
   int n_invalid = 0;
   
   while (event_reader->hasMoreData())
   {
      TcspcEvent p = event_reader->getEvent();

      sync_count_accum += p.macro_time_offset;
      long long cur_sync = p.macro_time + sync_count_accum;

      if (p.valid)
      {
         if (p.mark & markers.FrameMarker)
         {
            if (n_frame > 0 && get_single_frame_intensity)
               return;

            n_frame++;
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

         if ((p.mark == markers.PhotonMarker) && line_valid && frame_started && p.channel < n_chan)
         {
            int mapped_channel = channel_map[p.channel];
            if (mapped_channel > -1)
            {
               double cur_loc = ((cur_sync - sync_start) / sync_count_per_line - sync_offset) * (n_x_binned);

               if (cur_direction == -1)
                  cur_loc = n_x_binned - 1 - cur_loc;

               if ((cur_line % (spatial_binning * line_averaging)) == 0)
                  cur_loc += first_line_sync_offset * n_x_binned;

               int cur_px = static_cast<int>(cur_loc);

               int dtime = (p.micro_time + time_shifts_resunit[p.channel]) % t_rep_resunit;
               dtime = dtime < 0 ? dtime + t_rep_resunit : dtime;
               int bin = dtime >> downsampling;

               int x = cur_px;
               int y = cur_line / (spatial_binning * line_averaging);

               int frame_idx = n_frame - 1;
               if (frame_idx < frame_shifts.size())
               {
                  x -= std::round(frame_shifts[frame_idx].x / spatial_binning);
                  y -= std::round(frame_shifts[frame_idx].y / spatial_binning);
               }


               if ((bin < n_bin) && (x < n_x_binned) && (x >= 0) && (y < n_y_binned) && (y >= 0))
                  histogram[bin + n_bin * (mapped_channel + n_chan_stride * (x + n_x_binned * y))]++;
               else
                  n_invalid++;
            }
         }
      }
   }
   
   std::cout << "Num frames: " << n_frame << "\n";
   std::cout << "Num invalid: " << n_invalid << "\n";

}
*/