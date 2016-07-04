#pragma once
#include "FLIMReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

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

   Transform(double frame_)
   {
      frame = frame_;
      angle = 0.0;
      shift.x = 0.0;
      shift.y = 0.0;
   }

   Transform(double frame, double angle, cv::Point2d shift) :
      frame(frame), angle(angle), shift(shift)
   {
   }

   double frame;
   double angle;
   cv::Point2d shift;
};

class TransformInterpolator
{
public:

   TransformInterpolator()
   {
      frame_transform.push_back(Transform(0.0));
   }

   void setSize(int width, int height)
   {
      centre.x = width * 0.5;
      centre.y = height * 0.5;
   }

   bool empty()
   {
      return frame_transform.size() <= 1;
   }

   void clear()
   {
      frame_transform.clear();
      frame_transform.push_back(Transform(0.0));
   }

   void addTransform(Transform t)
   {
      assert(t.frame > frame_transform.back().frame);
      frame_transform.push_back(t);
   }

   void getAffine(double frame, cv::Mat& affine, cv::Point2d& shift)
   {
      assert(frame >= 0);

      if (frame == cache_frame)
      {
         affine = cache_affine;
         shift = cv::Point2d(0, 0);
      }
      else if (frame == 0.0 || frame_transform.size() == 1)
      {
         affine = cv::Mat::eye(2, 3, CV_64F);
         shift = cv::Point2d(0, 0);
      }
      else if (frame == frame_transform.back().frame)
      {
         interpolate(frame_transform.back(), frame_transform.back(), frame, affine, shift);
      }
      else
      {
         int idx = 0;
         while (frame > frame_transform[idx].frame)
            idx++;

         interpolate(frame_transform[idx - 1], frame_transform[idx], frame, affine, shift);
      }
   }

   void interpolate(Transform& t1, Transform& t2, double frame, cv::Mat& affine, cv::Point2d& shift)
   {
      double f = (frame - t1.frame) / (t2.frame - t1.frame);

      double angle = t2.angle * f + t1.angle * (1 - f);
      affine = cv::getRotationMatrix2D(centre, angle, 1.0);

      shift = t2.shift * f + t1.shift * (1 - f);
   }

private:
   std::vector<Transform> frame_transform;

   double cache_frame = -1;
   cv::Mat cache_affine;
   cv::Point2d cache_shift;

   cv::Point2d centre;
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
   TransformInterpolator transform_interpolator;
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
   
   cv::Mat affine;
   cv::Point2d shift;

   while (event_reader->hasMoreData())
   {
      TcspcEvent e = event_reader->getEvent();
      Photon p = processor.addEvent(e);
   
      if (p.valid && p.channel < n_chan)
      {
         int mapped_channel = channel_map[p.channel];

         if (mapped_channel == -1)
            continue;

         if (!transform_interpolator.empty())
         {
            double sub_frame = p.frame + static_cast<double>(p.y) / n_y;

            transform_interpolator.getAffine(sub_frame, affine, shift);

            p_pos[0] = p.x; p_pos[1] = p.y;
            tr_pos = affine * pos;
            p.x = p_tr_pos[0] - shift.x; 
            p.y = p_tr_pos[1] - shift.y;
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
}