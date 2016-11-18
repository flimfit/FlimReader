#pragma once
#include "FLIMReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>

<<<<<<< HEAD
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
=======
class Markers
{
public:
   uint8_t PhotonMarker = 0x0;
   uint8_t LineStartMarker = 0x0;
   uint8_t LineEndMarker = 0x0;
   uint8_t FrameMarker = 0x0;
   uint8_t Invalid = 0x80;
};

>>>>>>> stable

#include "FifoProcessor.h"

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, std::streamoff data_position) :
   data_position(data_position)
   {
      fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
      
      fs.seekg(0, fs.end);
      length = (double) (fs.tellg() - data_position);
      
      setToStart();

   }
   
   double getProgress()
   {
      return (fs.tellg() - data_position) / length;
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
   std::streamoff data_position;
   double length = 0;
};


class AbstractFifoReader : public FLIMReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   void stopReading() { terminate = true; }

   bool supportsRealignment() { return true; }

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
   double time_resolution_native_ps;
   double t_rep_ps;
   int n_timebins_native;

   SyncSettings sync;

   std::unique_ptr<AbstractEventReader> event_reader;
   Markers markers;
   
   bool terminate = false;

private:
   
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;
};





template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   terminate = false;

   assert(event_reader != nullptr);
   event_reader->setToStart();

   auto channels = validateChannels(channels_, n_chan_stride);

   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;
   
   int n_bin = 1 << temporal_resolution;
<<<<<<< HEAD
=======
   
   event_reader->setToStart();
   
   long long sync_count_accum = 0;
   int cur_line = 0;
   bool frame_started = markers.FrameMarker == 0x0;
   bool line_valid = false;
   long long sync_start = 0;
      
>>>>>>> stable
   int n_x_binned = n_x / spatial_binning;
   int n_y_binned = n_y / spatial_binning;
   int n_invalid = 0;

<<<<<<< HEAD
   FifoProcessor processor(markers, sync);

   cv::Mat pos(3, 1, CV_64F, cv::Scalar(0));
   cv::Mat tr_pos(3, 1, CV_64F, cv::Scalar(0));

   double* p_pos = pos.ptr<double>();
   double* p_tr_pos = pos.ptr<double>();
=======
   auto incrementFrame = [&]() {
      n_frame++;
      frame_started = true;
      cur_line = -1;
      cur_direction = 1;
   };
>>>>>>> stable
   
   cv::Mat affine;
   cv::Point2d shift;

   while (event_reader->hasMoreData() && !terminate)
   {
      TcspcEvent e = event_reader->getEvent();
      Photon p = processor.addEvent(e);
   
      if (p.valid && p.channel < n_chan)
      {
         int mapped_channel = channel_map[p.channel];

         if (mapped_channel == -1)
            continue;

<<<<<<< HEAD
         if (frame_aligner != nullptr)
            frame_aligner->shiftPixel(p.frame, p.x, p.y);

         p.x /= spatial_binning;
         p.y /= spatial_binning;

         int x = (int) std::round(p.x);
         int y = (int) std::round(p.y);

         int bin = (p.bin + time_shifts_resunit[p.channel]) % t_rep_resunit;
         bin = bin < 0 ? bin + t_rep_resunit : bin;
         bin = bin >> downsampling;

         if ((bin < n_bin) && (x < n_x_binned) && (x >= 0) && (y < n_y_binned) && (y >= 0))
            histogram[bin + n_bin * (mapped_channel + n_chan_stride * (x + n_x_binned * y))]++;
         else
            n_invalid++;
=======
      if (p.valid)
      {
         if (p.mark & markers.FrameMarker)
            incrementFrame();

         if (frame_started)
         {
            if ((p.mark & markers.LineEndMarker) && line_valid)
            {
               line_valid = false;
            }
            else if (p.mark & markers.LineStartMarker)
            {
               line_valid = true;
               sync_start = cur_sync;
               cur_line++;

               if ((cur_line >= n_y) && markers.FrameMarker == 0x0)
                  incrementFrame();

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

               int cur_px = static_cast<int>(cur_loc);

               if (cur_direction == -1)
                  cur_px = n_x_binned - 1 - cur_px;

               int dtime = (p.micro_time + time_shifts_resunit[p.channel]) % t_rep_resunit;
               dtime = dtime < 0 ? dtime + t_rep_resunit : dtime;
               int bin = dtime >> downsampling;

               int x = cur_px;
               int y = cur_line / (spatial_binning * line_averaging);

               if ((bin < n_bin) && (x < n_x_binned) && (x >= 0) && (y < n_y_binned) && (y >= 0))
                  histogram[bin + n_bin * (mapped_channel + n_chan_stride * (x + n_x_binned * y))]++;
               else
                  n_invalid++;
            }
         }
>>>>>>> stable
      }

   }
}