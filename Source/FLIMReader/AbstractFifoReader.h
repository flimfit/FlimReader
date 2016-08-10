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

#include "FifoProcessor.h"

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, std::streamoff data_position) :
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
   std::streamoff data_position;
};


class AbstractFifoReader : public FLIMReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
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
   //long long n_records;
   double time_resolution_native_ps;
   double t_rep_ps;
   int n_timebins_native;

   SyncSettings sync;

   std::unique_ptr<AbstractEventReader> event_reader;
   Markers markers;
   
private:
   
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;
   std::unique_ptr<AbstractFrameAligner> frame_aligner;
};





template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   alignFrames();

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

         frame_aligner->shiftPixel(p.frame, p.x, p.y);

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