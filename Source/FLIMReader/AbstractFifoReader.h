#pragma once
#include "FlimReader.h"

#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>


#include "FifoProcessor.h"

class AbstractEventReader
{
public:
   AbstractEventReader(const std::string& filename, std::streamoff data_position, uint packet_size) :
   data_position(data_position), packet_size(packet_size)
   {
      fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
      
      fs.seekg(0, fs.end);
      length = fs.tellg() - data_position;
      n_packet = length / packet_size;
      uint64_t n_block = n_packet / block_size + 1;
      data.reserve(n_block);
      
      reader_thread = std::thread(&AbstractEventReader::read, this);
      
      setToStart();
   }
   
   ~AbstractEventReader()
   {
      terminate = true;
      if (reader_thread.joinable())
         reader_thread.join();
   }


   void read()
   {
      uint64_t sz = block_size * packet_size;
      
      fs.seekg(data_position);

      size_t n_read = 0;
      do
      {
         std::vector<char> block(sz);
         fs.read(&block[0], sz);
         n_read = fs.gcount();

         std::unique_lock<std::mutex> lk(m);
         data.push_back(block);
         cv.notify_one();
      } while (n_read > 0 && !terminate);
   }

   double getProgress()
   {
      return ((double) cur_pos) / n_packet;
   }

   void setToStart()
   {
      cur_pos = 0;
   }

   bool hasMoreData()
   {
      return cur_pos < n_packet;
   }

   virtual TcspcEvent getEvent() = 0;

   const char* getPacket()
   {
      uint64_t block = cur_pos / block_size;
      uint64_t packet = cur_pos % block_size;
      
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [&]() { return block < data.size(); });
  
      cur_pos++;
      return &(data[block][packet*packet_size]);
   }
   
protected:
   
   std::ifstream fs;
   std::streamoff data_position;
   uint64_t packet_size;
   uint64_t length = 0;
   uint64_t n_packet = 0;
   std::vector<std::vector<char>> data;

   uint64_t block_size = 1024;

   uint64_t cur_pos = 0;

   std::thread reader_thread;
   std::mutex m;
   std::condition_variable cv;

   bool terminate = false;
};


class AbstractFifoReader : public FlimReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   void stopReading() { terminate = true; }

   bool supportsRealignment() { return true; }
   bool isBidirectional() { return sync.bi_directional; }
   void setBidirectionalPhase(double phase) { sync.phase = phase; }

   void alignFrames();

   float getProgress() { return event_reader->getProgress(); }

   void setTemporalResolution(int temporal_resolution);
      
protected:
   
   void readSettings();

   void computeIntensityNormalisation();
   
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

   computeIntensityNormalisation();

   auto channels = validateChannels(channels_, n_chan_stride);

   // Determine channel mapping
   std::vector<int> channel_map(n_chan, -1);
   int idx = 0;
   for (auto& c : channels)
      channel_map[c] = idx++;
   
   int n_bin = (int)timepoints_.size();
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

   while (event_reader->hasMoreData() && !terminate)
   {
      TcspcEvent e = event_reader->getEvent();
      Photon p = processor.addEvent(e);
   
      if (p.valid && p.channel < n_chan)
      {
         int mapped_channel = channel_map[p.channel];

         if (mapped_channel == -1)
            continue;

         if (frame_aligner != nullptr)
         {
            double correlation = frame_aligner->getFrameCorrelation(p.frame);
            double coverage = frame_aligner->getFrameCoverage(p.frame);
            if (correlation < realign_params.correlation_threshold || coverage < realign_params.coverage_threshold)
               continue;

            frame_aligner->shiftPixel(p.frame, p.x, p.y);
         }

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
      }

   }
}