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
#include <chrono>
#include <tuple>

#include "FifoProcessor.h"
#include "WriteMultipageTiff.h"
#include "AbstractEventReader.h"



class AbstractFifoReader : public FlimReader
{
public:
   
   AbstractFifoReader(const std::string& filename);
   ~AbstractFifoReader();
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   
   bool canReadNumZ() { return false; }
   void setNumZ(int n_z_) { n_z = n_z_; }


   void stopReading() 
   { 
      terminate = true; 
      realign_cv.notify_all();
   }

   void clearStopSignal() { terminate = false; }

   bool supportsRealignment() { return true; }
   bool isBidirectional() { return sync.bidirectional; }
   void setBidirectionalPhase(double phase) { sync.phase = phase; }

   double getProgress() { return event_reader->getProgress(); }

   void setTemporalResolution(int temporal_resolution);
      
protected:

   ImageScanParameters getImageScanParameters() {
      ImageScanParameters params(sync.count_per_line, sync.counts_interline, sync.counts_interframe, n_x, n_y, n_z, sync.bidirectional);
      return params;
   }
   
   void loadIntensityFramesImpl();
   int getNumIntensityFrames() { return (int) frames.size(); };

   void readSettings();

   template<typename T>
   void computeMeanArrivalImage(const T* histogram);

   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   
   bool useFrame(int frame) { return true; }

   void determineDwellTime();
      
   int line_averaging = 1;
   int downsampling;
   
   std::vector<float> time_shifts_ps;
   
   // Required Picoquant information
   int measurement_mode = 0;
   double time_resolution_native_ps = 0;
   double t_rep_ps = 0;
   int n_timebins_native = 0;

   SyncSettings sync;

   std::shared_ptr<AbstractEventReader> event_reader;
   Markers markers;
   
private:
   
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;

   int last_frame_binning = -1;

   bool save_mean_arrival_images = false;
   std::vector<cv::Mat> ma_image;
   
   bool finished_loading_intensity_frames = false;
};





template<typename T>
void AbstractFifoReader::readData_(T* histogram, const std::vector<int>& channels_, int n_chan_stride)
{
   if (terminate)
      return;

   if (realignment_complete) // we are reprocessing
      computeIntensityNormalisation();
   if (realign_params.type == RealignmentType::None)
      intensity_normalisation = cv::Mat();

   assert(event_reader != nullptr);
   event_reader->setToStart();

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
   int last_frame_written = 0;

   auto fifo_frame = std::make_shared<FifoFrame>(event_reader, markers);
   fifo_frame->loadNext();


   FifoProcessor processor(markers, sync);

   cv::Mat pos(3, 1, CV_64F, cv::Scalar(0));
   cv::Mat tr_pos(3, 1, CV_64F, cv::Scalar(0));

   double* p_pos = pos.ptr<double>();
   double* p_tr_pos = pos.ptr<double>();

   cv::Mat affine;
   cv::Point2d shift;

   ma_image.clear();

   int frame_idx = 0;

   while (event_reader->hasMoreData() && !terminate)
   {

      fifo_frame->loadNext();
      FifoProcessor2 processor2(markers, sync);
      processor2.setFrame(fifo_frame);

      int frame = frame_idx / n_z;
      int z = frame_idx % n_z;
      frame_idx++;

      Photon p;
      while (( p = processor2.getNextPhoton() ))
      {
         if (frame > last_frame_written)
         {
            computeMeanArrivalImage(histogram);
            last_frame_written = frame;
         }

         if (!useFrame(p.frame)) continue;
         if (!p.valid || p.channel > n_chan) continue;

         int mapped_channel = channel_map[p.channel];
         if (mapped_channel == -1) continue;

         p.z = z;

         if (frame_aligner != nullptr && frame_aligner->frameValid(frame))
         {
            // Check that we have realigned this frame
             if (!frame_aligner->frameReady(frame))
            {
               std::unique_lock<std::mutex> lk(realign_mutex);
               realign_cv.wait(lk, [this, frame] {
                  return (frame_aligner->frameReady(frame) || terminate);
               });
               lk.unlock();
            }

            if (terminate) break;

            double correlation = frame_aligner->getFrameCorrelation(frame);
            double coverage = frame_aligner->getFrameCoverage(frame);
            if (correlation < realign_params.correlation_threshold || coverage < realign_params.coverage_threshold)
               continue;

            frame_aligner->shiftPixel(frame, p.x, p.y, p.z);
         }

         p.x /= spatial_binning;
         p.y /= spatial_binning;

         int64_t x = (int64_t)std::round(p.x);
         int64_t y = (int64_t)std::round(p.y);
         int64_t z = (int64_t)std::round(p.z);

         int bin = p.bin;
         if (t_rep_resunit > 0)
         {
            bin = (bin + time_shifts_resunit[p.channel]) % t_rep_resunit;
            bin = bin < 0 ? bin + t_rep_resunit : bin;
         }
         bin = bin >> downsampling;

         if ((bin < n_bin) && (x < n_x_binned) && (x >= 0)
            && (y < n_y_binned) && (y >= 0)
            && (z < n_z) && (z >= 0))
         {
            size_t idx = (x + n_x_binned * y + n_x_binned * n_y_binned * z);
            histogram[bin + n_bin * (mapped_channel + n_chan_stride * idx)]++;
         }
         else
         {
            n_invalid++;
         }
      }
   }

   if (save_mean_arrival_images)
   {
      computeMeanArrivalImage(histogram);
      writeMultipageTiff("c:/users/cimlab/documents/test/ma-image.tif", ma_image);
   }
}

template<typename T>
void AbstractFifoReader::computeMeanArrivalImage(const T* histogram)
{
   if (!save_mean_arrival_images)
      return;

   cv::Mat mean_arrival_time(n_y, n_x, CV_32F, cv::Scalar(0));

   int n_px = n_x * n_y;
   for (int p = 0; p < n_px; p++)
   {
      const T* data_ptr = histogram + p * (timepoints_.size() * n_chan);
      uint16_t I = 0;
      float It = 0;
      for (int c = 0; c < 1; c++)
         for (int t = 0; t < this->timepoints_.size(); t++)
         {
            I += static_cast<uint16_t>(*data_ptr);
            It += static_cast<float>((*data_ptr) * timepoints_[t]);

            data_ptr++;
         }
      mean_arrival_time.at<float>(p) = It / I;
   }
   ma_image.push_back(mean_arrival_time);
}