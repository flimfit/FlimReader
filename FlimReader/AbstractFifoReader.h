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
   AbstractFifoReader();
   
   ~AbstractFifoReader();
   
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);
   
   bool canReadNumZ() const { return false; }
   void setNumZ(int n_z_) { n_z = n_z_; }

   bool canReadBidirectionalScan() const { return true; }
   void setBidirectionalScan(bool bidirectional_scan = true) {}; // do nothing
   bool getBidirectionalScan() const { return sync.bidirectional; }
   void setBidirectionalPhase(double phase) { sync.phase = phase; }

   void stopReading();
   void clearStopSignal() { terminate = false; }

   bool supportsRealignment() { return true; }
   double getProgress() { return event_reader->getProgress(); }


   void initaliseTimepoints(int n_timebins_native, double time_resolution_native_ps);
   void setTemporalDownsampling(int downsampling);

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
   std::vector<float> time_shifts_ps;
   double t_rep_ps = 0;
   SyncSettings sync;

   std::shared_ptr<AbstractEventReader> event_reader;
   Markers markers;
   
private:
   
   double time_resolution_native_ps = 0;
   int t_rep_resunit;
   std::vector<int> time_shifts_resunit;

   int last_frame_binning = -1;

   bool save_mean_arrival_images = false;
   std::vector<cv::Mat> ma_image;
};


template<typename T>
void AbstractFifoReader::computeMeanArrivalImage(const T* histogram)
{
   if (!save_mean_arrival_images)
      return;

   cv::Mat mean_arrival_time(n_y, n_x, CV_32F, cv::Scalar(0));

   int n_px = n_x * n_y;
   for (int p = 0; p < n_px; p++)
   {
      const T* data_ptr = histogram + p * (timepoints.size() * n_chan);
      uint16_t I = 0;
      float It = 0;
      for (int c = 0; c < 1; c++)
         for (int t = 0; t < this->timepoints.size(); t++)
         {
            I += static_cast<uint16_t>(*data_ptr);
            It += static_cast<float>((*data_ptr) * timepoints[t]);

            data_ptr++;
         }
      mean_arrival_time.at<float>(p) = It / I;
   }
   ma_image.push_back(mean_arrival_time);
}