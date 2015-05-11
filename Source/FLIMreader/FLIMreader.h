#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

class FLIMReader
{
public:
   
   static FLIMReader* createReader(std::string& filename);

   FLIMReader(const std::string& filename_);
    
   static std::string determineExtension(std::string& filename);
   std::vector<int> validateChannels(const std::vector<int> channels, int& n_chan_stride);

   virtual ~FLIMReader() {};

   virtual int numChannels() { return 1; };
   virtual void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;

   template<typename T>
   std::vector<T> readData(const std::vector<int>& channels = {})
   {
      int n_chan_stride = -1;
      std::vector<int> channels_ = validateChannels(channels, n_chan_stride);
      std::vector<T> data(channels_.size() * timepoints_.size());
      readData(data.data(), channels_);
      return data;
   }

   const std::vector<double>& timepoints() { return timepoints_; };
   int numX() { return n_x / spatial_binning_; }
   int numY() { return n_y / spatial_binning_; }

   virtual void setTemporalResolution(int temporal_resolution) {}; // do nothing in general case;
   int temporalResolution() { return temporal_resolution_; }

   int dataSizePerChannel();

   void setSpatialBinning(int spatial_binning)
   {
      if (n_x > spatial_binning && n_y > spatial_binning)
         spatial_binning_ = spatial_binning;
   }
   
   int spatialBinning() { return spatial_binning_; }
   
protected:

   std::vector<double> timepoints_;

   std::string filename;
   std::string extension;
   int temporal_resolution_;
   int spatial_binning_ = 1;
   int n_x;
   int n_y;
};
