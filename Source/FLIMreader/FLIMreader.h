#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

class RealignmentParameters 
{
public:
   bool use_realignment = false;
   int spatial_binning = 1;
   int frame_binning = 1;
   bool use_rotation = false;
};

class FLIMReader
{
public:
   
   static FLIMReader* createReader(const std::string& filename);

   FLIMReader(const std::string& filename_);
    
   static std::string determineExtension(const std::string& filename);
   std::vector<int> validateChannels(const std::vector<int> channels, int& n_chan_stride);

   virtual ~FLIMReader() {};

   virtual int getNumChannels() { return n_chan; };
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
   int numX() { return n_x / spatial_binning; }
   int numY() { return n_y / spatial_binning; }

   virtual void setTemporalResolution(int temporal_resolution) {}; // do nothing in general case;
   int getTemporalResolution() { return temporal_resolution; }

   int dataSizePerChannel();

   virtual bool supportsRealignment() { return false; }
   void setRealignmentParameters(RealignmentParameters realign_params_) { realign_params = realign_params_; }

   void setSpatialBinning(int spatial_binning_)
   {
      if (n_x > spatial_binning_ && n_y > spatial_binning_)
         spatial_binning = spatial_binning_;
   }
   
   int getSpatialBinning() { return spatial_binning; }
   
protected:

   std::vector<double> timepoints_;

   std::string filename;
   std::string extension;
   int temporal_resolution = 1;
   int spatial_binning = 1;
   int n_x = 0;
   int n_y = 0;
   int n_chan = 0;

   RealignmentParameters realign_params;
};
