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
   std::vector<int> validateChannels(const std::vector<int> channels);

   virtual ~FLIMReader() {};

   virtual int numChannels() { return 1; };
   virtual void readData(float* data, const std::vector<int>& channels = {}) = 0;
   virtual void readData(double* data, const std::vector<int>& channels = {}) = 0;
   virtual void readData(uint16_t* data, const std::vector<int>& channels = {}) = 0;

   template<typename T>
   std::vector<T> readData(const std::vector<int>& channels = {})
   {
      std::vector<int>& channels_ = validateChannels(channels);
      std::vector<T> data(channels_.size() * timepoints_.size());
      readData(data.data(), channels_);
      return data;
   }

   const std::vector<double>& timepoints() { return timepoints_; };
   int numX() { return n_x; }
   int numY() { return n_y; }

   virtual void setTemporalResolution(int temporal_resolution) {}; // do nothing in general case;
   int temporalResolution() { return temporal_resolution_; }

protected:

   std::vector<double> timepoints_;

   std::string filename;
   std::string extension;
   int temporal_resolution_;
   int n_x;
   int n_y;
};
