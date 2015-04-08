#pragma once

#include <vector>
#include <string>

class FLIMReader
{
public:
   
   FLIMReader(const std::string& filename_)
   {
      filename = filename_;
   }

   virtual ~FLIMReader() {};

   virtual int GetNumberOfChannels() { return 1; };
   const std::vector<double>& GetTimePoints() { return timepoints; };
   virtual void ReadData(const std::vector<int>& channels, float* data) = 0;
   //virtual void GetData(double* data) = 0;

   int GetNumX() { return n_x; }
   int GetNumY() { return n_y; }

   virtual void SetTemporalResolution(int temporal_resolution_) {}; // do nothing in general case;
   int GetTemporalResolution() { return temporal_resolution; }

protected:

   std::vector<double> timepoints;

   std::string filename;
   int temporal_resolution = 0;
   int n_x = 1;
   int n_y = 1;
};
