#pragma once

#include "HistogramReader.h"
#include <libics.h>

struct DimensionOrder
{
   int x = 0;
   int y = 1;
   int z = 2;
   int c = 3;
   int t = 4;
};

class IcsReader : public FlimReader
{
public:

   IcsReader(const std::string& filename);
   ~IcsReader();

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };

   bool canReadBidirectionalScan() const { return true; }
   void setBidirectionalScan(bool bidirectional_scan = true) {}; // do nothing
   bool getBidirectionalScan() const { return false; }
   

private:

   void readHeader();
   void check(Ics_Error err);

   template <class T>
   void readData__(T* data, const std::vector<int>& channels_, int n_chan_stride)
   {
      if (native_type == DataTypeUint16)
         readData_<uint16_t, T>(data, channels_, n_chan_stride);
      else if (native_type == DataTypeFloat)
         readData_<float, T>(data, channels_, n_chan_stride);
      else if (native_type == DataTypeDouble)
         readData_<double, T>(data, channels_, n_chan_stride);
      else
         throw std::runtime_error("Error reading ICS file: unknown data type");
   }

   template <class U, class T>
   void readData_(T* data, const std::vector<int>& channels_ = {}, int n_chan_stride = -1)
   {
      if (channels_.size() != 1 || channels_[0] != 0)
         throw std::runtime_error("IcsReader currently only supports a single channel");

      size_t data_size = IcsGetDataSize(ics);
      size_t buf_size = data_size / sizeof(U);

      std::vector<U> buf(buf_size);
      check(IcsGetData(ics, buf.data(), data_size));

      int n_xi = n_x / spatial_binning;
      int n_yi = n_y / spatial_binning;

      size_t n_timepoints_native = native_timepoints.size();
      size_t n_timepoints = timepoints.size();
      size_t n_el = n_xi * n_yi * n_timepoints;

      // Set to zero
      std::fill_n(data, n_el, (T)0);

      int yi, xi;
      int px = 0;

      for (int y = 0; y < n_y; y++)
      {
         yi = y / spatial_binning;
         for (int x = 0; x < n_x; x++)
         {
            xi = x / spatial_binning;
            {
               size_t pi = (yi*n_xi + xi)*n_timepoints;
               size_t idx = y * n_x + x;

               for (size_t t = 0; t < n_timepoints_native; t++)
                  data[pi + (t >> downsampling)] += static_cast<T>(buf[n_x*n_y*t + idx]);
            }
         }
      }
   }

   static std::mutex ics_mutex;
   ICS* ics;
   DimensionOrder dim_order;
};

