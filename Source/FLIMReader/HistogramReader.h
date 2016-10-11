#pragma once

#include "FLIMreader.h"
#include <fstream>

class HistogramReader : public FLIMReader
{
public:

   HistogramReader(const std::string& filename) :
      FLIMReader(filename)
   {}

protected:

   int numChannels() { return n_chan; };

   template <class U, class T>
   void readData_(T* data, const std::vector<int>& channels_ = {}, int n_chan_stride = -1)
   {
      auto channels = validateChannels(channels_, n_chan_stride);

      // Determine channel mapping
      std::vector<int> channel_map(n_chan, -1);
      int idx = 0;
      for (auto& c : channels)
         channel_map[c] = idx++;
      int n_chan_used = idx;

      std::ifstream fs(filename, std::ifstream::in | std::ifstream::binary);
      fs.seekg(data_position);

      size_t data_size = n_x * n_y * n_chan * n_timepoints;
      std::vector<U> data_buf(data_size);

      fs.read((char*)data_buf.data(), data_size * sizeof(U));

      int n_xi = n_x / spatial_binning;
      int n_yi = n_y / spatial_binning;

      size_t n_el = n_xi * n_yi * n_chan_used * n_timepoints;

      // Set to zero
      for (int i = 0; i < n_el; i++)
         data[i] = 0;

      int yi, xi;
      int px = 0;

      for (int y = 0; y < n_y; y++)
      {
         yi = y / spatial_binning;
         for (int x = 0; x < n_x; x++)
         {
            xi = x / spatial_binning;
            for (int c = 0; c < n_chan; c++)
            {
               int mapped_channel = channel_map[c];
               if (mapped_channel > -1)
               {
                  int p = ((yi*n_xi + xi)*n_chan_stride + mapped_channel)*n_timepoints;

                  for (int t = 0; t < n_timepoints; t++)
                  {
                     data[p + t] += data_buf[px++];
                  }
               }
            }
         }
      }
   }

   std::streamoff data_position;
   int n_timepoints = 1;
   bool has_multiple_channels = false;
};