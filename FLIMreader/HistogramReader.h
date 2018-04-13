#pragma once

#include "FlimReader.h"
#include <fstream>
#include "zlib.h"

class HistogramReader : public FlimReader
{
public:

   HistogramReader(const std::string& filename) :
      FlimReader(filename)
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

      std::vector<U> data_buf = readHistogramFromFile<U>();

      int n_xi = n_x / spatial_binning;
      int n_yi = n_y / spatial_binning;

      size_t n_el = n_xi * n_yi * n_chan_used * n_timepoints;

      // Set to zero
      for (int i = 0; i < n_el; i++)
         data[i] = 0;

      int yi, xi;
      int px = 0;

      for (int ci = 0; ci < channels.size(); ci++)
      {
         int c = channels[ci];

         for (int y = 0; y < n_y; y++)
         {
            yi = y / spatial_binning;
            for (int x = 0; x < n_x; x++)
            {
               xi = x / spatial_binning;
               {
                  int pi = ((yi*n_xi + xi)*n_chan_stride + ci)*n_timepoints;
                  int p = ((y*n_x + x)*n_chan + c)*n_timepoints;

                  for (int t = 0; t < n_timepoints; t++)
                     data[pi + t] += (T)data_buf[p++];
               }
            }
         }
      }
   }

   template <class U>
   std::vector<U> readHistogramFromFile()
   {
      std::ifstream fs(filename, std::ifstream::in | std::ifstream::binary);
      fs.seekg(data_position);

      size_t data_size = n_x * n_y * n_chan * n_timepoints;
      std::vector<U> data(data_size);

      if (compressed)
         readCompressedHistogramFromFile(data, fs);
      else
         readUncompressedHistogramFromFile(data, fs);

      return data;
   }

   template <class U>
   void readUncompressedHistogramFromFile(std::vector<U>& data, std::ifstream& fs)
   {
      fs.read((char*)data.data(), data.size() * sizeof(U));
   }

   template <class U>
   void readCompressedHistogramFromFile(std::vector<U>& data, std::ifstream& fs)
   {
      std::vector<unsigned char> buffer(compressed_size);
      fs.read((char*)buffer.data(), compressed_size);

      z_stream zInfo = { 0 };
      zInfo.total_in = zInfo.avail_in = compressed_size;
      zInfo.total_out = zInfo.avail_out = data.size() * sizeof(U);
      zInfo.next_in = buffer.data();
      zInfo.next_out = (unsigned char*) data.data();

      int nErr;
      nErr = inflateInit(&zInfo);
      if (nErr) throw std::runtime_error("Error decompressing file");
      nErr = inflate(&zInfo, Z_FINISH);     // zlib function

      if (zInfo.total_out != (data.size() * sizeof(U)) || nErr != Z_STREAM_END)
         throw std::runtime_error("Compressed data is incorrect size");
      inflateEnd(&zInfo);
   }

   std::streamoff data_position;
   int n_timepoints = 1;
   bool has_multiple_channels = false;
   bool compressed = false;
   size_t compressed_size = 0;
};