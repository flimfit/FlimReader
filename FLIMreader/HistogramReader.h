#pragma once

#include "FlimReader.h"
#include <fstream>
#include "zlib.h"

struct DataBlock
{
   std::streamoff position = 0;
   std::streamsize size = 0;
   bool compressed = false;
};


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

      std::vector<std::vector<U>> data_buf;

      if (channels_split)
      {
         if (blocks.size() < n_chan)
            throw std::runtime_error("Incorrect number of blocks");

         for (auto& c : channels)
            data_buf.emplace_back(readHistogramFromFile<U>(blocks[c]));
      }
      else
      {
        data_buf.emplace_back(readHistogramFromFile<U>(blocks[0]));
      }
        

      int n_xi = n_x / spatial_binning;
      int n_yi = n_y / spatial_binning;

      size_t n_timepoints_native = native_timepoints.size();
      size_t n_timepoints = timepoints.size();
      size_t n_el = n_xi * n_yi * n_chan_used * n_timepoints;

      // Set to zero
      std::fill_n(data, n_el, (T) 0);

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
                  size_t pi = ((yi*n_xi + xi)*n_chan_stride + ci)*n_timepoints;
                  std::vector<U>::iterator buf = channels_split ?
                     data_buf[ci].begin() + (y*n_x + x)*n_timepoints_native :
                     data_buf[0].begin() + ((y*n_x + x)*n_chan + c)*n_timepoints_native;

                  for (size_t t = 0; t < n_timepoints_native; t++)
                     data[pi + (t >> downsampling)] += static_cast<T>(*(buf++));;
               }
            }
         }
      }
   }

   template <class U>
   std::vector<U> readHistogramFromFile(DataBlock block)
   {
      std::ifstream fs(filename, std::ifstream::in | std::ifstream::binary);
      fs.seekg(block.position);

      size_t data_size = n_x * n_y * native_timepoints.size();
      data_size *= (channels_split ? 1 : n_chan);
      std::vector<U> data(data_size);

      if (block.compressed)
         readCompressedHistogramFromFile(data, fs, block.size);
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
   void readCompressedHistogramFromFile(std::vector<U>& data, std::ifstream& fs, std::streamsize compressed_size)
   {
      size_t buffer_size = (compressed_size == 0) ? 1024 * 1024 : compressed_size;
      std::vector<unsigned char> buffer(buffer_size);

      int wbits;

      // If we have a zip file, skip over header
      fs.read((char*)buffer.data(), 26);
      if (strncmp((char*)buffer.data(), "PK", 2) == 0)
      {
         uint16_t filename_len, extra_len;
         fs.read((char*)&filename_len, sizeof(filename_len));
         fs.read((char*)&extra_len, sizeof(extra_len));
         fs.ignore(filename_len + extra_len);
         wbits = -MAX_WBITS;
      }
      else
      {
         fs.seekg(-26, std::ios_base::cur);
         wbits = MAX_WBITS;
      }

      z_stream zInfo = { 0 };
//      fs.read((char*)buffer.data(), buffer.size());
      zInfo.next_in = buffer.data();
      zInfo.avail_in = 0;  (uInt)fs.gcount();

      int nErr = inflateInit2(&zInfo, wbits);
     
      if (nErr) throw std::runtime_error("Error decompressing file");

      do
      {
         if (zInfo.avail_in == 0)
         {
            fs.read((char*)buffer.data(), buffer.size());
            zInfo.avail_in = (uInt)fs.gcount();

         }

         zInfo.avail_out = (uInt)(data.size() * sizeof(U)) - zInfo.total_out;
         zInfo.next_in = buffer.data();
         zInfo.next_out = (unsigned char*) data.data() + zInfo.total_out;
         nErr = inflate(&zInfo, Z_SYNC_FLUSH);     // zlib function

      } while (nErr == Z_OK);

      if (zInfo.total_out != (data.size() * sizeof(U)) || nErr != Z_STREAM_END)
         throw std::runtime_error("Compressed data is incorrect size");
      inflateEnd(&zInfo);

   }

   std::vector<DataBlock> blocks;
   bool has_multiple_channels = false;
   bool channels_split = false;
};