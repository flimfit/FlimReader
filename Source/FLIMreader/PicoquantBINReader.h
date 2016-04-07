#include "FLIMreader.h"
#include <fstream>

class PicoquantBINReader : public FLIMReader
{
public:
   PicoquantBINReader(const std::string& filename, const std::string& ext) :
      FLIMReader(filename)
   {
      if (ext == "bin2")
         has_multiple_channels = true;

      readHeader();
   }

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };

private:

   void readHeader()
   {
      std::ifstream fs(filename, std::ifstream::in | std::ifstream::binary);

      int32_t PixX, PixY, NChan = 1, TCSPCChannels;
      float PixResol, TimeResol;

      fs.read((char*) &PixX, sizeof(PixX));
      fs.read((char*) &PixY, sizeof(PixY));
      fs.read((char*) &PixResol, sizeof(PixResol));

      if (has_multiple_channels)
         fs.read((char*) &NChan, sizeof(NChan));

      fs.read((char*) &TCSPCChannels, sizeof(TCSPCChannels));
      fs.read((char*) &TimeResol, sizeof(TimeResol));

      long long expected_size = PixX * PixY * NChan * TCSPCChannels * sizeof(float) + header_size;

      fs.seekg(0, std::ifstream::end);
      if (fs.tellg() < expected_size)
         throw std::runtime_error("File does not appear to be the correct size");

      n_x = PixX;
      n_y = PixY;
      n_timepoints = TCSPCChannels;
      n_chan = NChan;

      timepoints_.resize(n_timepoints);

      for (int i = 0; i < n_timepoints; i++)
         timepoints_[i] = i * TimeResol * 1000;

   }

   int numChannels() { return n_chan; };


   template <class T>
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
      fs.seekg(header_size);

      int n_xi = n_x / spatial_binning;
      int n_yi = n_y / spatial_binning;

      size_t n_el = n_xi * n_yi * n_chan_used * n_timepoints;

      // Set to zero
      for (int i = 0; i < n_el; i++)
         data[i] = 0;

      int yi, xi;
      
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
                  int p = ((yi*n_xi+xi)*n_chan_stride + mapped_channel)*n_timepoints;

                  for (int t = 0; t < n_timepoints; t++)
                  {
                     int value;
                     fs.read(reinterpret_cast<char*>(&value), 4);
                     data[p + t] += value;
                  }
               }
            }
         }
      }
   }

   const int header_size = 20;
   int n_timepoints = 1;
   bool has_multiple_channels = false;
};