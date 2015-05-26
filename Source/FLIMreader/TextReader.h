#pragma once

#include "FLIMReader.h"
#include <vector>
#include <map>

typedef std::map<std::string, std::string> FLIMMetadata;

class TextReader : public FLIMReader
{
public:

   TextReader(const std::string& filename);

   int numChannels() { return (int) data.size(); };
   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_(data, channels, n_chan_stride); };

protected:

   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {}, int n_chan_stride = -1);

   void readHeader();

   char delim = ',';
   char row_delim = '\r';
   int n_header_row = 0;

   std::vector<std::vector<float>> data;
   std::vector<FLIMMetadata> metadata;
};

template<typename T>
void TextReader::readData_(T* data_, const std::vector<int>& channels_, int n_chan_stride)
{
   auto channels = validateChannels(channels_, n_chan_stride);
   
   size_t n_t = timepoints_.size();
   size_t n_chan = channels.size();

   for (size_t c = 0; c < n_chan; c++)
      for (size_t i = 0; i < n_t; i++)
         data_[i + c*n_t] = static_cast<T>(data[channels[c]][i]);
}
