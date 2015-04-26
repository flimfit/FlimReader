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
   void readData(float* data, const std::vector<int>& channels = {}) { readData_(data, channels); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}) { readData_(data, channels); };

protected:

   template<typename T>
   void readData_(T* data, const std::vector<int>& channels = {});

   void readHeader();

   char delim = ',';
   char row_delim = '\n';
   int n_header_row = 0;

   std::vector<std::vector<float>> data;
   std::vector<FLIMMetadata> metadata;
};

template<typename T>
void TextReader::readData_(T* data_, const std::vector<int>& channels)
{
   size_t n_t = timepoints_.size();
   size_t n_chan = channels.size();

   for (size_t i = 0; i < n_t; i++)
      for (size_t c = 0; c < n_chan; c++)
         data_[i*n_chan + c] = static_cast<T>(data[c][i]);
}
