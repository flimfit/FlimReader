#pragma once
#include "HistogramReader.h"
#include "FfdHeader.h"

class FfhReader : public HistogramReader
{
public:
   FfhReader(const std::string& filename) : 
      HistogramReader(filename)
   {
      readHeader();
   }

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };

protected:

   void readHeader()
   {
      std::ifstream fs(filename, std::ifstream::binary);

      FfdHeader header(fs);

      data_position = header.data_position;
      tags = header.tags;

      if (header.type != FfdHeader::histogram)
         throw std::runtime_error("Must be histogramed data");

      TagMap::iterator it;

      it = tags.find("NumX");
      if (it != tags.end())
         n_x = (int) it->second.getValue<int64_t>();

      it = tags.find("NumY");
      if (it != tags.end())
         n_y = (int) it->second.getValue<int64_t>();

      it = tags.find("NumChannels");
      if (it != tags.end())
         n_chan = (int)it->second.getValue<int64_t>();

      it = tags.find("TimeBins");
      if (it != tags.end() && it->second.is_vector)
         timepoints_ = it->second.getVector<double>();

      it = tags.find("DataType");
      if (it != tags.end())
         data_type = it->second.getString();

      n_timepoints = (int) timepoints_.size();

      fs.close();

   }
   
private:

   std::string data_type;

   template <class T>
   void readData__(T* data, const std::vector<int>& channels_, int n_chan_stride)
   {
      if (data_type == "uint16_t")
         readData_<uint16_t, T>(data, channels_, n_chan_stride);
      else if (data_type == "double")
         readData_<double, T>(data, channels_, n_chan_stride);
      else if (data_type == "float")
         readData_<float, T>(data, channels_, n_chan_stride);
      else
         throw std::runtime_error("Error reading FFD file: unknown data type");
   }

};