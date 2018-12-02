#include "HistogramReader.h"
#include <fstream>

class PicoquantBINReader : public HistogramReader
{
public:
   PicoquantBINReader(const std::string& filename, const std::string& ext) :
      HistogramReader(filename)
   {
      if (ext == "bin2")
         has_multiple_channels = true;

      readHeader();
   }

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_<int, float>(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_<int, double>(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData_<int, uint16_t>(data, channels, n_chan_stride); };

   bool canReadBidirectionalScan() const { return true; }
   void setBidirectionalScan(bool bidirectional_scan = true) {}; // do nothing
   bool getBidirectionalScan() const { return false; }

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

      blocks.push_back({ header_size, expected_size });

      native_timepoints.resize(n_timepoints);

      for (int i = 0; i < n_timepoints; i++)
         native_timepoints[i] = i * TimeResol * 1000;

      initaliseTimepoints();
   }

   const int header_size = 20;
   int n_timepoints = 1;
   bool has_multiple_channels = false;
};