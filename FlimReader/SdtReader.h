#pragma once
#include "HistogramReader.h"
#include "SPC_data_structure.h"

class SdtReader : public HistogramReader
{
public:

   SdtReader(const std::string& filename);

   void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };
   void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) { readData__(data, channels, n_chan_stride); };

   bool canReadBidirectionalScan() const { return true; }
   void setBidirectionalScan(bool bidirectional_scan = true) {}; // do nothing
   bool getBidirectionalScan() const { return false; }

protected:
   void readHeader();

   template <class T>
   void readData__(T* data, const std::vector<int>& channels_, int n_chan_stride)
   {
      if (native_type == DataTypeUint16)
         readData_<uint16_t, T>(data, channels_, n_chan_stride);
      else
         throw std::runtime_error("Error reading SDT file: unknown data type");
   }

   bhfile_header header;
   BHBinHdr bin_header;
   SPCBinHdr spc_bin_header;
   SPCBinHdrExt spc_ext_header;
   MeasureInfo measure_info;
};