#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "AligningReader.h"
#include "MetadataTag.h"
#include "FlimCube.h"

typedef std::map<std::string, MetaDataTag> TagMap;
typedef std::map<std::string, cv::Mat> ImageMap;

class FlimReader : public AligningReader
{
public:

   static FlimReader* createReader(const std::string& filename);

   FlimReader(const std::string& filename_);
   FlimReader();

   static std::string determineExtension(const std::string& filename);
   std::vector<int> validateChannels(const std::vector<int> channels, int& n_chan_stride);

   virtual ~FlimReader() {};

   virtual int getNumChannels() const { return n_chan; };
   virtual void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;

   void readData(std::shared_ptr<FlimCube> cube, const std::vector<int>& channels = {});

   virtual void stopReading() {};
   virtual void clearStopSignal() {};
   virtual double getProgress() { return 0; }

   virtual FlimNativeType getPreferredType() { return (spectral_correction.empty()) ? native_type : DataTypeFloat; }

   virtual void setNumZ(int n_z_) { std::cout << "Setting n_z not supported\n"; }

   void setSpectralCorrection(const std::vector<cv::Mat>& spectral_correction);

   std::vector<bool> getRecommendedChannels();

   const std::vector<double>& getTimepoints() { return timepoints; };
   const std::vector<double>& getNativeTimepoints() { return native_timepoints; };
   int getNumX() const { return n_x / spatial_binning; }
   int getNumY() const { return n_y / spatial_binning; }
   int getNumZ() const { return n_z; }
   int dataSizePerChannel();

   FlimNativeType getNativeType() { return native_type; }

   virtual bool supportsRealignment() { return false; }
   int getSpatialBinning() { return spatial_binning; }
   double getRepRateHz() { return rep_rate_hz; }

   const std::string& getFilename() { return filename; }
   TagMap getTags() { return tags; }
   virtual TagMap getReaderTags();

   ImageMap getImageMap();

   virtual void setTemporalDownsampling(int downsampling_in_bits); 
   virtual void setBidirectionalPhase(double phase) { std::cout << "Warning: setting phase not supported by this reader\n"; }
   void setSpatialBinning(int spatial_binning_);

protected:

   virtual void initaliseTimepoints() { setTemporalDownsampling(0); }

   std::vector<double> timepoints;
   std::vector<double> native_timepoints;
   int downsampling;

   std::vector<cv::Mat> spectral_correction;
   std::vector<bool> recommended_channels;

   std::string filename;
   std::string extension;
   int spatial_binning = 1;
   int n_x = 0;
   int n_y = 0;
   int n_z = 1;
   int n_chan = 0;
   double rep_rate_hz = std::numeric_limits<double>::quiet_NaN(); // when we don't know 

   FlimNativeType native_type = DataTypeUint16;

   TagMap tags;
};

