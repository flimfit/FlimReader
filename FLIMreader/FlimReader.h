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

   static std::string determineExtension(const std::string& filename);
   std::vector<int> validateChannels(const std::vector<int> channels, int& n_chan_stride);

   virtual ~FlimReader() {};

   virtual int getNumChannels() { return n_chan; };
   virtual void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;

   template<typename T>
   void readData(std::shared_ptr<FlimCube<T>> cube, const std::vector<int>& channels = {});

   virtual void stopReading() {};
   virtual void clearStopSignal() {};
   virtual double getProgress() { return 0; }

   void setNumZ(int n_z_) { n_z = n_z_; }

   const std::vector<double>& getTimepoints() { return timepoints_; };
   int numX() { return n_x / spatial_binning; }
   int numY() { return n_y / spatial_binning; }
   int numZ() { return n_z; }
   int dataSizePerChannel();

   virtual bool isBidirectional() { return false; }
   virtual bool supportsRealignment() { return false; }
   int getTemporalResolution() { return temporal_resolution; }
   int getSpatialBinning() { return spatial_binning; }
   double getRepRateHz() { return rep_rate_hz; }

   const std::string& getFilename() { return filename; }
   TagMap getTags() { return tags; }
   virtual TagMap getReaderTags();

   ImageMap getImageMap();

   virtual void setTemporalResolution(int temporal_resolution) {}; // do nothing in general case;
   virtual void setBidirectionalPhase(double phase) {}
   void setSpatialBinning(int spatial_binning_);

protected:

   std::vector<double> timepoints_;

   std::string filename;
   std::string extension;
   int temporal_resolution = 1;
   int spatial_binning = 1;
   int n_x = 0;
   int n_y = 0;
   int n_z = 1;
   int n_chan = 0;
   double rep_rate_hz = std::numeric_limits<double>::quiet_NaN(); // when we don't know 

   TagMap tags;
};

template<typename T>
void FlimReader::readData(std::shared_ptr<FlimCube<T>> cube, const std::vector<int>& channels)
{
   int n_chan_stride = -1;
   std::vector<int> ch = validateChannels(channels, n_chan_stride);

   cube->init(timepoints_, (int)ch.size(), numX(), numY());
   readData(cube->getDataPtr(), ch);
}
