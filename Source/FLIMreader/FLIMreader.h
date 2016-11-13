#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "AbstractFrameAligner.h"
#include "MetadataTag.h"
#include "FlimCube.h"

typedef std::map<std::string, MetaDataTag> TagMap;

class FLIMReader
{
public:

   static FLIMReader* createReader(const std::string& filename);

   FLIMReader(const std::string& filename_);

   static std::string determineExtension(const std::string& filename);
   std::vector<int> validateChannels(const std::vector<int> channels, int& n_chan_stride);

   virtual ~FLIMReader() {};

   virtual void alignFrames() {};

   virtual int getNumChannels() { return n_chan; };
   virtual void readData(float* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(double* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;
   virtual void readData(uint16_t* data, const std::vector<int>& channels = {}, int n_chan_stride = -1) = 0;

   template<typename T>
   void readData(std::shared_ptr<FlimCube<T>> cube, const std::vector<int>& channels = {})
   {
      int n_chan_stride = -1;
      std::vector<int> ch = validateChannels(channels, n_chan_stride);

      cube->init(timepoints_, (int)ch.size(), numX(), numY());
      readData(cube->getDataPtr(), ch);
   }

   virtual void stopReading() {};
   virtual float getProgress() { return 0; }

   const std::vector<double>& timepoints() { return timepoints_; };
   int numX() { return n_x / spatial_binning; }
   int numY() { return n_y / spatial_binning; }

   virtual void setTemporalResolution(int temporal_resolution) {}; // do nothing in general case;
   int getTemporalResolution() { return temporal_resolution; }

   int dataSizePerChannel();

   virtual bool supportsRealignment() { return false; }
   void setRealignmentParameters(RealignmentParameters realign_params_) { realign_params = realign_params_; }

   void setSpatialBinning(int spatial_binning_)
   {
      if (n_x > spatial_binning_ && n_y > spatial_binning_)
         spatial_binning = spatial_binning_;
   }
   
   int getSpatialBinning() { return spatial_binning; }
   
   const std::string& getFilename() { return filename; }

   TagMap getTags() { return tags; }
   
   virtual TagMap getReaderTags()
   {
      TagMap reader_tags;
      reader_tags["SpatialBinning"] = spatial_binning;
      reader_tags["RealignmentType"] = realignmentTypeString(realign_params.type);
      reader_tags["RealignmentFrameBinning"] = realign_params.frame_binning;
      reader_tags["RealignmentSpatialBinning"] = realign_params.spatial_binning;
      reader_tags["RealignmentNumResamplingPoints"] = realign_params.n_resampling_points;

      return reader_tags;
   }

   const std::vector<RealignmentResult>& getRealignmentResults() { return realignment; }

protected:

   std::vector<double> timepoints_;

   std::string filename;
   std::string extension;
   int temporal_resolution = 1;
   int spatial_binning = 1;
   int n_x = 0;
   int n_y = 0;
   int n_chan = 0;


   TagMap tags;

   RealignmentParameters realign_params;
   std::unique_ptr<AbstractFrameAligner> frame_aligner;
   std::vector<cv::Mat> frames;
   std::vector<RealignmentResult> realignment;
};
