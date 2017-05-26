#include "FlimReader.h"
#include "PicoquantTTRReader.h"
#include "PicoquantPTUReader.h"
#include "PicoquantBINReader.h"
#include "FfdReader.h"
#include "FfhReader.h"
#include "TextReader.h"
#include "BhFifoReader.h"

FlimReader* FlimReader::createReader(const std::string& filename)
{
   std::string extension = determineExtension(filename);

   if (extension == "txt" || extension == "csv")
      return new TextReader(filename);
   else if (extension == "pt3")
      return new PicoquantTTTRReader(filename);
   else if (extension == "ptu")
      return new PicoquantPTUReader(filename);
   else if (extension == "bin" || extension == "bin2")
      return new PicoquantBINReader(filename, extension);
   else if (extension == "ffd")
      return new FfdReader(filename);
   else if (extension == "ffh")
      return new FfhReader(filename);
   else if (extension == "spc")
      return new BhFifoReader(filename);

   throw std::runtime_error("Unrecognised file format");
}

FlimReader::FlimReader(const std::string& filename_)
{
   filename = filename_;
   extension = determineExtension(filename);
}

std::string FlimReader::determineExtension(const std::string& filename)
{
   size_t last_dot, pos = 0;
   while ((pos = filename.find('.', pos + 1)) != std::string::npos)
      last_dot = pos + 1;
   return filename.substr(last_dot);
}

std::vector<int> FlimReader::validateChannels(std::vector<int> channels, int& n_chan_stride)
{
   std::vector<int> validated_channels;
   validated_channels.reserve(n_chan);

   if (channels.empty())
   {
      for (int i = 0; i < n_chan; i++)
         validated_channels.push_back(i);
   }
   else
   {
      for (auto& c : channels)
         if (c < n_chan)
            validated_channels.push_back(c);
   }

   if (n_chan_stride < validated_channels.size() || n_chan_stride == -1)
      n_chan_stride = (int) validated_channels.size();
   
   return validated_channels;
}

int FlimReader::dataSizePerChannel()
{
   int n_bin = (int) timepoints_.size();
   return n_bin * n_x * n_y / (spatial_binning * spatial_binning);
}

void FlimReader::setSpatialBinning(int spatial_binning_)
{
   if (n_x > spatial_binning_ && n_y > spatial_binning_)
      spatial_binning = spatial_binning_;
}

TagMap FlimReader::getReaderTags()
{
   TagMap reader_tags;
   reader_tags["SpatialBinning"] = spatial_binning;
   reader_tags["RealignmentType"] = realignmentTypeString(realign_params.type);
   reader_tags["RealignmentFrameBinning"] = realign_params.frame_binning;
   reader_tags["RealignmentSpatialBinning"] = realign_params.spatial_binning;
   reader_tags["RealignmentNumResamplingPoints"] = realign_params.n_resampling_points;
   reader_tags["RealignmentSmoothing"] = realign_params.smoothing;
   reader_tags["RealignmentCorrelationThreshold"] = realign_params.correlation_threshold;
   reader_tags["RealignmentCoverageThreshold"] = realign_params.correlation_threshold;

   return reader_tags;
}

ImageMap FlimReader::getImageMap()
{
   ImageMap images;
   if (!intensity_normalisation.empty())
      images["IntensityNormalisation"] = intensity_normalisation;
   return images;
}
