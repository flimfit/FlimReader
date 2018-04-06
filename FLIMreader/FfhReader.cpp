#include "FfhReader.h"

FfhReader::FfhReader(const std::string& filename) : 
   HistogramReader(filename)
{
   readHeader();

   auto it = images.find("IntensityNormalisation");
   if (it != images.end())
   {
      intensity_normalisation = it->second;
      auto size = intensity_normalisation.size();
      std::vector<int> dims = { 1, size.height, size.width };
      intensity_normalisation = intensity_normalisation.reshape(0, 3, &dims[0]);
   }
}

void FfhReader::readHeader()
{
   std::ifstream fs(filename, std::ifstream::binary);

   FfdHeader header(fs);

   data_position = header.data_position;
   tags = header.tags;

   if (header.type != FfdHeader::histogram)
      throw std::runtime_error("Must be histogramed data");

   uint64_t next_block_pos = 0;

   TagMap::iterator it;

   it = tags.find("NumX");
   if (it != tags.end())
      n_x = (int) it->second.getValue<int64_t>();
   else
      throw(std::runtime_error("NumX was not specified in data"));

   it = tags.find("NumY");
   if (it != tags.end())
      n_y = (int) it->second.getValue<int64_t>();
   else
      throw(std::runtime_error("NumY was not specified in data"));
   
   it = tags.find("NumChannels");
   if (it != tags.end())
      n_chan = (int)it->second.getValue<int64_t>();
   else
      throw(std::runtime_error("NumChannels was not specified in data"));
   
   it = tags.find("TimeBins");
   if (it != tags.end() && it->second.is_vector)
      timepoints_ = it->second.getVector<double>();
   else
      throw(std::runtime_error("TimeBins was not specified in data"));

   it = tags.find("DataType");
   if (it != tags.end())
   {
      std::string data_type = it->second.getString();
      if (data_type == "double") native_type = DataTypeDouble;
      else if (data_type == "float") native_type = DataTypeFloat;
      else if (data_type == "uint16_t") native_type = DataTypeUint16;
      else throw std::runtime_error("Unsupported data type");
   }
   else
      throw(std::runtime_error("DataType was not specified in data"));

   it = tags.find("NextBlock");
   if (it != tags.end())
      next_block_pos = it->second.getValue<uint64_t>();

   n_timepoints = (int) timepoints_.size();

   while(next_block_pos > 0)
   {
      fs.seekg(next_block_pos);
      auto block_tags = header.readTags(fs);

      std::string block_type, block_description;
      it = block_tags.find("BlockType");
      if (it != block_tags.end())
         block_type = it->second.getString();
      it = block_tags.find("BlockDescription");
      if (it != block_tags.end())
         block_description = it->second.getString();

      if (block_type == "Image")
      {
         int width = 0, height = 0, type = 0, data_length = 0;
         it = block_tags.find("ImageFormat");
         if (it != block_tags.end())
            type = it->second.getValue<int32_t>();
         it = block_tags.find("ImageWidth");
         if (it != block_tags.end())
            width = it->second.getValue<int32_t>();
         it = block_tags.find("ImageHeight");
         if (it != block_tags.end())
            height = it->second.getValue<int32_t>();
         it = block_tags.find("ImageDataLength");
         if (it != block_tags.end())
            data_length = it->second.getValue<int32_t>();

         cv::Mat im(height, width, type);
         fs.read(reinterpret_cast<char*>(im.data), data_length);

         images[block_description] = im;
      }

      next_block_pos = 0;
      it = block_tags.find("NextBlock");
      if (it != block_tags.end())
         next_block_pos = it->second.getValue<uint64_t>();
   }

   fs.close();

}
