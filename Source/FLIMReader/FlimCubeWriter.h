#pragma once

#include "MetadataTag.h"
#include "FlimCube.h"
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <typeinfo>

#define WRITE(fs, x) fs.write(reinterpret_cast<char *>(&x), sizeof(x))

template<typename T>
class FlimCubeWriter
{
public:

   FlimCubeWriter(const std::string& filename, std::shared_ptr<FlimCube<T>> cube, const TagMap& tags, const TagMap& reader_tags) :
      filename(filename)
   {

      // Write header
      uint32_t magic_number = 0xC0BE;
      uint32_t format_version = 2;
      uint32_t data_pos = 0;

      of = std::ofstream(filename, std::ifstream::binary);

      if (!of.is_open())
         throw std::runtime_error("Could not open file to write");

      WRITE(of, magic_number);
      WRITE(of, format_version);

      std::streampos data_pos_write = of.tellp();
      WRITE(of, data_pos);

      // get current time
      auto t = std::time(nullptr);
      std::ostringstream oss;
      oss << std::put_time(std::localtime(&t), "%FT%T");

      writeTag("NumTimeBins", cube->n_t);
      writeTag("NumX", cube->n_x);
      writeTag("NumY", cube->n_y);
      writeTag("NumChannels", cube->n_chan);
      writeTag("TimeBins", cube->timepoints);
      writeTag("DataType", getTypeName());
      writeTag("CreationDate", oss.str());

      for (auto t : reader_tags)
         writeTag(t.first, t.second);

      for (auto t : tags)
         writeTag("OriginalTags_" + t.first, t.second);
      
      writeTag("EndHeader", MetaDataTag());

      // Write correct data position
      data_pos = of.tellp();
      of.seekp(data_pos_write);
      WRITE(of, data_pos);
      of.seekp(data_pos);

      // Write data
      uint64_t data_size = cube->getDataSize();
      of.write((char*) cube->getDataPtr(), data_size);

      of.close();
   }

private:

   std::string getTypeName()
   {
      if (typeid(T) == typeid(uint16_t))
         return "uint16_t";
      if (typeid(T) == typeid(double))
         return "double";
      if (typeid(T) == typeid(float))
         return "float";
   }


   void writeTag(const std::string& name, const MetaDataTag& value)
   {
      uint32_t name_length = (uint32_t) name.length() + 1;
      uint16_t type = value.type | (value.is_vector * 0x80);

      name_length = std::min((uint32_t)255, name_length);

      WRITE(of, name_length);
      of.write(name.c_str(), name_length);
      WRITE(of, type);

      uint32_t length = 0;
      const char* data;

      if (value.is_vector)
      {
         length = (uint32_t) value.vector_data.size() * sizeof(uint64_t);
         data = (char*) value.vector_data.data();
      }
      else
      {
         switch (value.type)
         {
         case MetaDataTag::TagDouble:
            length = sizeof(double);
            data = (char*)&value.data;
            break;
         case MetaDataTag::TagUInt64:
            length = sizeof(uint64_t);
            data = (char*)&value.data;
            break;
         case MetaDataTag::TagInt64:
            length = sizeof(int64_t);
            data = (char*)&value.data;
            break;
         case MetaDataTag::TagBool:
            length = sizeof(bool);
            data = (char*)&value.data;
            break;
         case MetaDataTag::TagString:
         case MetaDataTag::TagDate:
            length = (uint32_t)value.string_data.size();
            data = value.string_data.c_str();
            break;
         }
      }

      WRITE(of, length);
      if (length > 0)
         of.write(data, length);
   }

   std::ofstream of;
   std::string filename;
};