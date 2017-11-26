#pragma once

#include <stdint.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <map>
#include "MetadataTag.h"

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

class FfdHeader
{
public:

   enum FfdType { fifo, histogram };
   
   enum FfdMetadataTag
   {
      TagDouble = 0,
      TagUInt64 = 1,
      TagInt64 = 2,
      TagBool = 4,
      TagString = 5,
      TagDate = 6,
      TagEndHeader = 7
   };

   FfdHeader() {};

   FfdHeader(std::ifstream& fs)
   {
      uint32_t magic;

      READ(fs, magic);

      if (!fs.is_open())
         throw std::runtime_error("Could not open file handle");

      if (magic == 0xF1F0)
         type = fifo;
      else if (magic == 0xC0BE)
         type = histogram;
      else
         throw std::runtime_error("Wrong magic string, this is not a valid FFD file");

      READ(fs, version);

      uint32_t dp;
      READ(fs, dp);
      data_position = dp;

      tags = readTags(fs);
   }

   TagMap readTags(std::ifstream& fs)
   {
      std::map<std::string, MetaDataTag> tags;

      char tag_name[255];
      uint32_t tag_name_length, tag_data_length;
      uint16_t tag_type;

      auto isTag = [&](const char* t) { return strncmp(t, tag_name, tag_name_length - 1) == 0; };

      do
      {
         READ(fs, tag_name_length);       
        if (tag_name_length > 255)
            throw(std::runtime_error("Unexpectedly large metadata tag name found, file is potentially corrupt."));

         std::fill_n(tag_name, 255, 0);
         fs.read(tag_name, tag_name_length);

         READ(fs, tag_type);
         READ(fs, tag_data_length);

         MetaDataTag tag;

         tag.is_vector = (tag_type & 0x80) != 0;
         tag.type = (MetaDataTag::TagType) (tag_type & 0x7F);

         if (tag.is_vector)
         {
            tag.vector_data.resize(tag_data_length / sizeof(uint64_t));
            fs.read((char*) tag.vector_data.data(), tag_data_length);
         }
         else if (tag.type == MetaDataTag::TagDate || tag.type == MetaDataTag::TagString)
         {
            tag.string_data.resize(tag_data_length);
            fs.read(&tag.string_data[0], tag_data_length);
         }
         else if (tag.type <= 0x4) // double, uint64_t, int64_t, bool
         {
            tag_data_length = std::min((uint32_t)8, tag_data_length);
            fs.read((char*)&tag.data, tag_data_length);

         }

         if (tag_type != TagEndHeader)
            tags[tag_name] = tag;

      } while (tag_type != TagEndHeader);

      return tags;
   }

   FfdType type;
   std::streamoff data_position;
   uint32_t version;
   std::map<std::string, MetaDataTag> tags;
   
};