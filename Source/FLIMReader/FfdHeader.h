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

      char tag_name[255];
      uint32_t tag_name_length, tag_data_length;
      uint16_t tag_type;

      auto isTag = [&](const char* t) { return strncmp(t, tag_name, tag_name_length - 1) == 0; };

      do
      {
         READ(fs, tag_name_length);

         tag_name_length = std::min(tag_name_length, (uint32_t)255);
         fs.read(tag_name, tag_name_length);

         READ(fs, tag_type);
         READ(fs, tag_data_length);

         std::vector<char> data(tag_data_length);
         char* data_ptr = data.data();
         fs.read(data_ptr, tag_data_length);

         if (tag_type == TagDouble)
         {
            tags[tag_name] = MetaDataTag(*(double*)data_ptr);
         }
         else if (tag_type == TagInt64)
         {
            tags[tag_name] = MetaDataTag(*(int64_t*)data_ptr);
         }
         else if (tag_type == TagUInt64)
         {
            tags[tag_name] = MetaDataTag(*(uint64_t*)data_ptr);
         }
         else if (tag_type == TagBool)
         {
            tags[tag_name] = MetaDataTag(*(bool*)data_ptr);
         }
         else if (tag_type == TagDate || tag_type == TagString)
         {
            std::string value;
            value.resize(tag_data_length);
            memcpy(&value[0], data_ptr, tag_data_length);

            tags[tag_name] = MetaDataTag(value);
         }
      } while (tag_type != TagEndHeader);

      data_position = fs.tellg();
   }

   FfdType type;
   std::streamoff data_position;
   uint32_t version;
   std::map<std::string, MetaDataTag> tags;
   
};