#pragma once

#include <stdint.h>
#include <string>

class MetaDataTag
{
public:
   enum TagType
   {
      TagDouble = 0,
      TagUInt64 = 1,
      TagInt64 = 2,
      TagBool = 4,
      TagString = 5
   };

   MetaDataTag()
   {
      type = TagUInt64;
      data = 0;
   }

   MetaDataTag(double v)
   {
      type = TagDouble;
      *reinterpret_cast<double*>(&data) = v;
   }

   MetaDataTag(uint64_t v)
   {
      type = TagUInt64;
      *reinterpret_cast<uint64_t*>(&data) = v;
   }

   MetaDataTag(int64_t v)
   {
      type = TagInt64;
      *reinterpret_cast<int64_t*>(&data) = v;
   }

   MetaDataTag(bool v)
   {
      type = TagBool;
      *reinterpret_cast<bool*>(&data) = v;
   }

   MetaDataTag(const std::string& v)
   {
      type = TagString;
      string_data = v;
   }


   template<typename T>
   T getValue()
   {
      switch (type)
      {
      case TagDouble:
         return (T) *reinterpret_cast<double*>(&data);
      case TagInt64:
         return (T) *reinterpret_cast<int64_t*>(&data);
      case TagUInt64:
         return (T) *reinterpret_cast<uint64_t*>(&data);
      case TagBool:
         return (T) *reinterpret_cast<bool*>(&data);
      default:
         return (T) 0;
      }
   }

   std::string getString()
   {
      switch (type)
      {
      case TagDouble:
         return std::to_string(*reinterpret_cast<double*>(&data));
      case TagInt64:
         return std::to_string(*reinterpret_cast<int64_t*>(&data));
      case TagUInt64:
         return std::to_string(*reinterpret_cast<uint64_t*>(&data));
      case TagBool:
         return std::to_string(*reinterpret_cast<bool*>(&data));
      case TagString:
         return string_data;
      }
   }



   TagType type;
   uint64_t data;
   std::string string_data;

};
