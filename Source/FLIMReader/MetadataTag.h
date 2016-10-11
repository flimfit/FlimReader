#pragma once

#include <stdint.h>
#include <string>
#include <vector>

class MetaDataTag
{
public:
   enum TagType
   {
      TagDouble = 0x0,
      TagUInt64 = 0x1,
      TagInt64 = 0x2,
      TagBool = 0x4,
      TagString = 0x5,
      TagDate = 0x6,
      TagEndHeader = 0x7
   };

   MetaDataTag() { setup(0, TagEndHeader); } // default is end tag
   MetaDataTag(double v) { setup(v, TagDouble); }
   MetaDataTag(uint64_t v) { setup(v, TagUInt64); }
   MetaDataTag(int64_t v) { setup(v, TagInt64); }
   MetaDataTag(bool v) { setup(v, TagBool); }
   MetaDataTag(uint32_t v) { setup((uint32_t) v, TagUInt64); }
   MetaDataTag(int32_t v) { setup((int32_t) v, TagInt64); }
   MetaDataTag(const std::string& v) { setup(v); }
   MetaDataTag(const std::vector<double>& v) { setupVector(v, TagDouble); }
   MetaDataTag(const std::vector<uint64_t>& v) { setupVector(v, TagUInt64); }
   MetaDataTag(const std::vector<int64_t>& v) { setupVector(v, TagInt64); }

   void operator=(const double &v) { setup(v, TagDouble); }
   void operator=(const uint64_t &v) { setup(v, TagUInt64); }
   void operator=(const int64_t &v) { setup(v, TagInt64); }
   void operator=(const bool &v) { setup(v, TagBool); }
   void operator=(const uint32_t &v) { setup((uint64_t) v, TagUInt64); }
   void operator=(const int32_t &v) { setup((uint64_t) v, TagInt64); }
   void operator=(const std::string& v) { setup(v); }
   void operator=(const std::vector<double>& v) { setupVector(v, TagDouble); }
   void operator=(const std::vector<uint64_t>& v) { setupVector(v, TagUInt64); }
   void operator=(const std::vector<int64_t>& v) { setupVector(v, TagInt64); }

   template<typename T>
   void setup(T v, TagType type_)
   {
      type = type_;
      *reinterpret_cast<T*>(&data) = v;
   }

   template<typename T>
   void setupVector(const std::vector<T>& v, TagType type_)
   {
      static_assert(sizeof(T) == sizeof(uint64_t), "Vector data size must equal sizeof(uint64_t)");
      type = type_;
      is_vector = true;
      vector_data.resize(v.size());
      std::copy_n((uint64_t*)v.data(), v.size(), vector_data.data());
   }

   void setup(const std::string& v)
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

   template<typename T>
   std::vector<T> getVector()
   {
      if (is_vector)
      {
         switch (type)
         {
         case TagDouble:
            return getVector_<T,double>();
         case TagInt64:
            return getVector_<T, int64_t>();
         case TagUInt64:
            return getVector_<T, uint64_t>();
         }
      }

      return std::vector<T>();
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



   TagType type = TagDouble;
   uint64_t data = 0;
   std::string string_data;
   std::vector<uint64_t> vector_data;
   bool is_vector = false;

private:

   template<typename T, typename U>
   std::vector<T> getVector_()
   {
      U* vd = (U*)vector_data.data();
      std::vector<T> v(vector_data.size());
      for (int i = 0; i < vector_data.size(); i++)
         v[i] = (T) vd[i];
      return v;
   }
};
