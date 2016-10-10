#pragma once

#include "MetadataTag.h"
#include "FlimCube.h"
#include <map>
#include <vector>
#include <string>
#include <fstream>

#define WRITE(fs, x) fs.write(reinterpret_cast<char *>(&x), sizeof(x))

template<typename T>
class FlimCubeWriter
{
public:

   FlimCubeWriter(const std::string& filename, std::shared_ptr<FlimCube<T>> cube, const TagMap& tags) :
      filename(filename)
   {

      // Write header
      uint32_t magic_number = 0xC0BE;
      uint32_t format_version = 2;

      of = std::ofstream(filename);

      if (!of.is_open())
         throw std::runtime_error("Could not open file to write");

      WRITE(of, magic_number);
      WRITE(of, format_version);

      writeTag("n_t", MetaDataTag((uint64_t)cube->n_t));
      writeTag("n_x", MetaDataTag((uint64_t)cube->n_x));
      writeTag("n_y", MetaDataTag((uint64_t)cube->n_y));
      writeTag("n_chan", MetaDataTag((uint64_t)cube->n_chan));

      for (auto t : tags)
      {
         std::string new_name = "original_" + t.first;
         writeTag(new_name, t.second);
      }

      uint64_t data_size = cube->getDataSize();
      of.write((char*) cube->getDataPtr(), data_size);

      of.close();


      /*
      header.clear();
      QBuffer buffer(&header);
      buffer.open(QIODevice::WriteOnly);
      header_stream.setDevice(&buffer);
      header_stream.setByteOrder(QDataStream::LittleEndian);

      writeTag("CreationDate", QDateTime::currentDateTime());
      writeTag("TcspcSystem", tcspc->describe());
      writeTag("SyncRate_Hz", tcspc->getSyncRateHz());
      writeTag("NumTimeBins", (int64_t)tcspc->getNumTimebins());
      writeTag("NumChannels", (int64_t)tcspc->getNumChannels());
      writeTag("MicrotimeResolutionUnit_ps", tcspc->getMicroBaseResolutionPs());
      writeTag("MacrotimeResolutionUnit_ps", tcspc->getMacroBaseResolutionPs());
      writeTag("L4ZCompression", use_compression);
      writeTag("L4ZMessageSize", lz4_stream.getMessageSize());
      writeTag("UsingPixelMarkers", tcspc->usingPixelMarkers());

      for (auto&& m : metadata)
         writeTag(m.first, m.second);

      writeEndTag();

      buffer.close();

      quint32 header_size = header.size();
      data_stream << magic_number << format_version << (header_size + 12); // + 12 for first three numbers 
      data_stream.writeRawData(header.data(), header.size());
      */
   }

private:

   void writeTag(const std::string& name, const MetaDataTag& value)
   {
      uint32_t name_length = (uint32_t) name.length();
      uint16_t type = value.type;

      name_length = std::min((uint32_t)255, name_length);

      WRITE(of, name_length);
      of.write(name.c_str(), name_length);
      WRITE(of, type);

      if (value.type == MetaDataTag::TagType::TagString)
      {
         uint32_t length = (uint32_t) value.string_data.length();
         WRITE(of, length);
         of.write(value.string_data.c_str(), length);
      }

   }

   std::ofstream of;
   std::string filename;
};