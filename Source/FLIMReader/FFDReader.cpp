#include "FfdReader.h"
#include "FfdHeader.h"

#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <functional>

FfdReader::FfdReader(const std::string& filename) :
   AbstractFifoReader(filename)
{
   readHeader();

   setTemporalResolution(12);

   markers.FrameMarker = 0x8;
   markers.LineEndMarker = 0x4;
   markers.LineStartMarker = 0x2;

   event_reader = std::unique_ptr<AbstractEventReader>(new FfdEventReader(filename, version, data_position));

   determineDwellTime();
}

void FfdReader::readHeader()
{
   std::ifstream fs(filename, std::ifstream::in | std::ifstream::binary);
   FfdHeader header(fs);

   version = header.version;
   data_position = header.data_position;
   tags = header.tags;

   if (header.type != FfdHeader::fifo)
      throw std::runtime_error("Must be fifo");

   TagMap::iterator it;
   
   it = tags.find("MicrotimeResolutionUnit_ps");
   if (it != tags.end())
      time_resolution_native_ps = it->second.getValue<double>();

   it = tags.find("SyncRate_Hz");
   if (it != tags.end())
      t_rep_ps = 1e12 / it->second.getValue<double>();

   it = tags.find("NumChannels");
   if (it != tags.end())
      n_chan = (int) it->second.getValue<int64_t>();

   it = tags.find("NumTimeBins");
   if (it != tags.end())
      n_timebins_native = (int) it->second.getValue<int64_t>();

   it = tags.find("BidirectionalScan");
   if (it != tags.end())
      sync.bi_directional = it->second.getValue<bool>();

   // Fix error in PLIM files
   it = tags.find("UsingPixelMarkers");
   if (it != tags.end())
   {
      bool using_pixel_markers = it->second.getValue<bool>();
      if (using_pixel_markers) // kludge for detecting PLIM
         t_rep_ps = 1e15;
   }

   it = tags.find("UseCompression");
   if (it != tags.end())
   {
      bool use_compression = it->second.getValue<bool>();
      if (use_compression)
         throw std::runtime_error("Compressed FFD files are no longer supported");
   }
}