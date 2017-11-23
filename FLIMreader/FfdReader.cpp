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
   markers.PixelMarker = 0x0; // no pixel marker

   event_reader = std::shared_ptr<AbstractEventReader>(new FfdEventReader(filename, version, data_position));

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

   it = tags.find("NumX");
   if (it != tags.end())
	   n_x = it->second.getValue<int>();

   it = tags.find("NumY");
   if (it != tags.end())
	   n_y = it->second.getValue<int>();


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


std::tuple<FifoEvent, uint64_t> FfdEventReader::getRawEvent()
{
   if (version == 1)
   {
      ffd_evt_v1 evt = getPacket<ffd_evt_v1>();
      return getFfdEvent(evt);
   }
   else
   {
      ffd_evt evt = getPacket<ffd_evt>();
      return getFfdEvent(evt);
   }

   assert(fs.good());

}

std::tuple<FifoEvent, uint64_t> FfdEventReader::getFfdEvent(ffd_evt_v1 evt)
{
   FifoEvent e;
   uint64_t macro_time_offset = 0;

   e.micro_time = evt.micro_time;
   e.macro_time = evt.macro_time;
   e.channel = evt.channel;
   e.mark = evt.mark;

   return std::tuple<FifoEvent, uint64_t>(e, macro_time_offset);
}

std::tuple<FifoEvent, uint64_t> FfdEventReader::getFfdEvent(ffd_evt evt)
{
   FifoEvent e;
   uint64_t macro_time_offset = 0;

   e.macro_time = evt.macro_time;
   e.channel = evt.micro_time & 0xF;

   if (e.channel == 0xF)
   {
      if (evt.micro_time == 0xF)
      {
         macro_time_offset = ((e.macro_time == 0) ? 1 : e.macro_time) * 0xFFFF;
         e.valid = false;
      }
      else
      {
         e.mark = evt.micro_time >> 4;
      }
   }
   else
   {
      e.mark = 0;
      e.micro_time = evt.micro_time >> 4;
   }

   return std::tuple<FifoEvent, uint64_t>(e, macro_time_offset);
}