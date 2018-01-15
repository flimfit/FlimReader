#include "BhFifoReader.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include "SPC_data_structure.h"

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

BhFifoReader::BhFifoReader(const std::string& filename) :
   AbstractFifoReader(filename)
{
   n_chan = 1;

   readHeader();

   n_timebins_native = 4096;
   time_resolution_native_ps = 50e3 / n_timebins_native / 4; // TODO; try and get TAC scaling from ini file
   setTemporalResolution((int) log2(n_timebins_native));

   markers.PixelMarker = 0x1;
   markers.LineStartMarker = 0x2;
   markers.LineEndMarker = 0x2;
   markers.FrameMarker = 0x4;

   event_reader = std::shared_ptr<AbstractEventReader>(new BhEventReader(filename, data_position));

   determineDwellTime();
}

#pragma pack(push)
#pragma pack(1)
struct HandheldScannerHeader
{
   uint32_t magic;
   uint32_t header_size;
   uint32_t format_version;
   uint32_t n_x;
   uint32_t n_y;
   uint32_t spc_header;
};
#pragma pack(pop)

void BhFifoReader::readHeader()
{
   std::string set_filename = filename;
   set_filename.replace(set_filename.end() - 3, set_filename.end(), "set");

   ifstream fs(set_filename, ios::binary);

   if (!fs.is_open())
   {
      // try reading header from file as saved by handheld scanner
      ifstream fs(filename, ios::binary);

      HandheldScannerHeader hdr;
      READ(fs, hdr);

      if (hdr.magic == 0xF1F0 && hdr.header_size == 4 && hdr.format_version == 1)
      {
         n_x = hdr.n_x - 1;
         n_y = hdr.n_y;
         data_position = fs.tellg();
         return;
      }
      else
      {
         throw std::runtime_error("Could not open set file");
      }
   }

   // Read file header
   bhfile_header hdr;
   READ(fs, hdr);

   // Read text info
   std::string file_info(hdr.info_length,' ');
   fs.read(&file_info[0], hdr.info_length);

   fs.seekg(hdr.setup_offs);
   std::string setup_info(hdr.setup_length, ' ');
   fs.ignore(hdr.setup_length, '\0'); // Ignore ASCII section
   uint32_t bin_len;
   BHBinHdr bh_bin_hdr;
   SPCBinHdr spc_bin_hdr;
   SPCBinHdrExt spc_bin_hdr_ext;
   SysParExt sys_par_ext;

   READ(fs, bin_len);
   uint64_t start = fs.tellg();

   READ(fs, bh_bin_hdr);
   READ(fs, spc_bin_hdr);

   fs.seekg(start + spc_bin_hdr.binhdrext_offs);
   READ(fs, spc_bin_hdr_ext);

   fs.seekg(start + spc_bin_hdr_ext.syspar_ext_offs);
   READ(fs, sys_par_ext);

   n_x = sys_par_ext.norm_ncx - 1; // to exclude final line
   n_y = sys_par_ext.norm_ncx; // maybe?
   n_chan = sys_par_ext.img_x;
   sync.bidirectional = false; //(sys_par_ext.scan_type == 1);
   
   data_position = 0;
}



std::tuple<FifoEvent, uint64_t> BhEventReader::getRawEvent()
{
   uint32_t evt = getPacket<uint32_t>();
   return getBhEvent(evt);
}

std::tuple<FifoEvent, uint64_t> BhEventReader::getBhEvent(uint32_t evt0)
{
   FifoEvent e;
   uint64_t macro_time_offset = 0;

   uint32_t evt = evt0;
   e.macro_time = evt & 0xFFF; evt >>= 12;
   e.channel = evt & 0xF; evt >>= 4;
   e.micro_time = evt & 0xFFF; evt >>= 12;
   e.micro_time = 4095 - e.micro_time; // Reverse start-stop

   bool is_mark = (evt & 0x1) != 0;
   bool gap = (evt & 0x2) != 0;
   bool mtov = (evt & 0x4) != 0;
   bool invalid = (evt & 0x8) != 0;

   e.valid = true;
   e.gap = gap;


   if (is_mark)
   {
      e.mark = e.channel;
   }
   if (mtov)
   {
      if (invalid && !is_mark)
      {
         e.valid = false;
         macro_time_offset = 0xFFF * (evt0 & 0xFFFFFFF);
      }
      else
      {
         macro_time_offset = 0xFFF;
      }
   }

   return std::tuple<FifoEvent, uint64_t>(e, macro_time_offset);
}