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
   readHeader();

   n_chan = 1; 

   n_timebins_native = 4096;
   time_resolution_native_ps = 4;
   setTemporalResolution(log2(n_timebins_native));

   markers.PixelMarker = 0x1;
   markers.LineStartMarker = 0x2;
   markers.LineEndMarker = 0x2;
   markers.FrameMarker = 0x4;

   event_reader = std::unique_ptr<AbstractEventReader>(new BhEventReader(filename, data_position));

   determineDwellTime();
}

void BhFifoReader::readHeader()
{
   std::string set_filename = filename;
   set_filename.replace(set_filename.end() - 3, set_filename.end(), "set");

   ifstream fs(set_filename, ios::binary);

   if (!fs.is_open())
      throw std::runtime_error("Could not open set file");

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

   n_x = sys_par_ext.fifo_ncx;
   n_y = sys_par_ext.fifo_ncx; // maybe?
   sync.bi_directional = false; //(sys_par_ext.scan_type == 1);
   
   data_position = 0;
}