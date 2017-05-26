#include "BhFifoReader.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>

#pragma pack(push,1)
struct bhfile_header {
   short    revision;  // software revision & module identification
                       //   lowest bits 0-3 -   software revision ( >= 12(decimal))
                       //        current value = 15 - support for huge data blocks >128MB <= 2GB
                       //   bits 11-4   - BH module type
                       //      meaning of this field values (hex):
                       //        0x20 -SPC-130, 0x21 - SPC-600, 0x22 - SPC-630,
                       //        0x23 -SPC-700, 0x24 - SPC-730, 0x25 - SPC-830,
                       //        0x26 -SPC-140, 0x27 - SPC-930, 0x28 - SPC-150,
                       //        0x29 -DPC-230, 0x2a - SPC-130EM
                       //   highest bits 15-12 - module subtype - not used yet
   long     info_offs; // offset of the info part which contains general text 
                       //   information (Title, date, time, contents etc.)
   short    info_length;  // length of the info part
   long     setup_offs;   // offset of the setup text data 
                          // (system parameters, display parameters, trace parameters etc.)
   short    setup_length;  // length of the setup data
   long     data_block_offs;   // offset of the first data block 
   short    no_of_data_blocks; // no_of_data_blocks valid only when in 0 .. 0x7ffe range,
                               // if equal to 0x7fff  the  field 'reserved1' contains 
                               //     valid no_of_data_blocks
   unsigned long     data_block_length;     // length of the longest block in the file  
                                            //        ( not compressed ) in bytes
   long     meas_desc_block_offs;  // offset to 1st. measurement description block 
                                   //   (system parameters connected to data blocks)
   short    no_of_meas_desc_blocks;  // number of measurement description blocks
   short    meas_desc_block_length;  // length of the measurement description blocks
   unsigned short    header_valid;   // valid: 0x5555, not valid: 0x1111
   unsigned long     reserved1;      // reserved1 now contains no_of_data_blocks
                                     // reserved2 now contains length (in int words) of data block extension, normally 0,
                                     //   data block extension contains info data for histograms data blocks
   unsigned short    reserved2;
   unsigned short    chksum;            // checksum of file header
};

typedef struct {
   unsigned long     soft_rev;       // software revision
   unsigned long     para_length;    // currently set to 0 
   unsigned long     reserved1;
   unsigned short    reserved2;
}BHBinHdr;


// all offsets in SPCBinHdr and SPCBinHdrExt are given from the beginning of 
//            BHBinHdr structure  ( just after 4-byte length of binary setup )
// all parts lengths are given in bytes

typedef struct {
   unsigned long     FCS_old_offs;    // offset of FCSParam structure for FCS parameters
   unsigned long     FCS_old_size;    //  FCSParam structure size
                                      //  FCSParam is obsolete - needed only for compatibility
                                      //  with software v. 8.1 - 8.2
                                      //  from v.8.3 FCS parameters are included further in HSTParam 
   unsigned long     gr1_offs;        // offset of GR_Param structure for main graph window
   unsigned long     gr1_size;        //  GR_Param structure size                       
   unsigned long     FCS_offs;        // offset of HSTParam structure for FCS parameters
   unsigned long     FCS_size;        //  HSTParam structure size
   unsigned long     FIDA_offs;       // offset of HSTParam structure for FIDA window parameters
   unsigned long     FIDA_size;       //  HSTParam structure size 
   unsigned long     FILDA_offs;      // offset of HSTParam structure for FILDA window parameters
   unsigned long     FILDA_size;      //  HSTParam structure size
   unsigned long     gr2_offs;        // offset of GR_Param structure for the 1st additional graph
   unsigned short    gr_no;           // number of additional GR_Param structures, ( size = gr1_size )
   unsigned short    hst_no;          // number of additional HSTParam structures, ( size = FCS_size )
   unsigned long     hst_offs;        // offset of the 1st additional HSTParam structure
   unsigned long     GVD_offs;        // offset of GVDParam structure for GVD parameters
   unsigned short    GVD_size;        //  GVDParam structure size
   unsigned short    FIT_offs;        // offset of FITSetup structure for FIT parameters
   unsigned short    FIT_size;        //  FITSetup structure size
   unsigned short    extdev_offs;     // offset of external devices setup structures
   unsigned short    extdev_size;     //  size of external devices setup structures
   unsigned long     binhdrext_offs;  // offset SPCBinHdrExt - extension of SPCBinHdr
   unsigned short    binhdrext_size;
}SPCBinHdr;

typedef struct {        // structure for extension of system parameters
   short  saved_in64b;      // parameters ( and file ) were saved in 64-bit SPCM version
   short  adc_bits_fifo;    // no of ADC bits for FIFO   modes
   short  adc_bits_scan;    // no of ADC bits for scan   modes
   short  adc_bits_fimg;    // no of ADC bits for FifoImage modes
   short  adc_bits_norm;    // no of ADC bits for all other modes
   short  img_ncx, img_ncy, img_x, img_y;  // Image size params for FifoImage mode
   short  fifo_ncx;
   short  norm_ncx, norm_ncy;
   short  wf_ncx, wf_x, wf_y;  // Image size params for Wide-Field Image mode
   int    adc_wf;
   short  flags;         // bit 0 - Repeat Time = Coll. Time, bit 1 - Display Time = Coll. Time,
                         //  not implemented yet
   short  skip_2nd_line_clk; // Fifo Image ( some microscopes delevers Line clock 
                             //             at the beg. and at the end of the line )
   short  scan_type;        // 0 - Unidir, 1 - Bidir
   unsigned int    right_border;     // for Bidir scanning
   char   reserve[208];     // keep fixed size of the structure = 256B
}SysParExt;

typedef struct {     // extension of SPCBinHdr
   unsigned long   MCS_img_offs;     // offset of MIMGParam structure for MCS Image parameters
   unsigned long   MCS_img_size;     //  MIMGParam structure size
   unsigned short  mom_no;           // number of MOMParam structures
   unsigned short  MOM_size;         //  MOMParam structure size
   unsigned long   mom_offs;         // offset of the 1st MOMParam structure
   unsigned long   syspar_ext_offs;  // offset of SysParExt structure for extension of system parameters
   unsigned long   syspar_ext_size;  //  SysParExt structure size
   unsigned long   mosaic_offs;      // offset of MosaicParam structure for extension of system parameters
   unsigned long   mosaic_size;      //  MosaicParam structure size
   unsigned long   WF_img_offs;      // offset of WFParam structure for Wide-field Image parameters
   unsigned short  WF_img_size;      //  WFParam structure size
   unsigned long   WndLayout_offs;   // offset of WndLayout structure for Windows Layout parameters
   unsigned short  WndLayout_size;   //  WndLayout structure size
   unsigned long   trpar_ext_offs;   // offset of TrParExt structure for extension of 3D Traces parameters
   unsigned short  trpar_ext_size;   //  TrParExt structure size
   unsigned long   CorPar_offs;      // offset of 1st CorPar structure for multichannel detector correction
   unsigned short  CorPar_size;      //  CorPar structure size
   unsigned short  CorPar_number;    //  number of CorPar structures - max 4 ( 1 per SPC module)
                                     //      bit 8 = 1 - Detector Correction Enabled
   unsigned long   LifeTrPar_offs;   // offset of 1st LifeTrPar structure for 3D Lifetime traces
   unsigned short  LifeTrPar_size;      //  LifeTrPar structure size
   unsigned short  LifeTrPar_number;    //  number of LifeTrPar structures - max 8 ( 1 per 3D trace)
   unsigned char   extension[176];   //  for future use, keep the size of SPCBinHdrExt = 240B
}SPCBinHdrExt;

#pragma pack(pop)

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