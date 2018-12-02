#include "SdtReader.h"
#include <boost/algorithm/string.hpp>


const std::string X_STRING = "#SP [SP_SCAN_X,I,";
const std::string Y_STRING = "#SP [SP_SCAN_Y,I,";
const std::string T_STRING = "#SP [SP_ADC_RE,I,";
const std::string C_STRING = "#SP [SP_SCAN_RX,I,";
const std::string X_IMG_STRING = "#SP [SP_IMG_X,I,";
const std::string Y_IMG_STRING = "#SP [SP_IMG_Y,I,";
const std::string BINARY_SETUP = "BIN_PARA_BEGIN:\0";

SdtReader::SdtReader(const std::string& filename) : 
   HistogramReader(filename)
{
   readHeader();
   initaliseTimepoints();
}

void SdtReader::readHeader()
{
   std::ifstream in(filename, std::ifstream::binary);

   // Read file header
   in.read((char*) &header, sizeof(header));

   // read file info
   in.seekg(header.info_offs);
   
   std::string info(header.info_length, ' '); 
   in.read(&info[0], header.info_length);

   std::stringstream info_stream(info); 
   std::string token, key, value; 
      
   while(std::getline(info_stream, token, '\n')) 
   {
      boost::trim(token);
      if (token.find(':') == -1) continue;
      key = token.substr(0, token.find(':'));
      value = token.substr(token.find(':') + 1);
      boost::trim(key);
      boost::trim(value);
      tags[key] = value;
   }

   // read setup
   in.seekg(header.setup_offs);

   std::string setup(header.setup_length, ' '); 
   in.read(&setup[0], header.setup_length);

   int textEnd = setup.find(BINARY_SETUP);
   if (textEnd > 0) {
      setup = setup.substr(0, textEnd);
      textEnd += BINARY_SETUP.length();
      in.seekg(header.setup_offs + textEnd);
   }

   // variables to hold height & width read from header string for measMode 13
   int mode13width = 0;
   int mode13height = 0;
   int n_timebins = 0;

   std::stringstream setup_stream(setup); 
      
   while(std::getline(setup_stream, token, '\n')) 
   {
      boost::trim(token);

      if (boost::starts_with(token, "#SP") || boost::starts_with(token, "#DI") ||
          boost::starts_with(token, "#PR") || boost::starts_with(token, "#MP"))
      {
        int open = token.find('[');
        key = token.substr(open + 1, token.find(",", open));
        value = token.substr(token.rfind(",") + 1, token.length() - 1);
      }
      else if (boost::starts_with(token, "#TR") || boost::starts_with(token, "#WI")) {
        key = token.substr(0, token.find('['));
        boost::trim(key);
        value = token.substr(token.find('[') + 1, token.find(']'));
      }

      if (!(key.empty() || value.empty())) tags[key] = value;

      if (token.find(X_STRING) != -1) {
        int ndx = token.find(X_STRING) + X_STRING.length();
        int end = token.find("]", ndx);
        n_x = std::stoi(token.substr(ndx, end));
      }
      else if (token.find(Y_STRING) != -1) {
        int ndx = token.find(Y_STRING) + Y_STRING.length();
        int end = token.find("]", ndx);
        n_y = std::stoi(token.substr(ndx, end));
      }
      else if (token.find(T_STRING) != -1) {
        int ndx = token.find(T_STRING) + T_STRING.length();
        int end = token.find("]", ndx);
        n_timebins = std::stoi(token.substr(ndx, end));
      }
      else if (token.find(C_STRING) != -1) {
        int ndx = token.find(C_STRING) + C_STRING.length();
        int end = token.find("]", ndx);
        n_chan = std::stoi(token.substr(ndx, end));
      }
      
      else if (token.find(X_IMG_STRING) != -1) {
        int ndx = token.find(X_IMG_STRING) + X_IMG_STRING.length();
        int end = token.find("]", ndx);
        mode13width = std::stoi(token.substr(ndx, end));
      }
      
      else if (token.find(Y_IMG_STRING) != -1) {
        int ndx = token.find(Y_IMG_STRING) + Y_IMG_STRING.length();
        int end = token.find("]", ndx);
        mode13height = std::stoi(token.substr(ndx, end));
      }
   }

   in.seekg(header.meas_desc_block_offs);
   for (int i=0; i<header.no_of_meas_desc_blocks; i++)
   {

      bool hasMeasureInfo = header.meas_desc_block_length >= 211;
      bool hasMeasStopInfo = header.meas_desc_block_length >= 211 + 60;
      bool hasMeasFCSInfo = header.meas_desc_block_length >= 211 + 60 + 38;
      bool hasExtendedMeasureInfo = header.meas_desc_block_length >= 211 + 60 + 38 + 26;
      bool hasMeasHISTInfo = header.meas_desc_block_length >= 211 + 60 + 38 + 26 + 24;

      in.read((char*) &measure_info, sizeof(measure_info));
      // extract dimensional parameters from measure info
      if (measure_info.scan_x > 0) n_x = measure_info.scan_x;
      if (measure_info.scan_y > 0) n_y = measure_info.scan_y;
      if (measure_info.adc_re > 0) n_timebins = measure_info.adc_re;
      if (measure_info.scan_rx > 0) n_chan = measure_info.scan_rx;
      
      // measurement mode 0 and 1 are both single-point data
      if (measure_info.meas_mode == 0 || measure_info.meas_mode == 1)  
      {
         n_x = 1;
         n_y = 1;
      }  
      
      // for measurement_mode 13 one channel is stored in each block 
      // & width & height are not in scanX & scanY
      if (measure_info.meas_mode == 13)  {
         n_x = mode13width;
         n_y = mode13height;
      }

      // This works if all blocks are same size etc or if we are in mode 13
      // Need to validate one of these is true...
      n_chan = header.no_of_meas_desc_blocks;
      channels_split = true;

   }

   in.seekg(header.data_block_offs);

   for (int i=0; i<header.no_of_data_blocks; i++) 
   {
      size_t next_block_offs;
      bool compressed = false;
      std::streamoff data_offset = 0;

      if (header.revision < 15)
      {
         BHFileBlockHeaderOld block_header;
         in.read((char*) &block_header, sizeof(block_header));

         next_block_offs = block_header.next_block_offs;
         compressed = block_header.block_type & DATA_ZIPPED;
         data_offset = block_header.data_offs;
      }
      else
      {
         BHFileBlockHeader block_header;
         in.read((char*) &block_header, sizeof(block_header));

         next_block_offs = block_header.next_block_offs;
         next_block_offs += ((size_t)block_header.next_block_offs_ext) << 32;
      
         compressed = block_header.block_type & DATA_ZIPPED;
         data_offset = block_header.data_offs;
         data_offset += ((std::streamoff)block_header.data_offs_ext) << 32;
      }

      blocks.push_back({ data_offset, 0, compressed });

      in.seekg(next_block_offs);
   }

   double time_resolution_native_ps = 1e12 * measure_info.tac_r / (n_timebins * measure_info.tac_g);
   native_timepoints.resize(n_timebins);
   for(int i=0; i<n_timebins; i++)
      native_timepoints[i] = i * time_resolution_native_ps;

}