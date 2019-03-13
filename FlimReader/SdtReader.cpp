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
   in.read((char*)&header, sizeof(header));

   // read file info
   in.seekg(header.info_offs);

   std::string info(header.info_length, ' ');
   in.read(&info[0], header.info_length);

   std::stringstream info_stream(info);
   std::string token, key, value;

   while (std::getline(info_stream, token, '\n'))
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

   size_t textEnd = setup.find(BINARY_SETUP);
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

   auto extract = [&](const std::string& token, const std::string& str, int& target) {
      if (token.find(str) != -1) {
         size_t ndx = token.find(str) + str.length();
         size_t end = token.find("]", ndx);
         target = std::stoi(token.substr(ndx, end));
      }
   };

   while (std::getline(setup_stream, token, '\n'))
   {
      boost::trim(token);

      key = ""; value = "";
      if (boost::starts_with(token, "#SP") || boost::starts_with(token, "#DI") ||
         boost::starts_with(token, "#PR") || boost::starts_with(token, "#MP"))
      {
         size_t open = token.find('[');
         key = token.substr(open + 1, token.find(",", open));
         value = token.substr(token.rfind(",") + 1, token.length() - 1);
      }
      else if (boost::starts_with(token, "#TR") || boost::starts_with(token, "#WI")) {
         key = token.substr(0, token.find('['));
         boost::trim(key);
         value = token.substr(token.find('[') + 1, token.find(']') - 1);
      }

      if (!(key.empty() || value.empty())) tags[key] = value;

      extract(token, X_STRING, n_x);
      extract(token, Y_STRING, n_y);
      extract(token, C_STRING, n_chan);
      extract(token, X_IMG_STRING, mode13width);
      extract(token, Y_IMG_STRING, mode13height);
   }

   in.seekg(header.meas_desc_block_offs);
   for (int i = 0; i < header.no_of_meas_desc_blocks; i++)
   {
      in.read((char*)&measure_info, sizeof(measure_info));

      // extract dimensional parameters from measure info
      if (measure_info.scan_x > 0) n_x = measure_info.scan_x;
      if (measure_info.scan_y > 0) n_y = measure_info.scan_y;
      if (measure_info.adc_re > 0) n_timebins = measure_info.adc_re;
      if (measure_info.scan_rx > 0) n_chan = measure_info.scan_rx;
      if (measure_info.image_rx > 0) n_chan = measure_info.image_rx;

      // measurement mode 0 and 1 are both single-point data
      if (measure_info.meas_mode == 0 || measure_info.meas_mode == 1)
      {
         n_x = 1;
         n_y = 1;
      }

      // for measurement_mode 13 one channel is stored in each block 
      // & width & height are not in scanX & scanY
      if (measure_info.meas_mode == 13) {
         n_x = mode13width;
         n_y = mode13height;
      }

      rep_rate_hz = measure_info.StopInfo.max_sync_rate;      
   }

   if (measure_info.meas_mode == 13)
   {
      n_chan = header.no_of_meas_desc_blocks;
      channels_split = true;
   }



   in.seekg(header.data_block_offs);

   for (int i = 0; i < header.no_of_data_blocks; i++)
   {
      size_t next_block_offs;
      bool compressed = false;
      std::streamoff data_offset = 0;

      if (header.revision < 15)
      {
         BHFileBlockHeaderOld block_header;
         in.read((char*)&block_header, sizeof(block_header));

         next_block_offs = block_header.next_block_offs;
         compressed = block_header.block_type & DATA_ZIPPED;
         data_offset = block_header.data_offs;
      }
      else
      {
         BHFileBlockHeader block_header;
         in.read((char*)&block_header, sizeof(block_header));

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
   for (int i = 0; i < n_timebins; i++)
      native_timepoints[i] = i * time_resolution_native_ps;

}