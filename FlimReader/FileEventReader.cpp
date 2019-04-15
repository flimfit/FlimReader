#include "FileEventReader.h"

FileEventReader::FileEventReader(const std::string& filename, std::streamoff data_position, unsigned int packet_size_) :
data_position(data_position), AbstractEventReader(packet_size_)
{
   fs = std::ifstream(filename, std::ifstream::in | std::ifstream::binary);
   
   fs.seekg(0, fs.end);
   length = fs.tellg() - data_position;
   n_packet = length / packet_size_;
   uint64_t n_block = n_packet / block_size + 1;
   data.reserve(n_block);
   
   reader_thread = std::thread(&FileEventReader::read, this);
   setToStart();
}


FileEventReader::~FileEventReader()
{
   terminate = true;
   if (reader_thread.joinable())
      reader_thread.join();
}


double FileEventReader::getProgress()
{
   return ((double) cur_pos) / n_packet;
}

bool FileEventReader::hasMoreData()
{
   return cur_pos < n_packet;
}

void FileEventReader::read()
{
   uint64_t sz = block_size * packet_size;
   
   fs.seekg(data_position);

   size_t n_read = 0;
   do
   {
      std::vector<char> block(sz);
      fs.read(&block[0], sz);
      n_read = fs.gcount();

      
      std::unique_lock<std::mutex> lk(m);
      data.push_back(block);
      cv.notify_one();
   } while (n_read > 0 && !terminate);
}
