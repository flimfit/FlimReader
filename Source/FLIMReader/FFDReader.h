#pragma once

#include "AbstractFifoReader.h"
#include "lz4.h"

struct ffd_evt
{
   uint64_t macro_time;
   uint16_t micro_time;
   uint8_t channel;
   uint8_t mark;
};

class FfdEvent : public TcspcEvent
{
public:
   FfdEvent(ffd_evt evt)
   {
      micro_time = evt.micro_time;
      macro_time = evt.macro_time;
      channel = evt.channel;
      mark = evt.mark;
   }
};


class FfdEventReader : public AbstractEventReader
{
public:

   FfdEventReader(const std::string& filename, int data_position, bool use_compression = false, size_t max_message_size = 16 * 1024)
      : AbstractEventReader(filename, data_position),
        use_compression(use_compression),
        max_message_size(max_message_size)
   {
      decode_buffer.resize(buffer_size);
      input_buffer.resize(LZ4_COMPRESSBOUND(max_message_size));
	  lz4StreamDecode_body = { 0 };
   }

   void decode()
   {
      finished_decoding = fs.eof();
        
      if (finished_decoding)
         return;

      int cmp_bytes;
      fs.read(reinterpret_cast<char*>(&cmp_bytes), sizeof(cmp_bytes));

      assert(fs);
         
      fs.read(input_buffer.data(), cmp_bytes);

      LZ4_streamDecode_t* lz4StreamDecode = &lz4StreamDecode_body;

      // Wraparound the ringbuffer offset
      if (decode_offset >= buffer_size - max_message_size) decode_offset = 0;

      char* const dec_ptr = &decode_buffer[decode_offset];
      decode_bytes = LZ4_decompress_safe_continue(
         lz4StreamDecode, input_buffer.data(), dec_ptr, cmp_bytes, max_message_size);

      finished_decoding = decode_bytes <= 0;
   }
   
   bool hasMoreData()
   {
      if (use_compression)
         return (decode_bytes > 0) || (finished_decoding == false);
      else
         return !fs.eof();
   }



   TcspcEvent getEvent()
   {
      if (use_compression)
      {
         int sz = sizeof(ffd_evt);

         if (decode_bytes == 0)
         decode();

         ffd_evt* evt = reinterpret_cast<ffd_evt*>(&decode_buffer[decode_offset]);
         decode_offset += sz;
         decode_bytes -= sz;
         return FfdEvent(*evt);
      }
      else
      {
         ffd_evt evt;
         fs.read(reinterpret_cast<char*>(&evt), sizeof(evt));
         return FfdEvent(evt);
      }
   }

protected: 

   static const int buffer_size = 128 * 1024;
   size_t max_message_size;

   std::vector<char> decode_buffer;
   std::vector<char> input_buffer;

   int decode_offset = 0;
   int decode_bytes = 0;
   LZ4_streamDecode_t lz4StreamDecode_body;
   bool finished_decoding = false;
   bool use_compression = false;
};


class FfdReader : public AbstractFifoReader
{
public:

   FfdReader(const std::string& filename);

protected:
   
   enum 
   {
      TagDouble = 0,
      TagUInt64 = 1,
      TagInt64 = 2,
      TagBool = 4,
      TagString = 5,
      TagDate = 6,
      TagEndHeader = 7
   } FlimMetadataTag;

   void readHeader();
   int data_position = 0;
   bool use_compression = false;
   size_t message_size = 16 * 1024;

};
