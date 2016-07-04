#pragma once

#include "AbstractFifoReader.h"
#include "lz4.h"

struct ffd_evt_v1
{
   uint64_t macro_time;
   uint16_t micro_time;
   uint8_t channel;
   uint8_t mark;
};


struct ffd_evt
{
   uint16_t macro_time;
   uint16_t micro_time;
};

class FfdEvent : public TcspcEvent
{
public:
   FfdEvent(ffd_evt_v1 evt)
   {
      micro_time = evt.micro_time;
      macro_time = evt.macro_time;
      channel = evt.channel;
      mark = evt.mark;
   }
   
   FfdEvent(ffd_evt evt)
   {
      macro_time = evt.macro_time;
      channel = evt.micro_time & 0xF;

      if (channel == 0xF)
      {
         if (evt.micro_time == 0xF)
         {
            macro_time_offset = ((macro_time == 0) ? 1 : macro_time) * 0xFFFF;
            valid = false;
         }
         else
         {
            mark = evt.micro_time >> 4;
         }
      }
      else
      {
         mark = 0;
         micro_time = evt.micro_time >> 4;
      }
   }
};


class FfdEventReader : public AbstractEventReader
{
public:

   FfdEventReader(const std::string& filename, int version, int data_position, bool use_compression = false, size_t max_message_size = 16 * 1024)
      : AbstractEventReader(filename, data_position),
        version(version),
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
         return !fs.eof() && !fs.fail();
   }



   TcspcEvent getEvent()
   {
      int a = sizeof(ffd_evt);
      if (use_compression)
      {

         if (decode_bytes == 0)
         decode();
        
         if (version == 1)
         {
            int sz = sizeof(ffd_evt_v1);
            ffd_evt_v1 evt = *reinterpret_cast<ffd_evt_v1*>(&decode_buffer[decode_offset]);
            decode_offset += sz;
            decode_bytes -= sz;
            return FfdEvent(evt);
         }
         else
         {
            int sz = sizeof(ffd_evt);
            ffd_evt evt = *reinterpret_cast<ffd_evt*>(&decode_buffer[decode_offset]);
            decode_offset += sz;
            decode_bytes -= sz;
            return FfdEvent(evt);
         }
      }
      else
      {
         if (version == 1)
         {
            ffd_evt_v1 evt;
            fs.read(reinterpret_cast<char*>(&evt), sizeof(evt));
            return FfdEvent(evt);
         }
         else
         {
            ffd_evt evt;
            fs.read(reinterpret_cast<char*>(&evt), sizeof(evt));
            return FfdEvent(evt);
         }

         assert(fs.good());
      }
   }

protected: 

   static const int buffer_size = 128 * 1024;
   size_t max_message_size;

   std::vector<char> decode_buffer;
   std::vector<char> input_buffer;

   uint32_t version;
   int decode_offset = 0;
   int decode_bytes = 0;
   uint64_t macro_time_offset = 0;
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
   uint32_t version;

};
