#include "AbstractEventReader.h"

AbstractEventReader::AbstractEventReader(unsigned int packet_size) :
   packet_size(packet_size)
{

}

FifoEvent AbstractEventReader::getEvent() 
{
   // Get event and compute absolute macro time
   auto t = getRawEvent();
   FifoEvent event = std::get<0>(t);
   sync_count_accum += std::get<1>(t);
   event.macro_time += sync_count_accum;
   return event;
}

void AbstractEventReader::setToStart()
{
   cur_pos = 0;
   sync_count_accum = 0;
}
