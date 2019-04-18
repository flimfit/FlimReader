#pragma once

#include <memory>
#include "AbstractEventReader.h"

class FifoFrame
{
public:
   FifoFrame(std::shared_ptr<AbstractEventReader> reader, Markers markers) :
      reader(reader), markers(markers), frame_number(-1)
   {
   }

   void loadNext();

   bool isEmpty();

   std::vector<FifoEvent> events;
   std::vector<FifoEvent> marker_events;
   FifoEvent next_frame_event;
   FifoEvent frame_start_event;
   int frame_number = -1;
   int image_number = 0;
   Markers getMarkers() { return markers; }

protected:

   std::shared_ptr<AbstractEventReader> reader;
   Markers markers;
};