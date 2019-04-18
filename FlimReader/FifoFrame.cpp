#include "FifoFrame.h"

bool FifoFrame::isEmpty()
{
   return marker_events.size() <= 1;
}

void FifoFrame::loadNext()
{
   do
   {
      frame_start_event = next_frame_event;

      events.clear();
      marker_events.clear();

      events.push_back(frame_start_event);
      marker_events.push_back(frame_start_event);

      while (reader->hasMoreData())
      {
         FifoEvent p = reader->getEvent();
         if (p.valid)
         {
            events.push_back(p);

            if (p.mark & markers.ImageMarker)
               image_number++;
            if (p.mark & markers.FrameMarker)
            {
               next_frame_event = p;
               break;
            }
            else if (p.mark)
            {
               marker_events.push_back(p);
            }
         }
      }
   } while (isEmpty() && reader->hasMoreData()); // skip empty frames 

   frame_number++;
}
