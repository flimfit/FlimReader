#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "AbstractEventReader.h"
#include "FifoFrame.h"

class Photon 
{
public:
   Photon() :
      valid(false)
   {}

   Photon(int image, int frame, double x, double y, double z, int channel, uint32_t micro_time, uint64_t macro_time) :
      image(image), frame(frame), x(x), y(y), z(0), channel(channel), micro_time(micro_time), macro_time(macro_time), valid(true)
   {}

   explicit operator bool() const { return valid; }

   int image;
   int frame;
   double x;
   double y;
   double z;
   int channel;
   uint32_t micro_time;
   uint64_t macro_time;
   bool valid;
};

class FifoInferredParameters
{
public:
   SyncSettings sync;
   int n_chan;
   std::vector<bool> recommended_channels;
};

class FifoProcessor
{
public:
   FifoProcessor(std::shared_ptr<FifoFrame> frame, SyncSettings sync);
   void determineLineStartTimes();
   Photon getNextPhoton();

   FifoInferredParameters determineSyncSettings(SyncSettings sync, int n_chan);


protected:
   std::shared_ptr<FifoFrame> frame;
   Markers markers;
   SyncSettings sync;

   std::vector<FifoEvent>::iterator event_it;
   size_t line_idx = 0;

   std::vector<uint64_t> real_line_time;

private:

   long long sync_start = 0;

   int cur_px = -1;
   int cur_line = -1;
   bool line_valid = false;
};