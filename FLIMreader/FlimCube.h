#pragma once
#include <stdint.h>

template<typename T>
class FlimCube
{
public:
   FlimCube()
   {
      data.resize(1);
   }
      
   void init(const std::vector<double>& timepoints_, int n_chan_, int n_x_, int n_y_, int n_z_ = 1)
   {
      timepoints = timepoints_;
      n_chan = n_chan_;
      n_x = n_x_;
      n_y = n_y_;
      n_z = n_z_;
      n_t = timepoints.size();
      n_frame_el = n_t * n_chan * n_x * n_y;
      data.resize(n_z * n_frame_el);
      ready = true;
   }

   T* getDataPtr(int z = 0)
   {
      if (!ready) throw std::runtime_error("Cube not initalized");
      if (z >= n_z) throw std::runtime_error("Invalid z frame requested");

      return data.data() + n_frame_el * z;
   }

   uint64_t getDataSize()
   {
      return n_frame_el * n_z * sizeof(T);
   }

   uint64_t getFrameSize()
   {
      return n_frame_el * sizeof(T);
   }

   uint64_t n_t = 1;
   uint64_t n_chan = 1;
   uint64_t n_x = 1;
   uint64_t n_y = 1;
   uint64_t n_z = 1;

   std::vector<double> timepoints;
   bool isReady() { return ready; }

protected:
   bool ready = false;
   std::vector<T> data;
   uint64_t n_frame_el = 1;
};