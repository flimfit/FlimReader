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
      
   void init(const std::vector<double>& timepoints_, int n_chan_, int n_x_, int n_y_)
   {
      timepoints = timepoints_;
      n_chan = n_chan_;
      n_x = n_x_;
      n_y = n_y_;
      n_t = timepoints.size();
      size_t n_el = timepoints.size() * n_chan * n_x * n_y;
      data.resize(n_el);
      ready = true;
   }

   T* getDataPtr()
   {
      assert(ready);
      return data.data();
   }

   uint64_t getDataSize()
   {
      return n_t * n_chan * n_y * n_x * sizeof(T);
   }

   uint64_t n_t = 1;
   uint64_t n_chan = 1;
   uint64_t n_x = 1;
   uint64_t n_y = 1;
   std::vector<double> timepoints;
   bool isReady() { return ready; }

protected:
   bool ready = false;
   std::vector<T> data;
};