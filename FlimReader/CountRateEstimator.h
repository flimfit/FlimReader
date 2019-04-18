#pragma once
#include <cstdint>
#include <queue>

class CountRateEstimator
{
public:
   CountRateEstimator(double macro_time_resolution_ps, double micro_time_resolution_ps, double frame_time_ps) :
      macro_time_resolution_ps(macro_time_resolution_ps),
      micro_time_resolution_ps(micro_time_resolution_ps),
      frame_time_ps(frame_time_ps)
   {}

   void setNewFrame()
   {
      count_rate = (1e12 / frame_time_ps) * counts_this_frame;
               
      //max_instant_count_rate[i] = max_rate_this_frame[i];
      //max_rate_this_frame[i] = 0;
      
      counts_this_frame = 0;
   }

   void addEvent(uint64_t macro_time, uint16_t micro_time)
   {
      counts_this_frame++;

      double cur_time = macro_time * macro_time_resolution_ps + micro_time * micro_time_resolution_ps;
      recent_photon_times.push(cur_time);

      // Calculate rate based on arrival times of most recent photons
      if (recent_photon_times.size() > 100)
      {
         double last_time = recent_photon_times.front();

         size_t n_photons = recent_photon_times.size();
         double rate = 1e12 * n_photons / (cur_time - last_time);

         if (rate > max_rate_this_frame)
            max_rate_this_frame = rate;

         recent_photon_times.pop();
      }
   }

   double getCountRate() { return count_rate; }

private:

   double macro_time_resolution_ps;
   double micro_time_resolution_ps;
   double frame_time_ps;
   
   uint64_t counts_this_frame = 0;

   double count_rate = 0;
   uint64_t max_pixel_counts = 0;
   double max_instant_count_rate = 0;
   double max_rate_this_frame = 0;

   std::queue<double> recent_photon_times;
};