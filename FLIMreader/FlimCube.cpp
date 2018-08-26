#include "FlimCube.h"

FlimCube::FlimCube()
{
   data.resize(1);
}

void FlimCube::init(FlimNativeType data_type_, const std::vector<double>& timepoints_, int n_chan_, int n_x_, int n_y_, int n_z_)
{
   data_type = data_type_;
   timepoints = timepoints_;
   n_chan = n_chan_;
   n_x = n_x_;
   n_y = n_y_;
   n_z = n_z_;
   n_t = timepoints.size();
   element_size = elementSize();
   n_frame_el = n_t * n_chan * n_x * n_y * element_size;
   data.resize(n_z * n_frame_el);
   ready = true;
}

uint8_t* FlimCube::getDataPtr(int z)
{
   if (!ready) throw std::runtime_error("Cube not initalized");
   if (z >= n_z) throw std::runtime_error("Invalid z frame requested");

   return data.data() + n_frame_el * z;
}

template<typename T>
void FlimCube::getIntensityAndMeanArrivalT(cv::Mat& intensity, cv::Mat& mar)
{
   std::vector<int> dims = { (int)n_z, (int)n_y, (int)n_x };
   intensity = cv::Mat(dims, CV_32F);
   mar = cv::Mat(dims, CV_32F);
   size_t n_px = n_z * n_y * n_x;

   T* data_ptr = reinterpret_cast<T*>(data.data());
   float* intensity_ptr = (float*)intensity.data;
   float* mean_arrival_time_ptr = (float*)mar.data;
   for (int p = 0; p < n_px; p++)
   {
      float I = 0;
      float It = 0;
      for (int c = 0; c < n_chan; c++)
         for (int t = 0; t < n_t; t++)
         {
            I += (float)*data_ptr;
            It += (float)((*data_ptr) * timepoints[t]);
            data_ptr++;
         }
      intensity_ptr[p] = I;
      mean_arrival_time_ptr[p] = It / I;
   }
}

void FlimCube::getIntensityAndMeanArrival(cv::Mat& intensity, cv::Mat& mar)
{
   if (!ready) return;

   switch (data_type)
   {
   case DataTypeUint16:
      getIntensityAndMeanArrivalT<uint16_t>(intensity, mar); return;
   case DataTypeFloat:
      getIntensityAndMeanArrivalT<float>(intensity, mar); return;
   case DataTypeDouble:
      getIntensityAndMeanArrivalT<double>(intensity, mar); return;
   }
}

uint64_t FlimCube::getDataSize()
{
   return n_frame_el * n_z;
}

uint64_t FlimCube::getFrameSize()
{
   return n_frame_el;
}

uint64_t FlimCube::elementSize()
{
   switch (data_type)
   {
   case DataTypeUint16:
      return 2;
   case DataTypeFloat:
      return 4;
   case DataTypeDouble:
      return 8;
   default:
      return 0;
   }
}