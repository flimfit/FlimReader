#pragma once
#include <stdint.h>
#include <opencv2/opencv.hpp>

enum FlimNativeType
{
   DataTypeUint16,
   DataTypeFloat,
   DataTypeDouble
};

class FlimCube
{
public:
   FlimCube();
      
   void init(FlimNativeType data_type_, const std::vector<double>& timepoints_, int n_chan_, int n_x_, int n_y_, int n_z_ = 1);

   uint8_t* getDataPtr(int z = 0);
   void getIntensityAndMeanArrival(cv::Mat& intensity, cv::Mat& mar);

   uint64_t getDataSize();
   uint64_t getFrameSize();
   uint64_t elementSize();

   uint64_t n_t = 1;
   uint64_t n_chan = 1;
   uint64_t n_x = 1;
   uint64_t n_y = 1;
   uint64_t n_z = 1;
   FlimNativeType data_type = DataTypeUint16;

   std::vector<double> timepoints;
   bool isReady() { return ready; }

protected:

   template<typename T>
   void getIntensityAndMeanArrivalT(cv::Mat& intensity, cv::Mat& mar);

   size_t element_size = 2;
   bool ready = false;
   std::vector<uint8_t> data;
   uint64_t n_frame_el = 1;
};