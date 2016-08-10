#pragma once

#include "AbstractFrameAligner.h"

class Transform
{
public:

   Transform(double frame_)
   {
      frame = frame_;
      angle = 0.0;
      shift.x = 0.0;
      shift.y = 0.0;
   }

   Transform(double frame, double angle, cv::Point2d shift) :
      frame(frame), angle(angle), shift(shift)
   {
   }

   double frame;
   double angle;
   cv::Point2d shift;
};

class TransformInterpolator : public AbstractFrameAligner
{
public:

   TransformInterpolator();

   bool empty();
   void clear();

   void setReference(int frame_t, const cv::Mat& reference_);
   void addFrame(int frame_t, const cv::Mat& frame); 
   void shiftPixel(int frame, int& x, int& y);
private:

   void addTransform(Transform t);
   void interpolate(Transform& t1, Transform& t2, double frame, cv::Mat& affine, cv::Point2d& shift);
   void getAffine(double frame, cv::Mat& affine, cv::Point2d& shift);

   std::vector<Transform> frame_transform;

   double cache_frame = -1;
   cv::Mat cache_affine;
   cv::Point2d cache_shift;

   cv::Point2d centre;
   cv::Point2d centre_binned;

   cv::Mat window;
   cv::Mat reference;

   cv::Mat log_polar0;
   cv::Mat log_polari;
   cv::Mat rotatedi;
};