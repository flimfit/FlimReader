#pragma once
#include <opencv2/opencv.hpp>

class RealignmentParameters
{
public:
   bool use_realignment = false;
   int spatial_binning = 1;
   int frame_binning = 1;
   bool use_rotation = false;
};

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

class TransformInterpolator
{
public:

   TransformInterpolator();

   bool empty();
   void clear();

   void setRealignmentParams(RealignmentParameters params_);
   void setReference(double frame_t, cv::Mat& reference_);
   void addFrame(double frame_t, cv::Mat& frame); 
   void getAffine(double frame, cv::Mat& affine, cv::Point2d& shift);
   void interpolate(Transform& t1, Transform& t2, double frame, cv::Mat& affine, cv::Point2d& shift);

protected:

   void addTransform(Transform t);

private:
   std::vector<Transform> frame_transform;
   RealignmentParameters realign_params;

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