#pragma once
#include <opencv2/opencv.hpp>
#include <memory>

enum class RealignmentType
{
   None        = 0,
   Translation = 1,
   RigidBody   = 2,
   Warp        = 3
};

std::string realignmentTypeString(RealignmentType t);

class RealignmentParameters
{
public:
   RealignmentType type = RealignmentType::None;
   int spatial_binning = 1;
   int frame_binning = 1;
   int n_resampling_points = 30;
   bool reprocessing = false;


   bool use_realignment() { return type != RealignmentType::None; }
   bool use_rotation() { return type == RealignmentType::RigidBody; }
};

class RealignmentResult
{
public:
   cv::Mat frame;
   cv::Mat realigned;
   double correlation = 0;
};

class ImageScanParameters
{
public:
   ImageScanParameters(double line_duration = 100, double interline_duration = 101, int n_x = 1, int n_y = 1) :
      line_duration(line_duration), interline_duration(interline_duration), n_x(n_x), n_y(n_y)
   {
      n_x = std::max(n_x, 1);
      n_y = std::max(n_y, 1);

      if (interline_duration <= line_duration)
         interline_duration = 1.01 * line_duration;

      pixel_duration = line_duration / n_x;
      frame_duration = n_y * interline_duration;
   }

   double line_duration;
   double interline_duration;
   double pixel_duration;
   double frame_duration;
   int n_x;
   int n_y;
};

class AbstractFrameAligner
{
public:

   static std::unique_ptr<AbstractFrameAligner> createFrameAligner(RealignmentParameters params);

   virtual bool empty() = 0;
   virtual void clear() = 0;

   virtual RealignmentType getType() = 0;

   void setRealignmentParams(RealignmentParameters params_) { realign_params = params_; }
   void setImageScanParams(ImageScanParameters params_) { image_params = params_; }
   virtual void setReference(int frame_t, const cv::Mat& reference_) = 0;
   virtual RealignmentResult addFrame(int frame_t, const cv::Mat& frame) = 0; // shold return aligned frame
   virtual void shiftPixel(int frame_t, double& x, double& y) = 0;
   virtual void setNumberOfFrames(int n_frames_) { n_frames = n_frames_; }

   virtual void reprocess() {};

protected:
   RealignmentParameters realign_params;
   ImageScanParameters image_params;
   cv::Mat reference;
   int n_frames = 1;
};
