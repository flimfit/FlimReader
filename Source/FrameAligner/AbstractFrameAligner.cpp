#include "RigidFrameAligner.h"
#include "FrameWarpAligner.h"

std::string realignmentTypeString(RealignmentType t)
{
   switch (t)
   {
   case RealignmentType::None:
      return "None";
   case RealignmentType::Translation:
      return "Translation";
   case RealignmentType::RigidBody:
      return "RigidBody";
   case RealignmentType::Warp:
      return "Warp";
   default:
      return "";
   }
}


std::unique_ptr<AbstractFrameAligner> AbstractFrameAligner::createFrameAligner(RealignmentParameters params)
{
   switch (params.type)
   {
   case RealignmentType::None:
      return nullptr;
   case RealignmentType::Warp:
      return std::unique_ptr<AbstractFrameAligner>(new FrameWarpAligner(params));
   default: // handles both Translation and RigidBody
      return std::unique_ptr<AbstractFrameAligner>(new RigidFrameAligner(params));
   }
}

cv::Mat downsample(const cv::Mat& im1, int factor)
{
   int w = im1.size().width;
   int h = im1.size().height;

   int nh = ceil(h / (double)factor);
   int nw = ceil(w / (double)factor);

   cv::Mat im2(nh, nw, im1.type(), cv::Scalar(0));

   for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
         im2.at<float>(y / factor, x / factor) += im1.at<float>(y, x);

   return im2;
}
