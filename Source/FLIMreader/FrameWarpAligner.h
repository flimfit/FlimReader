#pragma once

#include "TransformInterpolator.h"
#include <functional>
#include <opencv2/opencv.hpp>

class Range
{
public:
   int begin;
   int end;

   int interval() { return end - begin; }
};

class FrameWarpAligner : public AbstractFrameAligner
{
public:

   FrameWarpAligner(RealignmentParameters params);

   bool empty() { return false; };
   void clear() { Dstore.clear(); };

   RealignmentType getType() { return RealignmentType::Warp; };

   void setReference(int frame_t, const cv::Mat& reference_);
   RealignmentResult addFrame(int frame_t, const cv::Mat& frame);
   void shiftPixel(int frame_t, double& x, double& y);

   void reprocess();

protected:

   void precomputeInterp();
   void computeSteepestDecentImages(const cv::Mat& frame);
   double computeHessianEntry(int pi, int pj);
   void computeHessian();
   void steepestDecentUpdate(const cv::Mat& error_img, cv::Mat& sd);
   void warpImage(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D);
   cv::Point warpPoint(const std::vector<cv::Point2d>& D, int x, int y, int spatial_binning = 1);
   double computeErrorImage(cv::Mat& wimg, cv::Mat& error_img);
   std::vector<Range> D_range;

   cv::Mat Di;
   cv::Mat Df;
   cv::Mat H;

   cv::Point2d Dlast;

   std::map<int,std::vector<cv::Point2d>> Dstore;
   std::map<int,RealignmentResult> results;

   int nD = 10;

   int n_x_binned;
   int n_y_binned;

   std::vector<std::vector<double>> VI_dW_dp_x, VI_dW_dp_y;

   cv::Mat sum_1, sum_2;

   bool write_debug_images = false;
};