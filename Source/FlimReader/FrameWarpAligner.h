#pragma once

#include "RigidFrameAligner.h"
#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <dlib/optimization.h>

using namespace dlib;
typedef matrix<double, 0, 1> column_vector;

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
   
   ~FrameWarpAligner() {}

   bool empty() { return false; };
   void clear() { Dstore.clear(); };

   RealignmentType getType() { return RealignmentType::Warp; };

   void setNumberOfFrames(int n_frame);
   void setReference(int frame_t, const cv::Mat& reference_);
   RealignmentResult addFrame(int frame_t, const cv::Mat& frame);
   void shiftPixel(int frame_t, double& x, double& y);
   double getFrameCorrelation(int frame_t) { return results[frame_t].correlation; };
   double getFrameCoverage(int frame_t) { return results[frame_t].coverage; };
   void reprocess();

   void writeRealignmentInfo(std::string filename);

protected:

   void precomputeInterp();
   void computeSteepestDecentImages(const cv::Mat& frame);
   double computeHessianEntry(int pi, int pj);
   void computeHessian();
   void computeJacobian(const cv::Mat& error_img, column_vector& jac);
   void warpImage(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D, int invalid_value = 0);
   void warpImageIntensityPreserving(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D);
   void warpCoverage(cv::Mat& coverage, const std::vector<cv::Point2d>& D);
   cv::Point2d warpPoint(const std::vector<cv::Point2d>& D, int x, int y, int spatial_binning = 1);
   double computeErrorImage(cv::Mat& wimg, cv::Mat& error_img);
   std::vector<Range> D_range;

   cv::Mat smoothed_reference;

   cv::Mat Di;
   cv::Mat Df;
   matrix<double> H;

   cv::Point2d Dlast;

   std::vector<std::vector<cv::Point2d>> Dstore;
   std::vector<RealignmentResult> results;

   int nD = 10;
   int nx;

   int n_x_binned;
   int n_y_binned;

   std::mutex store_mutex;

   std::vector<std::vector<double>> VI_dW_dp_x, VI_dW_dp_y;

   cv::Mat sum_1, sum_2;

   std::unique_ptr<RigidFrameAligner> rigid_aligner;

   friend class OptimisationModel;
};


class OptimisationModel
{
   /*!
   This object is a "function model" which can be used with the
   find_min_trust_region() routine.
   !*/

public:
   typedef ::column_vector column_vector;
   typedef matrix<double> general_matrix;

   OptimisationModel(FrameWarpAligner* aligner, const cv::Mat& frame, const cv::Mat& raw_frame);

   double operator() (const column_vector& x) const;
   void get_derivative_and_hessian(const column_vector& x, column_vector& der, general_matrix& hess) const;

   cv::Mat getMask(const column_vector& x);
   cv::Mat getWarpedRawImage(const column_vector& x);
   cv::Mat getWarpedImage(const column_vector& x);

protected:

   FrameWarpAligner* aligner;
   cv::Mat raw_frame;
   cv::Mat frame;
   RealignmentParameters realign_params;
};
