#include "FrameWarpAligner.h"
#include <functional>
#include "LinearInterpolation.h"
#include <fstream>

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

double correlation(cv::Mat &image_1, cv::Mat &image_2, cv::Mat &mask)
{

   // convert data-type to "float"
   cv::Mat im_float_1;
   image_1.convertTo(im_float_1, CV_32F);
   cv::Mat im_float_2;
   image_2.convertTo(im_float_2, CV_32F);

   // Compute mean and standard deviation of both images
   cv::Scalar im1_Mean, im1_Std, im2_Mean, im2_Std;
   meanStdDev(im_float_1, im1_Mean, im1_Std, mask);
   meanStdDev(im_float_2, im2_Mean, im2_Std, mask);

   im_float_1 -= im1_Mean[0];
   im_float_2 -= im2_Mean[0];

   cv::multiply(im_float_1, im_float_2, im_float_1);

   // Compute covariance and correlation coefficient
   double covar = cv::mean(im_float_1, mask)[0];
   double correl = covar / (im1_Std[0] * im2_Std[0]);

   return correl;
}

void interpolatePoint2d(const std::vector<cv::Point2d>& Ds, std::vector<cv::Point2d>& D)
{
   if (D.size() == Ds.size())
   {
      D = Ds;
      return;
   }

   int nD = D.size();
   int nDs = Ds.size();
   auto ud = std::vector<double>(nDs);
   auto vd_x = std::vector<double>(nDs);
   auto vd_y = std::vector<double>(nDs);

   auto ui = std::vector<double>(nD);
   auto vi_x = std::vector<double>(nDs);
   auto vi_y = std::vector<double>(nDs);

   for (int i = 0; i < nDs; i++)
   {
      ud[i] = i / (nDs - 1.0);
      vd_x[i] = Ds[i].x;
      vd_y[i] = Ds[i].y;
   }

   for (int i = 0; i < nD; i++)
      ui[i] = i / (nD - 1.0);

   pwl_interp_1d(ud, vd_x, ui, vi_x);
   pwl_interp_1d(ud, vd_y, ui, vi_y);

   for (int i = 0; i < nD; i++)
   {
      D[i].x = vi_x[i];
      D[i].y = vi_y[i];
   }
}

FrameWarpAligner::FrameWarpAligner(RealignmentParameters params)
{
   realign_params = params;

   RealignmentParameters rigid_params = params;
   params.type = RealignmentType::Translation;
   params.spatial_binning = 4;
   rigid_aligner = std::unique_ptr<RigidFrameAligner>(new RigidFrameAligner(params));
}


void FrameWarpAligner::setReference(int frame_t, const cv::Mat& reference_)
{
   reference_.copyTo(reference);

   if (realign_params.smoothing > 0.0)
      cv::GaussianBlur(reference, smoothed_reference, cv::Size(0, 0), realign_params.smoothing, 1);
   else
      smoothed_reference = reference;

   cv::Mat f = downsample(reference, 4);
   rigid_aligner->setReference(frame_t, f);

   n_x_binned = image_params.n_x / realign_params.spatial_binning;
   n_y_binned = image_params.n_y / realign_params.spatial_binning;
   nD = realign_params.n_resampling_points;

   precomputeInterp();
   computeSteepestDecentImages(smoothed_reference);
   computeHessian();

   Dlast = cv::Point2d(0, 0);
}

void FrameWarpAligner::reprocess()
{
   sum_2 /= (double) results.size();
   setReference(0, sum_2);

//   for (auto iter : results)
//      addFrame(iter.first, iter.second.frame);
}

void D2col(const std::vector<cv::Point2d> &D, column_vector& col)
{
   int nD = D.size();
   col.set_size(nD * 2);

   for (int i = 0; i < nD; i++)
   {
      col(i) = D[i].x;
      col(i + nD) = D[i].y;
   }
}

void col2D(const column_vector& col, std::vector<cv::Point2d> &D)
{
   int nD = col.size() / 2;
   D.resize(nD);

   for (int i = 0; i < nD; i++)
   {
      D[i].x = col(i);
      D[i].y = col(i + nD);
   }
}

void FrameWarpAligner::setNumberOfFrames(int n_frames_)
{
   n_frames = n_frames_;

   Dstore.clear();
   results.clear();
   
   Dstore.resize(n_frames, std::vector<cv::Point2d>(nD));
   results.resize(n_frames);

   rigid_aligner->setNumberOfFrames(n_frames);
}

RealignmentResult FrameWarpAligner::addFrame(int frame_t, const cv::Mat& raw_frame)
{
   if (frame_t >= Dstore.size())
      throw std::runtime_error("Frame index not valid");

   cv::Mat frame;
   if (realign_params.smoothing > 0.0)
	   cv::GaussianBlur(raw_frame, frame, cv::Size(0, 0), realign_params.smoothing, 1);
   else
	   raw_frame.copyTo(frame);


   auto model = OptimisationModel(this, frame, raw_frame);


   std::vector<column_vector> starting_point(3, column_vector(2 * nD));

   // zero starting point
   std::fill(starting_point[0].begin(), starting_point[0].end(), 0);

   // last starting point
   std::vector<cv::Point2d> D(nD, cv::Point2d(0,0));
   interpolatePoint2d(Dstore[frame_t], D);
   D2col(D, starting_point[1]);

   // rigid starting point
   
   cv::Mat ff = downsample(frame, 4);
   rigid_aligner->addFrame(frame_t, ff);
   cv::Point2d rigid_shift = rigid_aligner->getRigidShift(frame_t);
   std::vector<cv::Point2d> D_rigid(nD, rigid_shift);
   D2col(D_rigid, starting_point[2]);
   
   double best = std::numeric_limits<double>::max();
   int best_start = 0;
   for (int i = 0; i < starting_point.size(); i++)
   {
      double new_value = model(starting_point[i]);
      if (new_value < best)
      {
         best_start = i;
         best = new_value;
      }
   }

   column_vector x = starting_point[best_start];

   auto f_der = [&](const column_vector& x) -> column_vector 
   {
      column_vector der;
      matrix<double> hess;
      model.get_derivative_and_hessian(x, der, hess);
      return der;
   };

   auto f = [&](const column_vector& x) -> double
   {
      return model(x);
   };

   /*
   auto d1 = derivative(f, 4)(x);
   column_vector d2 = f_der(x);
   double l = length(d1 - d2);

   std::cout << "Difference between analytic derivative and numerical approximation of derivative: "
      << l << std::endl;
      */

   try
   {
	   
      find_min_trust_region(dlib::objective_delta_stop_strategy(1e-5),
         model,
         x,
         40 // initial trust region radius
      );
   }
   catch (dlib::error e)
   {
      std::cout << e.info;
   }

   col2D(x, D);
   std::copy(D.begin(), D.end(), Dstore[frame_t].begin());

   Dlast = *(D.end() - 2);

   cv::Mat warped_smoothed = model.getWarpedImage(x);
   cv::Mat warped = model.getWarpedRawImage(x);
   cv::Mat mask = model.getMask(x);

   cv::Mat m;
   cv::compare(mask, 0, m, cv::CMP_GT);
   

   RealignmentResult r;
   r.frame = raw_frame;
   r.realigned = warped;
   r.mask = mask;
   r.correlation = correlation(warped_smoothed, smoothed_reference, m);
   r.unaligned_correlation = correlation(frame, smoothed_reference, m);
   r.coverage = ((double)cv::countNonZero(mask)) / mask.size().area();

   cv::Mat intensity_preserving(frame.size(), CV_16U, cv::Scalar(0));
   if (r.correlation >= realign_params.correlation_threshold && r.coverage >= realign_params.coverage_threshold)
      warpImageIntensityPreserving(raw_frame, intensity_preserving, D);
   r.realigned_preserving = intensity_preserving;

   results[frame_t] = r;

   return r;
}

OptimisationModel::OptimisationModel(FrameWarpAligner* aligner, const cv::Mat& frame, const cv::Mat& raw_frame) :
   aligner(aligner),
   raw_frame(raw_frame),
   frame(frame),
   realign_params(aligner->realign_params)
{
}


double OptimisationModel::operator() (const column_vector& x) const
{
   std::vector<cv::Point2d> D;
   cv::Mat warped_image, error_image;
   
   // Get displacement matrix from warp parameters
   col2D(x, D);

   aligner->warpImage(frame, warped_image, D, -1);
   double rms_error = aligner->computeErrorImage(warped_image, error_image);
   return rms_error;
}

void OptimisationModel::get_derivative_and_hessian(const column_vector& x, column_vector& der, general_matrix& hess) const
{
   std::vector<cv::Point2d> D;
   cv::Mat warped_image, error_image;

   // Get displacement matrix from warp parameters
   col2D(x, D);

   aligner->warpImage(frame, warped_image, D, -1);
   double rms_error = aligner->computeErrorImage(warped_image, error_image);

   aligner->computeJacobian(error_image, der);
   hess = aligner->H;
}

cv::Mat OptimisationModel::getMask(const column_vector& x)
{
   std::vector<cv::Point2d> D;
   col2D(x, D);
   
   cv::Mat mask;
   aligner->warpCoverage(mask, D);
   return mask;
}

cv::Mat OptimisationModel::getWarpedRawImage(const column_vector& x)
{
   std::vector<cv::Point2d> D;
   col2D(x, D);
   cv::Mat warped_image;
   aligner->warpImage(raw_frame, warped_image, D);
   return warped_image;
}

cv::Mat OptimisationModel::getWarpedImage(const column_vector& x)
{
	std::vector<cv::Point2d> D;
	col2D(x, D);
	cv::Mat warped_image;
	aligner->warpImage(frame, warped_image, D);
	return warped_image;
}

double FrameWarpAligner::computeErrorImage(cv::Mat& wimg, cv::Mat& error_img)
{
   cv::Size size = reference.size();
   int n_px = size.area();

   error_img = wimg - smoothed_reference;
   double n_include = 0;

   double ms_error = 0;
   for (int p = 0; p < n_px; p++)
   {
      if (wimg.at<float>(p) >= 0)
      {
         ms_error += error_img.at<float>(p) * error_img.at<float>(p);
         n_include++;
      }
      else
      {
         error_img.at<float>(p) = 0;
         wimg.at<float>(p) = 0;
      }
   }

   return ms_error;
}

void FrameWarpAligner::shiftPixel(int frame_t, double& x, double& y)
{
   if (!realign_params.use_realignment())
      return;

   cv::Point2d loc = warpPoint(Dstore[frame_t], x, y);

   x -= loc.x;
   y -= loc.y;
}



/*
Jacobian structure:

0 1 2 3 4 5
|/|/|/|/|/|
-----------
0:|\|
1:|/|\|
2:  |/|\|
3:    |/|\|
4:      |/|\|
5:        |/|
*/


void FrameWarpAligner::precomputeInterp()
{
   cv::Size size = smoothed_reference.size();

   double pixel_duration = image_params.pixel_duration * realign_params.spatial_binning;
   double frame_duration = image_params.frame_duration;
   double interline_duration = image_params.interline_duration * realign_params.spatial_binning;

   D_range.resize(nD);

   Di = cv::Mat(size, CV_16U);
   Df = cv::Mat(size, CV_64F);
   
   double Di_xy;
   int i;
   int last_i = -1;

   for (int y = 0; y < size.height; y++)
   {
      for (int x = 0; x < size.width; x++)
      {
         double t = y * interline_duration + x * pixel_duration;
         double f = modf(t / frame_duration * (nD - 1), &Di_xy);
         i = (int)Di_xy;

         if (i >= (nD-1))
         {
            i = nD - 2;
            f = 1;
         }

         int x_true = x;
         if (image_params.bidirectional && ((y % 2) == 1))
            x_true = size.width - x - 1;

         Di.at<uint16_t>(y, x_true) = i;
         Df.at<double>(y, x_true) = f;

         if (i > last_i)
         {
            if (last_i >= 0)
               D_range[last_i].end = y*size.width + x_true - 1;
            D_range[i].begin = y*size.width + x_true;
            last_i = i;
         }
      }
   }

   D_range[i].end = size.area() - 1;

   int max_interval = 0;
   for (int i = 0; i < nD; i++)
   {
      int interval = D_range[i].interval();
      if (interval > max_interval)
         max_interval = D_range[i].interval();

   }

   int max_VI_dW_dp = size.area(); //max_interval * 2;

   VI_dW_dp_x.clear();
   VI_dW_dp_x.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
   VI_dW_dp_y.clear();
   VI_dW_dp_y.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
}

void FrameWarpAligner::computeSteepestDecentImages(const cv::Mat& frame)
{
   auto size = frame.size();

   // Evaluate gradient of reference
   cv::Mat nabla_Tx, nabla_Ty;
   cv::Scharr(frame, nabla_Tx, CV_64F, 1, 0, 1.0 / 32.0);
   cv::Scharr(frame, nabla_Ty, CV_64F, 0, 1, 1.0 / 32.0);

   for (int i = 1; i < nD; i++)
   {
      int p0 = D_range[i - 1].begin;
      int p1 = D_range[i - 1].end;
      for (int p = p0; p < p1; p++)
      {
         double jac = Df.at<double>(p);
         VI_dW_dp_x[i][p] = nabla_Tx.at<double>(p) * jac;
         VI_dW_dp_y[i][p] = nabla_Ty.at<double>(p) * jac;
      }
   }
   
   for(int i = 0; i < (nD - 1); i++)
   {
      int p0 = D_range[i].begin;
      int p1 = D_range[i].end;
      for (int p = p0; p < p1; p++)
      {
         double jac = 1 - Df.at<double>(p);
         VI_dW_dp_x[i][p] = nabla_Tx.at<double>(p) * jac;
         VI_dW_dp_y[i][p] = nabla_Ty.at<double>(p) * jac;
      }
   }
}

double FrameWarpAligner::computeHessianEntry(int pi, int pj)
{
   auto getV = [this](int i) -> const std::vector<double>& { return (i < nD) ? VI_dW_dp_x[i] : VI_dW_dp_y[i - nD]; };

   int i = pi % nD;
   int j = pj % nD;

   if (j > i) return computeHessianEntry(pj, pi);
   if ((i - j) > 1) return 0;

   auto v1 = getV(pi);
   auto v2 = getV(pj);
  
   int p0 = (i==0) ? D_range[i].begin : D_range[i-1].begin;
   int p1 = (i==(nD-1)) ? D_range[i-1].end : D_range[i].end;

   double h = 0;
   for (int p = p0; p < p1; p++)
      h += v1[p] * v2[p];
      
   return h;
}

void FrameWarpAligner::computeHessian()
{   
   H.set_size(2 * nD, 2 * nD);
   std::fill(H.begin(), H.end(), 0);

   for (int pi = 0; pi < nD * 2; pi++)
      H(pi, pi) += computeHessianEntry(pi, pi);

   for (int pi = 1; pi < nD * 2; pi++)
   {
      double h = computeHessianEntry(pi, pi - 1);
      H(pi, pi - 1) = h;
      H(pi - 1, pi) = h;
   }
   
   for (int i = 0; i < nD; i++)
   {
      for (int j = std::max(0, i - 1); j < std::min(nD, i + 1); j++)
      {
         double h = computeHessianEntry(i, j + nD);
         H(i, j + nD) += h;
         H(j + nD, i) += h;
      }
   }
}


void FrameWarpAligner::computeJacobian(const cv::Mat& error_img, column_vector& jac)
{
   jac.set_size(nD * 2);
   std::fill(jac.begin(), jac.end(), 0);

   for (int i = 1; i < nD; i++)
   {
      int p0 = D_range[i - 1].begin;
      int p1 = D_range[i - 1].end;
      for (int p = p0; p < p1; p++)
      {
         jac(i) += VI_dW_dp_x[i][p] * error_img.at<float>(p); // x 
         jac(i+nD) += VI_dW_dp_y[i][p] * error_img.at<float>(p); // y
      }
   }
   for (int i = 0; i < (nD - 1); i++)
   {
      int p0 = D_range[i].begin;
      int p1 = D_range[i].end;
      for (int p = p0; p < p1; p++)
      {
         jac(i) += VI_dW_dp_x[i][p] * error_img.at<float>(p); // x
         jac(i+nD) += VI_dW_dp_y[i][p] * error_img.at<float>(p); // y
      }
   }
}

void FrameWarpAligner::warpImage(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D, int invalid_value)
{
   auto size = img.size();
   wimg = cv::Mat(size, CV_32F, cv::Scalar(invalid_value));
   
   cv::Rect2i img_rect(cv::Point2i(0, 0), size);

   for (int y = 0; y < size.height; y++)
      for (int x = 0; x < size.width; x++)
      {
         cv::Point2d loc = warpPoint(D, x, y, realign_params.spatial_binning);
         
         loc.x += x;
         loc.y += y;

         cv::Point loc0(floor(loc.x), floor(loc.y));
         cv::Point loc1 = loc0 + cv::Point(1, 1);

         cv::Point2d locf(loc.x - loc0.x, loc.y - loc0.y);

         if (img_rect.contains(loc0) && img_rect.contains(loc1))
         {
            wimg.at<float>(y, x) =
               img.at<float>(loc0.y, loc0.x) * (1 - locf.y) * (1 - locf.x) + 
               img.at<float>(loc1.y, loc0.x) * (    locf.y) * (1 - locf.x) + 
               img.at<float>(loc0.y, loc1.x) * (1 - locf.y) * (    locf.x) + 
               img.at<float>(loc1.y, loc1.x) * (    locf.y) * (    locf.x);
         }
              
      }
}

void FrameWarpAligner::warpImageIntensityPreserving(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D)
{
   auto size = img.size();
   wimg = cv::Mat(size, CV_16U, cv::Scalar(0));

   cv::Rect2i img_rect(cv::Point2i(0, 0), size);

   for (int y = 0; y < size.height; y++)
      for (int x = 0; x < size.width; x++)
      {
         cv::Point2d loc = warpPoint(D, x, y, realign_params.spatial_binning);

         loc.x = x - loc.x;
         loc.y = y - loc.y;

         if (img_rect.contains(loc))
            wimg.at<uint16_t>(loc) += img.at<float>(y, x);
      }
}


void FrameWarpAligner::warpCoverage(cv::Mat& coverage, const std::vector<cv::Point2d>& D)
{
   auto size = reference.size();
   coverage = cv::Mat(size, CV_16U, cv::Scalar(0));

   cv::Rect2i img_rect(cv::Point2i(0, 0), size);

   for (int y = 0; y < size.height; y++)
      for (int x = 0; x < size.width; x++)
      {
         cv::Point2d loc = warpPoint(D, x, y, realign_params.spatial_binning);
         loc.x = x - loc.x;
         loc.y = y - loc.y;

         if (img_rect.contains(loc))
            coverage.at<uint16_t>(loc)++;
      }
}

cv::Point2d FrameWarpAligner::warpPoint(const std::vector<cv::Point2d>& D, int x, int y, int spatial_binning)
{
   double factor = ((double)realign_params.spatial_binning) / spatial_binning;
   int xs = (int) (x / factor);
   int ys = (int) (y / factor);

   if ((xs < 0) || (xs >= n_x_binned) || (ys < 0) || (ys >= n_y_binned))
      return cv::Point(0,0);

   double f = Df.at<double>(ys, xs);
   int i = Di.at<uint16_t>(ys, xs);

   cv::Point2d p = f * D[i + 1] + (1 - f) * D[i];
   p *= factor;

   return p;
}

void FrameWarpAligner::writeRealignmentInfo(std::string filename)
{
   if (Dstore.empty())
      return;
   
   std::ofstream os(filename);
   
   os << "Line Duration," << image_params.line_duration << "\n";
   os << "Interline Duration," << image_params.interline_duration << "\n";
   os << "Frame Duration," << image_params.frame_duration << "\n";
   os << "Interframe Duration," << image_params.interframe_duration << "\n";
   os << "Lines," << image_params.n_y << "\n";

   os << "Frame, UnalignedCorrelation, Correlation, Coverage";
   for (int j = 0; j < Dstore[0].size(); j++)
      os << ", p_" << j;
   os << "\n";
   for (int i = 0; i < Dstore.size(); i++)
   {
      os << i << "," << results[i].unaligned_correlation << "," << results[i].correlation << ", " << results[i].coverage;
      for (int j = 0; j < Dstore[i].size(); j++)
         os << ", " << Dstore[i][j].x << std::showpos << Dstore[i][j].y << std::noshowpos << "i";
      os << "\n";
   }
}
