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

void interpolatePoint3d(const std::vector<cv::Point3d>& Ds, std::vector<cv::Point3d>& D)
{
   if (Ds.empty())
      return;

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
   auto vd_z = std::vector<double>(nDs);

   auto ui = std::vector<double>(nD);
   auto vi_x = std::vector<double>(nDs);
   auto vi_y = std::vector<double>(nDs);
   auto vi_z = std::vector<double>(nDs);

   for (int i = 0; i < nDs; i++)
   {
      ud[i] = i / (nDs - 1.0);
      vd_x[i] = Ds[i].x;
      vd_y[i] = Ds[i].y;
      vd_z[i] = Ds[i].z;
   }

   for (int i = 0; i < nD; i++)
      ui[i] = i / (nD - 1.0);

   pwl_interp_1d(ud, vd_x, ui, vi_x);
   pwl_interp_1d(ud, vd_y, ui, vi_y);
   pwl_interp_1d(ud, vd_z, ui, vi_z);

   for (int i = 0; i < nD; i++)
   {
      D[i].x = vi_x[i];
      D[i].y = vi_y[i];
      D[i].z = vi_z[i];
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

cv::Mat FrameWarpAligner::reshapeForOutput(cv::Mat& m)
{
   if (dims[Z] == 1)
      return m.reshape(0, 2, &dims[Y]);
   else
      return m;
}

cv::Mat FrameWarpAligner::reshapeForProcessing(cv::Mat& m)
{
   return m.reshape(0, 3, dims.data());
}


void FrameWarpAligner::smoothStack(const cv::Mat& in, cv::Mat& out)
{

   if (realign_params.smoothing > 0.0)
   {
      out = cv::Mat(dims, CV_32F);
      cv::Range range[] = { cv::Range::all(), cv::Range::all(), cv::Range::all() };
      for(int i=0; i<dims[Z]; i++)
      {
         range[0] = cv::Range(i,i+1);
         cv::Mat in_i = in(range).reshape(0, 2, &dims[1]);
         cv::Mat out_i = out(range).reshape(0, 2, &dims[1]);
         
         cv::GaussianBlur(in_i, out_i, cv::Size(0, 0), realign_params.smoothing, 1);
      }
   }
   else
   {
      out = in;
   }
   
}


void FrameWarpAligner::setReference(int frame_t, const cv::Mat& reference_)
{
   rigid_aligner->setNumberOfFrames(n_frames);

   n_x_binned = image_params.n_x / realign_params.spatial_binning;
   n_y_binned = image_params.n_y / realign_params.spatial_binning;
   dims = {image_params.n_z, n_x_binned, n_y_binned};

   reference_.copyTo(reference);
   
   reference = reshapeForProcessing(reference);   
   smoothStack(reference, smoothed_reference);

   cv::Mat f = downsample(reference, 4);
   rigid_aligner->setReference(frame_t, f);

   nD = realign_params.n_resampling_points;

   precomputeInterp();
   computeSteepestDecentImages(smoothed_reference);
   computeHessian();

   Dlast = cv::Point3d(0, 0, 0);
}

void FrameWarpAligner::reprocess()
{
   sum_2 /= (double) results.size();
   setReference(0, sum_2);

//   for (auto iter : results)
//      addFrame(iter.first, iter.second.frame);
}

void D2col(const std::vector<cv::Point3d> &D, column_vector& col, int n_dim)
{
   int nD = D.size();
   col.set_size(nD * n_dim);

   for (int i = 0; i < nD; i++)
   {
      col(i) = D[i].x;
      col(i + nD) = D[i].y;
      if (n_dim == 3)
            col(i + 2*nD) = D[i].z;
   }
}

void col2D(const column_vector& col, std::vector<cv::Point3d> &D, int n_dim)
{
   int nD = col.size() / n_dim;
   D.resize(nD);

   for (int i = 0; i < nD; i++)
   {
      D[i].x = col(i);
      D[i].y = col(i + nD);
      if (n_dim == 3) D[i].z = col(i + nD);
   }
}


RealignmentResult FrameWarpAligner::addFrame(int frame_t, const cv::Mat& raw_frame_)
{
   cv::Mat raw_frame, frame;
   raw_frame_.copyTo(raw_frame);
   
   raw_frame = reshapeForProcessing(raw_frame);
   smoothStack(raw_frame, frame);

   auto model = OptimisationModel(this, frame, raw_frame);

   std::vector<column_vector> starting_point(2, column_vector(nD * n_dim));

   // zero starting point
   std::fill(starting_point[0].begin(), starting_point[0].end(), 0);

   // last starting point
   std::vector<cv::Point3d> D(nD, cv::Point3d(0,0,0));
   if (Dstore.count(frame_t) == 1)
      interpolatePoint3d(Dstore[frame_t], D);
   D2col(D, starting_point[1], n_dim);

   // rigid starting point // to do for 3d
   /*
   cv::Mat ff = downsample(frame, 4);
   rigid_aligner->addFrame(frame_t, ff);
   cv::Point2d rigid_shift = rigid_aligner->getRigidShift(frame_t);
   cv::Point3d rigid_shift_3d(rigid_shift.x, rigid_shift.y, 0);
   std::vector<cv::Point3d> D_rigid(nD, rigid_shift_3d);
   D2col(D_rigid, starting_point[2], n_dim);
   */

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

   col2D(x, D, n_dim);
   Dstore[frame_t] = D;
   Dlast = *(D.end() - 2);

   std::cout << "*";

   cv::Mat warped_smoothed = model.getWarpedImage(x);
   cv::Mat warped = model.getWarpedRawImage(x);
   cv::Mat mask = model.getMask(x);

   cv::Mat m;
   cv::compare(mask, 0, m, cv::CMP_GT);
   

   RealignmentResult r;
   r.frame = reshapeForOutput(raw_frame);
   r.realigned = reshapeForOutput(warped);
   r.mask = reshapeForOutput(mask);
   r.correlation = correlation(warped_smoothed, smoothed_reference, m);
   r.unaligned_correlation = correlation(frame, smoothed_reference, m);
   r.coverage = ((double)cv::countNonZero(mask)) / mask.size().area();

   cv::Mat intensity_preserving(frame.size(), CV_16U, cv::Scalar(0));
   if (r.correlation >= realign_params.correlation_threshold && r.coverage >= realign_params.coverage_threshold)
      warpImageIntensityPreserving(raw_frame, intensity_preserving, D);
   r.realigned_preserving = reshapeForOutput(intensity_preserving);

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
   std::vector<cv::Point3d> D;
   cv::Mat warped_image, error_image;
   
   // Get displacement matrix from warp parameters
   col2D(x, D, aligner->n_dim);

   aligner->warpImage(frame, warped_image, D, -1);
   double rms_error = aligner->computeErrorImage(warped_image, error_image);
   return rms_error;
}

void OptimisationModel::get_derivative_and_hessian(const column_vector& x, column_vector& der, general_matrix& hess) const
{
   std::vector<cv::Point3d> D;
   cv::Mat warped_image, error_image;

   // Get displacement matrix from warp parameters
   col2D(x, D, aligner->n_dim);

   aligner->warpImage(frame, warped_image, D, -1);
   double rms_error = aligner->computeErrorImage(warped_image, error_image);

   aligner->computeJacobian(error_image, der);
   hess = aligner->H;
}

cv::Mat OptimisationModel::getMask(const column_vector& x)
{
   std::vector<cv::Point3d> D;
   col2D(x, D, aligner->n_dim);
   
   cv::Mat mask;
   aligner->warpCoverage(mask, D);
   return mask;
}

cv::Mat OptimisationModel::getWarpedRawImage(const column_vector& x)
{
   std::vector<cv::Point3d> D;
   col2D(x, D, aligner->n_dim);
   cv::Mat warped_image;
   aligner->warpImage(raw_frame, warped_image, D);
   return warped_image;
}

cv::Mat OptimisationModel::getWarpedImage(const column_vector& x)
{
	std::vector<cv::Point3d> D;
	col2D(x, D, aligner->n_dim);
	cv::Mat warped_image;
	aligner->warpImage(frame, warped_image, D);
	return warped_image;
}

double FrameWarpAligner::computeErrorImage(cv::Mat& wimg, cv::Mat& error_img)
{
   int n_px = dims[X] * dims[Y] * dims[Z];

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

void FrameWarpAligner::shiftPixel(int frame_t, double& x, double& y, double& z)
{
   if (!realign_params.use_realignment())
      return;

   assert(Dstore.size() > frame_t);

   auto& D = Dstore[frame_t];
   cv::Point3d loc = warpPoint(D, x, y, z);

   x -= loc.x;
   y -= loc.y;
   z -= loc.z;
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
   double pixel_duration = image_params.pixel_duration * realign_params.spatial_binning;
   double frame_duration = image_params.frame_duration;
   double interline_duration = image_params.interline_duration * realign_params.spatial_binning;

   D_range.resize(nD);

   Di = cv::Mat(dims, CV_16U);
   Df = cv::Mat(dims, CV_64F);
   
   double Di_xy;
   int i;
   int last_i = -1;

   for (int z = 0; z < dims[Z]; z++)
   {
      for (int y = 0; y < dims[Y]; y++)
      {
         for (int x = 0; x < dims[X]; x++)
         {
            double t = z * frame_duration + y * interline_duration + x * pixel_duration;
            double f = modf(t / frame_duration * (nD - 1), &Di_xy);
            i = (int)Di_xy;

            int x_true = x;
            if (image_params.bidirectional && ((y % 2) == 1))
               x_true = dims[X] - x - 1;

            Di.at<uint16_t>(z, y, x_true) = i;
            Df.at<double>(z, y, x_true) = f;

            if (i > last_i)
            {
               D_range[i].begin = (z*dims[Y] + y)*dims[X] + x_true;
               if (last_i >= 0)
                  D_range[last_i].end = D_range[i].begin - 1;
               last_i = i;
            }
         }
      }
   }

   D_range[i].end = dims[X]*dims[Y]*dims[Z] - 1;

   int max_interval = 0;
   for (int i = 0; i < nD; i++)
   {
      int interval = D_range[i].interval();
      if (interval > max_interval)
         max_interval = D_range[i].interval();

   }

   int max_VI_dW_dp = dims[X] * dims[Y] * dims[Z]; //max_interval * 2;

   VI_dW_dp_x.clear();
   VI_dW_dp_y.clear();
   VI_dW_dp_z.clear();

   VI_dW_dp_x.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
   VI_dW_dp_y.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
   VI_dW_dp_z.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
}

void FrameWarpAligner::computeSteepestDecentImages(const cv::Mat& frame)
{
   // Evaluate gradient of reference
   cv::Mat nabla_Tx(dims, CV_64F);
   cv::Mat nabla_Ty(dims, CV_64F);
   cv::Mat nabla_Tz(dims, CV_64F);


   cv::Range range[] = { cv::Range::all(), cv::Range::all(), cv::Range::all() };
   for(int i=0; i<dims[Z]; i++)
   {
      range[0] = cv::Range(i,i+1);
      cv::Mat frame_i = frame(range).reshape(0, 2, &dims[1]);
      cv::Mat nabla_Tx_i = nabla_Tx(range).reshape(0, 2, &dims[1]);
      cv::Mat nabla_Ty_i = nabla_Ty(range).reshape(0, 2, &dims[1]);

      cv::Scharr(frame_i, nabla_Tx_i, CV_64F, 1, 0, 1.0 / 32.0);
      cv::Scharr(frame_i, nabla_Ty_i, CV_64F, 0, 1, 1.0 / 32.0);
   }   

   //#pragma omp parallel for
   for (int i = 1; i < nD; i++)
   {
      int p0 = D_range[i - 1].begin;
      int p1 = D_range[i - 1].end;
      for (int p = p0; p < p1; p++)
      {
         double jac = Df.at<double>(p);
         VI_dW_dp_x[i][p] = nabla_Tx.at<double>(p) * jac;
         VI_dW_dp_y[i][p] = nabla_Ty.at<double>(p) * jac;

         if (n_dim == 3)
            VI_dW_dp_z[i][p] = nabla_Tz.at<double>(p) * jac;
      }
   }
   
   //#pragma omp parallel for
   for(int i = 0; i < (nD - 1); i++)
   {
      int p0 = D_range[i].begin;
      int p1 = D_range[i].end;
      for (int p = p0; p < p1; p++)
      {
         double jac = 1 - Df.at<double>(p);
         VI_dW_dp_x[i][p] = nabla_Tx.at<double>(p) * jac;
         VI_dW_dp_y[i][p] = nabla_Ty.at<double>(p) * jac;

         if (n_dim == 3)
            VI_dW_dp_z[i][p] = nabla_Tz.at<double>(p) * jac;
      }
   }
}

double FrameWarpAligner::computeHessianEntry(int pi, int pj)
{
   auto getV = [this](int i) -> const std::vector<double>& { 
      if (i < nD)
         return VI_dW_dp_x[i];
      else if (i < 2*nD)
         return VI_dW_dp_y[i - nD];
      else
         return VI_dW_dp_z[i - 2*nD];
      };

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
//   H = cv::Mat(2 * nD, 2 * nD, CV_64F, cv::Scalar(0));
   
   H.set_size(2 * nD, 2 * nD);
   std::fill(H.begin(), H.end(), 0);

  // #pragma omp parallel for
   for (int pi = 0; pi < nD * 2; pi++)
      H(pi, pi) += computeHessianEntry(pi, pi);

   //#pragma omp parallel for
   for (int pi = 1; pi < nD * 2; pi++)
   {
      double h = computeHessianEntry(pi, pi - 1);
      H(pi, pi - 1) = h;
      H(pi - 1, pi) = h;
   }
   
  // #pragma omp parallel for
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
   jac.set_size(nD * n_dim);
   std::fill(jac.begin(), jac.end(), 0);

   // we are ignoring stride here, ok as set up
   float* err_ptr = reinterpret_cast<float*>(error_img.data);

   for (int i = 1; i < nD; i++)
   {
      int p0 = D_range[i - 1].begin;
      int p1 = D_range[i - 1].end;
      for (int p = p0; p < p1; p++)
      {
         jac(i) += VI_dW_dp_x[i][p] * err_ptr[p]; // x 
         jac(i+nD) += VI_dW_dp_y[i][p] * err_ptr[p]; // y
         if (n_dim == 3)
            jac(i+2*nD) += VI_dW_dp_z[i][p] * err_ptr[p]; // y         
      }
   }
   for (int i = 0; i < (nD - 1); i++)
   {
      int p0 = D_range[i].begin;
      int p1 = D_range[i].end;
      for (int p = p0; p < p1; p++)
      {
         jac(i) += VI_dW_dp_x[i][p] * err_ptr[p]; // x
         jac(i+nD) += VI_dW_dp_y[i][p] * err_ptr[p]; // y
         if (n_dim == 3)
            jac(i+2*nD) += VI_dW_dp_z[i][p] * err_ptr[p]; // y
         
      }
   }
}


void FrameWarpAligner::warpImage(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point3d>& D, int invalid_value)
{
   wimg = cv::Mat(dims, CV_32F, cv::Scalar(invalid_value));
   
   for (int z = 0; z < dims[Z]; z++)
      for (int y = 0; y < dims[Y]; y++)
         for (int x = 0; x < dims[X]; x++)
         {
            cv::Point3d loc = warpPoint(D, x, y, z, realign_params.spatial_binning);
            loc += cv::Point3d(x,y,z);

            cv::Point3i loc0(floor(loc.x), floor(loc.y), floor(loc.z));
            cv::Point3i loc1 = loc0 + cv::Point3i(1, 1, 1);
            cv::Point3d locf(loc.x - loc0.x, loc.y - loc0.y, loc.z - loc0.z);

            if (isValidPoint(loc0))
            {
               wimg.at<float>(z, y, x) =
                  (1 - locf.z) * (
                     img.at<float>(loc0.z, loc0.y, loc0.x) * (1 - locf.y) * (1 - locf.x) + 
                     img.at<float>(loc0.z, loc1.y, loc0.x) * (    locf.y) * (1 - locf.x) + 
                     img.at<float>(loc0.z, loc0.y, loc1.x) * (1 - locf.y) * (    locf.x) + 
                     img.at<float>(loc0.z, loc1.y, loc1.x) * (    locf.y) * (    locf.x)
                  );
               if (n_dim == 3)
                  wimg.at<float>(z, y, x) +=
                     locf.z * (
                        img.at<float>(loc0.z, loc0.y, loc0.x) * (1 - locf.y) * (1 - locf.x) + 
                        img.at<float>(loc0.z, loc1.y, loc0.x) * (    locf.y) * (1 - locf.x) + 
                        img.at<float>(loc0.z, loc0.y, loc1.x) * (1 - locf.y) * (    locf.x) + 
                        img.at<float>(loc0.z, loc1.y, loc1.x) * (    locf.y) * (    locf.x)
                     );
            }
               
         }
}

void FrameWarpAligner::warpImageIntensityPreserving(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point3d>& D)
{
   wimg = cv::Mat(dims, CV_16U, cv::Scalar(0));

   for (int z = 0; z < dims[Z]; z++)
      for (int y = 0; y < dims[Y]; y++)
         for (int x = 0; x < dims[X]; x++)
         {
            cv::Point3d loc = warpPoint(D, x, y, z, realign_params.spatial_binning);

            cv::Vec<int,3> locr = { (int)round(z - loc.z),
                                    (int)round(y - loc.y),
                                    (int)round(x - loc.x) };              
               
            if (isValidPoint(locr))
               wimg.at<uint16_t>(locr) += img.at<float>(z, y, x);
         }
}


void FrameWarpAligner::warpCoverage(cv::Mat& coverage, const std::vector<cv::Point3d>& D)
{
   coverage = cv::Mat(dims, CV_16U, cv::Scalar(0));

   for(int z = 0; z < dims[Z]; z++)
   for (int y = 0; y < dims[Y]; y++)
      for (int x = 0; x < dims[X]; x++)
      {
         cv::Point3d loc = warpPoint(D, x, y, z, realign_params.spatial_binning);

         cv::Vec<int,3> locr = { (int)round(z - loc.z),
                                 (int)round(y - loc.y),
                                 (int)round(x - loc.x) };
                  
         if (isValidPoint(locr))
            coverage.at<uint16_t>(locr)++;
      }
}

cv::Point3d FrameWarpAligner::warpPoint(const std::vector<cv::Point3d>& D, int x, int y, int z, int spatial_binning)
{
   double factor = ((double)realign_params.spatial_binning) / spatial_binning;
   int xs = (int) (x / factor);
   int ys = (int) (y / factor);

   if (   (xs < 0) || (xs >= dims[X]) 
       || (ys < 0) || (ys >= dims[Y])
       || (z  < 0) || (z  >= dims[Z]))
      return cv::Point3d(0,0,0);

   double* Df_d = reinterpret_cast<double*>(Df.data);
   uint16_t* Di_d = reinterpret_cast<uint16_t*>(Di.data);
   int loc = (z * dims[Y] + ys) * dims[X] + xs;

   double f = Df_d[loc];
   int i = Di_d[loc];

   if (i >= (nD-1))
      int a = 1;

   cv::Point3d p = f * D[i + 1] + (1 - f) * D[i];
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
