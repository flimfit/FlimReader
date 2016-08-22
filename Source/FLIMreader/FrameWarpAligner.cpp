#pragma once

#include "FrameWarpAligner.h"
#include <functional>

FrameWarpAligner::FrameWarpAligner(RealignmentParameters params)
{
   realign_params = params;
}


void FrameWarpAligner::setReference(int frame_t, const cv::Mat& reference_)
{
   reference = reference_;

   n_x_binned = image_params.n_x / realign_params.spatial_binning;
   n_y_binned = image_params.n_y / realign_params.spatial_binning;
   nD = realign_params.n_resampling_points;

   assert(n_x_binned == image_params.n_x);
   assert(n_y_binned == image_params.n_y);

   precomputeInterp();
   computeSteepestDecentImages();
   computeHessian();
   
   if (write_debug_images)
   {
      cv::Mat out;
      reference.convertTo(out, CV_8U, 5);
      cv::transpose(out, out);
      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/no-correction-" + std::to_string(frame_t) + ".png";
      cv::imwrite(ref_im_file, out);

   }

   reference.convertTo(sum_1, CV_32F);
   reference.convertTo(sum_2, CV_32F);
}

void FrameWarpAligner::addFrame(int frame_t, const cv::Mat& frame)
{
   int n_iter = 200;

   cv::Size size = reference.size();

   int n_px = size.area();
   cv::Mat wimg, error_img;
   cv::Mat sd(nD * 2, 1, CV_64F, cv::Scalar(0));
   cv::Mat delta_p(nD * 2, 1, CV_64F, cv::Scalar(0));
   std::vector<cv::Point2d> X(nD);
   std::vector<cv::Point2d> D(nD);

   double last_rms_error = std::numeric_limits<double>::max();

   for (int f = 0; f < n_iter; f++)
   {
      warpImage(frame, wimg, D);
      error_img = wimg - reference;

      double rms_error = 0;
      for (int p = 0; p < n_px; p++)
      {
         if (wimg.at<float>(p) >= 0)
         {
            rms_error += error_img.at<float>(p) * error_img.at<float>(p);
         }
         else
         {
            error_img.at<float>(p) = 0;
            wimg.at<float>(p) = 0;
         }
      }

      std::cout << f << " ===> rms error: " << sqrt(rms_error) << "\n";

      // TODO : check convergence
      if ((last_rms_error - rms_error) / rms_error < 1e-4)
         break;
      last_rms_error = rms_error;

      steepestDecentUpdate(error_img, sd);

      int a = cv::solve(H, sd, delta_p, cv::DECOMP_CHOLESKY);

      for (int i = 0; i < nD; i++)
      {
         D[i].x -= delta_p.at<double>(i);
         D[i].y -= delta_p.at<double>(i + nD);
      }

   }

   Dstore[frame_t] = D;
   
   
   if (write_debug_images)
   {
      sum_1 += frame;
      sum_2 += wimg;


      cv::Mat img;

      cv::transpose(frame, img);
      img.convertTo(img, CV_8U, 5);
      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/nocorrection-" + std::to_string(frame_t) + ".png";
      cv::imwrite(ref_im_file, img);

      cv::transpose(wimg, wimg);
      wimg.convertTo(wimg, CV_8U, 5);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/corrected-" + std::to_string(frame_t) + ".png";
      cv::imwrite(ref_im_file, wimg);

      double min, max;
      cv::minMaxIdx(sum_2, &min, &max);

      sum_1.convertTo(img, CV_8U, 350.0 / max);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/nocorrection-final.png";
      cv::imwrite(ref_im_file, img);

      sum_2.convertTo(img, CV_8U, 350.0 / max);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/corrected-final.png";
      cv::imwrite(ref_im_file, img);
   }
   
}


void FrameWarpAligner::shiftPixel(int frame_t, int& x, int& y)
{
   if (Dstore.count(frame_t) == 0)
      return;

   auto& D = Dstore[frame_t];
   cv::Point loc = warpPoint(D, x, y);

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
   cv::Size size = reference.size();

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

         Di.at<uint16_t>(y, x) = i;
         Df.at<double>(y, x) = f;

         if (i > last_i)
         {
            if (last_i >= 0)
               D_range[last_i].end = y*size.width + x - 1;
            D_range[i].begin = y*size.width + x;
            last_i = i;
         }
      }
   }

   D_range[i].end = size.area() - 1;

   int max_interval = 0;
   for (int i = 0; i < nD; i++)
      if (D_range[i].interval() > max_interval)
         max_interval = D_range[i].interval();

   int max_VI_dW_dp = size.area(); //max_interval * 2;

   VI_dW_dp_x.clear();
   VI_dW_dp_x.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
   VI_dW_dp_y.clear();
   VI_dW_dp_y.resize(nD, std::vector<double>(max_VI_dW_dp, 0.0));
}

void FrameWarpAligner::computeSteepestDecentImages()
{
   auto size = reference.size();

   // Evaluate gradient of reference
   cv::Mat nabla_Tx, nabla_Ty;
   cv::Scharr(reference, nabla_Tx, CV_64F, 1, 0, 1.0 / 32.0);
   cv::Scharr(reference, nabla_Ty, CV_64F, 0, 1, 1.0 / 32.0);

   #pragma omp parallel for
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
   
   #pragma omp parallel for
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
   auto getV = [this](int i) -> const auto& { return (i < nD) ? VI_dW_dp_x[i] : VI_dW_dp_y[i - nD]; };

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
   H = cv::Mat(2 * nD, 2 * nD, CV_64F, cv::Scalar(0));
   
   #pragma omp parallel for
   for (int pi = 0; pi < nD * 2; pi++)
      H.at<double>(pi, pi) += computeHessianEntry(pi, pi);

   #pragma omp parallel for
   for (int pi = 1; pi < nD * 2; pi++)
   {
      double h = computeHessianEntry(pi, pi - 1);
      H.at<double>(pi, pi - 1) = h;
      H.at<double>(pi - 1, pi) = h;
   }
   
   #pragma omp parallel for
   for (int i = 0; i < nD; i++)
   {
      for (int j = std::max(0, i - 1); j < std::min(nD, i + 1); j++)
      {
         double h = computeHessianEntry(i, j + nD);
         H.at<double>(i, j + nD) += h;
         H.at<double>(j + nD, i) += h;
      }
   }
   

}

void FrameWarpAligner::steepestDecentUpdate(const cv::Mat& error_img, cv::Mat& sd)
{
   sd.setTo(0);

   // we are ignoring stride here, ok as set up
   float* err_ptr = reinterpret_cast<float*>(error_img.data);
   double* sd_ptr_x = reinterpret_cast<double*>(sd.data);
   double* sd_ptr_y = sd_ptr_x + nD;


   for (int i = 1; i < nD; i++)
   {
      int p0 = D_range[i - 1].begin;
      int p1 = D_range[i - 1].end;
      for (int p = p0; p < p1; p++)
      {
         sd_ptr_x[i] += VI_dW_dp_x[i][p] * err_ptr[p];
         sd_ptr_y[i] += VI_dW_dp_y[i][p] * err_ptr[p];
      }
   }
   for (int i = 0; i < (nD - 1); i++)
   {
      int p0 = D_range[i].begin;
      int p1 = D_range[i].end;
      for (int p = p0; p < p1; p++)
      {
         sd_ptr_x[i] += VI_dW_dp_x[i][p] * err_ptr[p];
         sd_ptr_y[i] += VI_dW_dp_y[i][p] * err_ptr[p];
      }
   }
}

void FrameWarpAligner::warpImage(const cv::Mat& img, cv::Mat& wimg, const std::vector<cv::Point2d>& D)
{
   auto size = img.size();
   wimg = cv::Mat(size, CV_32F, cv::Scalar(-1));

   cv::Rect2i img_rect(cv::Point2i(0, 0), size);

   for (int y = 0; y < size.height; y++)
      for (int x = 0; x < size.width; x++)
      {
         cv::Point2i loc = warpPoint(D, x, y, realign_params.spatial_binning);
         loc.x += x;
         loc.y += y;

         if (img_rect.contains(loc))
            wimg.at<float>(y, x) = img.at<float>(loc);
      }
}

cv::Point FrameWarpAligner::warpPoint(const std::vector<cv::Point2d>& D, int x, int y, int spatial_binning)
{
   double factor = ((double)realign_params.spatial_binning) / spatial_binning;
   int xs = (int) (x / factor);
   int ys = (int) (y / factor);

   if ((xs < 0) || (xs >= n_x_binned) || (ys < 0) || (ys >= n_y_binned))
      return cv::Point(x,y);

   double f = Df.at<double>(ys, xs);
   int i = Di.at<uint16_t>(ys, xs);
   cv::Point2d p = f * D[i + 1] + (1 - f) * D[i];
   p *= factor;

   return cv::Point((int)p.x, (int)p.y);
}