#pragma once

#include "TransformInterpolator.h"
#include <functional>

class Range
{
public:
   int begin;
   int end;

   int interval() { return end - begin; }
};

class FrameWarpAligner : AbstractFrameAligner
{
public:

   bool empty() { return false; };
   void clear() { };


   void setReference(double frame_t, const cv::Mat& reference_)
   {
      reference = reference_;

      precomputeInterp();
      computeSteepestDecentImages();
      computeHessian();
   }

   void addFrame(double frame_t, const cv::Mat& frame)
   {
      int n_iter = 50;

      cv::Size size = reference.size();

      int n_px = size.area();
      cv::Mat wimg, error_img;
      cv::Mat sd(nD * 2, 1, CV_64F, cv::Scalar(0));
      cv::Mat delta_p(nD * 2, 1, CV_64F, cv::Scalar(0));
      std::vector<cv::Point2d> D(nD);

      for (int f = 0; f < n_iter; f++)
      {
         warpImage(frame, wimg, D);
         error_img = wimg - reference;

         double rms_error = 0;
         for (int p = 0; p < n_px; p++)
         {
            if (wimg.at<int16_t>(p) >= 0)
               rms_error += ((double)error_img.at<int16_t>(p)) * error_img.at<int16_t>(p);
            else
               error_img.at<int16_t>(p) = 0;
         }

         std::cout << f << " ===> rms error: " << sqrt(rms_error) << "\n";

         // TODO : check convergence

         steepestDecentUpdate(error_img, sd);
  
         cv::solve(H, sd, delta_p, cv::DECOMP_CHOLESKY);

         for (int i = 0; i < nD; i++)
         {
            D[i].x -= delta_p.at<double>(i);
            D[i].y -= delta_p.at<double>(i + nD);
         }

      }

      cv::transpose(wimg, wimg);

      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/image-out.png";
      cv::imwrite(ref_im_file, wimg);

   }

protected:



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


   void precomputeInterp()
   {
      cv::Size size = reference.size();
      
      double pixel_duration = line_duration / size.width;
      double frame_duration = size.height * interline_duration;

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
            double f = modf(t / frame_duration * (nD-1), &Di_xy);
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

      int max_VI_dW_dp = max_interval * 2;

      VI_dW_dp_x.clear();
      VI_dW_dp_x.resize(nD, std::vector<double>(max_VI_dW_dp));
      VI_dW_dp_y.clear();
      VI_dW_dp_y.resize(nD, std::vector<double>(max_VI_dW_dp));
   }

   void computeSteepestDecentImages()
   {
      auto size = reference.size();

      // Evaluate gradient of reference
      cv::Mat nabla_Tx, nabla_Ty;
      cv::Scharr(reference, nabla_Tx, CV_64F, 1, 0, 1.0/32.0);
      cv::Scharr(reference, nabla_Ty, CV_64F, 0, 1, 1.0/32.0);

      for (int i = 0; i < nD; i++)
      {
         int p_off = 0;
         if (i > 0)
         {
            int np = D_range[i-1].interval();
            for (int p = 0; p < np; p++)
            {
               int pos = p + D_range[i-1].begin;
               double jac = Df.at<double>(pos);
               VI_dW_dp_x[i][p] = nabla_Tx.at<double>(pos) * jac;
               VI_dW_dp_y[i][p] = nabla_Ty.at<double>(pos) * jac;
            }
            p_off = np;
         }
         if (i < (nD - 1))
         {
            int np = D_range[i].interval();
            for (int p = 0; p < np; p++)
            {
               int pos = p + D_range[i].begin;
               double jac = 1 - Df.at<double>(pos);
               VI_dW_dp_x[i][p + p_off] = nabla_Tx.at<double>(pos) * jac;
               VI_dW_dp_y[i][p + p_off] = nabla_Ty.at<double>(pos) * jac;
            }
         }
      }
   }

   double computeHessianEntry(int pi, int pj)
   {
      auto getV = [this](int i) -> const auto& { return (i < nD) ? VI_dW_dp_x[i] : VI_dW_dp_y[i - nD]; };

      double h = 0;

      int i = pi % nD;
      int j = pj % nD;

      if (j > i) return computeHessianEntry(pj, pi);
      if ((i - j) > 1) return 0;

      auto v1 = getV(pi);
      auto v2 = getV(pj);

      int p_off1 = 0;
      int p_off2 = 0;

      if (i == j)
      {
         int np = D_range[i].interval();
         if (i > 0)
            np += D_range[i - 1].interval();

         for (int p = 0; p < np; p++)
            h += v1[p] * v2[p];
      }
      else if (i > 0) // we know that i == j + 1 here
      {
         int p_off_j = (j == 0) ? 0 : D_range[j - 1].interval();
         int np = D_range[i - 1].interval();
         
         for (int p = 0; p < np; p++)
            h += v1[p] * v2[p + p_off2];
      }

      return h;
   }

   void computeHessian()
   {
      H = cv::Mat(2 * nD, 2 * nD, CV_64F, cv::Scalar(0));

      for (int pi = 0; pi < nD * 2; pi++)
         H.at<double>(pi, pi) += computeHessianEntry(pi, pi);
      
      for (int pi = 1; pi < nD * 2; pi++)
      {
         double h = computeHessianEntry(pi, pi - 1);
         H.at<double>(pi, pi - 1) = h;
         H.at<double>(pi - 1, pi) = h;
      }

      /* These don't seem to help
      for (int i = 0; i < nD; i++)
      {
         for (int j = std::max(0, i - 1); j < std::min(nD, i + 1); j++)
         {
            double h = computeHessianEntry(i, j + nD);
            H.at<double>(i, j + nD) += h;
            H.at<double>(j + nD, i) += h;

            h = computeHessianEntry(i + nD, j);
            H.at<double>(i + nD, j) += h;
            H.at<double>(j, i + nD) += h;
         }

      }
      */

      auto a = H.data;
      a = a;
   }

   void steepestDecentUpdate(const cv::Mat& error_img, cv::Mat& sd)
   {
      sd.setTo(0);

      for (int i = 0; i < nD; i++)
      {
         int p_off = 0;
         if (i > 0)
         {
            int np = D_range[i - 1].interval();
            for (int p = 0; p < np; p++)
            {
               int pos = p + D_range[i - 1].begin;
               sd.at<double>(i) += VI_dW_dp_x[i][p] * error_img.at<int16_t>(pos);
               sd.at<double>(i + nD) += VI_dW_dp_y[i][p] * error_img.at<int16_t>(pos);
            }
            p_off = np;
         }
         if (i < (nD - 1))
         {
            int np = D_range[i].interval();
            for (int p = 0; p < np; p++)
            {
               int pos = p + D_range[i].begin;
               double jac = 1 - Df.at<double>(pos);
               sd.at<double>(i) += VI_dW_dp_x[i][p + p_off] * error_img.at<int16_t>(pos);
               sd.at<double>(i + nD) += VI_dW_dp_y[i][p + p_off] * error_img.at<int16_t>(pos);
            }
         }
      }

      int a = 1;
   }

   void warpImage(const cv::Mat& img, cv::Mat& wimg, std::vector<cv::Point2d> D)
   {
      auto size = img.size(); 
      wimg = cv::Mat(size, CV_16S, cv::Scalar(-1));
      
      cv::Rect2i img_rect(cv::Point2i(0, 0), size);

      for (int y = 0; y < size.height; y++)
      {
         for (int x = 0; x < size.width; x++)
         {
            double f = Df.at<double>(y, x);
            int i = Di.at<uint16_t>(y, x);

            cv::Point2d p = f * D[i + 1] + (1 - f) * D[i];
            cv::Point2i loc(x + p.x, y + p.y);

            if (img_rect.contains(loc))
               wimg.at<int16_t>(y, x) = img.at<int16_t>(loc);
         }
      }
   }


   double line_duration = 100;
   double interline_duration = 101;
   std::vector<Range> D_range;

   cv::Mat Di;
   cv::Mat Df;
   cv::Mat H;

   int nD = 30;

   std::vector<std::vector<double>> VI_dW_dp_x, VI_dW_dp_y;
};