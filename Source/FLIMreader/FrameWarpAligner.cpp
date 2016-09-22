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

   precomputeInterp();
   computeSteepestDecentImages(reference);
   computeHessian();

   if (write_debug_images)
   {
      cv::Mat out;
      reference.convertTo(out, CV_8U, 5);
      cv::transpose(out, out);
      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/no-correction-" + std::to_string(frame_t) + ".png";
      //cv::imwrite(ref_im_file, out);

   }

   Dlast = cv::Point2d(0, 0);

   reference.convertTo(sum_1, CV_32F);
   reference.convertTo(sum_2, CV_32F);
}

void FrameWarpAligner::addFrame(int frame_t, const cv::Mat& frame)
{
   int max_n_iter = 200;

   cv::Mat wimg, wimg0, error_img, error_img0, error_img_trial, H_lm;
   cv::Mat sd(nD * 2, 1, CV_64F, cv::Scalar(0));
   cv::Mat delta_p(nD * 2, 1, CV_64F, cv::Scalar(0));
   std::vector<cv::Point2d> X(nD);
   std::vector<cv::Point2d> D(nD), Dtrial(nD);

   double last_rms_error = std::numeric_limits<double>::max();
   double delta = 1e-9;

   warpImage(frame, wimg0, D);
   double rms_error0 = computeErrorImage(wimg0, error_img0);

   if (Dstore.count(frame_t) == 1)
      D = Dstore[frame_t];
   else
      std::fill(D.begin(), D.end(), Dlast);

   warpImage(frame, wimg, D);
   double rms_error = computeErrorImage(wimg, error_img);

   if (rms_error0 < rms_error)
   {
      std::fill(D.begin(), D.end(), cv::Point(0,0));
      rms_error = rms_error0;
      wimg0.copyTo(wimg);
      error_img0.copyTo(error_img);
   }

   for (int f = 0; f < max_n_iter; f++)
   {
      steepestDecentUpdate(error_img, sd);

      H.copyTo(H_lm);
      for (int i = 0; i < (nD * 2); i++)
         H_lm.at<double>(i, i) *= (1 + delta);

      int a = cv::solve(H_lm, sd, delta_p, cv::DECOMP_CHOLESKY);

      for (int i = 0; i < nD; i++)
      {
         Dtrial[i].x = D[i].x - delta_p.at<double>(i);
         Dtrial[i].y = D[i].y - delta_p.at<double>(i + nD);
      }

      warpImage(frame, wimg, Dtrial);
      double rms_error_trial = computeErrorImage(wimg, error_img_trial);

      if (rms_error_trial <= rms_error) // good step
      {
         delta *= 0.2;
         rms_error = rms_error_trial;
         error_img_trial.copyTo(error_img);
         for (int i = 0; i < nD; i++)
            D[i] = Dtrial[i];
         //std::cout << f << " good step ===> " << sqrt(rms_error_trial) << "\n";

         if (((last_rms_error - rms_error) < 1e-10) && f > 20)
         {
            //std::cout << " objective function convergence criteria met \n";
            break;
         }

      }
      else // bad step
      {
         delta *= 5;
         //std::cout << f << " bad step ===> " << sqrt(rms_error_trial) << "\n";
      }

      last_rms_error = rms_error;
     
   }

   //std::cout << frame_t << ": " << sqrt(rms_error) << "\n";

   Dstore[frame_t] = D;
   Dlast = *(D.end()-2);
   
   
   if (write_debug_images)
   {
      sum_1 += frame;
      sum_2 += wimg;


      cv::Mat img;

      cv::transpose(frame, img);
      img.convertTo(img, CV_8U, 5);
      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/nocorrection-" + std::to_string(frame_t) + ".png";
      //cv::imwrite(ref_im_file, img);

      cv::transpose(wimg, wimg);
      wimg.convertTo(wimg, CV_8U, 5);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/corrected-" + std::to_string(frame_t) + ".png";
      //cv::imwrite(ref_im_file, wimg);

      double min, max;
      cv::minMaxIdx(sum_2, &min, &max);

      sum_1.convertTo(img, CV_8U, 350.0 / max);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/nocorrection-final.png";
      //cv::imwrite(ref_im_file, img);

      sum_2.convertTo(img, CV_8U, 350.0 / max);
      ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/corrected-final.png";
      //cv::imwrite(ref_im_file, img);
   }
   
}

double FrameWarpAligner::computeErrorImage(cv::Mat& wimg, cv::Mat& error_img)
{
   cv::Size size = reference.size();
   int n_px = size.area();

   error_img = wimg - reference;
   int n_include = 0;

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
   if (Dstore.count(frame_t) == 0)
      return;

   auto& D = Dstore[frame_t];
   cv::Point2d loc = warpPoint(D, x, y);

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

void FrameWarpAligner::computeSteepestDecentImages(const cv::Mat& frame)
{
   auto size = frame.size();

   // Evaluate gradient of reference
   cv::Mat nabla_Tx, nabla_Ty;
   cv::Scharr(frame, nabla_Tx, CV_64F, 1, 0, 1.0 / 32.0);
   cv::Scharr(frame, nabla_Ty, CV_64F, 0, 1, 1.0 / 32.0);

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
      return cv::Point(0,0);

   double f = Df.at<double>(ys, xs);
   int i = Di.at<uint16_t>(ys, xs);
   cv::Point2d p = f * D[i + 1] + (1 - f) * D[i];
   p *= factor;

   return cv::Point((int)p.x, (int)p.y);
}