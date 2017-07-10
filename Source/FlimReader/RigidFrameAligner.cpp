#include "RigidFrameAligner.h"
#include <opencv2/imgproc.hpp>

RigidFrameAligner::RigidFrameAligner(RealignmentParameters params)
{
   realign_params = params;
   frame_transform.push_back(Transform(0.0));
}

bool RigidFrameAligner::empty()
{
   return frame_transform.size() <= 1;
}

void RigidFrameAligner::clear()
{
   frame_transform.clear();
}

void RigidFrameAligner::setReference(int frame_t, const cv::Mat& reference_)
{
   frame_transform.resize(n_frames+1);

   reference_.copyTo(reference);

   auto size = reference.size();
   cv::createHanningWindow(window, reference.size(), CV_32F);

   addTransform(0, Transform(0));
   addTransform(1, Transform(0.5*realign_params.frame_binning));

   centre = cv::Point2d(image_params.n_x, image_params.n_y) * 0.5;
   centre_binned = centre / realign_params.spatial_binning;

   cv::logPolar(reference, log_polar0, centre, 1.0, CV_WARP_FILL_OUTLIERS);
}

RealignmentResult RigidFrameAligner::addFrame(int frame_t, const cv::Mat& frame)
{
   Transform transform(realign_params.frame_binning*(frame_t+0.5));

   cv::Mat log_polari, rotatedi, refi, rotatedm;
   cv::Mat mask(frame.size(), CV_16U, cv::Scalar(1));

   if (realign_params.use_rotation())
   {
      cv::logPolar(frame, log_polari, centre, 1.0, CV_WARP_FILL_OUTLIERS);
      auto p = cv::phaseCorrelate(log_polar0, log_polari, window);
      double rotation = p.y * 90.0 / frame.size().height - 45.0;

      cv::Mat t = cv::getRotationMatrix2D(centre_binned, rotation, 1);
      cv::warpAffine(frame, rotatedi, t, frame.size());
      cv::warpAffine(mask, rotatedm, t, frame.size());

      transform.angle = rotation;
   }
   else
   {
      frame.copyTo(rotatedi);
      mask.copyTo(rotatedm);
   }

   reference.copyTo(refi);
   double response;
   auto p = cv::phaseCorrelate(refi, rotatedi, window, &response);

   transform.shift = p * realign_params.spatial_binning;

   cv::Mat m(2, 3, CV_32F, cv::Scalar(0));
   m.at<float>(0, 0) = 1;
   m.at<float>(1, 1) = 1;
   m.at<float>(0, 2) = (float) -p.x;
   m.at<float>(1, 2) = (float) -p.y;

   cv::Mat shifted, shiftedm;
   frame.copyTo(shifted);
   mask.copyTo(shiftedm);
   cv::warpAffine(rotatedi, shifted, m, frame.size());
   cv::warpAffine(rotatedm, shiftedm, m, frame.size());

   addTransform(frame_t,transform);

   RealignmentResult r;
   r.frame = frame;
   r.realigned = shifted;
   r.correlation = response;
   r.mask = shiftedm;

   return r; // TODO
}

void RigidFrameAligner::shiftPixel(int frame, double& x, double& y)
{
   cv::Mat pos(3, 1, CV_64F, cv::Scalar(0));
   cv::Mat tr_pos(3, 1, CV_64F, cv::Scalar(0));

   pos.at<double>(0) = x;
   pos.at<double>(1) = y;

   cv::Mat affine;
   cv::Point2d shift;

   double frame_t = frame + (y*image_params.interline_duration + x*image_params.pixel_duration) / image_params.frame_duration;

   getAffine(frame_t, affine, shift);
   tr_pos = affine * pos;

   x = (int) std::round(tr_pos.at<double>(0) - shift.x);
   y = (int) std::round(tr_pos.at<double>(1) - shift.y);
}


void RigidFrameAligner::getAffine(double frame, cv::Mat& affine, cv::Point2d& shift)
{
   assert(frame >= 0);

   if (frame == cache_frame)
   {
      affine = cache_affine;
      shift = cache_shift;
   }
   else if (frame == 0.0 || frame_transform.size() == 1)
   {
      affine = cv::Mat::eye(2, 3, CV_64F);
      shift = cv::Point2d(0, 0);
   }
   else if (frame >= frame_transform.back().frame)
   {
      interpolate(frame_transform.back(), frame_transform.back(), frame, affine, shift);
   }
   else
   {
      int idx = 0;
      while (frame > frame_transform[idx].frame)
         idx++;

      interpolate(frame_transform[idx - 1], frame_transform[idx], frame, affine, shift);
   }

   cache_frame = frame;
   cache_affine = affine;
   cache_shift = shift;
}

void RigidFrameAligner::interpolate(Transform& t1, Transform& t2, double frame, cv::Mat& affine, cv::Point2d& shift)
{
   double f = (frame - t1.frame) / (t2.frame - t1.frame);

   double angle = t2.angle * f + t1.angle * (1 - f);
   affine = cv::getRotationMatrix2D(centre, angle, 1.0);

   shift = t2.shift * f + t1.shift * (1 - f);
}


cv::Point2d RigidFrameAligner::getRigidShift(int frame)
{
   return frame_transform[frame + 1].shift;
}

void RigidFrameAligner::addTransform(int frame_t, Transform t)
{
   frame_transform[frame_t+1] = t;
}

