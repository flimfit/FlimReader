#pragma once

#include "TransformInterpolator.h"


TransformInterpolator::TransformInterpolator(RealignmentParameters params)
{
   realign_params = params;
   frame_transform.push_back(Transform(0.0));
}

bool TransformInterpolator::empty()
{
   return frame_transform.size() <= 1;
}

void TransformInterpolator::clear()
{
   frame_transform.clear();
   frame_transform.push_back(Transform(0.0));
}

void TransformInterpolator::setReference(int frame_t, const cv::Mat& reference_)
{
   reference = reference_;

   auto size = reference.size();
   cv::createHanningWindow(window, reference.size(), CV_32F);

   addTransform(Transform(0.5*realign_params.frame_binning));

   centre = cv::Point2d(image_params.n_x, image_params.n_y) * 0.5;
   centre_binned = centre / realign_params.spatial_binning;

   cv::logPolar(reference, log_polar0, centre, 1.0, CV_WARP_FILL_OUTLIERS);
}

void TransformInterpolator::addFrame(int frame_t, const cv::Mat& frame)
{
   Transform transform(realign_params.frame_binning*(frame_t+0.5));

   if (realign_params.use_rotation)
   {
      cv::logPolar(frame, log_polari, centre, 1.0, CV_WARP_FILL_OUTLIERS);
      auto p = cv::phaseCorrelate(log_polar0, log_polari, window);
      double rotation = p.y * 90.0 / frame.size().height - 45.0;

      cv::Mat t = cv::getRotationMatrix2D(centre_binned, rotation, 1);
      cv::warpAffine(frame, rotatedi, t, frame.size());

      transform.angle = rotation;
   }
   else
   {
      rotatedi = frame;
   }

   auto p = cv::phaseCorrelate(reference, rotatedi, window);

   transform.shift = p * realign_params.spatial_binning;

   addTransform(transform);
}

void TransformInterpolator::shiftPixel(int frame, int& x, int& y)
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


void TransformInterpolator::getAffine(double frame, cv::Mat& affine, cv::Point2d& shift)
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

void TransformInterpolator::interpolate(Transform& t1, Transform& t2, double frame, cv::Mat& affine, cv::Point2d& shift)
{
   double f = (frame - t1.frame) / (t2.frame - t1.frame);

   double angle = t2.angle * f + t1.angle * (1 - f);
   affine = cv::getRotationMatrix2D(centre, angle, 1.0);

   shift = t2.shift * f + t1.shift * (1 - f);
}

void TransformInterpolator::addTransform(Transform t)
{
   assert(t.frame > frame_transform.back().frame);
   frame_transform.push_back(t);
}

