#include "TransformInterpolator.h"
#include "FrameWarpAligner.h"

std::unique_ptr<AbstractFrameAligner> AbstractFrameAligner::createFrameAligner(RealignmentParameters params)
{
   return std::unique_ptr<AbstractFrameAligner>(new FrameWarpAligner(params));
   //return std::unique_ptr<TransformInterpolator>(new TransformInterpolator(params));
}