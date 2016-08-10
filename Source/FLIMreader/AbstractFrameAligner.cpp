#include "TransformInterpolator.h"
#include "FrameWarpAligner.h"

std::unique_ptr<AbstractFrameAligner> AbstractFrameAligner::createFrameAligner(RealignmentParameters params)
{
   switch (params.type)
   {
   case RealignmentType::None:
      return nullptr;
   case RealignmentType::Warp:
      return std::unique_ptr<AbstractFrameAligner>(new FrameWarpAligner(params));
   default: // handles both Translation and RigidBody
      return std::unique_ptr<AbstractFrameAligner>(new TransformInterpolator(params));
   }
}