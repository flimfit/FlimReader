#pragma once

#include "AbstractFrameAligner.h"
#include <thread>
#include <mutex>

class AligningReader
{
public:

   const std::vector<RealignmentResult>& getRealignmentResults() { return realignment; }
   const std::unique_ptr<AbstractFrameAligner>& getFrameAligner() { return frame_aligner; }

   void setRealignmentParameters(RealignmentParameters realign_params_) { realign_params = realign_params_; }   
   cv::Mat getIntensityNormalisation() { return intensity_normalisation; }
   void setReferenceIndex(int reference_index_) { reference_index = reference_index_; }
   
   void alignFrames();   
   void waitForAlignmentComplete();

protected:

   virtual void getIntensityFrames() {};
   virtual ImageScanParameters getImageScanParameters() { return ImageScanParameters(); }

   void computeIntensityNormalisation();
   void alignFramesImpl();

   std::thread realignment_thread;
   std::mutex realign_mutex;
   std::condition_variable realign_cv;
   bool realignment_complete = false;
   bool terminate = false;

   RealignmentParameters realign_params;
   std::unique_ptr<AbstractFrameAligner> frame_aligner;
   std::vector<cv::Mat> frames;
   std::vector<RealignmentResult> realignment;
   int reference_index = 0;

   cv::Mat intensity_normalisation;
};
