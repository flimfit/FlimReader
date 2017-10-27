#pragma once

#include "AbstractFrameAligner.h"
#include <thread>
#include <mutex>

class AligningReader
{
public:

   ~AligningReader();

   const std::vector<RealignmentResult>& getRealignmentResults() { return realignment; }
   const std::unique_ptr<AbstractFrameAligner>& getFrameAligner() { return frame_aligner; }

   void setRealignmentParameters(RealignmentParameters realign_params_) { realign_params = realign_params_; }   
   cv::Mat getIntensityNormalisation() { return intensity_normalisation; }
   void setReferenceIndex(int reference_index_) { reference_index = reference_index_; }
   
   void alignFrames();   
   void waitForAlignmentComplete();

protected:

   void loadIntensityFrames();
   cv::Mat getIntensityFrame(int frame);
   virtual cv::Mat getIntensityFrameImmediately(int frame) { return getIntensityFrame(frame); };

   virtual void loadIntensityFramesImpl() {};
   virtual int getNumIntensityFrames() { return 0; };
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
   std::vector<RealignmentResult> realignment;
   int reference_index = 0;

   cv::Mat intensity_normalisation;

   std::thread frame_thread;
   std::mutex frame_mutex;
   std::condition_variable frame_cv;
   std::vector<cv::Mat> frames;

   bool async_load_intensity_frames = false;
};
