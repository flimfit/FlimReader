#pragma once

#include "AbstractFrameAligner.h"
#include <thread>
#include <mutex>
#include <map>
#include "Cv3dUtils.h"
#include "CvCache.h"

class AligningReader
{
public:

   AligningReader();
   ~AligningReader();

   const std::map<size_t,RealignmentResult>& getRealignmentResults() { return frame_aligner->getRealignmentResults(); }
   const std::unique_ptr<AbstractFrameAligner>& getFrameAligner() { return frame_aligner; }

   void setRealignmentParameters(RealignmentParameters realign_params_) { realign_params = realign_params_; }   
   cv::Mat getIntensityNormalisation() { return intensity_normalisation; }
   cv::Mat getFloatIntensityNormalisation();
   void setReferenceIndex(int reference_index_) { reference_index = reference_index_; }
   virtual int getNumChannels() const = 0;
   
   virtual bool canReadBidirectionalScan() const = 0;
   virtual void setBidirectionalScan(bool bidirectional_scan = true) = 0;
   virtual bool getBidirectionalScan() const = 0;

   virtual bool canReadNumZ() const { return true; }
   virtual void setNumZ(int n_z_) {} // ignore usually
   virtual int getNumZ() const = 0;

   void setUseAllChannels();
   void setChannelsToUse(const std::vector<bool>& use_channel);

   void alignFrames();   
   void waitForAlignmentComplete();

   void setReferenceFrame(cv::Mat reference_frame_) { reference_frame = reference_frame_; }
   cv::Mat getReferenceFrame() { return reference_frame; }

protected:

   void waitForFrameReady(int frame);

   void loadIntensityFrames();
   CachedMat getIntensityFrame(int frame);
   virtual cv::Mat getIntensityFrameImmediately(int frame) { return getIntensityFrame(frame)->get(); };

   void setIntensityFrame(int frame_idx, const cv::Mat frame);

   virtual void loadIntensityFramesImpl() {}
   virtual int getNumIntensityFrames() { return 0; };
   virtual ImageScanParameters getImageScanParameters() { return ImageScanParameters(); }

   void computeIntensityNormalisation();
   void alignFramesImpl();

   cv::Mat reference_frame;

   bool terminate = false;

   std::thread realignment_thread;
   std::mutex realign_mutex;
   std::condition_variable realign_cv;
   bool realignment_complete = false;

   RealignmentParameters realign_params;
   std::unique_ptr<AbstractFrameAligner> frame_aligner;
   int reference_index = -1;

   cv::Mat intensity_normalisation;

   std::thread frame_thread;
   std::mutex frame_mutex;
   std::condition_variable frame_cv;
   std::map<size_t,CachedMat> frames;
   std::vector<bool> use_channel;

   bool async_load_intensity_frames = false;

   int64_t tick_count_start;
};