#include "AligningReader.h"
//#ifdef __APPLE__
//#import <Foundation/Foundation.h>
//#endif

void AligningReader::alignFrames()
{
   if (!realign_params.use_realignment())
   {
      frame_aligner = nullptr;
      std::cout << "No realigmment requested\n";
      return;
   }

   if (frame_aligner == nullptr || frame_aligner->getType() != realign_params.type)
      frame_aligner = std::unique_ptr<AbstractFrameAligner>(AbstractFrameAligner::createFrameAligner(realign_params));

   getIntensityFrames();

   if ((frames.size() == 0) || terminate)
   {
      std::cout << "No frames\n";
      return;      
   }


   ImageScanParameters image_params = getImageScanParameters();
   
   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames((int)frames.size());

   int max_idx = reference_index;
   frame_aligner->setReference(max_idx, frames[max_idx]);

   realignment = std::vector<RealignmentResult>(frames.size());
   realignment_complete = false;

   intensity_normalisation = cv::Mat(frames[0].dims, frames[0].size.p, CV_16U, cv::Scalar(1));


   if (realignment_thread.joinable())
      realignment_thread.join();

   realignment_thread = std::thread(&AligningReader::alignFramesImpl, this);
}

void AligningReader::waitForAlignmentComplete()
{
   if (realignment_thread.joinable())
      realignment_thread.join();
}



void AligningReader::alignFramesImpl()
{
   try {


   auto fcn = [this](int i)
   {
      if (terminate) return;
      realignment[i] = frame_aligner->addFrame(i, frames[i]);

      {
         std::lock_guard<std::mutex> lk(realign_mutex);
         if ((realignment[i].correlation >= realign_params.correlation_threshold) &&
            (realignment[i].coverage >= realign_params.coverage_threshold))
               intensity_normalisation += realignment[i].mask;
      }

      std::cout << "*";
      realign_cv.notify_all();
   };


#ifdef __APPLE__
   //dispatch_queue_t c_queue = dispatch_queue_create("Frame queue", DISPATCH_QUEUE_CONCURRENT);
   //dispatch_apply(I, c_queue, ^fcn);
   for (int i = 0; i < frames.size(); i++)
      fcn(i);
#else
   #pragma omp parallel for schedule(dynamic,1)
   for (int i = 0; i < frames.size(); i++)
      fcn(i);
#endif


   realignment_complete = true;
   realign_cv.notify_all();

   }
   catch(cv::Exception e)
   {
      std::cout << "Error during realignment: " << e.what();
   }
}

void AligningReader::computeIntensityNormalisation()
{
   if (!realignment.empty())
   {
      // Get intensity
      cv::Mat intensity(intensity_normalisation.dims, intensity_normalisation.size.p, CV_16U, cv::Scalar(1));
      for (int i = 0; i < realignment.size(); i++)
      {
         if ((realignment[i].correlation >= realign_params.correlation_threshold) &&
             (realignment[i].coverage >= realign_params.coverage_threshold) &&
              realignment[i].done &&
             (!realignment[i].mask.empty()))
            intensity += realignment[i].mask;
      }
      intensity_normalisation = intensity / realignment.size();
   }
}