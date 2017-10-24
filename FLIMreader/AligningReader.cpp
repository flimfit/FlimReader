#include "AligningReader.h"
#include <future>

template <class F>
void par_for(int begin, int end, F fn) {
  std::atomic<int> idx;
  idx = begin;

  int num_cpus = std::thread::hardware_concurrency();
  std::vector<std::future<void>> futures(num_cpus);
  for (int cpu = 0; cpu != num_cpus; ++cpu) {
    futures[cpu] = std::async(
      std::launch::async,
      [cpu, &idx, end, &fn]() {
        for (;;) {
          int i = idx++;
          if (i >= end) break;
          fn(i, cpu);
        }
      }
    );
  }
  for (int cpu = 0; cpu != num_cpus; ++cpu) {
    futures[cpu].get();
  }
};

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

   par_for(0, frames.size(), [this](int i, int thread)
   {
      if (terminate) return;

      try 
      {
         realignment[i] = frame_aligner->addFrame(i, frames[i]);
      }
      catch (cv::Exception e)
      {
         std::cout << "Error during realignment: " << e.what();
      }

      {
         std::lock_guard<std::mutex> lk(realign_mutex);
         if ((realignment[i].correlation >= realign_params.correlation_threshold) &&
            (realignment[i].coverage >= realign_params.coverage_threshold))
               intensity_normalisation += realignment[i].mask;
      }

      std::cout << "*";
      realign_cv.notify_all();
   });

   realignment_complete = true;
   realign_cv.notify_all();

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