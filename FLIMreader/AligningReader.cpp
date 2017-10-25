#include "AligningReader.h"
#include <future>
#include "Cv3dUtils.h"

template <class F>
void par_for(int begin, int end, F fn) {
  std::atomic<int> idx;
  idx = begin;

  int num_cpus = std::thread::hardware_concurrency();
  num_cpus = std::min(num_cpus, end - begin);
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

void AligningReader::loadIntensityFrames()
{
   std::lock_guard<std::mutex> lk(frame_mutex);

   if (!frames.empty() || frame_thread.joinable())
      return;

   frame_thread = std::thread(&AligningReader::loadIntensityFramesImpl, this);
}

cv::Mat AligningReader::getIntensityFrame(int frame)
{
   if (frame >= getNumIntensityFrames())
      throw std::runtime_error("Invalid frame index");

   std::unique_lock<std::mutex> lk(frame_mutex);
   frame_cv.wait(lk, [&] { return ((frame < frames.size()) && (area(frames[frame]) > 0)); });
   return frames[frame];
}



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

   loadIntensityFrames();

   int n_frames = getNumIntensityFrames();
   reference_index = n_frames / 2;
   
   if ((n_frames == 0) || terminate)
   {
      std::cout << "No frames\n";
      return;      
   }

   ImageScanParameters image_params = getImageScanParameters();
   
   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames(n_frames);

   cv::Mat ref_frame = getIntensityFrame(reference_index);
   frame_aligner->setReference(reference_index, ref_frame);

   realignment = std::vector<RealignmentResult>(n_frames);
   realignment_complete = false;

   intensity_normalisation = cv::Mat(ref_frame.dims, ref_frame.size.p, CV_16U, cv::Scalar(1));

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
   int n_frames = getNumIntensityFrames();
   par_for(0, n_frames, [this](int i, int thread)
   {
      if (terminate) return;

      try 
      {
         realignment[i] = frame_aligner->addFrame(i, getIntensityFrame(i));
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