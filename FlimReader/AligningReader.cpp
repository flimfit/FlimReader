#include "AligningReader.h"
#include <future>
#include "Cv3dUtils.h"
#include "CvCache.h"
#include "Cache_impl.h"

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

AligningReader::AligningReader()
{
   frame_aligner = std::make_unique<NullFrameAligner>();
}

AligningReader::~AligningReader()
{
   terminate = true;
   if (realignment_thread.joinable())
      realignment_thread.join();

   if (frame_thread.joinable())
      frame_thread.join();
}

void AligningReader::setUseAllChannels()
{
   use_channel = std::vector<bool>(getNumChannels(), true);
}

void AligningReader::setChannelsToUse(const std::vector<bool>& use_channel_)
{
   setUseAllChannels();
   for(int i=0; i<std::min((size_t) getNumChannels(), use_channel_.size()); i++)
      use_channel[i] = use_channel_[i];
}

void AligningReader::loadIntensityFrames()
{
   {
      std::lock_guard<std::mutex> lk(frame_mutex);

      if (!frames.empty())
         return;
   }

   if (async_load_intensity_frames)
   {
      if (frame_thread.joinable())
         frame_thread.join();
      frame_thread = std::thread(&AligningReader::loadIntensityFramesImpl, this);
   }
   else
   {
      loadIntensityFramesImpl();
   }
}

CachedMat AligningReader::getIntensityFrame(int frame)
{
   if (frame >= getNumIntensityFrames())
      throw std::runtime_error("Invalid frame index");

   std::unique_lock<std::mutex> lk(frame_mutex);
   frame_cv.wait(lk, [&] { return frames.count(frame) > 0; });
   return frames[frame];
}


void AligningReader::setIntensityFrame(int frame_idx, const cv::Mat frame)
{
   Cache<cv::Mat>* cache = Cache<cv::Mat>::getInstance();

   cv::Mat frame_cpy;
   frame.copyTo(frame_cpy);

   {
      std::lock_guard<std::mutex> lk(frame_mutex);
      frames[frame_idx] = cache->add(frame_cpy);
   }
   frame_cv.notify_all();
}



void AligningReader::alignFrames()
{
   if (frame_aligner->getType() != realign_params.type)
      frame_aligner = std::unique_ptr<AbstractFrameAligner>(AbstractFrameAligner::createFrameAligner(realign_params));

   if (!realign_params.use_realignment())
   {
      std::cout << "No realigmment requested\n";
      return;
   }

   loadIntensityFrames();

   int n_frames = getNumIntensityFrames();
   
   if ((n_frames == 0) || terminate)
   {
      std::cout << "No frames\n";
      return;      
   }

   ImageScanParameters image_params = getImageScanParameters();
   

   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames(n_frames);

   if (reference_frame.empty())
   {
      if (reference_index == -1)
      {
         switch (realign_params.default_reference_frame)
         {
         case DefaultReferenceFrame::FirstFrame:
            reference_index = 0;
            break;
         case DefaultReferenceFrame::MiddleFrame:
            reference_index = n_frames / 2;
            break;
         case DefaultReferenceFrame::LastFrame:
            reference_index = n_frames - 1;
            break;
         }
      }
      reference_frame = getIntensityFrameImmediately(reference_index);
   }


   tick_count_start = cv::getTickCount();

   frame_aligner->setReference(reference_index, reference_frame);

//   realignment.clear();
   realignment_complete = false;

   intensity_normalisation = cv::Mat(reference_frame.dims, reference_frame.size.p, CV_32F, cv::Scalar(0));

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
         frame_aligner->addFrame(i, getIntensityFrame(i));
      }
      catch (cv::Exception e)
      {
         std::cout << "Error during realignment: " << e.what();
      }

         auto result = frame_aligner->getRealignmentResult(i);
         if (result.useFrame(realign_params))
            intensity_normalisation += result.mask->get();

      realign_cv.notify_all();
   });

   frame_aligner->clearTemp();

   realignment_complete = true;
   realign_cv.notify_all();

   int64 duration_ticks = cv::getTickCount() - tick_count_start;
   double duration_s = duration_ticks / cv::getTickFrequency();

   std::cout << "Realignment took: " << duration_s << "s\n";

}

void AligningReader::computeIntensityNormalisation()
{
   // Get intensity
   cv::Mat intensity = intensity_normalisation.clone();
   intensity = 1.0;

   for (auto& r : frame_aligner->getRealignmentResults())
   {
      const RealignmentResult& result = r.second;
      cv::Mat mask = result.mask->get();
      if (result.useFrame(realign_params) && result.done && !mask.empty())
         intensity += mask;
      intensity.copyTo(intensity_normalisation);
   }
}

cv::Mat AligningReader::getFloatIntensityNormalisation()
{
   auto& results = frame_aligner->getRealignmentResults();

   int n_frame_norm = std::max((int)results.size(), 1);
   cv::Mat fl_intensity_normalisation = intensity_normalisation / n_frame_norm;
   return fl_intensity_normalisation;
}

void AligningReader::waitForFrameReady(int frame)
{
   // Check that we have realigned this frame
   if (!frame_aligner->frameReady(frame))
   {
      std::unique_lock<std::mutex> lk(realign_mutex);
      realign_cv.wait(lk, [this, frame] {
         return (frame_aligner->frameReady(frame) || terminate);
      });
      lk.unlock();
   }
}