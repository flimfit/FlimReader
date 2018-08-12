#include "AligningReader.h"
#include <future>
#include "Cv3dUtils.h"
#include "Cache.h"

#include "Cache_impl.h"

Cache<cv::Mat>* Cache<cv::Mat>::instance = nullptr;

template<>
size_t Cache<cv::Mat>::getSize(const cv::Mat& obj)
{
   return obj.total() * obj.elemSize();
}


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
   Cache<cv::Mat>::getInstance();
}

AligningReader::~AligningReader()
{
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

CachedObject<cv::Mat> AligningReader::getIntensityFrame(int frame)
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

   if (reference_index == -1)
   {
      switch(realign_params.default_reference_frame)
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
   
   if ((n_frames == 0) || terminate)
   {
      std::cout << "No frames\n";
      return;      
   }

   ImageScanParameters image_params = getImageScanParameters();
   
   frame_aligner->setRealignmentParams(realign_params);
   frame_aligner->setImageScanParams(image_params);
   frame_aligner->setNumberOfFrames(n_frames);

   cv::Mat ref_frame = getIntensityFrameImmediately(reference_index);

   tick_count_start = cv::getTickCount();

   frame_aligner->setReference(reference_index, ref_frame);

   realignment.clear();
   realignment_complete = false;

   intensity_normalisation = cv::Mat(ref_frame.dims, ref_frame.size.p, CV_16U, cv::Scalar(0));

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
         cv::Mat m16;
         std::lock_guard<std::mutex> lk(realign_mutex);
         if (realignment[i].useFrame(realign_params))
         {
            cv::Mat mask = realignment[i].mask;
            mask.convertTo(m16, CV_16U);
            intensity_normalisation += m16;

         }
      }

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
   if (!realignment.empty())
   {
      // Get intensity
      cv::Mat intensity(intensity_normalisation.dims, intensity_normalisation.size.p, CV_16U, cv::Scalar(1));
      cv::Mat m16;

      for (auto& r : realignment)
      {
         RealignmentResult& result = r.second;
         cv::Mat mask = result.mask;
         if (result.useFrame(realign_params) &&
             result.done && !mask.empty())
         {
            mask.convertTo(m16, CV_16U);
            intensity += m16;
         }
      }
      intensity.copyTo(intensity);
   }
}

cv::Mat AligningReader::getFloatIntensityNormalisation()
{
   cv::Mat fl_intensity_normalisation;
   int n_frame_norm = std::max((int)realignment.size(), 1);
   intensity_normalisation.convertTo(fl_intensity_normalisation, CV_32F, 1.0 / n_frame_norm, 0);

   return fl_intensity_normalisation;
}