#include "FLIMreader.h"

//#define CATCH_CONFIG_MAIN 
#include "catch.hpp"

#include "FrameWarpAligner.h"

using namespace std;

//TEST_CASE("Loading data", "[load]")
void main()
{
   //std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";
   std::string filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/output/JNK Caspase FOV=0 t=-4.ptu";
   
   //filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/james.pt3";
   //filename = "/Users/sean/Documents/FLIMTestData/Frame marker test/1000hz 64 frames 1 line.ptu";
   filename = "C:/Users/sean/Downloads/16.11.15 bin files/VLDLR_mGFP parallel channel.bin";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/LeicaSP8_Picoquant_emilie_wientjes.pt3";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/Invivo_LateralMotion.ffd";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/LeicaSP8_FFD_Peristalsis2.ffd";
   unique_ptr<FLIMReader> reader(FLIMReader::createReader(filename));
   reader->setTemporalResolution(4);
   reader->setSpatialBinning(2);
   
   
   int sz = reader->dataSizePerChannel();
   std::vector<uint16_t> d(sz);

   RealignmentParameters params;
   params.frame_binning = 1;
   params.spatial_binning = 2;
   params.use_realignment = true;
   
   reader->setRealignmentParameters(params);

   reader->readData(d.data(), { 0 });
}

/*
int main()
{
   bool error = false;
   {
      std::string ref_im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/image1.png";
      std::string im_file = "C:/Users/CIMLab/Documents/flim-data-zoo/warp/image2.png";

      cv::Mat ref_im = cv::imread(ref_im_file, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat im = cv::imread(im_file, CV_LOAD_IMAGE_GRAYSCALE);

      cv::transpose(ref_im, ref_im);
      cv::transpose(im, im);

      ref_im.convertTo(ref_im, CV_16S);
      im.convertTo(im, CV_32F);

      FrameWarpAligner aligner;
      aligner.setReference(0, ref_im);
      aligner.addFrame(1, im);
   }


   return 0;
}
*/