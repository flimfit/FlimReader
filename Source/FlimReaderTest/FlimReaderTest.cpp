#include "FlimReader.h"

//#define CATCH_CONFIG_MAIN 
#include "catch.hpp"
#include <chrono>

#include "FrameWarpAligner.h"
#include <opencv2/highgui.hpp>

using namespace std;

//TEST_CASE("Loading data", "[load]")
void test_load()
{
   //std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";
   std::string filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FlimReader/PTU Splitter/output/JNK Caspase FOV=0 t=-4.ptu";
   
   //filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FlimReader/PTU Splitter/james.pt3";
   //filename = "/Users/sean/Documents/FLIMTestData/Frame marker test/1000hz 64 frames 1 line.ptu";
   filename = "C:/Users/sean/Downloads/16.11.15 bin files/VLDLR_mGFP parallel channel.bin";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/LeicaSP8_Picoquant_emilie_wientjes.pt3";
   //filename = "C:/Users/CIMLab/Documents/flim-data-zoo/ANRN-Rac1f9 T=0 pre-injection pancreas_7_1_001.ffd";
   //filename = "C:/Users/CIMLab/Documents/flim-data-zoo/Lijuan Zhang/ATTO_488_10_4_2/ATTO_488_10_4_2.ptu";

   //filename = "Z:/James/FLIM Lifetimes/20161128 PLIMFLIM/PLIM/Mouse=30741 Treatment=AZD25 Image=1_001_001.ffd";

   //filename = "C:/Users/CIMLab/Documents/flim-data-zoo/Imspector.msr";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/hetero-FRET ptu/D1.ptu";
   filename = "E:/151116_JB_Lifetime_tests_p2.sptw/151116_mFlex_seFRET/151116_mFlex_seFRET 4.ptu";

   auto start = chrono::high_resolution_clock::now();

   unique_ptr<FlimReader> reader(FlimReader::createReader(filename));
   //reader->setTemporalResolution(4);
   reader->setSpatialBinning(1);
   
   
   int sz = reader->dataSizePerChannel();
   std::vector<uint16_t> d(sz);

   RealignmentParameters params;
   params.frame_binning = 1;
   params.spatial_binning = 4;
   params.type = RealignmentType::None;
   params.n_resampling_points = 10;
   
   //reader->setRealignmentParameters(params);

   reader->readData(d.data(), { 0 });

   auto end = chrono::high_resolution_clock::now();

   cout << chrono::duration <double, milli>(end-start).count() << " ms" << endl;

}

void test_flimreader()
{
   std::string file = "test_stack.tif";
   std::vector<cv::Mat> stack;
   cv::imreadmulti(file, stack, cv::IMREAD_GRAYSCALE);

   int n_z = 10;

   std::vector<cv::Mat> frames;

   cv::Size imsz = stack[0].size();

   std::vector<int> dims = {n_z, imsz.height, imsz.width};

   for(int i=0; i<stack.size(); i++)
   {
      int z = i % n_z;
      int f = i / n_z;

      if (frames.size() <= f)
         frames.push_back( cv::Mat(dims, CV_32F) );

      cv::Mat fz(imsz, CV_32F, frames[f].data + z * imsz.area() * sizeof(float));
      stack[i].convertTo(fz, CV_32F);
   }

   RealignmentParameters params;
   params.type = RealignmentType::Warp;
   FrameWarpAligner aligner(params);

}

int main()
{
   test_load();

   return 0;
}