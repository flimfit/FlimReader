#include "FLIMreader.h"

//#define CATCH_CONFIG_MAIN 
#include "catch.hpp"

#include "FrameWarpAligner.h"

using namespace std;

//TEST_CASE("Loading data", "[load]")
void test_load()
{
   //std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";
   std::string filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/output/JNK Caspase FOV=0 t=-4.ptu";
   
   //filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/james.pt3";
   //filename = "/Users/sean/Documents/FLIMTestData/Frame marker test/1000hz 64 frames 1 line.ptu";
   filename = "C:/Users/sean/Downloads/16.11.15 bin files/VLDLR_mGFP parallel channel.bin";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/LeicaSP8_Picoquant_emilie_wientjes.pt3";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/ANRN-Rac1f9 T=0 pre-injection pancreas_7_1_001.ffd";
   filename = "C:/Users/CIMLab/Documents/flim-data-zoo/Lijuan Zhang/ATTO_488_10_4_2/ATTO_488_10_4_2.ptu";

   //filename = "C:/Users/CIMLab/Documents/flim-data-zoo/Imspector.msr";

   unique_ptr<FLIMReader> reader(FLIMReader::createReader(filename));
   reader->setTemporalResolution(4);
   reader->setSpatialBinning(1);
   
   
   int sz = reader->dataSizePerChannel();
   std::vector<uint16_t> d(sz);

   RealignmentParameters params;
   params.frame_binning = 1;
   params.spatial_binning = 4;
   params.type = RealignmentType::None;
   params.n_resampling_points = 10 ;
   
   reader->setRealignmentParameters(params);

   reader->readData(d.data(), { 1 });
}



int main()
{
   test_load();

   return 0;
}