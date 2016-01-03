#include "PicoquantTTRReader.h"

using namespace std;

int main()
{
   //std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";
   std::string filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/output/JNK Caspase FOV=0 t=-4.ptu";
   
   //filename = "/Users/sean/repositories/FLIMfit/FLIMfitLibrary/FLIMreader/PTU Splitter/james.pt3";
   //filename = "/Users/sean/Documents/FLIMTestData/Frame marker test/1000hz 64 frames 1 line.ptu";
   filename = "C:/Users/sean/Downloads/16.11.15 bin files/VLDLR_mGFP parallel channel.bin";
   unique_ptr<FLIMReader> reader(FLIMReader::createReader(filename));
   reader->setTemporalResolution(8);
//   reader->setSpatialBinning(2);
   
   
   int sz = reader->dataSizePerChannel();
   std::vector<uint16_t> d(sz);

   reader->readData(d.data(), { 1 });


   return 0;
}