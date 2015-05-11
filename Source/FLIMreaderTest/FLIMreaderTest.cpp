#include "PicoquantTTRReader.h"

using namespace std;

int main()
{
   //std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";
   std::string filename = "/Users/sean/Documents/FLIMTestData/data rep=2 frames=3 lineacc=2 facc=64.ptu";
   
   unique_ptr<FLIMReader> reader(FLIMReader::createReader(filename));
   reader->setTemporalResolution(8);
   reader->setSpatialBinning(2);
   
   
   int sz = reader->dataSizePerChannel();
   std::vector<float> d(sz*2);

   reader->readData(d.data(), { 0, 1 });


   return 0;
}