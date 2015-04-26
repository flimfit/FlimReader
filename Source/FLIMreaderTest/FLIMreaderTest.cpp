#include "PicoquantTTRReader.h"

using namespace std;

int main()
{
   std::string filename = "C:/Users/sean/Documents/FLIMreader/TestData/BaseName_9_1.pt3";

   PicoquantTTTRReader reader(filename);
   reader.setTemporalResolution(8);
   int sz = reader.dataSizePerChannel();
   std::vector<float> d(sz);

   reader.readData(d.data(), { 1 });


   return 0;
}