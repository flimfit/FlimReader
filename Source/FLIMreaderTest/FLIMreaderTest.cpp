#include "PicoquantTTRReader.h"

using namespace std;

int main()
{
   std::string filename = "C:/Users/sean/Documents/FLIMreader/TestData/BaseName_9_1.pt3";

   PicoquantTTTRReader reader(filename);
   reader.SetTemporalResolution(8);
   int sz = reader.GetDataSizePerChannel();
   std::vector<float> d(sz);

   reader.ReadData({ 1 }, d.data());


   return 0;
}