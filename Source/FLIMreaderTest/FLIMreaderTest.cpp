#include "PicoquantTTRReader.h"

using namespace std;

int main()
{
   std::string filename = "E:/User Data/James/JC280415/Plasmid=CC3 Ex=800 Em=624.pt3";

   PicoquantTTTRReader reader(filename);
   reader.setTemporalResolution(8);
   int sz = reader.dataSizePerChannel();
   std::vector<float> d(sz*2);

   reader.readData(d.data(), { 0, 1 });


   return 0;
}