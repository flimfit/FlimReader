#include "FLIMreader.h"

using namespace std;

int main()
{
   std::string filename = "C:/Users/sean/Documents/FLIMreader/TestData/BaseName_9_1.pt3";

   PicoquantTTTRReader reader(filename);

   return 0;
}