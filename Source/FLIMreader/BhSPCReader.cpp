#include "BhSPCReader.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>

using namespace std;

#define READ(fs, x) fs.read(reinterpret_cast<char *>(&x), sizeof(x))

BhSPCReader::BhSPCReader(const std::string& filename) :
   AbstractFifoReader(filename)
{
   readHeader();

   n_timebins_native = 1 << 14;
   setTemporalResolution(14);

   markers.FrameMarker = 0x4;
   markers.LineEndMarker = 0x2;
   markers.LineStartMarker = 0x1;

   event_reader = std::unique_ptr<AbstractEventReader>(new BhEventReader(filename, data_position));

   determineDwellTime();
}

void BhSPCReader::readHeader()
{
   ifstream fs(filename, ifstream::in | ifstream::binary);

   char magic[8];
   char version[8];

   fs.read(magic, sizeof(magic));

   if (string("PQTTTR") != magic)
      throw runtime_error("Wrong magic string, this is not a PTU file");

   n_chan = 1;
   fs.read(version, sizeof(version));

   data_position = fs.tellg();
}