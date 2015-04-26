#include "TextReader.h"
#include <fstream>
#include <sstream>
#include <cassert>
#include <cctype>

using namespace std;

TextReader::TextReader(const std::string& filename) :
   FLIMReader(filename)
{
   if (extension == "txt")
      delim = '\t';
   else if (extension == "csv")
      delim = ',';

   readHeader();
}

void TextReader::readHeader()
{
   ifstream fs(filename, ifstream::in);

   assert(fs.is_open());

   int min_n_cols = numeric_limits<int>::max();
   bool is_data = false;

   int data_row = 0;

   for (string row; getline(fs, row, row_delim);) 
   {
      istringstream ss(row);

      string header;
      getline(ss, header, delim);

      is_data |= (header.size() > 0) && isdigit(header[0]);

      if (is_data)
      {
         timepoints_.push_back(stof(header));

         // Read in data line
         int chan = 0;
         for (string field; getline(ss, field, delim);)
         {
            if (data.size() <= chan)
               data.resize(chan + 1, vector<float>(data_row, 0));

            data[chan].push_back(stof(field));
            chan++;
         }
      }
      else
      {
         // Read in header line

         int chan = 0;
         for (string field; getline(ss, field, delim);)
         {
            if (metadata.size() <= chan)
               metadata.resize(chan + 1);

            metadata[chan][header] = field;
            chan++;
         }
      }

   }

   size_t n_t = timepoints_.size();
   for (size_t i = 0; i < data.size(); i++)
   {
      if (data[i].size() < n_t)
         data[i].resize(n_t, 0);
   }

}

