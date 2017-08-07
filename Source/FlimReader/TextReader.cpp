#include "TextReader.h"
#include <fstream>
#include <sstream>
#include <cassert>
#include <cctype>
#include <iostream>

using namespace std;

TextReader::TextReader(const std::string& filename) :
   FlimReader(filename)
{
   if (extension == "txt")
      delim = '\t';
   else if (extension == "csv")
      delim = ',';

   n_x = n_y = 1;

   readHeader();
}

void TextReader::readHeader()
{
   ifstream fs(filename, ifstream::in);

   if (!fs.is_open())
      throw std::runtime_error("Could not open file");
   
   bool is_data = false;
   int data_row = 0;

   for (std::string row; getline(fs, row, row_delim);) 
   {
      istringstream ss(row);

      string header;
      getline(ss, header, delim);

      is_data |= (header.size() > 0) && 
         (isdigit(header[0]) || header[0] == '-' || header[0] == '.');

      if (is_data)
      {
         timepoints_.push_back(stod(header));

         // Read in data line
         int chan = 0;
         for (std::string field; getline(ss, field, delim);)
         {
            if (data.size() <= chan)
               data.resize(chan + 1, std::vector<float>(data_row, 0));

            try
            {
               data[chan].push_back((float) stod(field));
            }
            catch(std::out_of_range)
            {
               data[chan].push_back(0.0);
               std::cout << "Warning, out of range for a double:  " << field << "\n";
            }
            
            chan++;
         }
      }
      else
      {
         // Read in header line

         int chan = 0;
         for (std::string field; getline(ss, field, delim);)
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

