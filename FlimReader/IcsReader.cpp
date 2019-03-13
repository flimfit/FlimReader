
#include "IcsReader.h"

std::mutex IcsReader::ics_mutex;


std::vector<std::string> split(const char *s, char delim = ' ') 
{
   std::vector<std::string> result;
   std::stringstream ss;
   std::string item;

   ss << s;

   while (getline(ss, item, delim)) {
      result.push_back(item);
   }

   return result;
}

std::vector<double> splitAsDouble(const char *s, char delim = ' ')
{
   std::vector<double> result;
   std::stringstream ss;
   std::string item;

   ss << s;

   while (getline(ss, item, delim)) {
      result.push_back(stod(item));
   }

   return result;
}


void IcsReader::check(Ics_Error err)
{
   if (err != IcsErr_Ok)
      throw std::runtime_error(IcsGetErrorText(err));
}

IcsReader::IcsReader(const std::string& filename) :
   FlimReader(filename)
{
   check(IcsOpen(&ics, filename.c_str(), "r"));
   readHeader();
   initaliseTimepoints();
}

IcsReader::~IcsReader()
{
   if (ics)
      IcsClose(ics);
   ics = nullptr;
}



void IcsReader::readHeader()
{
   std::lock_guard<std::mutex> lk(ics_mutex);

   Ics_DataType data_type;
   int ndims;
   size_t dims[ICS_MAXDIM];

   IcsGetLayout(ics, &data_type, &ndims, dims);

   int n_history;
   check(IcsGetNumHistoryStrings(ics, &n_history));

   std::vector<std::string> labels, units;
   std::vector<double> extents, offsets;


   char key[ICS_STRLEN_TOKEN];
   char value[ICS_LINE_LENGTH];
   Ics_HistoryWhich which = IcsWhich_First;
   for (int i = 0; i < n_history; i++)
   {
      check(IcsGetHistoryKeyValue(ics, key, value, which));
      which = IcsWhich_Next;

      if (strncmp(key, "labels", ICS_STRLEN_TOKEN) == 0)
         labels = split(value);
      if (strncmp(key, "extents", ICS_STRLEN_TOKEN) == 0)
         extents = splitAsDouble(value);
      if (strncmp(key, "offsets", ICS_STRLEN_TOKEN) == 0)
         offsets = splitAsDouble(value);
      if (strncmp(key, "units", ICS_STRLEN_TOKEN) == 0)
         units = split(value);
   }

   if (ndims != labels.size())
      throw std::runtime_error("Incorrect number of labels");
   if (ndims != extents.size())
      throw std::runtime_error("Incorrect number of extents");
   if (ndims != offsets.size())
      throw std::runtime_error("Incorrect number of offsets");

   switch (data_type)
   {
   case Ics_uint16:
      native_type = DataTypeUint16; break;
   case Ics_real32:
      native_type = DataTypeFloat; break;
   case Ics_real64:
      native_type = DataTypeDouble; break;

      /*
         case Ics_uint8:
            cv_type = CV_8U; break;
         case Ics_sint8:
            cv_type = CV_8S; break;
         case Ics_sint16:
            cv_type = CV_16S; break;
         case Ics_uint32:
         case Ics_sint32:
            cv_type = CV_32S; break; */
   default:
      throw std::runtime_error("unsupported ICS data type");

   }

   int n_t = 0;

   const char* order;
   const char* label;
   for (int d = 0; d < ndims; d++)
   {
      if (labels.empty())
         check(IcsGetOrderF(ics, d, &order, &label));
      else
         order = labels[d].c_str();

      if (strcmp(order, "x") == 0)
      {
         dim_order.x = d;
         n_x = (int)dims[d];
      }
      else if (strcmp(order, "y") == 0)
      {
         dim_order.y = d;
         n_y = (int)dims[d];
      }
      else if (strcmp(order, "z") == 0)
      {
         throw std::runtime_error("Z-resolved ICS data is not currently supported");
      }
      else if (strcmp(order, "t") == 0)
      {
         dim_order.t = d;
         n_t = (int)dims[d];
      }
      else
      {
         throw std::runtime_error("Multichannel ICS data is not currently supported");
      }
   }

   double unit_to_ps = 1e12; // assume seconds
   if (units.size() >= dim_order.t)
   {
      if (units[dim_order.t] == "ps")
         unit_to_ps = 1;
      else if (units[dim_order.t] == "ns")
         unit_to_ps = 1e3;
      else if (units[dim_order.t] == "us")
         unit_to_ps = 1e6;
      else if (units[dim_order.t] == "ms")
         unit_to_ps = 1e9;
   }


   double dt = extents[dim_order.t] / n_t * unit_to_ps;
   double t0 = offsets[dim_order.t] * unit_to_ps;

   native_timepoints.resize(n_t);
   for (int i = 0; i < n_t; i++)
      native_timepoints[i] = t0 + dt * i;

   n_chan = 1;
   n_z = 1;
}