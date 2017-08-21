#pragma once

#include <mex.h>
#include <string>
#include <vector>

#define AssertInputCondition(x) checkInputCondition(#x, x);
void checkInputCondition(const char* text, bool condition)
{
   if (!condition)
      mexErrMsgIdAndTxt("FLIMfitMex:invalidInput", text);
}

std::string getStringFromMatlab(const mxArray* dat)
{
   if (mxIsChar(dat))
   {
      size_t buflen = mxGetN(dat) * sizeof(mxChar) + 1;
      char* buf = (char*)mxMalloc(buflen);

      mxGetString(dat, buf, (mwSize)buflen);

      return std::string(buf);
   }

   mexWarnMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
      "Unable to retrieve string");

   return std::string();
}

template<typename T, typename U>
std::vector<T> getUVector(const mxArray* v)
{
   std::vector<T> vec;
   if (mxIsNumeric(v))
      if (U* data = (U*)mxGetData(v))
      {
         size_t len = mxGetNumberOfElements(v);
         vec.resize(len);

         for (size_t i = 0; i < len; i++)
            vec[i] = static_cast<T>(data[i]);
      }
   return vec;

};

template<typename T>
std::vector<T> getVector(const mxArray* v)
{
   if (mxIsDouble(v))
      return getUVector<T, double>(v);
   else if (mxIsSingle(v))
      return getUVector<T, float>(v);
   else if (mxIsInt64(v))
      return getUVector<T, int64_t>(v);
   else if (mxIsInt32(v))
      return getUVector<T, int32_t>(v);
   else if (mxIsInt16(v))
      return getUVector<T, int16_t>(v);
   else if (mxIsInt8(v))
      return getUVector<T, int8_t>(v);
   else if (mxIsUint64(v))
      return getUVector<T, uint64_t>(v);
   else if (mxIsUint32(v))
      return getUVector<T, uint32_t>(v);
   else if (mxIsUint16(v))
      return getUVector<T, uint16_t>(v);
   else if (mxIsUint8(v))
      return getUVector<T, uint8_t>(v);

   return std::vector<T>();
}

void checkSize(const mxArray* array, int needed)
{
   if (needed != mxGetNumberOfElements(array))
      mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
         "Input array is the wrong size");
}

mxArray* getFieldFromStruct(const mxArray* s, const char *field)
{
   int field_number = mxGetFieldNumber(s, field);
   if (field_number == -1)
   {
      std::string err = std::string("Missing field in structure: ").append(field);
      mexErrMsgIdAndTxt("FLIMfit:missingField", err.c_str());
   }

   return mxGetFieldByNumber(s, 0, field_number);
}

double getValueFromStruct(const mxArray* s, const char *field, double default_value)
{
   int field_number = mxGetFieldNumber(s, field);
   if (field_number == -1)
      return default_value;

   const mxArray* v = mxGetFieldByNumber(s, 0, field_number);

   if (!mxIsScalar(v))
   {
      std::string err = std::string("Expected field to be scalar: ").append(field);
      mexErrMsgIdAndTxt("FLIMfit:missingField", err.c_str());
   }

   return mxGetScalar(v);
}

double getValueFromStruct(const mxArray* s, const char *field)
{
   const mxArray* v = getFieldFromStruct(s, field);

   if (!mxIsScalar(v))
   {
      std::string err = std::string("Expected field to be scalar: ").append(field);
      mexErrMsgIdAndTxt("FLIMfit:missingField", err.c_str());
   }

   return mxGetScalar(v);
}


template<typename T>
std::vector<T> getVectorFromStruct(const mxArray* s, const char *field)
{
   mxArray* v = getFieldFromStruct(s, field);
   return getVector<T>(v);
}

bool isArgument(int nrhs, const mxArray *prhs[], const char* arg, int nstart = 0)
{
   for (int i = nstart; (i + 1) < nrhs; i++)
   {
      if (mxIsChar(prhs[i]) && getStringFromMatlab(prhs[i]) == arg)
         return true;
   }
   return false;
}

const mxArray* getNamedArgument(int nrhs, const mxArray *prhs[], const char* arg, int nstart = 0)
{
   for (int i = nstart; (i + 1) < nrhs; i++)
   {
      if (mxIsChar(prhs[i]) && getStringFromMatlab(prhs[i]) == arg)
         return prhs[i + 1];
   }

   std::string err = std::string("Missing argument: ").append(arg);
   mexErrMsgIdAndTxt("FLIMfit:missingArgument", err.c_str());
   return static_cast<const mxArray*>(nullptr);
}

void checkInput(int nrhs, int needed)
{
   if (nrhs < needed)
      mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
         "Not enough input arguments");
}

