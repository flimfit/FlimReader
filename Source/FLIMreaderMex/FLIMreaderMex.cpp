#include "PicoquantTTRReader.h"
#include "MexUtils.h"

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <limits>

using namespace std;

void CheckInput(int nrhs, int needed);
void ErrorCheck(int nlhs, int nrhs, const mxArray *prhs[]);
void CheckSize(const mxArray* array, int needed);
void Cleanup();

vector<unique_ptr<FLIMReader>> readers;

void mexFunction(int nlhs, mxArray *plhs[],
   int nrhs, const mxArray *prhs[])
{
   ErrorCheck(nlhs, nrhs, prhs);
   mexAtExit(Cleanup);

   try
   {
      if ((nrhs == 1) && mxIsChar(prhs[0]))
      {
         string filename = GetStringFromMatlab(prhs[0]);

         auto reader = unique_ptr<FLIMReader>(new PicoquantTTTRReader(filename));

         // Make sure we have an empty place
         if (readers.empty() || readers[readers.size()-1] != nullptr)
            readers.push_back(unique_ptr<FLIMReader>(nullptr));

         int i = 0;
         for (; i < readers.size(); i++)
            if (readers[i] == nullptr)
            {
               readers[i].swap(reader);
               break;
            }
               
         if (nlhs > 0)
            plhs[0] = mxCreateDoubleScalar(i);
      }
      else if (nrhs >= 2)
      {
         if (!mxIsChar(prhs[1]))
            mexErrMsgIdAndTxt("FLIMreaderMex:invalidInput",
            "Second argument should be a command string");

         int idx = static_cast<int>(mxGetScalar(prhs[0]));
         string command = GetStringFromMatlab(prhs[1]);

         if (idx >= readers.size() || readers[idx] == nullptr)
            mexErrMsgIdAndTxt("FLIMreaderMex:invalidReader",
            "Invalid reader index specified");

         if (command == "GetTimePoints")
         {
            const vector<double>& timepoints = readers[idx]->timepoints();

            plhs[0] = mxCreateDoubleMatrix(1, timepoints.size(), mxREAL);
            double* t = mxGetPr(plhs[0]);
            for (int i = 0; i < timepoints.size(); i++)
               t[i] = timepoints[i];
         }
         else if (command == "GetNumberOfChannels" && nlhs > 0)
         {
            int n_chan = readers[idx]->numChannels();
            plhs[0] = mxCreateDoubleScalar(n_chan);
         }
         else if (command == "GetImageSize" && nlhs > 0)
         {
            plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
            double* d = mxGetPr(plhs[0]);

            d[0] = readers[idx]->numX();
            d[1] = readers[idx]->numY();
         }
         else if (command == "GetData" && nlhs > 0)
         {
            if (nrhs < 3)
               mexErrMsgIdAndTxt("FLIMreaderMex:invalidInput",
               "Not enough input arguments");

            vector<int> channels = GetVector<int>(prhs[2]);

            mwSize n_t = readers[idx]->timepoints().size();
            mwSize n_chan = channels.size();
            mwSize n_x = readers[idx]->numX();
            mwSize n_y = readers[idx]->numY();

            mwSize dims[4] = { n_t, n_chan, n_x, n_y };

            plhs[0] = mxCreateNumericArray(4, dims, mxSINGLE_CLASS, mxREAL);
            float* d = reinterpret_cast<float*>(mxGetData(plhs[0]));
            readers[idx]->readData(d, channels);
         } 
         else if (command == "Delete")
         {
            readers[idx] = nullptr;
         }
      }
   }
   catch (exception e)
   {
      mexErrMsgIdAndTxt("FLIMreaderMex:exceptionOccurred",
         e.what());
   }
}


void Cleanup()
{
   readers.clear();
}



void ErrorCheck(int nlhs, int nrhs, const mxArray *prhs[])
{
   if (nrhs == 0)
      mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
      "Not enough input arguments");
}

