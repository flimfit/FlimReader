#include "FLIMreader.h"
#include "MexUtils.h"

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

void CheckInput(int nrhs, int needed);
void ErrorCheck(int nlhs, int nrhs, const mxArray *prhs[]);
void CheckSize(const mxArray* array, int needed);
void Cleanup();


void mexFunction(int nlhs, mxArray *plhs[],
   int nrhs, const mxArray *prhs[])
{
   ErrorCheck(nlhs, nrhs, prhs);
   mexAtExit(Cleanup);

   const mxArray* handle = prhs[0];
   string command = GetStringFromMatlab(prhs[1]);
}


void Cleanup()
{
}



void ErrorCheck(int nlhs, int nrhs, const mxArray *prhs[])
{

   if (nrhs < 2)
      mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
      "Not enough input arguments");

   if (!mxIsChar(prhs[1]))
      mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput",
      "Second argument should be a command string");

}

