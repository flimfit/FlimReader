#include "PicoquantTTRReader.h"
#include "MexUtils.h"

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <limits>

using namespace std;


void Cleanup();

vector<unique_ptr<FlimReader>> readers;

void mexFunction(int nlhs, mxArray *plhs[],
   int nrhs, const mxArray *prhs[])
{
   mexAtExit(Cleanup);
   AssertInputCondition(nrhs >= 1);

   try
   {
      if ((nrhs == 1) && mxIsChar(prhs[0]))
      {
         string filename = getStringFromMatlab(prhs[0]);

         auto reader = unique_ptr<FlimReader>(FlimReader::createReader(filename));

         // Make sure we have an empty place
         if (readers.empty() || readers[readers.size()-1] != nullptr)
            readers.push_back(unique_ptr<FlimReader>(nullptr));

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
            mexErrMsgIdAndTxt("FlimReaderMex:invalidInput",
            "Second argument should be a command string");

         int idx = static_cast<int>(mxGetScalar(prhs[0]));
         string command = getStringFromMatlab(prhs[1]);

         if (idx >= readers.size() || readers[idx] == nullptr)
            mexErrMsgIdAndTxt("FlimReaderMex:invalidReader",
            "Invalid reader index specified");

         if (command == "GetTimePoints")
         {
            AssertInputCondition(nlhs >= 1);
            const std::vector<double>& timepoints = readers[idx]->getTimepoints();

            plhs[0] = mxCreateDoubleMatrix(1, timepoints.size(), mxREAL);
            double* t = mxGetPr(plhs[0]);
            for (int i = 0; i < timepoints.size(); i++)
               t[i] = timepoints[i];
         }
         else if (command == "GetNumberOfChannels")
         {
            AssertInputCondition(nlhs >= 1);
            int n_chan = readers[idx]->getNumChannels();
            plhs[0] = mxCreateDoubleScalar(n_chan);
         }
         else if (command == "GetImageSize")
         {
            AssertInputCondition(nlhs >= 1);
            plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
            double* d = mxGetPr(plhs[0]);

            d[0] = readers[idx]->numX();
            d[1] = readers[idx]->numY();
         }
         else if (command == "GetData" && nlhs > 0)
         {
            AssertInputCondition(nlhs >= 1);
            AssertInputCondition(nrhs >= 3);

            std::vector<int> channels = getVector<int>(prhs[2]);

            mwSize n_t = readers[idx]->getTimepoints().size();
            mwSize n_chan = channels.size();
            mwSize n_x = readers[idx]->numX();
            mwSize n_y = readers[idx]->numY();

            mwSize dims[4] = { n_t, n_chan, n_x, n_y };

            plhs[0] = mxCreateNumericArray(4, dims, mxUINT16_CLASS, mxREAL);
			   uint16_t* d = reinterpret_cast<uint16_t*>(mxGetData(plhs[0]));
            readers[idx]->readData(d, channels);
         }
         else if (command == "GetSpatialBinning")
         {
            AssertInputCondition(nlhs >= 1);
            int spatial_binning = readers[idx]->getSpatialBinning();
            plhs[0] = mxCreateDoubleScalar(spatial_binning);
         }
         else if (command == "SetSpatialBinning")
         {
            AssertInputCondition(nrhs >= 3);
            int spatial_binning = (int) mxGetScalar(prhs[2]);
            readers[idx]->setSpatialBinning(spatial_binning);
         }
         else if (command == "GetNumTemporalBits")
         {
            AssertInputCondition(nlhs >= 1);
            int n_bits = readers[idx]->getTemporalResolution();
            plhs[0] = mxCreateDoubleScalar(n_bits);
         }
         else if (command == "SetNumTemporalBits")
         {
            AssertInputCondition(nrhs >= 3);
            int n_bits = (int)mxGetScalar(prhs[2]);
            readers[idx]->setTemporalResolution(n_bits);
         }
         else if (command == "SupportsRealignment")
         {
            AssertInputCondition(nlhs >= 1);
            bool supports_realignment = readers[idx]->supportsRealignment();
            plhs[0] = mxCreateLogicalMatrix(1, 1);
            *(mxGetLogicals(plhs[0])) = supports_realignment;
         }
         else if (command == "IsBidirectional")
         {
            AssertInputCondition(nlhs >= 1);
            bool is_bidirectional = readers[idx]->isBidirectional();
            plhs[0] = mxCreateLogicalMatrix(1, 1);
            *(mxGetLogicals(plhs[0])) = is_bidirectional;
         }
         else if (command == "GetRepRate")
         {
            AssertInputCondition(nlhs >= 1);
            double rep_rate = readers[idx]->getRepRateHz();
            plhs[0] = mxCreateDoubleScalar(rep_rate);
         }
         else if (command == "SetRealignmentParameters")
         {
            AssertInputCondition(nrhs >= 3);
            AssertInputCondition(mxIsStruct(prhs[2]));
            RealignmentParameters params;
            params.type = static_cast<RealignmentType>((int) getValueFromStruct(prhs[2], "type", false));
            params.frame_binning = (int) getValueFromStruct(prhs[2],"frame_binning", 4);
            params.spatial_binning = (int) getValueFromStruct(prhs[2], "spatial_binning", 4);
            params.n_resampling_points = (int) getValueFromStruct(prhs[2], "n_resampling_points", 10);
            readers[idx]->setRealignmentParameters(params);
         }
         else if (command == "SetBidirectionalPhase")
         {
            AssertInputCondition(nrhs >= 3);
            double phase = mxGetScalar(prhs[2]);
            readers[idx]->setBidirectionalPhase(phase);
         }
         else if (command == "GetIntensityNormalisation")
         {
            AssertInputCondition(nlhs >= 1);
            cv::Mat intensity_normalisation = readers[idx]->getIntensityNormalisation();
            auto size = intensity_normalisation.size();
            plhs[0] = mxCreateNumericMatrix(size.width, size.height, mxUINT16_CLASS, mxREAL);
            uchar* out = (uchar*) mxGetData(plhs[0]);
            copy_n((uchar*)intensity_normalisation.data, size.area() * sizeof(uint16_t), out);
         }
         else if (command == "GetMetadata")
         {
            AssertInputCondition(nlhs >= 1);
            auto tags = readers[idx]->getTags();

            size_t n_tags = tags.size();
            std::vector<const char*> names;
            names.reserve(n_tags);
            for (auto p : tags)
               names.push_back(p.first.data());

            plhs[0] = mxCreateStructMatrix(1, 1, n_tags, names.data());

            int i = 0;
            for (auto p : tags)
            {
               mxArray* v; 
               if (p.second.is_vector)
               {
                  v = mxCreateDoubleScalar(0); // TODO
               }
               else
               {
                  switch (p.second.type)
                  {
                  case MetaDataTag::TagBool:
                  case MetaDataTag::TagDouble:
                  case MetaDataTag::TagUInt64:
                  case MetaDataTag::TagInt64:
                     v = mxCreateDoubleScalar(p.second.getValue<double>());
                     break;
                  case MetaDataTag::TagString:
                  case MetaDataTag::TagDate:
                     v = mxCreateString(p.second.getString().c_str());
                     break;
                  default:
                     return;
                  }
               }
               mxSetFieldByNumber(plhs[0], 0, i++, v);
            }
         }

         else if (command == "Delete")
         {
            readers[idx] = nullptr;
         }
      }
   }
   catch (std::runtime_error e)
   {
      mexErrMsgIdAndTxt("FlimReaderMex:exceptionOccurred",
         e.what());
   }
   catch (exception e)
   {
      mexErrMsgIdAndTxt("FlimReaderMex:exceptionOccurred",
         e.what());
   }
}


void Cleanup()
{
   readers.clear();
}
