#include "tiffio.h"
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

void writeMultipageTiff(const std::string& filename, const std::vector<cv::Mat>& planes)
{
   TIFF *out = TIFFOpen(filename.c_str(), "w");

   if (!out)
      throw std::runtime_error("Could not open file");

   const int npages = planes.size();
   int page;
   for (page = 0; page < npages; page++) {

      auto& plane = planes[page];

      uint32 imagelength = plane.size().height;
      uint32 imagewidth = plane.size().width;
      uint32 bytespersample = plane.elemSize1();
      uint32 nsamples = plane.channels();
      uint16 config = PLANARCONFIG_CONTIG;
      int type = plane.type() & 0x7; // pull out channel format

      TIFFSetField(out, TIFFTAG_IMAGELENGTH, imagelength);
      TIFFSetField(out, TIFFTAG_IMAGEWIDTH, imagewidth);
      TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
      TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, nsamples);
      TIFFSetField(out, TIFFTAG_COMPRESSION, COMPRESSION_LZW);
      TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, bytespersample * 8);
      TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(out, imagewidth*nsamples));
      TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

      switch (type)
      {
      case CV_8U:
      case CV_16U:
         TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
         break;
      case CV_8S:
      case CV_16S:
      case CV_32S:
         TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_INT);
         break;
      case CV_32F:
      case CV_64F:
         TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
         break;
      }

		/* We are writing single page of the multipage file */
		TIFFSetField(out, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);
		TIFFSetField(out, TIFFTAG_PAGENUMBER, page, npages);

		for (uint row = 0; row < imagelength; row++) 
		{
			auto buf = plane.row(row).data;
			if (TIFFWriteScanline(out, buf, row) != 1)
			{
				printf("Unable to write a row\n");
				break;
			}
		}
		TIFFWriteDirectory(out);
	}
	TIFFClose(out);
}