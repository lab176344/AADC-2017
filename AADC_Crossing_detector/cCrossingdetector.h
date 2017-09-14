/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _CROSSING_DETECTOR_HEADER_
#define _CROSSING_DETECTOR_HEADER_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"
#define OID_ADTF_CROSSINGDETECTOR_FILTER  "adtf.aadc.aadc_crossingfilter"
class cCrossingdetector : public adtf::cFilter
{
/*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_CROSSINGDETECTOR_FILTER, "Crossing Detector", adtf::OBJCAT_DataFilter);

protected:
    /*! input for rgb image */
    cVideoPin           m_oVideoInputPin;
	
	// Video Output
	
	cVideoPin 			m_oVideoOutputPin;
	//Edges Detect Flag
	
	cVideoPin 			m_oVideoOutputPin1;
	
	// Distance output for the edges	
	cOutputPin			m_Edgeonedetect;
	
	
	cObjectPtr<IMediaTypeDescription> m_pDesEdgedetect;
    
	
	// Stop line detection
	
	cOutputPin			m_Stoplinedist;
	
	cObjectPtr<IMediaTypeDescription> m_pDesStoplinedist;
	
	
public:
    // default constructor 
    cCrossingdetector(const tChar* __info);
    /*! default destructor */
    virtual ~cCrossingdetector();
	tResult PropertyChanged(const char* strProperty);

    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Start(ucom::IException** __exception_ptr = NULL);
    tResult Stop(ucom::IException** __exception_ptr = NULL);

	struct EuclideanDistanceFunctor
	{
    	int _dist2;
        EuclideanDistanceFunctor(int dist) : _dist2(dist*dist) {}

   	 bool operator()(const Point& lhs, const Point& rhs) const
   	 {
      	  return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < _dist2;
   	 }
	};

	struct myclass {
   	 bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y > pt2.y);}
			} myobject;


private:

    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);
    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);
    tResult UpdateOutputImageFormat2(const cv::Mat& outputImage);
	tResult ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime);
    tResult ProcessVideo(IMediaSample* pSample);
	tResult ReadProperties(const tChar* strPropertyName);
	tResult writeoutput(tUInt32 timeStamp);
	
	tBool m_bFirstFrame;
	tBool imshowflag;		// bool values for edge detection
    tFloat32 edge_dst[4],stop_line_dst;  // distance values of the edges
	tBitmapFormat m_sInputFormat;    
    tBitmapFormat m_sOutputFormat;
    tBitmapFormat m_sOutputFormat2;
    Mat m_inputImage;
	tInt row1,row2,col1,col2;
	tInt  m_nThresholdValue;
	
	
	tFloat64	m_nThresholdValue2;
	tFloat64	m_nThresholdValueBeforeNorm;
    tFloat64    m_nCornerHarrisparamK;
    tInt        m_nCornerHarrisblockSize;
    tInt        m_nCornerHarrisksize;
	tInt        m_nThresholdValueCanny;
    tInt        m_nThresholdValueHough;
	tInt		m_nThresholdValueHoughSMALL;
	
	

};

/** @} */ // end of group

#endif  //_CROSSING_DETECTOR_HEADER_
