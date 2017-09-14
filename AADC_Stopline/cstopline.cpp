/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "cstopline.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   cstopline)

#define MD_RANGE_ROWS_R1 "StopLine::ROW1"
#define MD_RANGE_ROWS_R2 "StopLine::ROW2"
#define MD_RANGE_COLS_C1 "StopLine::COL1"
#define MD_RANGE_COLS_C2 "StopLine::COL2"
#define MD_GREY_TRESHOLD "StopLine::GreyThresholdValue"

cstopline::cstopline(const tChar* __info) : cFilter(__info)
{

 SetPropertyInt(MD_GREY_TRESHOLD, 200);
    SetPropertyBool(MD_GREY_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_GREY_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");

 SetPropertyInt(MD_RANGE_ROWS_R1, 520);
    SetPropertyBool(MD_RANGE_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2, 600);
    SetPropertyBool(MD_RANGE_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2 NSSUBPROP_DESCRIPTION,"MarkerDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1, 450);
    SetPropertyBool(MD_RANGE_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2, 750);
    SetPropertyBool(MD_RANGE_COLS_C2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2 NSSUBPROP_DESCRIPTION, "MarkerDetection::The second col C2 to be taken (,C1) (,C2)");
}

cstopline::~cstopline()
{
}

tResult cstopline::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cstopline::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult cstopline::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));


        // Video Input
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));
	// Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin1.Create("Video_Output Raw", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin1));
	//stop line	
	tChar const * strDescSignaldistancestop = pDescManager->GetMediaDescription("tStoplineStruct");
	RETURN_IF_POINTER_NULL(strDescSignaldistancestop);
	cObjectPtr<IMediaType> pTypeSignaldistancestop = new cMediaType(0, 0, 0, "tStoplineStruct", strDescSignaldistancestop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignaldistancestop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesStoplinedist));
	RETURN_IF_FAILED(m_Stoplinedist.Create("distance stop line", pTypeSignaldistancestop, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_Stoplinedist));	
	//stop line Orientation
	tChar const * strDescSignalOrientationstop = pDescManager->GetMediaDescription("tStoplineStruct");
	RETURN_IF_POINTER_NULL(strDescSignalOrientationstop);
	cObjectPtr<IMediaType> pTypeSignalOrientationstop = new cMediaType(0, 0, 0, "tStoplineStruct", strDescSignalOrientationstop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalOrientationstop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesStoplineOrient));
	RETURN_IF_FAILED(m_StoplineOrient.Create("Orientation", pTypeSignalOrientationstop, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_StoplineOrient));	

	// Input LaneFollowerStart
        tChar const * strDescSignalValueInputLaneFollowerStart = pDescManager->GetMediaDescription("tStoplineStruct");
        RETURN_IF_POINTER_NULL(strDescSignalValueInputLaneFollowerStart);
        cObjectPtr<IMediaType> pTypeSignalValueInputLaneFollowerStart = new cMediaType(0, 0, 0, "tStoplineStruct", strDescSignalValueInputLaneFollowerStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueInputLaneFollowerStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalStop));
        RETURN_IF_FAILED(m_oStopline.Create("Stopline Det", pTypeSignalValueInputLaneFollowerStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oStopline));
		
    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
	stop_detect=tFalse;
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
    }

    RETURN_NOERROR;
}
tResult cstopline::ReadProperties(const tChar* strPropertyName)
{
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_GREY_TRESHOLD))
	{
		m_nThresholdValue = GetPropertyInt(MD_GREY_TRESHOLD);
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R1))
	{
		row1 = GetPropertyInt(MD_RANGE_ROWS_R1);
	}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R2))
			{
				row2 = GetPropertyInt(MD_RANGE_ROWS_R2);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C1))
			{
				col1 = GetPropertyInt(MD_RANGE_COLS_C1);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C2))
			{
				col2 = GetPropertyInt(MD_RANGE_COLS_C2);
			}
	RETURN_NOERROR;
		}

tResult cstopline::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cstopline::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
            }

            ProcessVideo(pMediaSample);
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
        }
    }
    RETURN_NOERROR;
}

tResult cstopline::ProcessVideo(IMediaSample* pSample)
{
    // VideoInput
     RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImage;
    const tVoid* l_pSrcBuffer;
    tUInt32 timeStamp = 0;
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
	Mat imagecut,color_dst,imagecut2,imagecut3,imagecut4;
	Mat skel(greythresh.size(), CV_8UC1, cv::Scalar(0));
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
        {
        //copy the data to matrix (make a copy, not change the sample content itself!)
        memcpy(m_inputImage.data, l_pSrcBuffer, size_t(m_sInputFormat.nSize));
        //or just set the data pointer of matrix because we create a new matrix later one
        //m_inputImage.data = (uchar*)(l_pSrcBuffer);
	imagecut = m_inputImage(Range(520, 600), Range(450, 750)).clone(); 
        //Erzeugen eines Graustufenbildes
	GaussianBlur(imagecut,imagecut2,Size(13,13),0,0, BORDER_DEFAULT); 
        cvtColor(imagecut2, grey, CV_BGR2GRAY);
        threshold(grey, greythresh, 155, 500, THRESH_BINARY);
	
	Mat temp(greythresh.size(), CV_8UC1);
	Mat element = getStructuringElement(MORPH_RECT, Size(4, 4));
        erode(greythresh, greythresh, element);
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
	cannysize = linecanny.size();
        vector<Vec2f> lines;
        // How long must be the line to be detected as roadlines -> HoughLines
	HoughLines(linecanny, lines, 1, CV_PI / 180, 50, 0,0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );

       tFloat32 thetaAll[1000];
       tFloat32 rhoAll[1000];
       tFloat32 distanceCal;

        for (tInt i = 0; i < (tInt)(lines.size()); i++)
        {
            thetaAll[i] = lines[i][1];
            rhoAll[i] = lines[i][0];
        }

        tFloat32 thetaNoRep[1000];
        tFloat32 rhoNoRep[1000];
        tInt sizeNoRep = 0;

        for (tInt i = 0; i < (tInt)(lines.size()); i++)
        {
            tInt rep = 0;
            for (tInt j = 0; j < i; j++)
            {
               if ((abs(thetaAll[i] - thetaAll[j]) <= 0.2) && (abs(rhoAll[i] - rhoAll[j]) <= 30))
		 //if ((abs(thetaAll[i] - thetaAll[j]) <= 0.2))
                {
                    rep = 1;		// One point of the Line is repeated, ignore the whole line
                }
            }	
          // Filter
          //if ((rep == 0) &&((thetaAll[i]<CV_PI*0.48889) || (thetaAll[i]>CV_PI*0.51111)))
          if ((rep == 0) && ((thetaAll[i]>CV_PI*0.4444) && (thetaAll[i]<CV_PI*0.5556)))
          //if ((rep == 0) && (( thetaAll[i]<CV_PI*0.36111) || (thetaAll[i]>CV_PI*0.63889)))
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
            }
        }

        tFloat32 DistPack[sizeNoRep];
	// Print the detected lines on the screen
	for(tInt i = 0; i < sizeNoRep; i++ )
        {
            double a = cos(thetaNoRep[i]), b = sin(thetaNoRep[i]);
            double x0 = a*rhoNoRep[i], y0 = b*rhoNoRep[i];
	    LOG_INFO(adtf_util::cString::Format("X not %f Y not %f",x0,y0));
            
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	    tInt newv=80-y0;
	    distanceCal = 0.0001*pow(newv,3)-0.0045*pow(newv,2)+0.8652*newv+28.8300;
            DistPack[i] = distanceCal;
        }
	if (sizeNoRep == 1)
	{
	distance = DistPack[0];
	Orientation = thetaNoRep[0];
	}
	if (sizeNoRep == 2)
	{
        	if (DistPack[0]<DistPack[1])
        	{
		distance = DistPack[0];
		Orientation = thetaNoRep[0];}

		else
		{
		distance = DistPack[1];
		Orientation = thetaNoRep[1];}
        	}
	}
	Orientation = Orientation*180/CV_PI;
	   LOG_INFO(adtf_util::cString::Format("distance %f Orientation %f", distance,Orientation));

    if (!imagecut.empty())
    {
        UpdateOutputImageFormat2(imagecut);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, imagecut.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin1.Transmit(pMediaSample));
        //RETURN_IF_FAILED(m_oVideoOutputPin1.Transmit(pMediaSample));
        imagecut.release();
    }
       if (!color_dst.empty())
       {
        UpdateOutputImageFormat(color_dst);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, color_dst.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        color_dst.release();
      }
        pSample->Unlock(l_pSrcBuffer);
    }

    RETURN_NOERROR;
}

tResult cstopline::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

tResult cstopline::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult cstopline::UpdateOutputImageFormat2(const cv::Mat& outputImage2)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage2.total() * outputImage2.elemSize()) != m_sOutputFormat2.nSize)
    {
        Mat2BmpFormat(outputImage2, m_sOutputFormat2);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat2.nWidth, m_sOutputFormat2.nHeight, m_sOutputFormat2.nBytesPerLine, m_sOutputFormat2.nSize, m_sOutputFormat2.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin1.SetFormat(&m_sOutputFormat2, NULL);
    }
    RETURN_NOERROR;
}

tResult cstopline::writeOutputs1(tFloat32 distance, tUInt32 timeStamp)
{
   
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);

	// acceleration
	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pDesStoplinedist->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDesStoplinedist->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("bValue", (tVoid*)&(stop_detect));
	pCoderOutputaccelerate->Set("f32Distance", (tVoid*)&(distance));
	pCoderOutputaccelerate->Set("f32Orientation", (tVoid*)&(Orientation));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDesStoplinedist->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_Stoplinedist.Transmit(pMediaSampleaccelerate);

}
