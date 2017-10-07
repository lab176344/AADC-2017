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
#include "cTrafficsign.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   cOpenCVTemplate)



cOpenCVTemplate::cOpenCVTemplate(const tChar* __info) : cFilter(__info)
{
}

cOpenCVTemplate::~cOpenCVTemplate()
{
}

tResult cOpenCVTemplate::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cOpenCVTemplate::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult cOpenCVTemplate::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
		
		// create pin for start signal
		tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalstart);
		cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
		RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStart));


        // Video Input
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));


		// create pin for check traffic for crossing
		tChar const * strDescSignalCheckTraffic = pDescManager->GetMediaDescription("tCheckTrafficForCrossing");
		RETURN_IF_POINTER_NULL(strDescSignalCheckTraffic);
		cObjectPtr<IMediaType> pTypeSignalCheckTraffic = new cMediaType(0, 0, 0, "tCheckTrafficForCrossing", strDescSignalCheckTraffic, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalCheckTraffic->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCheckTraffic));
		RETURN_IF_FAILED(m_oCheckTraffic.Create("CheckTraffic", pTypeSignalCheckTraffic, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oCheckTraffic));

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

        // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
	m_firstimage=tFalse;
	
	m_bStart=tFalse;

	m_bTrafficOnLeft=tFalse;
	m_bTrafficOnCenter=tFalse;
	m_bTrafficOnRight=tFalse;
	
	
	bFlagLeft=tFalse;
	bFlagCenter=tFalse;
	bFlagRight=tFalse;
    }

    RETURN_NOERROR;
}



tResult cOpenCVTemplate::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cOpenCVTemplate::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
		// Input signal at Start
		if (pSource == &m_oStart)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			m_pDescStart->Unlock(pCoderInput);
			TransmitOutput();
		}
        else if (pSource == &m_oVideoInputPin)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
            }
			
			if(m_bStart)
			{
				ProcessVideo(pMediaSample);
				TransmitOutput();
			}
			else
			{
				m_bTrafficOnLeft=tFalse;
				m_bTrafficOnCenter=tFalse;
				m_bTrafficOnRight=tFalse;
				
				
				bFlagLeft=tFalse;
				bFlagCenter=tFalse;
				bFlagRight=tFalse;
			}
			
				
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

tResult cOpenCVTemplate::ProcessVideo(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImage_left,outputImage_center,outputImage_right,nonzeroimage_left,nonzeroimage_center,nonzeroimage_right,src1Gray,imagecut_left,imagecut_right,imagecut_center;
    const tVoid* l_pSrcBuffer;

    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
        {
            //copy the data to matrix (make a copy, not change the sample content itself!)
            memcpy(m_inputImage.data, l_pSrcBuffer, size_t(m_sInputFormat.nSize));
            //or just set the data pointer of matrix because we create a new matrix later one
            //m_inputImage.data = (uchar*)(l_pSrcBuffer);
           // Canny(m_inputImage, outputImage, 100, 200);// Detect Edges
	   cvtColor(m_inputImage, src1Gray, CV_RGB2GRAY);
	   //cvtColor(m_inputImage, src1Gray, CV_BGR2HSV);
	
	// Imagecut Left
	 imagecut_left = src1Gray(Range(450, 570), Range(0, 300)).clone(); 
	
	// Imagecut Center
	 imagecut_center = src1Gray(Range(450, 570), Range(470, 580)).clone();
	
	// Imagecut Right
	 imagecut_right = src1Gray(Range(450, 570), Range(700, 950)).clone(); 
	
	   if(!m_firstimage)
	   {
	   	m_previousimage_left=imagecut_left.clone();
		m_previousimage_center=imagecut_center.clone();
		m_previousimage_right=imagecut_right.clone();
	   	m_firstimage=tTrue;
	   }

	   absdiff(m_previousimage_left,imagecut_left,outputImage_left);
	   absdiff(m_previousimage_center,imagecut_center,outputImage_center);
	   absdiff(m_previousimage_right,imagecut_right,outputImage_right);

	   m_previousimage_left=imagecut_left.clone();
	   m_previousimage_center=imagecut_center.clone();
	   m_previousimage_right=imagecut_right.clone();

	   cv::Mat foregroundMask_left = cv::Mat::zeros(outputImage_left.rows, outputImage_left.cols, CV_8UC1);
	   cv::Mat foregroundMask_center = cv::Mat::zeros(outputImage_center.rows, outputImage_center.cols, CV_8UC1);
	   cv::Mat foregroundMask_right = cv::Mat::zeros(outputImage_right.rows, outputImage_right.cols, CV_8UC1);

   	float threshold = 15.0f;
   	float dist_left   = 0;
	float dist_center = 0;
	float dist_right  = 0;

// Left Image
   	 for(int j=0; j<outputImage_left.rows; ++j)       	 
	 for(int i=0; i<outputImage_left.cols; ++i)
            {
            cv::Vec3b pix_left = outputImage_left.at<cv::Vec3b>(j,i);

            dist_left = (pix_left[0]*pix_left[0] + pix_left[1]*pix_left[1] + pix_left[2]*pix_left[2]);
            dist_left = sqrt(dist_left);

		    if(dist_left>threshold)
		    {
		       	foregroundMask_left.at<unsigned char>(j,i) = 255;
		    }
            }
		findNonZero(foregroundMask_left,nonzeroimage_left);


// Center Image
   	 for(int j=0; j<outputImage_center.rows; ++j)       	 
	 for(int i=0; i<outputImage_center.cols; ++i)
            {
            cv::Vec3b pix_center = outputImage_center.at<cv::Vec3b>(j,i);

            dist_center = (pix_center[0]*pix_center[0] + pix_center[1]*pix_center[1] + pix_center[2]*pix_center[2]);
            dist_center = sqrt(dist_center);

		    if(dist_center>threshold)
		    {
		        foregroundMask_center.at<unsigned char>(j,i) = 255;
		    }
            }
            	findNonZero(foregroundMask_center,nonzeroimage_center);


// Right Image
   	 for(int j=0; j<outputImage_right.rows; ++j)    	 
	 for(int i=0; i<outputImage_right.cols; ++i)
            {
            cv::Vec3b pix_right = outputImage_right.at<cv::Vec3b>(j,i);

            dist_right = (pix_right[0]*pix_right[0] + pix_right[1]*pix_right[1] + pix_right[2]*pix_right[2]);
            dist_right = sqrt(dist_right);

		    if(dist_right>threshold)
		    {
		        foregroundMask_right.at<unsigned char>(j,i) = 255;
		    }
            }
            	findNonZero(foregroundMask_right,nonzeroimage_right);

	}
	 
	   tInt m_noofelem_left=nonzeroimage_left.total();
	   tInt m_noofelem_center=nonzeroimage_center.total();
	   tInt m_noofelem_right=nonzeroimage_right.total();

	   LOG_INFO(adtf_util::cString::Format("left: %d \t center: %d \t right %d", m_noofelem_left, m_noofelem_center, m_noofelem_right));


	tInt obj_thres_left   = 600;
	tInt obj_thres_center = 250;
	tInt obj_thres_right  = 500;


	// Check if Obstacle left
	if(m_noofelem_left>obj_thres_left)
	{
		// detected obstacle on left
		m_bTrafficOnLeft = tTrue;
		bFlagLeft=tFalse;
	}
	else
	{
		// no obstacle on left detected
		if(!bFlagLeft)
		{
			// save timestamp and set flag true
			timeStampLeft = _clock->GetStreamTime();
			bFlagLeft=tTrue;
		}
		else
		{
			// no obstacle on left side for more than [1s]
			if(_clock->GetStreamTime()-timeStampLeft > 1e6)
			{
				// no obstacle on left side
				m_bTrafficOnLeft = tFalse;
			}
		}
	}

	// Check if Obstacle center
	if(m_noofelem_center>obj_thres_center)
	{
		// detected obstacle on center
		m_bTrafficOnCenter = tTrue;
		bFlagCenter=tFalse;
	}
	else
	{
		// no obstacle on center detected
		if(!bFlagCenter)
		{
			// save timestamp and set flag true
			timeStampCenter = _clock->GetStreamTime();
			bFlagCenter=tTrue;
		}
		else
		{
			// no obstacle on center for more than [1s]
			if(_clock->GetStreamTime()-timeStampCenter > 1e6)
			{
				// no obstacle on center
				m_bTrafficOnCenter = tFalse;
			}
		}
	}


	// Check if Obstacle right
	if(m_noofelem_right>obj_thres_right)
	{
		// detected obstacle on rigth
		m_bTrafficOnRight = tTrue;
		bFlagRight=tFalse;
	}
	else
	{
		// no obstacle on right detected
		if(!bFlagRight)
		{
			// save timestamp and set flag true
			timeStampRight = _clock->GetStreamTime();
			bFlagRight=tTrue;
		}
		else
		{
			// no obstacle on right side for more than [1s]
			if(_clock->GetStreamTime()-timeStampRight > 1e6)
			{
				// no obstacle on right side
				m_bTrafficOnRight = tFalse;
			}
		}
	}
}

/*
    if (!outputImage_left.empty())
    {
        UpdateOutputImageFormat(outputImage_left);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage_left.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        outputImage_left.release();
    }
*/ pSample->Unlock(l_pSrcBuffer);

    RETURN_NOERROR;
}



tResult cOpenCVTemplate::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult cOpenCVTemplate::UpdateOutputImageFormat(const cv::Mat& outputImage_left)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage_left.total() * outputImage_left.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage_left, m_sOutputFormat);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}



tResult cOpenCVTemplate::TransmitOutput()
{
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleCheckTraffic;

	AllocMediaSample((tVoid**)&pMediaSampleCheckTraffic);
	
	tTimeStamp timestamp=_clock->GetStreamTime();


	// bool values
	cObjectPtr<IMediaSerializer> pSerializerCheckTraffic;
	m_pDescCheckTraffic->GetMediaSampleSerializer(&pSerializerCheckTraffic);
	tInt nSizeCheckTraffic = pSerializerCheckTraffic->GetDeserializedSize();
	pMediaSampleCheckTraffic->AllocBuffer(nSizeCheckTraffic);
	cObjectPtr<IMediaCoder> pCoderOutputCheckTraffic;
	m_pDescCheckTraffic->WriteLock(pMediaSampleCheckTraffic, &pCoderOutputCheckTraffic);
	pCoderOutputCheckTraffic->Set("bRigth", (tVoid*)&(m_bTrafficOnRight));
	pCoderOutputCheckTraffic->Set("bStraight", (tVoid*)&(m_bTrafficOnCenter));
	pCoderOutputCheckTraffic->Set("bLeft", (tVoid*)&(m_bTrafficOnLeft));
	pCoderOutputCheckTraffic->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescCheckTraffic->Unlock(pCoderOutputCheckTraffic);
	pMediaSampleCheckTraffic->SetTime(_clock->GetStreamTime());
	m_oCheckTraffic.Transmit(pMediaSampleCheckTraffic);

	RETURN_NOERROR;
}

