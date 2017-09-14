/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "cCrossingdetector.h"

#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

#define MD_RANGE_ROWS_R1 "CrossDetection::ROW1"
#define MD_RANGE_ROWS_R2 "CrossDetection::ROW2"
#define MD_RANGE_COLS_C1 "CrossDetection::COL1"
#define MD_RANGE_COLS_C2 "CrossDetection::COL2"
#define MD_GREY_TRESHOLD "CrossDetection::GreyThresholdValue"
#define MD_ENABLE_IMSHOW  "CrossDetection::Enableimshow"
#define CrossingDetect_PROP_BEFORENORM "CrossingDetect::ThresholdBeforeNorm"
#define CrossingDetect_PROP_TRESHOLD2 "CrossingDetect::ThresholdValue2"

#define CrossingDetect_PROP_CornerHarrisparamK "CrossingDetect::CornerHarrisparamK"
#define CrossingDetect_PROP_CornerHarrisblockSize "CrossingDetect::CornerHarrisblockSize"
#define CrossingDetect_PROP_CornerHarrisksize "CrossingDetect::CornerHarrisksize"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Crossing_Detector", OID_ADTF_CROSSINGDETECTOR_FILTER,cCrossingdetector);


cCrossingdetector::cCrossingdetector(const tChar* __info) : cFilter(__info)
{
	SetPropertyBool(MD_ENABLE_IMSHOW,tFalse);
    SetPropertyBool(MD_ENABLE_IMSHOW NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_ENABLE_IMSHOW NSSUBPROP_DESCRIPTION, "If true imshow will be enabled");

    SetPropertyInt(MD_GREY_TRESHOLD, 200);
    SetPropertyBool(MD_GREY_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_GREY_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");
	
	SetPropertyFloat(CrossingDetect_PROP_TRESHOLD2, 120);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The third threshold value.");


    SetPropertyInt(MD_RANGE_ROWS_R1, 300);
    SetPropertyBool(MD_RANGE_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2, 480);
    SetPropertyBool(MD_RANGE_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2 NSSUBPROP_DESCRIPTION,"MarkerDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1, 300);
    SetPropertyBool(MD_RANGE_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2, 640);
    SetPropertyBool(MD_RANGE_COLS_C2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2 NSSUBPROP_DESCRIPTION, "MarkerDetection::The second col C2 to be taken (,C1) (,C2)");
	
	SetPropertyInt(CrossingDetect_PROP_BEFORENORM, 40);
    SetPropertyBool(CrossingDetect_PROP_BEFORENORM NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_BEFORENORM NSSUBPROP_DESCRIPTION, "The threshold value (2nd) before the normalization.");
	
	
	SetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize, 5);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(CrossingDetect_PROP_CornerHarrisksize, 11);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(CrossingDetect_PROP_CornerHarrisparamK, 0.08);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");
	//Bool value Initialization
	
	// distance value initialization
    edge_dst[4]=0;
	//stop line distance
	stop_line_dst=0;
	
	ReadProperties(NULL);
}

cCrossingdetector::~cCrossingdetector()
{
}


tResult cCrossingdetector::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cCrossingdetector::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cCrossingdetector::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
        // Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));
	// Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin1.Create("Video_Output Raw", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin1));
		/*
		//distance outputs
		//edge 1
			tChar const * strDescSignaldistanceedgeone = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignaldistanceedgeone);
			cObjectPtr<IMediaType> pTypeSignaldistanceedgeone = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceedgeone, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignaldistanceedgeone->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesEdgedetect));
			RETURN_IF_FAILED(m_Edgeonedetect.Create("Distance_Edge", pTypeSignaldistanceedgeone, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Edgeonedetect));	
		//stop line	
			tChar const * strDescSignaldistancestop = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignaldistancestop);
			cObjectPtr<IMediaType> pTypeSignaldistancestop = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistancestop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignaldistancestop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesStoplinedist));
			RETURN_IF_FAILED(m_Stoplinedist.Create("Distance_Edge one", pTypeSignaldistancestop, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Stoplinedist));	
		// Flag outputs
		// edge one
			tChar const * strDescSignaleone = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignaleone);
			cObjectPtr<IMediaType> pTypeSignaleone = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaleone, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignaleone->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesEdgeoneflag));
			RETURN_IF_FAILED(m_Edgeoneflag.Create("start", pTypeSignaleone, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Edgeoneflag));
			*/
			
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
    }

    RETURN_NOERROR;
}



tResult cCrossingdetector::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cCrossingdetector::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}

tResult cCrossingdetector::ReadProperties(const tChar* strPropertyName)
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
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_BEFORENORM))
			{
				m_nThresholdValueBeforeNorm = GetPropertyInt(CrossingDetect_PROP_BEFORENORM);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLD2))
			{
				m_nThresholdValue2 = GetPropertyInt(CrossingDetect_PROP_TRESHOLD2);
			}
			  if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisblockSize))
			{
				m_nCornerHarrisblockSize = GetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize);
			}
			
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisparamK))
			{
				float f;
				f = GetPropertyFloat(CrossingDetect_PROP_CornerHarrisparamK);
				m_nCornerHarrisparamK = static_cast<double>(f);
			}

			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisblockSize))
			{
				m_nCornerHarrisblockSize = GetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize);
			}
			RETURN_NOERROR;
		}

tResult cCrossingdetector::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
tTimeStamp InputTimeStamp;
	    InputTimeStamp = pMediaSample->GetTime();
            if (m_bFirstFrame)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrame = tFalse;
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

tResult cCrossingdetector::ProcessVideo(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    const tVoid* l_pSrcBuffer;
    cObjectPtr<IMediaSample> pNewRGBSample;
    cv::Mat grey_image,greythresh,canny_image,outputImage,m_matThres2,image_right,outputImage2;
    int x[100],y[100],pointsx[100],pointsy[100];

    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat, be sure to select the right pixelformat
        // ### START Image Processing ###
        IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        img->imageData = (char*)l_pSrcBuffer;
        //Übergang von OpenCV1 auf OpenCV2
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
   	Mat image_right= image(cv::Range(row1, row2), cv::Range(col1, col2)).clone();
	
	vector<Vec2f> hough_lines;
	//GaussianBlur(image_right, image_right, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
	cvtColor(image_right ,grey_image,CV_BGR2GRAY);// Grey Image
	threshold(grey_image, greythresh, m_nThresholdValue, 255,THRESH_BINARY);// Generate Binary Image
	greythresh=greythresh-m_nThresholdValue;
	medianBlur(greythresh, outputImage,3);
	cornerHarris(outputImage, outputImage, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
	//goodFeaturesToTrack(outputImage, outputImage2,10,0.01,20,NULL,3,1,0.04);
	threshold(outputImage, outputImage, 0.01, 255,THRESH_BINARY); // in case of no real corners set everything to zero - otherwise everywhere corners because of normalize		     
	//threshold(outputImage, outputImage, m_nThresholdValueBeforeNorm, 255,THRESH_TRUNC);
	normalize(outputImage, outputImage, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
	convertScaleAbs(outputImage, outputImage); 
	threshold(outputImage, m_matThres2,  m_nThresholdValue2, 255,THRESH_BINARY); 
	cv::Mat nonzerocoordinates;
	vector<Point> pts;
	findNonZero(m_matThres2,pts);
	/*int noofpoint=nonzerocoordinates.total();
	LOG_INFO(adtf_util::cString::Format(" size %d",noofpoint));
	int finalPoint = 0;
	for(int i=0;i<nonzerocoordinates.total();i++)
	{
		x[i]=nonzerocoordinates.at<Point>(i).x;
		y[i]=nonzerocoordinates.at<Point>(i).y;
		//LOG_INFO(adtf_util::cString::Format("i is %d ,X : %d,Y :%d",i,x[i],y[i]));
		
		} */
	int euclidean_distance = 20;

    // Apply partition 
    // All pixels within the the given distance will belong to the same cluster

    vector<int> labels;

    // With functor
    int n_labels = partition(pts, labels, EuclideanDistanceFunctor(euclidean_distance));

    // With lambda function
    //int th2 = euclidean_distance * euclidean_distance;
   //int n_labels = partition(pts, labels, EuclideanDistanceFunctor());


    // Store all points in same cluster, and compute centroids
    vector<vector<Point> > clusters(n_labels);
    vector<Point> centroids(n_labels, Point(0,0));
 
    for (int i = 0; i < pts.size(); ++i)
    {
        clusters[labels[i]].push_back(pts[i]);
        centroids[labels[i]] += pts[i];
    }
   	tInt abcd[n_labels];
    for (int i = 0; i < n_labels; ++i)
    {
        centroids[i].x /= clusters[i].size();
        centroids[i].y /= clusters[i].size();

    }
	sort(centroids.begin(), centroids.end(), myobject);
	LOG_INFO(adtf_util::cString::Format("labels: %d X sorted : %d,Y sorted :%d",n_labels,centroids[0].x ,centroids[0].y));
	for (int i = 0; i < n_labels; ++i)
    {	
	// tInt newv=80-centroids[i].y;	
	tInt RightBottomY =  80-centroids[0].y;
	tInt RightBottomDistance = 0.0001*pow(newv,3)-0.0045*pow(newv,2)+0.8652*newv+28.8300;
	LOG_INFO(adtf_util::cString::Format("Output points  FINAL DISTANCE %d",distance));
    }

        pSample->Unlock(l_pSrcBuffer);
    }
    if (!grey_image.empty())
    {
        UpdateOutputImageFormat2(grey_image);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, grey_image.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));
        //RETURN_IF_FAILED(m_oVideoOutputPin1.Transmit(pMediaSample));
        grey_image.release();
    }
    if (!m_matThres2.empty())
    {
        UpdateOutputImageFormat(m_matThres2);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat2.nWidth, m_sOutputFormat2.nHeight, m_sOutputFormat2.nBitsPerPixel, m_sOutputFormat2.nBytesPerLine, m_matThres2.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin1.Transmit(pMediaSample));
        //RETURN_IF_FAILED(m_oVideoOutputPin1.Transmit(pMediaSample));

        m_matThres2.release();
    }

    RETURN_NOERROR;
}

tResult cCrossingdetector::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult cCrossingdetector::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin1.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult cCrossingdetector::UpdateOutputImageFormat2(const cv::Mat& outputImage2)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage2.total() * outputImage2.elemSize()) != m_sOutputFormat2.nSize)
    {
        Mat2BmpFormat(outputImage2, m_sOutputFormat2);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat2.nWidth, m_sOutputFormat2.nHeight, m_sOutputFormat2.nBytesPerLine, m_sOutputFormat2.nSize, m_sOutputFormat2.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat2, NULL);
    }
    RETURN_NOERROR;
}
