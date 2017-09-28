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
#include "ccrossline.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   ccrossline)

#define MD_RANGE_ROWS_R1 "Stopline::ROW1"
#define MD_RANGE_ROWS_R2 "Stopline::ROW2"
#define MD_RANGE_COLS_C1 "Stopline::COL1"
#define MD_RANGE_COLS_C2 "Stopline::COL2"
#define MD_GREY_TRESHOLD "Stopline::GreyThresholdValue"
#define MD_ENABLE_IMSHOW  "CrossDetection::Enableimshow"
#define CrossingDetect_PROP_BEFORENORM "CrossingDetect::ThresholdBeforeNorm"
#define CrossingDetect_PROP_TRESHOLD2 "CrossingDetect::ThresholdValue2"

#define CrossingDetect_PROP_CornerHarrisparamK "CrossingDetect::CornerHarrisparamK"
#define CrossingDetect_PROP_CornerHarrisblockSize "CrossingDetect::CornerHarrisblockSize"
#define CrossingDetect_PROP_CornerHarrisksize "CrossingDetect::CornerHarrisksize"


ccrossline::ccrossline(const tChar* __info) : cFilter(__info)
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


 SetPropertyInt(MD_RANGE_ROWS_R1, 520);
    SetPropertyBool(MD_RANGE_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2, 600);
    SetPropertyBool(MD_RANGE_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2 NSSUBPROP_DESCRIPTION,"MarkerDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1, 600);
    SetPropertyBool(MD_RANGE_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2, 800);
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


    SetPropertyBool("Debug Output to Console",false);
}

ccrossline::~ccrossline()
{
}

tResult ccrossline::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult ccrossline::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult ccrossline::Init(tInitStage eStage, __exception)
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
	RETURN_IF_FAILED(m_Stoplinedist.Create("distance cross line", pTypeSignaldistancestop, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_Stoplinedist));	
	// Media description for Signal Value
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Create Pins for selection of crossing edges
        //create pin for steering signal output
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesEdgeselect));
        RETURN_IF_FAILED(m_edgeselect.Create("Crossing select", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_edgeselect));
        //distance outputs edges
        //edge 1
        tChar const * strDescSignaldistanceedgeone = pDescManager->GetMediaDescription("tEdgeStruct");
        RETURN_IF_POINTER_NULL(strDescSignaldistanceedgeone);
        cObjectPtr<IMediaType> pTypeSignaldistanceedgeone = new cMediaType(0, 0, 0, "tEdgeStruct", strDescSignaldistanceedgeone, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignaldistanceedgeone->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesEdgedetect));
        RETURN_IF_FAILED(m_Edgeonedetect.Create("Distance_Edge", pTypeSignaldistanceedgeone, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_Edgeonedetect));
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
	edge_detect=tFalse;
	distance_prev=0;
	distance=0;
	distance_edge=0;
	Orientation=0;
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("StopLine Detection: Invalid Input Format for this filter");
        }
    }

    RETURN_NOERROR;
}
tResult ccrossline::ReadProperties(const tChar* strPropertyName)
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
        m_bDebugOutputEnabled = GetPropertyBool("Debug Output to Console");

	RETURN_NOERROR;
}

tResult ccrossline::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult ccrossline::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
	    ProcessEdge(pMediaSample);
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

tResult ccrossline::ProcessVideo(IMediaSample* pSample)
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
	imagecut = m_inputImage(Range(520, 600), Range(680, 850)).clone(); 
        //Erzeugen eines Graustufenbildes
	GaussianBlur(imagecut,imagecut2,Size(13,13),0,0, BORDER_DEFAULT); 
        cvtColor(imagecut2, grey, CV_BGR2GRAY);
        threshold(grey, greythresh, 130, 500, THRESH_BINARY);
	
	Mat temp(greythresh.size(), CV_8UC1);
	Mat element = getStructuringElement(MORPH_RECT, Size(4, 4));
        erode(greythresh, greythresh, element);
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
	cannysize = linecanny.size();
        vector<Vec2f> lines;
        // How long must be the line to be detected as roadlines -> HoughLines
	HoughLines(linecanny, lines, 1, CV_PI / 180, 75, 0,0);
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
                {
                    rep = 1;		// One point of the Line is repeated, ignore the whole line
                }
            }	
          
          if ((rep == 0) && ((thetaAll[i]>CV_PI*0.4444) && (thetaAll[i]<CV_PI*0.5556)))
          
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
            if(m_bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: X not %f Y not %f",x0,y0));
            
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	    tInt newv=80-y0;
	    distanceCal = 0.0001*pow(newv,3)-0.0045*pow(newv,2)+0.8652*newv+28.8300;
            DistPack[i] = distanceCal;
        }
	if (sizeNoRep==0)
	{
	distance = 0;
	Orientation = 0;
	stop_detect=tFalse;
	}
	
	if (sizeNoRep == 1)
	{
	distance = DistPack[0];
	Orientation = thetaNoRep[0];
	 tFloat32 difference_dist=abs(distance_prev-distance);
		if(difference_dist>=60)
		{
			stop_detect=tFalse;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(0,timeStamp);
		}
		else
		{
			stop_detect=tTrue;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(distance,timeStamp);
		}
		distance_prev=distance;
	}
	if (sizeNoRep == 2)
	{
        	if (DistPack[0]<DistPack[1])
        	{
		distance = DistPack[0];
		Orientation = thetaNoRep[0];
		tFloat32 difference_dist=abs(distance_prev-distance);
		if(difference_dist>=60)
		{
			stop_detect=tFalse;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(0,timeStamp);
		}
		else
		{
			stop_detect=tTrue;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(distance,timeStamp);
		}
		distance_prev=distance;	
		}

		else
		{
		distance = DistPack[1];
		Orientation = thetaNoRep[1];
		stop_detect=tFalse;		
		}	
        	}
	
	}
	Orientation = Orientation*180/CV_PI;
        if(m_bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: distance %f Orientation %f", distance,Orientation));
	writeOutputs1(distance,timeStamp);

  
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

tResult ccrossline::ProcessEdge(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    const tVoid* l_pSrcBuffer;
    cObjectPtr<IMediaSample> pNewRGBSample;
    cv::Mat grey_image,greythresh,canny_image,outputImage,m_matThres2,image_right,outputImage2;

	/*switch(m_edgeselected)
	{
	case 0:
		{
			col1=600;col2=750;
			break;
		}
	case 1:
		{
			col1=300;col2=400;
			break;
		}
	case 2:
		{
			col1=450;col2=750;
			break;
		}
	default:
		{
		break;
		}

	}*/
    tUInt32 timeStamp = 0;
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
   	Mat image_right= image(cv::Range(520, 600), cv::Range(600, 750)).clone();
	
	vector<Vec2f> hough_lines;
	cvtColor(image_right ,grey_image,CV_BGR2GRAY);// Grey Image
	threshold(grey_image, greythresh, 200, 255,THRESH_BINARY);// Generate Binary Image
	greythresh=greythresh-150;
	medianBlur(greythresh, outputImage,3);
	cornerHarris(outputImage, outputImage, 5, 11, 0.08, BORDER_DEFAULT);// preprocess corners
	
	threshold(outputImage, outputImage, 0.01, 255,THRESH_BINARY); // in case of no real corners set everything to zero - otherwise everywhere corners because of normalize		     
	threshold(outputImage, outputImage, 40, 255,THRESH_TRUNC);
	normalize(outputImage, outputImage, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
	convertScaleAbs(outputImage, outputImage); 
	threshold(outputImage, m_matThres2,  120, 255,THRESH_BINARY); 
	cv::Mat nonzerocoordinates;
	vector<Point> pts;
	findNonZero(m_matThres2,pts);
	tInt euclidean_distance = 20;

    // Apply partition 
    // All pixels within the the given distance will belong to the same cluster

    vector<tInt> labels;

    // With functor
    tInt n_labels = partition(pts, labels, EuclideanDistanceFunctor(euclidean_distance));
    // Store all points in same cluster, and compute centroids
    vector<vector<Point> > clusters(n_labels);
    vector<Point> centroids(n_labels, Point(0,0));
 
    for (tInt i = 0; i < pts.size(); ++i)
    {
        clusters[labels[i]].push_back(pts[i]);
        centroids[labels[i]] += pts[i];
    }
   
    for (tInt i = 0; i < n_labels; ++i)
    {
        centroids[i].x /= clusters[i].size();
        centroids[i].y /= clusters[i].size();

    }
    sort(centroids.begin(), centroids.end(), myobject);
   // LOG_INFO(adtf_util::cString::Format("cENTROID1 Y %d n lable %d",centroids[0].y,n_labels));
   // LOG_INFO(adtf_util::cString::Format("cENTROID2 Y %d n lable %d",centroids[1].y,n_labels));
   // LOG_INFO(adtf_util::cString::Format("cENTROID3 Y %d n lable %d",centroids[2].y,n_labels));

    vector<Point> centroidsnorep(n_labels, Point(0,0));
    tInt Ynorep[5];
    tInt norep=0;
    tFloat32 distanceCal;
    for(tInt i=0; i<n_labels;i++)
    {
	tInt rep=0;
	for(tInt j=0; j<i;j++)
	{	
	if(abs(centroids[i].y-centroids[j].y) <= 5)
	{
	rep = 1;
	}
        }
	if ( rep==0 )
	{
	Ynorep[norep]=centroids[i].y;
	//LOG_INFO(adtf_util::cString::Format("in %d no rep %d",Ynorep[norep],norep));
	norep++;
        }
    }
    tFloat32 DistPack[norep];
    for(tInt i = 0; i < norep; i++ )
    {
	 tInt newv=80-Ynorep[i];
	 distanceCal = 0.0001*pow(newv,3)-0.0045*pow(newv,2)+0.8652*newv+28.8300;
         DistPack[i] = distanceCal;
    }
    if (norep==0)
    {
	distance_edge = 0;
        edge_detect=tFalse;
    } 
    if (norep == 1)
    {
    distance_edge = DistPack[0];
    tFloat32 difference_dist=abs(distance_prev-distance_edge);
    if(difference_dist>=60)
    {
	edge_detect=tFalse;
	timeStamp = _clock->GetStreamTime() / 1000;
	writeOutputs2(0,timeStamp);
    }
    else
    {
	edge_detect=tTrue;
	timeStamp = _clock->GetStreamTime() / 1000;
	writeOutputs2(distance_edge,timeStamp);
    }
	distance_prev=distance_edge;
    }
    if (norep == 2)
    {
       	if (DistPack[0]<DistPack[1])
       	{
	distance_edge = DistPack[0];
	tFloat32 difference_dist=abs(distance_prev-distance_edge);
	if(difference_dist>=60)
            {
		edge_detect=tFalse;
		timeStamp = _clock->GetStreamTime() / 1000;
		writeOutputs2(0,timeStamp);
	    }
	else
            {
		edge_detect=tTrue;
		timeStamp = _clock->GetStreamTime() / 1000;
		writeOutputs2(distance_edge,timeStamp);
	    }
	distance_prev=distance_edge;	
	}

	else
	    {
	        distance_edge = DistPack[1];
		stop_detect=tFalse;		
	    }	
    }
    writeOutputs2(distance_edge,timeStamp);
    if (!m_matThres2.empty())
    {
        UpdateOutputImageFormat2(m_matThres2);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, m_matThres2.data);

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
    pSample->Unlock(l_pSrcBuffer);
    }
    RETURN_NOERROR;
}

tResult ccrossline::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        if(m_bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

tResult ccrossline::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        if(m_bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult ccrossline::UpdateOutputImageFormat2(const cv::Mat& outputImage2)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage2.total() * outputImage2.elemSize()) != m_sOutputFormat2.nSize)
    {
        Mat2BmpFormat(outputImage2, m_sOutputFormat2);

        if(m_bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat2.nWidth, m_sOutputFormat2.nHeight, m_sOutputFormat2.nBytesPerLine, m_sOutputFormat2.nSize, m_sOutputFormat2.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin1.SetFormat(&m_sOutputFormat2, NULL);
    }
    RETURN_NOERROR;
}

tResult ccrossline::writeOutputs1(tFloat32 distance, tUInt32 timeStamp)
{
   
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);

	// Line distance
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
tResult ccrossline::writeOutputs2(tFloat32 distance_edge, tUInt32 timeStamp)
{
   
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);

	// Edge distance
	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pDesEdgedetect->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDesEdgedetect->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("bValue1", (tVoid*)&(edge_detect));
	pCoderOutputaccelerate->Set("f32Distance1", (tVoid*)&(distance_edge));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp1", (tVoid*)&timeStamp);
	m_pDesEdgedetect->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_Edgeonedetect.Transmit(pMediaSampleaccelerate);

}
