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
#include "lanefollower.h"

using namespace std;
using namespace cv;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Lane_v_2", OID_ADTF_LaneFollower, clanefollower)
clanefollower::clanefollower(const tChar* __info) : cFilter(__info)
{
    SetPropertyInt("KpsiRIGHT", 29);
    SetPropertyBool("KpsiRIGHT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("KpsiLEFT", 28);
    SetPropertyBool("KpsiLEFT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("Ky", 38);
    SetPropertyBool("Ky" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("vMax", -10);
    SetPropertyBool("vMax" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("vMin", -10);  // Ignore the variable
    SetPropertyBool("vMin" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("cameraoffset", 0);
    SetPropertyBool("cameraoffset" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("thresholdvalue", 150);
    SetPropertyBool("thresholdvalue" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyBool("imshow", tFalse);
    SetPropertyBool("imshow" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("houghlines", 63);
    SetPropertyBool("houghlines" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("row1", 460);
    SetPropertyBool("row1" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("row2", 600);
    SetPropertyBool("row2" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("col1", 400);
    SetPropertyBool("col1" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("col2", 880);
    SetPropertyBool("col2" NSSUBPROP_ISCHANGEABLE, tTrue);
	m_time=tFalse;
    
    writeZeroOutputs = tFalse;
    blind_count = 0;
	//m_szIdsOutputSpeedSet = tFalse;
}

clanefollower::~clanefollower()
{

}

tResult clanefollower::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult clanefollower::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult clanefollower::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // steering output
        tChar const * strDescSignalValueOutputSteer = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputSteer);
        cObjectPtr<IMediaType> pTypeSignalValueOutputSteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputSteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputSteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputSteer));
        RETURN_IF_FAILED(m_oSteer.Create("steering_angle", pTypeSignalValueOutputSteer, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteer));
        // acceleration output
        tChar const * strDescSignalValueOutputAccel = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputAccel);
        cObjectPtr<IMediaType> pTypeSignalValueOutputAccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputAccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputAccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputAccel));
        RETURN_IF_FAILED(m_oAccelerate.Create("acceleration", pTypeSignalValueOutputAccel, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oAccelerate));


        // Video output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

    }
    else if (eStage == StageNormal)
    {
	firstFrame = tTrue;
        imagecount = 0;

        KpsiRIGHT = GetPropertyInt("KpsiRIGHT");
        KpsiLEFT = GetPropertyInt("KpsiLEFT");
        Ky = GetPropertyInt("Ky");
        vMax = GetPropertyInt("vMax");
        vMin = GetPropertyInt("vMin");
        cameraoffest = GetPropertyInt("cameraoffset");
        thresholdvalue = GetPropertyInt("thresholdvalue");
        enable_imshow = GetPropertyBool("imshow");
        houghlinesvalue = GetPropertyInt("houghlines");
        row1 = GetPropertyInt("row1");
        row2 = GetPropertyInt("row2");
        col1 = GetPropertyInt("col1");
        col2 = GetPropertyInt("col2");

    }

    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
	accel =0;
       steeringAngle = 0;
       steeringAngle_previous = 0;
        mean_theta_previous = 0;
          // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
	if(!m_time)
	{
	starttime= _clock->GetStreamTime();
	}    
    }

    RETURN_NOERROR;
}



tResult clanefollower::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult clanefollower::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

	//if(_clock->GetStreamTime()-starttime< 3000000)
	//{
	//	accel=0;
	//	writeOutputs(0);
	//	RETURN_NOERROR;
	//}
		


        if (pSource == &m_oVideoInputPin)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
           }
    	    tUInt32 timeStamp = 0;
            ProcessVideo(pMediaSample);
	    if (firstFrame)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL)
                {
                    LOG_ERROR("Spurerkennung: No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight = pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                firstFrame = tFalse;

            }
            
            steeringAngle = evaluateSteeringAngle(pMediaSample);
		if(blind_count > 30)
              {
                  LOG_INFO(adtf_util::cString::Format("Output: Lakshman acceleration loop"));
            	accel = vMax;
                }
              else
               {
			LOG_INFO(adtf_util::cString::Format("Output: Lakshman acceleration loop2"));
               		 accel = vMin;
                }
            
            timeStamp = _clock->GetStreamTime() / 1000;
            LOG_INFO(adtf_util::cString::Format("TimeStamp %d, accel %f, steer %f",timeStamp,accel,steeringAngle));
           // writeOutputs(timeStamp);
		writeOutputs1(accel,timeStamp);
		writeOutputs2(steeringAngle,timeStamp);
	    	

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

tResult clanefollower::ProcessVideo(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImage;
    const tVoid* l_pSrcBuffer;
    Mat imagecut,color_dst, imagecut2,imagecut3,imagecut4;
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
            Canny(m_inputImage, outputImage, 100, 200);// Detect Edges

        
        imagecut = m_inputImage(Range(row1, row2), Range(col1, col2)).clone(); //Range(200, 400), Range(99, 600)
        
	GaussianBlur(imagecut,imagecut2,Size(11,11),0,0, BORDER_DEFAULT); 
        //Erzeugen eines Graustufenbildes
        cvtColor(imagecut2, grey, CV_BGR2GRAY);
        
        
        //Treshold um Binärbild zu erhalten
        threshold(grey, greythresh, thresholdvalue, 585, THRESH_BINARY);
        //namedWindow("Grey Threshold");
        //imshow("Grey Threshold",greythresh);
        //waitKey(1);
        //Kantendedektion
        //Canny(greythresh, linecanny, 250, 350);	
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
        cannysize = linecanny.size();
        vector<Vec2f> lines;
        HoughLines(linecanny, lines, 2, CV_PI / 180, houghlinesvalue, 0, 0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );

        tFloat32 thetaAll[1000];
        tFloat32 rhoAll[1000];
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
                if (abs(thetaAll[i] - thetaAll[j]) <= 0.2)
                {
                    rep = 1;		// Line is repeated, ignore this one
                }
            }
            if ((rep == 0) && (thetaAll[i]<CV_PI*0.4 || thetaAll[i]>CV_PI*0.60)) //theta not between 72 and 108
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
		//LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta %f",sizeNoRep,thetaNoRep[i]));
		//LOG_INFO(adtf_util::cString::Format("Output HoughLine: size %d th0 %f rh0  %f",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
		//LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %d",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
            }

        }
        for(tInt i = 0; i < sizeNoRep+1; i++ )
        {
            double a = cos(thetaNoRep[i]), b = sin(thetaNoRep[i]);
            double x0 = a*rhoNoRep[i], y0 = b*rhoNoRep[i];
	    //LOG_INFO(adtf_util::cString::Format("Y not %f",y0));
            LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[0],rhoNoRep[0]));
            LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[1],rhoNoRep[1]));
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
        }
	tFloat32 mean_theta = 0;
        tFloat32 x1, x2, deltaR, deltaL, deltaRL = 0;
        for (tInt i = 0; i < sizeNoRep; i++)  // foreach distinct lines
        {
            if (thetaNoRep[i] >= CV_PI / 2)
                mean_theta += thetaNoRep[i] - (tFloat32)CV_PI;
            else
                mean_theta += thetaNoRep[i];
	LOG_INFO(adtf_util::cString::Format("Mean theta is %f",mean_theta));
        }
	if (sizeNoRep >= 2)
{
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (260 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            x2 = rhoNoRep[1] * cos(thetaNoRep[1]) - (260 - rhoNoRep[1] * sin(thetaNoRep[1]))*tan(thetaNoRep[1]);
		LOG_INFO(adtf_util::cString::Format("Output: x1 %f : x2 %f",x1,x2));
            if (x1 > x2)
{
                deltaR = x1 - 240;
                deltaL = 240 - x2;
			
            }
            else
{
                deltaR = x2 - 240;
                deltaL = 240 - x1;
            }
            deltaRL = deltaR - deltaL  ;
        }
	if (sizeNoRep == 1){
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (260 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            if (x1 > 240){
                deltaRL = x1 - 240;
            }
            else{
                deltaRL = x1; // deltaRL = x1 - 0;
            }
        }
        if (sizeNoRep == 0)
        {
        blind_count++;
        }
	LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaL %f",deltaL));	
        LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaR %f",deltaR));
	LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaRL %f",deltaRL));
	LOG_INFO(adtf_util::cString::Format("Output: Lakshman blindcount %f",blind_count));
	if (abs(deltaRL * Ky / 1000) >= 25)
            deltaRL = 0;

        if ((sizeNoRep != 0) && (sizeNoRep != 1)){
            mean_theta /= sizeNoRep;
	//LOG_INFO(adtf_util::cString::Format("Mean theta%f",mean_theta));
            if (mean_theta > 0)
                steeringAngle = KpsiRIGHT * mean_theta + Ky / 1000 * deltaRL;
            else
                steeringAngle = KpsiLEFT * mean_theta + Ky / 1000 * deltaRL;
        }
        else if (sizeNoRep == 1)
        if (mean_theta > 0)
            {
            blind_count = 0;
            steeringAngle = KpsiRIGHT * mean_theta / 2 + Ky / 1000 * deltaRL;
            }
        else
            {
            blind_count = 0;
            steeringAngle = KpsiLEFT * mean_theta / 2 + Ky / 1000 * deltaRL;
            }
        else
            {
            steeringAngle = steeringAngle_previous;
            blind_count++;
            }

      	 	steeringAngle_previous = steeringAngle;
               mean_theta_previous = mean_theta;

/*
   this->steeringAng = 0;
	
    if (steeringAngle != 0)
        	this->steeringAng = (steeringAngle*0.856) + 0;		// after 90 is -ve, then 0.856 refers to 60/70
	
   	  
   	if (this->steeringAng > 30)
  	    this->steeringAng = 30;
   	if (this->steeringAng < -30)
    	   this->steeringAng = -30;

    	return this->steeringAng;
	//return this->steeringAngle;
*/
       //return this->steeringAngle;

	
	
	
	  }

        pSample->Unlock(l_pSrcBuffer);


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
    }/*
if (!greythresh.empty())
    {
        UpdateOutputImageFormat(greythresh);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, greythresh.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        greythresh.release();
    }*/




    RETURN_NOERROR;
}
tFloat32 clanefollower::evaluateSteeringAngle(IMediaSample* pSample)
{
    // VideoInput
    RETURN_IF_POINTER_NULL(pSample);

    cObjectPtr<IMediaSample> pNewRGBSample;

    const tVoid* l_pSrcBuffer;

    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {

        IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        img->imageData = (char*)l_pSrcBuffer;
        //Übergang von OpenCV1 auf OpenCV2
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
        pSample->Unlock(l_pSrcBuffer);
        //Zuschneiden des Bildes
	Mat imagecut,color_dst,imagecut2,imagecut3,imagecut4;
           imagecut = m_inputImage(Range(row1, row2), Range(col1, col2)).clone(); //Range(200, 400), Range(99, 600)
        
	GaussianBlur(imagecut,imagecut2,Size(11,11),0,0, BORDER_DEFAULT); 
        //Erzeugen eines Graustufenbildes
        cvtColor(imagecut2, grey, CV_BGR2GRAY);
        
        
        //Treshold um Binärbild zu erhalten
        threshold(grey, greythresh, thresholdvalue, 600, THRESH_BINARY);
        //namedWindow("Grey Threshold");
        //imshow("Grey Threshold",greythresh);
        //waitKey(1);
        //Kantendedektion
        //Canny(greythresh, linecanny, 250, 350);	
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
        cannysize = linecanny.size();
        vector<Vec2f> lines;
        HoughLines(linecanny, lines, 2, CV_PI / 180, houghlinesvalue, 0, 0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );

        tFloat32 thetaAll[1000];
        tFloat32 rhoAll[1000];
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
                if (abs(thetaAll[i] - thetaAll[j]) <= 0.2)
                {
                    rep = 1;		// Line is repeated, ignore this one
                }
            }
            if ((rep == 0) && (thetaAll[i]<CV_PI*0.40 || thetaAll[i]>CV_PI*0.60)) //theta not between 72 and 108
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
		//LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta %f",sizeNoRep,thetaNoRep[i]));
		LOG_INFO(adtf_util::cString::Format("Output HoughLine: size %d th0 %f rh0  %f",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
		//LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %d",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
            }

        }
        for( size_t i = 0; i < sizeNoRep+1; i++ )
        {
            double a = cos(thetaNoRep[i]), b = sin(thetaNoRep[i]);
            double x0 = a*rhoNoRep[i], y0 = b*rhoNoRep[i];
	    LOG_INFO(adtf_util::cString::Format("Y not %f",y0));
            LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[0],rhoNoRep[0]));
            LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[1],rhoNoRep[1]));
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
        }
	tFloat32 mean_theta = 0;
        tFloat32 x1, x2, deltaR, deltaL, deltaRL = 0;
        for (tInt i = 0; i < sizeNoRep; i++)  // foreach distinct lines
        {
            if (thetaNoRep[i] >= CV_PI / 2)
                mean_theta += thetaNoRep[i] - (tFloat32)CV_PI;
            else
                mean_theta += thetaNoRep[i];
	
        }
	LOG_INFO(adtf_util::cString::Format("Mean theta is %f",mean_theta));
	if (sizeNoRep >= 2)
{
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (260 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            x2 = rhoNoRep[1] * cos(thetaNoRep[1]) - (260 - rhoNoRep[1] * sin(thetaNoRep[1]))*tan(thetaNoRep[1]);
		LOG_INFO(adtf_util::cString::Format("Output: x1 %f : x2 %f",x1,x2));
            if (x1 > x2)
{
                deltaR = x1 - 240;
                deltaL = 240 - x2;
			
            }
            else
{
                deltaR = x2 - 240;
                deltaL = 240 - x1;
            }
            deltaRL = deltaR - deltaL  ;
        }
	if (sizeNoRep == 1){
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (260 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            if (x1 > 240){
                deltaRL = x1 - 240;
            }
            else{
                deltaRL = x1; // deltaRL = x1 - 0;
            }
        }
        if (sizeNoRep == 0)
        {
        blind_count++;
        }

        if (abs(deltaRL * Ky / 1000) >= 25)
            deltaRL = 0;

        if ((sizeNoRep != 0) && (sizeNoRep != 1)){
            mean_theta /= sizeNoRep;
	//LOG_INFO(adtf_util::cString::Format("Mean theta%f",mean_theta));
            if (mean_theta > 0)
                steeringAngle = KpsiRIGHT * mean_theta + Ky / 1000 * deltaRL;
            else
                steeringAngle = KpsiLEFT * mean_theta + Ky / 1000 * deltaRL;
        }
        else if (sizeNoRep == 1)
        if (mean_theta > 0)
            {
            blind_count = 0;
            steeringAngle = KpsiRIGHT * mean_theta / 2 + Ky / 1000 * deltaRL;
            }
        else
            {
            blind_count = 0;
            steeringAngle = KpsiLEFT * mean_theta / 2 + Ky / 1000 * deltaRL;
            }
        else
            {
            steeringAngle = steeringAngle_previous;
            blind_count++;
            }

        steeringAngle_previous = steeringAngle;
        mean_theta_previous = mean_theta;

    }
	LOG_INFO(adtf_util::cString::Format("steeering angle: %f", steeringAngle));

	
   	this->steeringAng = 0;
	
       if (steeringAngle != 0)
        	this->steeringAng = (steeringAngle*0.856) + 0;		// after 90 is -ve, then 0.856 refers to 60/70
	
   	  
   	if (this->steeringAng > 30)
  	    this->steeringAng = 30;
   	if (this->steeringAng < -30)
    	   this->steeringAng = -30;
	LOG_INFO(adtf_util::cString::Format("steering angle: %f", steeringAngle));	
	LOG_INFO(adtf_util::cString::Format("Final Steering angle : %f", steeringAng));
    	return this->steeringAng;
	

      // return this->steeringAngle;
}
tResult clanefollower::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult clanefollower::UpdateOutputImageFormat(const cv::Mat& outputImage)
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

tResult clanefollower::writeOutputs1(tFloat32 accel, tUInt32 timeStamp)
{
   
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	

	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	

	// acceleration
	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pCoderDescSignalOutputAccel->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pCoderDescSignalOutputAccel->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("f32Value", (tVoid*)&(accel));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalOutputAccel->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_oAccelerate.Transmit(pMediaSampleaccelerate);



}

tResult clanefollower::writeOutputs2(tFloat32 steeringAngle, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleSteer;
    AllocMediaSample((tVoid**)&pMediaSampleSteer);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerSteer = NULL;
    m_pCoderDescSignalOutputSteer->GetMediaSampleSerializer(&pSerializerSteer);
    tInt nSizeSteer = pSerializerSteer->GetDeserializedSize();
    pMediaSampleSteer->AllocBuffer(nSizeSteer);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputSteer;
    m_pCoderDescSignalOutputSteer->WriteLock(pMediaSampleSteer, &pCoderOutputSteer);
    // ...
    pCoderOutputSteer->Set("f32Value", (tVoid*)&(steeringAngle));
    pCoderOutputSteer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalOutputSteer->Unlock(pCoderOutputSteer);
    //transmit media sample over output pin
    pMediaSampleSteer->SetTime(_clock->GetStreamTime());
    m_oSteer.Transmit(pMediaSampleSteer);

    RETURN_NOERROR;
}

tResult clanefollower::writeOutputs(tUInt32 timeStamp)
{

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleAccel;
    AllocMediaSample((tVoid**)&pMediaSampleAccel);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerAccel;
    m_pCoderDescSignalOutputAccel->GetMediaSampleSerializer(&pSerializerAccel);
    tInt nSizeAccel = pSerializerAccel->GetDeserializedSize();
    pMediaSampleAccel->AllocBuffer(nSizeAccel);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputAccel;
    m_pCoderDescSignalOutputAccel->WriteLock(pMediaSampleAccel, &pCoderOutputAccel);
    // ...
    pCoderOutputAccel->Set("f32Value", (tVoid*)&(accel));
    pCoderOutputAccel->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalOutputAccel->Unlock(pCoderOutputAccel);
    //transmit media sample over output pin
    pMediaSampleAccel->SetTime(_clock->GetStreamTime());
    m_oAccelerate.Transmit(pMediaSampleAccel);

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleSteer;
    AllocMediaSample((tVoid**)&pMediaSampleSteer);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerSteer;
    m_pCoderDescSignalOutputSteer->GetMediaSampleSerializer(&pSerializerSteer);
    tInt nSizeSteer = pSerializerSteer->GetDeserializedSize();
    pMediaSampleSteer->AllocBuffer(nSizeSteer);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputSteer;
    m_pCoderDescSignalOutputSteer->WriteLock(pMediaSampleSteer, &pCoderOutputSteer);
    // ...
    pCoderOutputSteer->Set("f32Value", (tVoid*)&(steeringAngle));
    pCoderOutputSteer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalOutputSteer->Unlock(pCoderOutputSteer);
    //transmit media sample over output pin
    pMediaSampleSteer->SetTime(_clock->GetStreamTime());
    m_oSteer.Transmit(pMediaSampleSteer);

    RETURN_NOERROR;

}
/*
tResult clanefollower::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);
	  m_szIdsOutputSpeedSet=tFalse;

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignalOutputAccel->GetMediaSampleSerializer(&pSerializer);

	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutputAccel, pMediaSample, pCoderOutput);

        	if(!m_szIdsOutputSpeedSet)
        	{
				// (tVoid*) before the pointer?
                pCoderOutput->GetID("f32Value", m_szIdOutputSpeedControllerValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSpeedControllerTs);
        		m_szIdsOutputSpeedSet = tTrue;
       		}

    		pCoderOutput->Set(m_szIdOutputSpeedControllerValue, (tVoid*)&speed);
    		pCoderOutput->Set(m_szIdOutputSpeedControllerTs, (tVoid*)&timestamp);
	
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oAccelerate.Transmit(pMediaSample);

	RETURN_NOERROR;

}

*/
