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
#include "cImageCut_RF1RDF.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   cImageCut_RF1RDF)



cImageCut_RF1RDF::cImageCut_RF1RDF(const tChar* __info) : cFilter(__info)
{
}

cImageCut_RF1RDF::~cImageCut_RF1RDF()
{
}

tResult cImageCut_RF1RDF::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cImageCut_RF1RDF::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult cImageCut_RF1RDF::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Video Visualized Depth Input
        m_oInputVideoDepthVis.Create("DepthVisualization_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this));
        RegisterPin(&m_oInputVideoDepthVis);


        // Video Input
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

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



tResult cImageCut_RF1RDF::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cImageCut_RF1RDF::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
            LOG_INFO(adtf_util::cString::Format("Process Video"));
        }
        else if (pSource == &m_oInputVideoDepthVis)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oInputVideoDepthVis.GetFormat()));
            }
            pMediaSampleDepth = pMediaSample;
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

tResult cImageCut_RF1RDF::ProcessVideo(IMediaSample* pSampleRGB)
{

   /* RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImage;
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
            Canny(m_inputImage, outputImage, 100, 200);// Detect Edges
        }
        pSample->Unlock(l_pSrcBuffer);
    }
    */
    // RGB
    RETURN_IF_POINTER_NULL(pSampleRGB);
    const tVoid* l_pSrcBufferRGB;
    IplImage* imgRGB;
    imgRGB = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
    pSampleRGB->Lock(&l_pSrcBufferRGB);
    // Get the image
    imgRGB->imageData = (char*)l_pSrcBufferRGB;
    // Set the image as a Mat
    cv::Mat matImageRGB;
    matImageRGB = cvarrToMat(imgRGB);
    pSampleRGB->Unlock(&l_pSrcBufferRGB);



    // Depth
    RETURN_IF_POINTER_NULL(pMediaSampleDepth);
    const tVoid* l_pSrcBufferDepth;
    IplImage* imgDepth;
    imgDepth = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_16U, 1);
    pMediaSampleDepth->Lock(&l_pSrcBufferDepth);
    // Get the image
   // imgDepth->imageData = (char*)l_pSrcBufferDepth;
    // Set the image as a Mat
  //  cv::Mat matImageDepth(cvarrToMat(imgDepth));

    cv::Mat matImageDepth2;
    cv::Mat ImageUShort = cv::Mat(cv::Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_16U, (char*)l_pSrcBufferDepth, m_sInputFormat.nBytesPerLine);
    double min, max;
    cv::minMaxLoc(ImageUShort, &min, &max);
    ImageUShort.convertTo(matImageDepth2, CV_16U, 65535 / (max - min));
    pMediaSampleDepth->Unlock(&l_pSrcBufferDepth);





    imwrite("test/RGB.jpg", matImageRGB);
   // imwrite("test/Depth1.jpg", matImageDepth);
    imwrite("test/Depth2.jpg", matImageDepth2);

    LOG_INFO(adtf_util::cString::Format("safe mat"));



    cv::Mat matDepthOffsetCUT;
    matDepthOffsetCUT = matImageDepth2(Range(31,480),Range(51,640)).clone();

    cv::Mat matRGBOffsetCUT;
    matRGBOffsetCUT = matImageRGB(Range(1,450),Range(0,619)).clone();


    imwrite("test/CUT_RGB.jpg", matRGBOffsetCUT);
   // imwrite("test/Depth1.jpg", matImageDepth);
    imwrite("test/CUT_Depth.jpg", matDepthOffsetCUT);


    CreateColorImageMappedToDepth(matImageDepth2,matImageRGB);
/*
    if (!outputImage.empty())
    {
        UpdateOutputImageFormat(outputImage);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        outputImage.release();
    }

    */

    RETURN_NOERROR;
}

tResult cImageCut_RF1RDF::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult cImageCut_RF1RDF::UpdateOutputImageFormat(const cv::Mat& outputImage)
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
