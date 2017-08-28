#ifndef _LaneFollower_FILTER_HEADER_
#define _LaneFollower_FILTER_HEADER_

#define OID_ADTF_LaneFollower  "adtf.aadc_Lane_v_2"

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"

class clanefollower : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LaneFollower, "Lane_v_2", adtf::OBJCAT_Tool, "AADC LaneFollower", 1, 0, 0, "Beta Version");

protected:
    /*! input for rgb image */
    cVideoPin           m_oVideoInputPin;

    /*! output for rgb image */
    cVideoPin           m_oVideoOutputPin;
    cOutputPin      m_oSteer;
    cOutputPin      m_oAccelerate;
    tBufferID	m_szIdOutputSpeedControllerValue;	// Speed-Value
    tBufferID	m_szIdOutputSpeedControllerTs;		// Timestamp
    tBool	m_szIdsOutputSpeedSet;			// Bool first received
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputSteer;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputAccel;

public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */

   tUInt8 blind_count;
   tFloat32 steeringAng;
   
   tFloat32 steeringAngle_previous;
   tFloat32 mean_theta_previous;
   tFloat32 KpsiRIGHT, KpsiLEFT, Ky;
   tFloat32 vMax, vMin;
   tBool writeZeroOutputs;
   tBool enable_imshow;
   tInt row1,row2,col1,col2;
   tInt cameraoffest,thresholdvalue,houghlinesvalue;
    tFloat32 accel;
    tFloat32 steeringAngle;
    tUInt32 timeStamp;
	tBool m_time;
    cv::Mat Line;					/* matrix for the line*/
    cv::Mat grey;					/* matrix for the gray image*/
    cv::Mat greythresh;				/* matrix for the gray threshold*/
    cv::Size cannysize;				/* size for the canny detector*/
    cv::Mat linecanny;				/* size for the canny lines*/
    clanefollower(const tChar* __info);
    

    /*! default destructor */
    virtual ~clanefollower();

    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tFloat32 evaluateSteeringAngle(IMediaSample* pSample);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult Start(ucom::IException** __exception_ptr = NULL);

    tResult Stop(ucom::IException** __exception_ptr = NULL);


private: // private methods

    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);

    tResult ProcessVideo(IMediaSample* pSample);
    tResult writeOutputs2(tFloat32 accel, tUInt32 timeStamp);
    tResult writeOutputs1(tFloat32 steeringAngle, tUInt32 timeStamp);
     tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
	tResult writeOutputs(tUInt32 timeStamp);
    /*! bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;

    /*! tha last received input image*/
    Mat m_inputImage;
    tBool firstFrame;				/* flag for the first frame*/
    tUInt8 imagecount;				/* counter for the images*/

	tTimeStamp starttime;
	tTimeStamp delaytime;

};

/** @} */ // end of group

#endif  //_LaneFollower_FILTER_HEADER_
