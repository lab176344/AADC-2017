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
#ifndef _CROSSING_H_
#define _CROSSING_H_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"


#define OID_ADTF_FILTER_DEF "adtf.aadc.Crossing" //unique for a filter
#define ADTF_FILTER_DESC "Crossing"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "OpenCVTemplateFilter"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small OpenCV Template Filter \n$Rev:: 62948"


class cCrossing : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

protected:

	/* INPUT PINS */

    //input for rgb image
    cVideoPin           m_oVideoInputPin;
    /*! output for rgb image */
    cVideoPin           m_oVideoOutputPin;

	// Start Testing
	cInputPin m_oStart;
	cObjectPtr<IMediaTypeDescription> m_pDescStart;


	// input from situation detection
	cInputPin m_oSituationDetection;
	cObjectPtr<IMediaTypeDescription> m_pDescSituationDetection;

	// Distance over all
	cInputPin m_oDistanceOverall;
	cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;

	// Steering Input from LaneFollower
	cInputPin m_oSteeringOfLaneFollower;
	cObjectPtr<IMediaTypeDescription> m_pDescSteeringOfLaneFollower;

	// InerMeasUnit
	cInputPin m_oInerMeasUnit;
	cObjectPtr<IMediaTypeDescription> m_pDescInerMeasUnit;

	// Infos about Stop Line
	cInputPin m_oStopLine;
	cObjectPtr<IMediaTypeDescription> m_pDescStopLine;

	// Infos about Edges
	cInputPin m_oEdgeLine;
	cObjectPtr<IMediaTypeDescription> m_pDescEdgeLine;
	cInputPin m_oEdgePoint;
	cObjectPtr<IMediaTypeDescription> m_pDescEdgePoint;

	// Infos about RoadSignExt
	cInputPin m_oRoadSignExt;
	cObjectPtr<IMediaTypeDescription> m_pDescRoadSignExt;

	// Check Traffic for Crossing
	cInputPin m_oCheckTraffic;
	cObjectPtr<IMediaTypeDescription> m_pDescCheckTraffic;

	// critical section for current traffic sign
	cCriticalSection m_critSecCurrentTrafficSign;

	// member current traffic sign
	tRoadSign m_oCurrentTrafficSign;


    /* OUTPUT PINS */

	// output steering
	cOutputPin    m_oOutputSteering;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteering;

	// output acceleration
	cOutputPin m_oOutputAcceleration;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputAcceleration;

	// output turnSignalLeftEnabled
	cOutputPin m_oOutputTurnSignalLeftEnabled;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnSignalLeftEnabled;

	// output turnSignalRightEnabled
	cOutputPin m_oOutputTurnSignalRightEnabled;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnSignalRightEnabled;

	// output FinishFlag
	cOutputPin m_oOutputFinishFlag;
	cObjectPtr<IMediaTypeDescription> m_pDescFinishFlag;

	// output check traffic
	cOutputPin m_oOutputStartCheckTraffic;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionStartCheckTraffic;

	/* MEMBER VARIABLES INPUT*/

	tBool m_bStart;

	// InerMeasUnit
	tFloat32 m_fYawAngle;
	tFloat32 m_fYawAngleAtStart;

	// infos about stopline
	tBool m_bStopLineDetected;
	tFloat32 m_fDist2StopLine;
	tFloat32 m_fOrientation2StopLine;
	tBool m_bFlagNoStopLine;

	// infos about EdgeLine
	tBool m_bEdgeLineDetected;
	tFloat32 m_fDist2EdgeLine;
	tFloat32 m_fOrientation2EdgeLine;
	tBool m_bFlagNoEdgeLine;

	// infos about edges
	tBool m_bEdge1;
	tFloat32 m_fDistEdge1;
	tBool m_bEdge2;
	tFloat32 m_fDistEdge2;

	// Infos about RoadSign
	tFloat32 m_f32DistanceRoadSignY;
	tFloat32 m_f32DistanceRoadSignX;
	tFloat32 m_f32DistanceOverall_SignPassed;
	tBool	 m_bStopSignPassed;
	tFloat32 m_iDistance2driveAfterRoadSign;
	tFloat32 m_f32MinDistance2SeeRoadSign;
	tFloat32 m_f32ArrayDistanceRoadSignX[10];
	tBool	 m_bRoadSignDetected;
        tInt     m_iRoadSignNotDetectedCounter;
        tBool    m_bRoadSignPassed;

	// steering input of lanefollower
	tFloat32 m_fInputSteeringOfLaneFollower;

	// check traffic for crossing
	tBool m_bTrafficOnLeft;
	tBool m_bTrafficOnStraight;
	tBool m_bTrafficOnRight;


	/* DEBUG */

	tBool m_bDebugModeEnabled;

	/* MEMBER VARIABLES PROCESS*/
	tBool m_bStarted;
	tBool m_bStoppedAtLine;
	tBool m_bWaited;
	tBool m_bTurned;
	tBool m_bFinished;

	tTimeStamp stop_time;

	// time stamp
	tUInt32 timestamp;

	// result of function TwoLinesDetected
	tBool m_bTwoLinesDetected;
	
	// traffic sign
	//tFloat32 m_fTrafficSignImageSize;
	tInt16 m_iTrafficSignID;
	tInt8 m_iManeuverID;

	// distance over all
	tFloat32 m_fDistanceOverall;
	tFloat32 m_fDistanceOverall_Start;

	// distance and orientation until stop line
	tFloat32 m_fDist2Go;
	tFloat32 m_fOrientation2Go;

	/* MEMBER VARIABLES OUTPUT*/
	tFloat32 m_fSteeringOutput;
	tFloat32 m_fAccelerationOutput;
	tBool m_bTurnSignalLeftEnabled;
	tBool m_bTurnSignalRightEnabled;
	tBool m_bFinishFlag;

	tBool m_bStartCheckTraffic;

	// state of turn
	tInt16 m_iStateOfTurn;

	enum StateOfTurnEnums{
        SOT_NOSTART = 0,
        SOT_GOTOSTOP,
        SOT_WAIT,
        SOT_PRIORITYINTRAFFIC,
        SOT_TURN,
        SOT_GOSTRAIGHT,
        SOT_FINISH      
    };

public:

   tUInt8 blind_count;
   tFloat32 steeringAng;
   
   tFloat32 steeringAngle_previous,steermulti;
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


    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    cCrossing(const tChar* __info);

    /*! default destructor */
    virtual ~cCrossing();

	tResult PropertyChanged(const char* strProperty);

    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \result Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

	// function to decide the maneuver
	tResult ProcessManeuver();

	// function to turn right, left or go straight
	tResult Maneuver(tInt8 iManeuverID, tBool bHaveToStop);

	tResult TurnRight(tBool bHaveToStop);
	tResult TurnLeft(tBool bHaveToStop);
	tResult GoStraight(tBool bHaveToStop);

	tResult ReadProperties(const tChar* strPropertyName);

	tResult TransmitOutput();

	tResult TransmitStartTraffic(tBool i_start);


private: // private methods

    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);

    tResult TwoLinesDetected(IMediaSample* pSample);

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

	// Properties
	tFloat32    propDist2Stopline, propDistRightTurn, propDistLeftTurn, propDistStraight, propDistGoStraightAfterTurn;
	tFloat32    propSteerRight, propSteerLeft;
	tFloat32    speed2stopline,speed_curve, speed_straight;
	tFloat32	propKpSteering;

};

/** @} */ // end of group

#endif  //_CROSSING_H_
