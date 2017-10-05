/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cScanning.h"




#define DISTANCE_LANE_FOLLOW 			"cScanning::DIST_LANEFOLLOW"
#define SPEED_LANE_FOLLOW 			"cScanning::SPEED_LANEFOLLOW"


#define DISTANCE_SCAN_SLOT_LENGTH 			"cScanning::DIST_SCAN_SLOTLENGHT"
#define DISTANCE_SCAN_SLOT_DEPTH 			"cScanning::DIST_SCAN_SLOTDEPTH"
#define DISTANCE_SCAN_SPACE 			"cScanning::DIST_SCANSPACE"


/// Create filter shell
ADTF_FILTER_PLUGIN("Scanning", OID_ADTF_SCANNING, cScanning);

using namespace roadsignIDs;
using namespace std;

cScanning::cScanning(const tChar* __info):cFilter(__info)
{
// set properties for Dibug
    // m_bDebugModeEnabled = tTrue;
    SetPropertyBool("Debug Output to Console",false);
// set properties for lanefollow front
    // set properties for lanefollow front
            SetPropertyFloat(DISTANCE_LANE_FOLLOW,1.5);
        SetPropertyBool(DISTANCE_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
        SetPropertyStr(DISTANCE_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the distance for the lanefollow");

            SetPropertyFloat(SPEED_LANE_FOLLOW,0.3);
        SetPropertyBool(SPEED_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
        SetPropertyStr(SPEED_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the speed for the lanefollow");


// set property for distance for scanning slot lenght
        SetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH,0.46);
    SetPropertyBool(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_DESCRIPTION, "the distance for One slot Area");
// set property for distance for scanning slot depth
        SetPropertyFloat(DISTANCE_SCAN_SLOT_DEPTH,0.7);
    SetPropertyBool(DISTANCE_SCAN_SLOT_DEPTH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SLOT_DEPTH NSSUBPROP_DESCRIPTION, "the distance for One slot Area");
	// set property for distance for scanning desired space for parking
        SetPropertyFloat(DISTANCE_SCAN_SPACE,0.35);
    SetPropertyBool(DISTANCE_SCAN_SPACE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SPACE NSSUBPROP_DESCRIPTION, "the distance for desired space for parking");
}



cScanning::~cScanning()
{

}

tResult cScanning::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

                cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

                tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

//create pin for start pin input
                tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalstart);
                cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
                RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oStart));


//create pin for Distance over all input
                tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
                cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
                RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));

// Input - US Struct
				tChar const * strUSStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
				RETURN_IF_POINTER_NULL(strUSStruct);
				cObjectPtr<IMediaType> pTypeUSStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUSStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeUSStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputUSStruct));
				RETURN_IF_FAILED(m_oInputUSStruct.Create("US Struct", pTypeUSStruct, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oInputUSStruct));

//create pin for US Rightside input
                tChar const * strDescSignalUSRightside = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalUSRightside);
                cObjectPtr<IMediaType> pTypeSignalUSRightside = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUSRightside, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalUSRightside->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUSRightside));
                RETURN_IF_FAILED(m_oUSRightside.Create("US_Rightside", pTypeSignalUSRightside, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oUSRightside));

//create pin for lanefollower steering
                tChar const * strDescSignalLane_steer = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalLane_steer);
                cObjectPtr<IMediaType> pTypeSignalLane_steer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalLane_steer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalLane_steer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescLane_steer));
                RETURN_IF_FAILED(m_oLane_steer.Create("Lane_steer", pTypeSignalLane_steer, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oLane_steer));

// create pin for Infos about Edges
                tChar const * strDescEdges = pDescManager->GetMediaDescription("tStoplineStruct");
                RETURN_IF_POINTER_NULL(strDescEdges);
                cObjectPtr<IMediaType> pTypeSignalEdges = new cMediaType(0, 0, 0, "tStoplineStruct", strDescEdges, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalEdges->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescEdges));
                RETURN_IF_FAILED(m_oEdges.Create("Edges", pTypeSignalEdges, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oEdges));

// Output - Scan_Finish
                tChar const * strDescSignalScan_Finish = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalScan_Finish);
                cObjectPtr<IMediaType> pTypeSignalScan_Finish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalScan_Finish, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalScan_Finish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputScan_Finish));
                RETURN_IF_FAILED(m_oOutputScan_Finish.Create("Scan Finish", pTypeSignalScan_Finish, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputScan_Finish));

//create pin for steering signal output
                RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
                RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

//create pin for speed signal output
                tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
                cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
                RETURN_IF_FAILED(m_oOutputAcceleration.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration));

// Input pin for position input ADD IN Scanning
			  tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
			  RETURN_IF_POINTER_NULL(strDescPos);
			  cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			  RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
			  RETURN_IF_FAILED(m_InputPostion.Create("Position", pTypePos, static_cast<IPinEventSink*> (this)));
			  RETURN_IF_FAILED(RegisterPin(&m_InputPostion));

//output pin for traffic update
			  tChar const * strDescParkingSpace = pDescManager->GetMediaDescription("tParkingSpace");
			  RETURN_IF_POINTER_NULL(strDescParkingSpace);
			  cObjectPtr<IMediaType> pTypeParkingSpace = new cMediaType(0, 0, 0, "tParkingSpace", strDescParkingSpace, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			  RETURN_IF_FAILED(pTypeParkingSpace->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingSpace));
			  RETURN_IF_FAILED(m_InputParkingSpace.Create("ParkingSpace", pTypeParkingSpace, static_cast<IPinEventSink*> (this)));
			  RETURN_IF_FAILED(RegisterPin(&m_InputParkingSpace));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
	m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
		m_bStart = tFalse;
		m_bFinished = tFalse;
		m_bTransmitout = tFalse;
		m_bTransmitstop = tFalse;
		m_bImage_used = tFalse;
		m_bFree_space_by_sideUS = tFalse;
                m_iStateOfScan = SOS_NOSTART;
		m_bFront_Slot_Detected = tFalse;
		m_fcover_dist = 0;
		m_iParking_slot = 0;
		m_iScanning_slot = 0;
		m_fslot_space_start = 0;
		m_fnextslot_dist_start = 0;
		m_fImage_detection_distance = 0;
		m_fmax_threshold = 105;
		m_fmin_threshold = 45;
		// Variables for input position of car
		m_szF32X=0;			// input of car position to be added with the image distance 			
		m_szF32Y=0;
		m_szF32Radius=0;
		m_szF32Speed=0;
		m_szF32Heading=0;
		// Variables forparking update
		m_parkingI16Id=0;		// parking slot ID	
		m_parkingF32X=0;		// Parking spot position in X  (=m_szF32X+image distance)
		m_parkingF32Y=0;		// Parking spot position in Y
		m_parkingUI16Status=0;		// (Parking status free or occupied)
                m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
		FirstHeadingDetected = tFalse;
		FirstHeading1=0;
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cScanning::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {

                m_bStart= tFalse;

                // initial values for steering and acceleration
                m_fAccelerationOutput=0;
                m_fSteeringOutput=0;
                // init process values
                m_bFinished=tFalse;
                m_bTransmitout= tFalse;
				m_bTransmitstop = tFalse;
                m_iStateOfScan=SOS_NOSTART;
                m_bFront_Slot_Detected=tFalse;
				m_bFree_space_by_sideUS = tFalse;
				m_bImage_used = tFalse;
				m_fcover_dist = 0;
                m_iParking_slot=0;
                m_iScanning_slot=0;
                                m_fmax_threshold = 110;
				m_fmin_threshold = 45;
                m_fslot_space_start= 0;
                m_fnextslot_dist_start=0;
                m_fImage_detection_distance=0;
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cScanning::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{

  cObjectPtr<IMediaCoder> pCoderInput;
  RETURN_IF_FAILED(m_pDescriptionPos->Lock(pMediaSampleIn, &pCoderInput));
  pCoderInput->Get("f32x", (tVoid*)&m_szF32X);
  pCoderInput->Get("f32y", (tVoid*)&m_szF32Y);
  pCoderInput->Get("f32radius", (tVoid*)&m_szF32Radius);
  pCoderInput->Get("f32speed", (tVoid*)&m_szF32Speed);
  pCoderInput->Get("f32heading", (tVoid*)&m_szF32Heading);
  m_pDescriptionPos->Unlock(pCoderInput);

RETURN_NOERROR;
}

tResult cScanning::PropertyChanged(const char* strProperty)
{
        ReadProperties(strProperty);
        RETURN_NOERROR;
}

tResult cScanning::computepose()
{
	 if(!FirstHeadingDetected && m_szF32Heading!=0)
 	{
	FirstHeading1=m_szF32Heading;
	FirstHeadingDetected = tTrue;
	LOG_INFO(cString::Format("First Heading %f From car %f",FirstHeading1,m_szF32Heading));
 	}
	tFloat32 m_angle_change=FirstHeading1-m_szF32Heading;
        m_parkingF32X = m_fposx*std::cos(m_angle_change)-m_fposy*std::sin(m_angle_change);
        m_parkingF32Y = m_fposy*std::cos(m_angle_change)+m_fposx*std::sin(m_angle_change);

	LOG_INFO(cString::Format("Park X %f Park Y %f Angle change %f",m_parkingF32X,m_parkingF32Y,m_angle_change));
}
tResult cScanning::ReadProperties(const tChar* strPropertyName)
{
//check properties for lanefollow front
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_LANE_FOLLOW))
        {
                DIST_LANEFOLLOW = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_LANE_FOLLOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_LANE_FOLLOW))
        {
                SPEED_LANEFOLLOW = static_cast<tFloat32> (GetPropertyFloat(SPEED_LANE_FOLLOW));
        }

//check property for Scanning
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SLOT_LENGTH))
        {
                DIST_SCAN_SLOTLENGHT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH));
        }
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SLOT_DEPTH))
        {
                DIST_SCAN_SLOTDEPTH = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SLOT_DEPTH));
        }
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SPACE))
        {
                DIST_SCANSPACE = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SPACE));
        }
        RETURN_NOERROR;
}

tResult cScanning::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        if (pMediaSample != NULL)
        {
                // Input signal at Start
                if (pSource == &m_oStart)
                {
					if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool input"));
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_bStart);
                    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescStart->Unlock(pCoderInput);
                    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool input %d", m_bStart));
                    if(!m_bStart)
          	    {	
               		stop_time = _clock->GetStreamTime();		
                    m_bTransmitstop=tTrue;
				}
                }


                // Input signal at Distance Overall
                else if (pSource == &m_oDistanceOverall)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescdistanceoverall->Unlock(pCoderInput);

                }
                else if (pSource == &m_oLane_steer)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescLane_steer->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fLane_steer);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescLane_steer->Unlock(pCoderInput);

                }
				else if (pSource == &m_oInputUSStruct)
				{
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pDescriptionInputUSStruct->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("tFrontRight.f32Value", (tVoid*)&m_fUSFrontRightside);
					pCoderInput->Get("tSideRight.f32Value", (tVoid*)&m_fUSRightside);
					pCoderInput->Get("tFrontCenterRight.f32Value", (tVoid*)&m_fUSCenterRight);

					m_pDescriptionInputUSStruct->Unlock(pCoderInput);

				}

                else if (pSource == &m_oUSRightside)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescUSRightside->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fUSRightside);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescUSRightside->Unlock(pCoderInput);	
                }

                // infos about Edges
                else if (pSource == &m_oEdges)
                {
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescEdges->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_bFront_Slot_Detected);
                    pCoderInput->Get("f32Distance", (tVoid*)&m_fImage_detection_distance);
                    pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2StopLine);
                    m_pDescEdges->Unlock(pCoderInput);
                }
				else  if (pSource == &m_InputPostion)
				{
				 tTimeStamp tsInputTime;
				 tsInputTime = pMediaSample->GetTime();
				 //Process Sample
				 RETURN_IF_FAILED(ProcessInputPosition(pMediaSample, tsInputTime));
				}

	  	// stop signal
                if(!m_bStart)
                {
                 // LOG_INFO(cString::Format("Start bool false"));
                        m_fAccelerationOutput=0;
                        m_fSteeringOutput=0;

                        m_iStateOfScan=SOS_NOSTART;

                        if(m_bTransmitstop)
                        {
                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("stop signal"));
                        TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput, 0);
                        m_bTransmitstop= tFalse;
                        }


                }
            else if (m_bStart)
                {
                                                stage_scan();
						TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput, 0);

                }
                else
                {
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("no bool"));
                }
    }


}
RETURN_NOERROR;
}



tResult cScanning::stage_scan()
{
//scanning code
	if(m_bDebugModeEnabled) LOG_INFO(cString::Format("in side scan"));
	switch(m_iStateOfScan)
		{
        case SOS_NOSTART:
                {

                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 0:no start"));

                        //reset finish flag
                        m_bFinished=tFalse;
                        m_bFront_Slot_Detected=tFalse;
                        //reset Values
                        m_iParking_slot=0;
                        m_iScanning_slot=0;
						m_fmax_threshold = 105;
						m_fmin_threshold = 45;
                        m_fnextslot_dist_start = m_fDistanceOverall;
                        m_fDistanceOverall_Start=m_fDistanceOverall;
                        m_fImage_detection_distance=0;
                        m_fcover_dist = DIST_LANEFOLLOW;
						m_bnext_slot_freespace = tTrue;
						


                        // change state of turn when StartSignal is true
                        if(m_bStart)
                                {
                                        //save the first distance and get started
                                        m_iStateOfScan=SOS_Start_step1;
                                        m_iScanning_slot=0;
                                        if(m_bFront_Slot_Detected && m_fImage_detection_distance < 60)
                                        {
                                            m_fcover_dist = (m_fImage_detection_distance/100)+0.23;
                                            m_bFront_Slot_Detected=tFalse;
                                        }
                                }
                        break;
                }
        case SOS_Start_step1:
                {
                        //if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1:desired distance"));
                                    if (m_fDistanceOverall - m_fDistanceOverall_Start < m_fcover_dist) // distance for first_lanefollow
                                        {
                                                if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Step 1:desired distance"));
                                                m_fSteeringOutput = m_fLane_steer;		//accuator output
                                                m_fAccelerationOutput= SPEED_LANEFOLLOW ;
                                                stop_time = _clock->GetStreamTime();
                                                if (m_bFront_Slot_Detected && m_fImage_detection_distance < 60 )  // to check with laxman
                                                        {

                                                        m_fnextslot_dist_start=m_fDistanceOverall;
                                                        m_iScanning_slot=0;
                                                        m_fcover_dist = (m_fImage_detection_distance / 100) + 0.23; //Image detection distance is in cm
                                                        m_bFront_Slot_Detected=tFalse;
                                                        m_iStateOfScan=SOS_Reach_slot;
                                                        m_fDistanceOverall_Start=m_fDistanceOverall;
							m_fposx=(m_fImage_detection_distance / 100);
							m_fposy=0;
							computepose();
							m_parkingF32X = m_szF32X + m_parkingF32X;	
							m_parkingF32Y = m_szF32Y + m_parkingF32Y;
							//m_parkingF32X = m_szF32X + (m_fImage_detection_distance / 100); //parking spot update

                                                        }

                                        }
                                        else
                                        {

                                                if((_clock->GetStreamTime() - stop_time)/1000000 < 3) //time for wait
                                                {

                                                        m_fSteeringOutput=0;		//accuator output
                                                        m_fAccelerationOutput= 0;
                                                }
                                                else
                                                {

                                                        m_iStateOfScan=SOS_Reach_slot;
                                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                                        stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                                }

                                        }
                        break;
                }
        case SOS_Reach_slot:
            {
                       if (m_fDistanceOverall - m_fDistanceOverall_Start < m_fcover_dist) // distance for first_lanefollow
                            {
                               //if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Step 1:desired distance"));
                               m_fSteeringOutput = m_fLane_steer;		//accuator output
                               m_fAccelerationOutput = SPEED_LANEFOLLOW;

                               LOG_INFO(cString::Format("Step 2:reach distance"));
                               stop_time = _clock->GetStreamTime();
                                                           if ((m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start)) <= 0.5)
							   {
                                                               m_fmax_threshold = 45 + (105* (m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start))); // detection range decresing as per going furture
                                                               m_fmin_threshold = 5 + (105* (m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start)));
                                                               LOG_INFO(cString::Format("1st slot scanning bz front max threshold = %f & min threshold = %f",m_fmax_threshold,m_fmin_threshold));
								   if ((m_fUSFrontRightside <= m_fmax_threshold && m_fUSFrontRightside >= m_fmin_threshold) || (m_fUSCenterRight <= m_fmax_threshold && m_fUSCenterRight >= m_fmin_threshold))
								   {
									   m_bnext_slot_freespace = tFalse;
								   }


							   }



                            }
                            else
                            {

                            if ((_clock->GetStreamTime() - stop_time) / 1000000 < 1) //time for wait
                            {
                                       if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1:wait"));
                                        m_fSteeringOutput = 0;		//accuator output
                                        m_fAccelerationOutput = 0;

                            }
                            else
                            {

                                       m_bFront_Slot_Detected = tFalse;
                                       m_iStateOfScan = SOS_Start_currentslot;
                                       m_fScan_dist_start= m_fDistanceOverall;
                                       m_fnextslot_dist_start= m_fDistanceOverall;
                                       m_fslot_space_start=m_fDistanceOverall;
                                       m_fScan_distance= 0;
                                       stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                       m_iScanning_slot++; //increment of scanning slot
                                       m_fSlot_Length=DIST_SCAN_SLOTLENGHT;
                                       m_bFree_space_by_sideUS = tFalse;
                                       m_bFree_space_by_frontUS = m_bnext_slot_freespace;
                                       stop_time = _clock->GetStreamTime(); //store time data for waitning in next case


                            }

                       }
             break;
            }
        case SOS_Start_currentslot:
                {
                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step :current slot"));
                        //----------------------------------------check image detection-------------------------------
			if (m_bFront_Slot_Detected && m_fImage_detection_distance < 60 && !m_bImage_used)
                        {
                                if (m_bDebugModeEnabled) LOG_INFO(cString::Format("image detect"));
                                m_fnextslot_dist_start=m_fDistanceOverall;
                                m_fSlot_Length = (m_fImage_detection_distance / 100)-DIST_SCAN_SLOTLENGHT + 0.23;  //Image detection distance is in cm
                                m_bFront_Slot_Detected=tFalse;
				m_bImage_used = tTrue;
				if (m_iScanning_slot<=3)
					{
					m_fposx=(m_fImage_detection_distance / 100) - DIST_SCAN_SLOTLENGHT; //parking spot update
					m_fposy=0;
					computepose();
					m_parkingF32X = m_szF32X + m_parkingF32X;	
					m_parkingF32Y = m_szF32Y + m_parkingF32Y;
					} 

				else 
					{
					m_fposx=m_fposx + 0.46; //parking spot update
					m_fposy=0;
					computepose();
					m_parkingF32X = m_szF32X + m_parkingF32X;	
					m_parkingF32Y = m_szF32Y + m_parkingF32Y;
					}
                        }
                        //--------------------------------------------------------------------------------------------

                        if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Scanning slot =%d",m_iScanning_slot));
                        if(m_fDistanceOverall-m_fnextslot_dist_start < m_fSlot_Length) // distance for scanning area in meter
                                {

                                    m_fSteeringOutput = m_fLane_steer;		//accuator output
                                    m_fAccelerationOutput = SPEED_LANEFOLLOW;
                                                                        m_fmax_threshold = 45 + (102*(m_fSlot_Length- (m_fDistanceOverall - m_fnextslot_dist_start))); // detection range decrtesing as per going furture
                                                                        m_fmin_threshold = 5 + (102*(m_fSlot_Length- (m_fDistanceOverall - m_fnextslot_dist_start)));

                                    //----------------------------------------object detection in current parking slot--------------
                                    if (m_fUSRightside < ((DIST_SCAN_SLOTDEPTH)*100)) //scan slot depth
                                            {
                                                    m_fslot_space_start= m_fDistanceOverall;  // if object in parking slot detected  ---> no space distance update
                                                    m_fScan_distance= m_fDistanceOverall-m_fslot_space_start; // =0
                                            }
                                    else
                                            {
                                                    m_fScan_distance= m_fDistanceOverall-m_fslot_space_start;  // no object dected ----> space distance calculate
                                            }
                                    //-------------------------------------------------------------------------------------------
									//--------------------------------------- object detection in next parking slot-----------------
									if ((m_fUSFrontRightside <= m_fmax_threshold && m_fUSFrontRightside >= m_fmin_threshold) || (m_fUSCenterRight <= m_fmax_threshold && m_fUSCenterRight >= m_fmin_threshold))
											{
												m_bnext_slot_freespace = tFalse;
												LOG_INFO(cString::Format(" : scan distance = %f",m_fScan_distance));

											}
									//----------------------------------------------------------------------------------------------
                                    //----------------------------------------decision on above detections-----------------------
                                    if(m_fScan_distance>=DIST_SCANSPACE)  // if desired space is available
                                            {

                                                                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("space_available : scan distance = %f",m_fScan_distance));
                                                                m_iParking_slot=m_iScanning_slot;
								m_bFree_space_by_sideUS = tTrue;
                                                                m_fScan_dist_start=m_fDistanceOverall;
                                                                stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                            }
                                   else
                                            {
                                                    stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                            }

                                }

                        else
                                {

                                    if(m_fScan_distance>=DIST_SCANSPACE)  // if desired space is available
                                            {

                                                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("space_available : scan distance = %f",m_fScan_distance));
                                                m_fScan_dist_start=m_fDistanceOverall;
                                                m_iParking_slot=m_iScanning_slot;
                                                stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
												m_bFree_space_by_sideUS = tTrue;

                                            }
                                    
									LOG_INFO(cString::Format("scanned slot =%d ,by sidesensor = %d, by frontsensor = %d ,freespace = %d", m_iScanning_slot, m_bFree_space_by_sideUS, m_bFree_space_by_frontUS, m_bFree_space_by_sideUS && m_bFree_space_by_frontUS));
									m_parkingI16Id = m_iScanning_slot;
									if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS) m_parkingUI16Status = 1;
									else m_parkingUI16Status = 0;
                                                                        SendParkingData();

                                    if (m_iScanning_slot <= 3) // if parking available slot are 4
                                           {
                                               if (m_bDebugModeEnabled) LOG_INFO(cString::Format("going to next slot"));
                                               m_iStateOfScan = SOS_go_nextslot;  //if no space found so going for next slot
											   m_bImage_used = tFalse;
                                               stop_time = _clock->GetStreamTime(); //lane follow bool true ----remaining
                                               m_fSlot_Length = DIST_SCAN_SLOTLENGHT;
											   m_bFree_space_by_sideUS = tFalse;
                                           }

                                           else
                                           {
                                                m_iStateOfScan=SOS_Finished;
												m_bFree_space_by_sideUS = tFalse;
												m_bImage_used = tFalse;
                                           }
                                }
                        break;
                }
        case SOS_go_nextslot:
                {

                        if((_clock->GetStreamTime() - stop_time)/1000000 < 1) // distance for backside_lanefollow
                                {


                                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Wait before going to next slot"));
                                        m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                        m_fAccelerationOutput=0;

                                }
                        else
                                {

                                        m_iScanning_slot++;
                                        m_fScan_dist_start = m_fDistanceOverall;
                                        m_fslot_space_start = m_fDistanceOverall;
                                        m_fScan_distance = 0;
                                        m_fnextslot_dist_start = m_fDistanceOverall;
                                        stop_time = _clock->GetStreamTime();
                                        m_iStateOfScan = SOS_Start_currentslot;
                                        m_fSlot_Length = DIST_SCAN_SLOTLENGHT;
                                        m_bFree_space_by_frontUS = m_bnext_slot_freespace; // transfer the detection of next slot to current slot


                                 }

                        break;
                }
		case SOS_Finished:
			{
                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step: Finished"));

                                        m_bStart=tFalse;
                                        m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                        m_fAccelerationOutput=0;
                                        m_bFinished=tTrue;
                                        TransmitFinish();
                                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("no free space"));
                                        m_iStateOfScan=SOS_NOSTART;
                                        m_iParking_slot=0;
                                        m_iScanning_slot=0;
                                        stop_time = _clock->GetStreamTime();
                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                        m_bnext_slot_freespace = tTrue;
                                        m_bFree_space_by_frontUS = tTrue;


            break;
			}
		default :
                    {
                            m_iStateOfScan=SOS_NOSTART;
                            m_fScan_dist_start=m_fDistanceOverall;
                           if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Scanning default case"));
                    }
	}

	RETURN_NOERROR;
}



tResult cScanning::TransmitFinish()
{
	// Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleScan_Finish;
        AllocMediaSample((tVoid**)&pMediaSampleScan_Finish);

        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerScan_Finish;
        m_pDescriptionOutputScan_Finish->GetMediaSampleSerializer(&pSerializerScan_Finish);
        tInt nSizeScan_Finish = pSerializerScan_Finish->GetDeserializedSize();
        pMediaSampleScan_Finish->AllocBuffer(nSizeScan_Finish);
        cObjectPtr<IMediaCoder> pCoderOutputScan_Finish;
        m_pDescriptionOutputScan_Finish->WriteLock(pMediaSampleScan_Finish, &pCoderOutputScan_Finish);
        pCoderOutputScan_Finish->Set("bValue", (tVoid*)&(m_bFinished));
        pCoderOutputScan_Finish->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputScan_Finish->Unlock(pCoderOutputScan_Finish);
        pMediaSampleScan_Finish->SetTime(_clock->GetStreamTime());
        m_oOutputScan_Finish.Transmit(pMediaSampleScan_Finish);
        RETURN_NOERROR;
}
tResult cScanning::TransmitOutput(tFloat32 speed,tFloat32 steering, tUInt32 timestamp)
{

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleaccelerate;
        cObjectPtr<IMediaSample> pMediaSamplesteer;

        AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
        AllocMediaSample((tVoid**)&pMediaSamplesteer);

        // acceleration
        cObjectPtr<IMediaSerializer> pSerializeraccelerate;
        m_pDescriptionOutputAcceleration->GetMediaSampleSerializer(&pSerializeraccelerate);
        tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
        pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
        cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
        m_pDescriptionOutputAcceleration->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
        pCoderOutputaccelerate->Set("f32Value", (tVoid*)&(m_fAccelerationOutput));
        pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputAcceleration->Unlock(pCoderOutputaccelerate);
        pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
        m_oOutputAcceleration.Transmit(pMediaSampleaccelerate);

        // steering
        cObjectPtr<IMediaSerializer> pSerializersteer;
        m_pDescriptionOutputSteering->GetMediaSampleSerializer(&pSerializersteer);
        tInt nSizesteer = pSerializersteer->GetDeserializedSize();
        pMediaSamplesteer->AllocBuffer(nSizesteer);
        cObjectPtr<IMediaCoder> pCoderOutputsteer;
        m_pDescriptionOutputSteering->WriteLock(pMediaSamplesteer, &pCoderOutputsteer);
        pCoderOutputsteer->Set("f32Value", (tVoid*)&(m_fSteeringOutput));
        pCoderOutputsteer->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputSteering->Unlock(pCoderOutputsteer);
        pMediaSamplesteer->SetTime(_clock->GetStreamTime());
        m_oOutputSteering.Transmit(pMediaSamplesteer);

        RETURN_NOERROR;
}
tResult cScanning::SendParkingData() // add this in Scanning Code

{
		cObjectPtr<IMediaSample> pMediaSampleTraffic;
		AllocMediaSample((tVoid**)&pMediaSampleTraffic);
 		// Parking space data
        cObjectPtr<IMediaSerializer> pSerializerTraffic;
        m_pDescriptionParkingSpace->GetMediaSampleSerializer(&pSerializerTraffic);
        tInt nSizesteer = pSerializerTraffic->GetDeserializedSize();
        pMediaSampleTraffic->AllocBuffer(nSizesteer);
        cObjectPtr<IMediaCoder> pCoderOutputTraffic;
        m_pDescriptionParkingSpace->WriteLock(pMediaSampleTraffic, &pCoderOutputTraffic);
        pCoderOutputTraffic->Set("i16Identifier", (tVoid*)&(m_parkingI16Id));
		pCoderOutputTraffic->Set("f32x", (tVoid*)&(m_parkingF32X));
		pCoderOutputTraffic->Set("f32y", (tVoid*)&(m_parkingF32Y));
		pCoderOutputTraffic->Set("ui16Status", (tVoid*)&(m_parkingUI16Status));
        m_pDescriptionOutputSteering->Unlock(pCoderOutputTraffic);
        pMediaSampleTraffic->SetTime(_clock->GetStreamTime());
        m_oOutputSteering.Transmit(pMediaSampleTraffic);

        RETURN_NOERROR;
}


