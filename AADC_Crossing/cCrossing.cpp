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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cCrossing.h"

#define DISTANCE2STOPLINE		"cCrossing::distance2stopline"
#define DISTANCERIGHTTURN		"cCrossing::distancerightturn"
#define DISTANCELEFTTURN		"cCrossing::distanceleftturn"
#define DISTANCESTRAIGHT		"cCrossing::distancestraight"
#define DISTANCEGOSTRAIGHTAFTERTURN		"cCrossing::distancestraightafterturn"

#define STEER_RIGHT 			"cCrossing::steerright"
#define STEER_LEFT				"cCrossing::steerleft"

#define SPEED2STOPLINE			"cCrossing::speed2stopline"
#define SPEEDCURVE		 		"cCrossing::speedcurve"
#define SPEEDSTRAIGHT		 	"cCrossing::speedstraight"

#define KPSTEERING				"cCrossing::kpsteering"

/// Create filter shell
ADTF_FILTER_PLUGIN("Crossing", OID_ADTF_CROSSING_FILTER, cCrossing);

using namespace roadsignIDs;

cCrossing::cCrossing(const tChar* __info):cFilter(__info)
{
	SetPropertyBool("Debug Output to Console",true);

	SetPropertyFloat(DISTANCE2STOPLINE,40);
    SetPropertyBool(DISTANCE2STOPLINE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE2STOPLINE NSSUBPROP_DESCRIPTION, "max. distance until stop line");

	SetPropertyFloat(DISTANCERIGHTTURN,50);
    SetPropertyBool(DISTANCERIGHTTURN NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCERIGHTTURN NSSUBPROP_DESCRIPTION, "max. distance for right turn");

	SetPropertyFloat(DISTANCELEFTTURN,110);
    SetPropertyBool(DISTANCELEFTTURN NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCELEFTTURN NSSUBPROP_DESCRIPTION, "max. distance for left turn");

	SetPropertyFloat(DISTANCESTRAIGHT,20);
    SetPropertyBool(DISTANCESTRAIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCESTRAIGHT NSSUBPROP_DESCRIPTION, "max. distance for going straight");

	SetPropertyFloat(DISTANCEGOSTRAIGHTAFTERTURN,0.0);
    SetPropertyBool(DISTANCEGOSTRAIGHTAFTERTURN NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCEGOSTRAIGHTAFTERTURN NSSUBPROP_DESCRIPTION, "max. distance for going straight after a turn");

	SetPropertyFloat(STEER_RIGHT,90);
    SetPropertyBool(STEER_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_RIGHT NSSUBPROP_DESCRIPTION, "steering for right turn");

	SetPropertyFloat(STEER_LEFT,-60);
    SetPropertyBool(STEER_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_LEFT NSSUBPROP_DESCRIPTION, "steering for left turn");

	SetPropertyFloat(SPEED2STOPLINE,-15);
    SetPropertyBool(SPEED2STOPLINE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED2STOPLINE NSSUBPROP_DESCRIPTION, "the speed for going to stop line");

	SetPropertyFloat(KPSTEERING,20);
    SetPropertyBool(KPSTEERING NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(KPSTEERING NSSUBPROP_DESCRIPTION, "Kp value for steering control until stop line");

	SetPropertyFloat(SPEEDCURVE,-20);
    SetPropertyBool(SPEEDCURVE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEEDCURVE NSSUBPROP_DESCRIPTION, "the speed for curve (left and right)");

	SetPropertyFloat(SPEEDSTRAIGHT,-20);
    SetPropertyBool(SPEEDSTRAIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEEDSTRAIGHT NSSUBPROP_DESCRIPTION, "the speed for going straight");

}

cCrossing::~cCrossing()
{

}

tResult cCrossing::Init(tInitStage eStage, __exception)
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

		tChar const * strDescCrossingStruct = pDescManager->GetMediaDescription("tCrossingStruct");
        RETURN_IF_POINTER_NULL(strDescCrossingStruct);
        cObjectPtr<IMediaType> pTypeCrossingStruct = new cMediaType(0, 0, 0, "tCrossingStruct", strDescCrossingStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//create pin for Distance over all input
		tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
		cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
		RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));


		// create pin for signal from situation detection
		tChar const * strDescSituationDetection = pDescManager->GetMediaDescription("tCrossingStruct");
		RETURN_IF_POINTER_NULL(strDescSituationDetection);
		cObjectPtr<IMediaType> pTypeSignalSituationDetection = new cMediaType(0, 0, 0, "tCrossingStruct", strDescSituationDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSituationDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSituationDetection));
		RETURN_IF_FAILED(m_oSituationDetection.Create("SituationDetectionStart", pTypeSignalSituationDetection, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSituationDetection));

		// create pin for yaw angle of AHRS Calculator
		tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
		cObjectPtr<IMediaType> pTypeSignalInerMeasUnit = new cMediaType(0, 0, 0, "tSignalValue", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescInerMeasUnit));
		RETURN_IF_FAILED(m_oInerMeasUnit.Create("yaw angle", pTypeSignalInerMeasUnit, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInerMeasUnit));

		// create pin for Infos about Stop Line
		tChar const * strDescStopLine = pDescManager->GetMediaDescription("tStoplineStruct");
		RETURN_IF_POINTER_NULL(strDescStopLine);
		cObjectPtr<IMediaType> pTypeSignalStopLine = new cMediaType(0, 0, 0, "tStoplineStruct", strDescStopLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalStopLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStopLine));
		RETURN_IF_FAILED(m_oStopLine.Create("StopLine", pTypeSignalStopLine, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopLine));

		// create pin for Infos about Edges
		tChar const * strDescEdges = pDescManager->GetMediaDescription("tEdgeStruct");
		RETURN_IF_POINTER_NULL(strDescEdges);
		cObjectPtr<IMediaType> pTypeSignalEdges = new cMediaType(0, 0, 0, "tEdgeStruct", strDescEdges, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalEdges->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescEdges));
		RETURN_IF_FAILED(m_oEdges.Create("Edges", pTypeSignalEdges, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oEdges));


		/*
		// create pin for start signal
		tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalstart);
		cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
		RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStart));
		
		//create pin for traffic sign input
		RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("TrafficSign", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));

		*/

		//create pin for steering signal output
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
        RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

		//create pin for acceleration signal output
		tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
		cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
		RETURN_IF_FAILED(m_oOutputAcceleration.Create("Acceleration", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration));

		//create pin for turnSignalLeftEnabled output
		tChar const * strDescSignalTurnSignalLeftEnabled = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
		RETURN_IF_POINTER_NULL(strDescSignalTurnSignalLeftEnabled);
		cObjectPtr<IMediaType> pTypeSignalTurnSignalLeftEnabled = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnSignalLeftEnabled, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalTurnSignalLeftEnabled->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnSignalLeftEnabled));
		RETURN_IF_FAILED(m_oOutputTurnSignalLeftEnabled.Create("turnSignalLeftEnabled", pTypeSignalTurnSignalLeftEnabled, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnSignalLeftEnabled));

		//create pin for turnSignalRightEnabled output
		tChar const * strDescSignalTurnSignalRightEnabled = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
		RETURN_IF_POINTER_NULL(strDescSignalTurnSignalRightEnabled);
		cObjectPtr<IMediaType> pTypeSignalTurnSignalRightEnabled = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnSignalRightEnabled, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalTurnSignalRightEnabled->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnSignalRightEnabled));
		RETURN_IF_FAILED(m_oOutputTurnSignalRightEnabled.Create("turnSignalRightEnabled", pTypeSignalTurnSignalRightEnabled, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnSignalRightEnabled));

		//create pin for FinishFlag output
		tChar const * strDescSignalfinishflag = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
		RETURN_IF_POINTER_NULL(strDescSignalfinishflag);
		cObjectPtr<IMediaType> pTypeSignalfinishflag = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalfinishflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalfinishflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescFinishFlag));
		RETURN_IF_FAILED(m_oOutputFinishFlag.Create("FinishFlag", pTypeSignalfinishflag, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputFinishFlag));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.

		m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageGraphReady)
    {
		/*
        // no ids were set so far
        m_bIDsRoadSignSet = tFalse;
		//m_bIDsSterringSet = tFalse;
		*/

		m_bStart= tFalse;

		

		// no ids were set so far

		// initial values for steering and acceleration
		m_fAccelerationOutput=0;
		m_fSteeringOutput=0;
		m_bStart=tFalse;

		// init process values
		m_bStarted=tFalse;
		m_bStoppedAtLine=tFalse;
		m_bWaited=tFalse;
		m_bTurned=tFalse;
		m_bFinished=tFalse;

		m_bStopLineDetected=tFalse;
		m_bFlagNoStopLine=tFalse;

		// output signals
		m_bTurnSignalLeftEnabled=tFalse;
		m_bTurnSignalRightEnabled=tFalse;
		m_bFinishFlag=tFalse;

		m_iStateOfTurn=SOT_NOSTART;
    }

    RETURN_NOERROR;
}

tResult cCrossing::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
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

tResult cCrossing::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}
		
tResult cCrossing::ReadProperties(const tChar* strPropertyName)
{
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE2STOPLINE))
	{
		propDist2Stopline = static_cast<tFloat32> (GetPropertyFloat(DISTANCE2STOPLINE));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCERIGHTTURN))
	{
		propDistRightTurn = static_cast<tFloat32> (GetPropertyFloat(DISTANCERIGHTTURN));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCELEFTTURN))
	{
		propDistLeftTurn = static_cast<tFloat32> (GetPropertyFloat(DISTANCELEFTTURN));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCESTRAIGHT))
	{
		propDistStraight = static_cast<tFloat32> (GetPropertyFloat(DISTANCESTRAIGHT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCEGOSTRAIGHTAFTERTURN))
	{
		propDistGoStraightAfterTurn = static_cast<tFloat32> (GetPropertyFloat(DISTANCEGOSTRAIGHTAFTERTURN));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_RIGHT))
	{
		propSteerRight = static_cast<tFloat32> (GetPropertyFloat(STEER_RIGHT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_LEFT))
	{
		propSteerLeft = static_cast<tFloat32> (GetPropertyFloat(STEER_LEFT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED2STOPLINE))
	{
		speed2stopline = static_cast<tFloat32> (GetPropertyFloat(SPEED2STOPLINE));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, KPSTEERING))
	{
		propKpSteering = static_cast<tFloat32> (GetPropertyFloat(KPSTEERING));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEEDCURVE))
	{
		speed_curve = static_cast<tFloat32> (GetPropertyFloat(SPEEDCURVE));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEEDSTRAIGHT))
	{
		speed_straight = static_cast<tFloat32> (GetPropertyFloat(SPEEDSTRAIGHT));
	}
	RETURN_NOERROR;
}

tResult cCrossing::OnPinEvent(IPin* pSource,
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
		if(pSource == &m_oSituationDetection)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescSituationDetection->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("i8RoadSignID", (tVoid*)&m_iTrafficSignID);
			pCoderInput->Get("i8CrossingManeuverID", (tVoid*)&m_iManeuverID);
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescSituationDetection->Unlock(pCoderInput);
		}

		// Input signal at Distance Overall
		else if (pSource == &m_oDistanceOverall)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);

			// change from [m] to [cm]
			m_fDistanceOverall=m_fDistanceOverall*100;

			ProcessManeuver();
		}
		// Input signal at InerMeasUnit
		else if (pSource == &m_oInerMeasUnit)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescInerMeasUnit->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fYawAngle);
			//pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescInerMeasUnit->Unlock(pCoderInput);

			// transform it from -180/+180 to 0/+360 coordinate
			m_fYawAngle=m_fYawAngle+180;
		}
		// infos about StopLine
		else if (pSource == &m_oStopLine)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStopLine->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStopLineDetected);
			pCoderInput->Get("f32Distance", (tVoid*)&m_fDist2StopLine);
			pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2StopLine);
			m_pDescStopLine->Unlock(pCoderInput);

			m_fDist2StopLine=m_fDist2StopLine;

			// set flag if the stop line is not detected
			if(!m_bStopLineDetected)
			{
				m_bFlagNoStopLine=tTrue;
			}
		}
		// infos about Edges
		else if (pSource == &m_oEdges)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescEdges->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue1", (tVoid*)&m_bEdge1);
			pCoderInput->Get("f32Distance1", (tVoid*)&m_fDistEdge1);
			pCoderInput->Get("bValue2", (tVoid*)&m_bEdge2);
			pCoderInput->Get("f32Distance2", (tVoid*)&m_fDistEdge2);
			m_pDescEdges->Unlock(pCoderInput);
		}

    }
    RETURN_NOERROR;
}
}

tResult cCrossing::ProcessManeuver()
{
	// StartSignal is true
	//if(m_bStart)
	{
		// use mutex to access current traffic sign
	__synchronized_obj(m_critSecCurrentTrafficSign);
	
	// decision which traffic sign is dectected
	switch(m_iTrafficSignID)
	{
	// right before left
	case MARKER_ID_UNMARKEDINTERSECTION:
		{
			Maneuver(m_iManeuverID, tFalse);
			break;
		}
	// give away
	case MARKER_ID_STOPANDGIVEWAY:
	case MARKER_ID_GIVEWAY:
		{
			Maneuver(m_iManeuverID, tTrue);
			break;
		}
	// parking --> stop crossing
	case MARKER_ID_PARKINGAREA:
	case MARKER_ID_PEDESTRIANCROSSING:
		{
			Maneuver(m_iManeuverID, tTrue);
			break;
		}
	// have away
	case MARKER_ID_HAVEWAY:
		{
			Maneuver(m_iManeuverID, tFalse);
			break;
		}
	// not implemented because not used in AADC 2017
	case MARKER_ID_AHEADONLY:
	case MARKER_ID_ROUNDABOUT:
	case MARKER_ID_NOOVERTAKING:
	case MARKER_ID_NOENTRYVEHICULARTRAFFIC:
	case MARKER_ID_TESTCOURSEA9:
	case MARKER_ID_ONEWAYSTREET:
	case MARKER_ID_ROADWORKS:
	case MARKER_ID_KMH50: // not important for crossing
	case MARKER_ID_KMH100: // not important for crossing
	case MARKER_ID_NOMATCH:
		{
			break;
		}
	}

	}

	//if(m_bStart){
		TransmitOutput();
	//}

	RETURN_NOERROR;
}

tResult cCrossing::Maneuver(tInt8 iID, tBool i_bHaveToStop)
{
	switch(m_iStateOfTurn)
	{
	case SOT_NOSTART:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//reset finish flag
			m_bFinishFlag=tFalse;
			m_bFlagNoStopLine=tFalse;
			// change state of turn when StartSignal is true
			if(m_bStart)
			{
				m_iStateOfTurn=SOT_GOTOSTOP;
				//save the first distance and get started
				m_fDistanceOverall_Start=m_fDistanceOverall;
				//save the property distance to the stop line
				m_fDist2Go=propDist2Stopline;
			}
			break;
		}
	case SOT_GOTOSTOP:
		{
			// save new values about the stop line
			if(!m_bFlagNoStopLine)
			{
				m_fDist2Go=m_fDist2StopLine;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Dist %f    diff %f", m_fDist2Go, (m_fDistanceOverall-m_fDistanceOverall_Start)));
			//change state of turn if your [5 cm] before stop line
			if (m_fDistanceOverall-m_fDistanceOverall_Start > m_fDist2Go-10)
			{
				m_iStateOfTurn=SOT_WAIT;
				stop_time = _clock->GetStreamTime();
			}
			// stop line not reached --> continue going straight
			else
			{
				// linear calculation of the steering by KP
				m_fSteeringOutput=propKpSteering*(m_fOrientation2StopLine-90);

				m_fAccelerationOutput=speed2stopline;
			}
			break;
		}
    case SOT_WAIT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;

			if(i_bHaveToStop)
			{
				// waittime on stopping line
				if((_clock->GetStreamTime() - stop_time) > 3e6)
				{
					m_iStateOfTurn=SOT_PRIORITYINTRAFFIC;
				}
			}
			else
			{
				m_iStateOfTurn=SOT_PRIORITYINTRAFFIC;
			}
			break;
		}
	// all other states are different by left/straight/rigth
	default:
		{
				if(iID==-1)
				{
					RETURN_NOERROR;
				}
				else if(iID==0)
				{
					// turn left
					TurnLeft(i_bHaveToStop);
				}
				else if(iID==1)
				{
					// go straight
					GoStraight(i_bHaveToStop);
				}
				else if(iID==2)
				{
					// turn right
					TurnRight(i_bHaveToStop);
				}
				break;
		}

	}

	RETURN_NOERROR;
}

tResult cCrossing::TurnRight(tBool bHaveToStop)
{
	if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Turn Right State of Turn %d", m_iStateOfTurn));

	switch(m_iStateOfTurn)
	{
    case SOT_PRIORITYINTRAFFIC:
		{
			// Beachtung der Vorfahrtsregeln

			// tbd

			// save start values
			m_fDistanceOverall_Start=m_fDistanceOverall;
			m_fYawAngleAtStart=m_fYawAngle;
			// change state of turn
			m_iStateOfTurn=SOT_TURN;
			break;
		}
    case SOT_TURN:
		{
			m_fSteeringOutput=propSteerRight;
			m_fAccelerationOutput=speed_curve;

			// calculate yaw angle border
			tFloat32 yawangleborder=m_fYawAngleAtStart-90;
			tBool FlagCrossedBorder=tFalse;
			if(yawangleborder<0)
			{
				yawangleborder=yawangleborder+360;
				FlagCrossedBorder=tTrue;
			}

			if(FlagCrossedBorder)
			{
				//if ((m_fDistanceOverall-m_fDistanceOverall_Start > propDistRightTurn)||((m_fYawAngle < yawangleborder)&&(m_fYawAngle>yawangleborder+2)))
				if ((m_fDistanceOverall-m_fDistanceOverall_Start) > propDistRightTurn)
				{
					m_iStateOfTurn=SOT_GOSTRAIGHT;
					m_fDistanceOverall_Start=m_fDistanceOverall;
				}
			}
			else
			{
				//if ((m_fDistanceOverall-m_fDistanceOverall_Start > propDistRightTurn)||(m_fYawAngle < yawangleborder))
				if ((m_fDistanceOverall-m_fDistanceOverall_Start) > propDistRightTurn)
				{
					m_iStateOfTurn=SOT_GOSTRAIGHT;
					m_fDistanceOverall_Start=m_fDistanceOverall;
				}
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_straight;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > propDistGoStraightAfterTurn)
			{
				m_iStateOfTurn=SOT_FINISH;
			}
			break;
		}
    case SOT_FINISH:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//tbd
			m_iStateOfTurn=SOT_NOSTART;
			m_bFinishFlag=tTrue;
			m_bStart=tFalse;
			TransmitOutput();
			break;
		}
	}	
	RETURN_NOERROR;
}


tResult cCrossing::TurnLeft(tBool bHaveToStop)
{
	if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Turn Left State of Turn %d", m_iStateOfTurn));

	switch(m_iStateOfTurn)
	{
    case SOT_PRIORITYINTRAFFIC:
		{
			// Beachtung der Vorfahrtsregeln

			// tbd

			// save start values
			m_fDistanceOverall_Start=m_fDistanceOverall;
			m_fYawAngleAtStart=m_fYawAngle;
			// change state of turn
			m_iStateOfTurn=SOT_TURN;
			break;
		}
    case SOT_TURN:
		{
			m_fSteeringOutput=propSteerLeft;
			m_fAccelerationOutput=speed_curve;

			// calculate yaw angle border
			tFloat32 yawangleborder=m_fYawAngleAtStart-90;
			tBool FlagCrossedBorder=tFalse;
			if(yawangleborder<0)
			{
				yawangleborder=yawangleborder+360;
				FlagCrossedBorder=tTrue;
			}

			if(FlagCrossedBorder)
			{
				//if ((m_fDistanceOverall-m_fDistanceOverall_Start > propDistRightTurn)||((m_fYawAngle < yawangleborder)&&(m_fYawAngle>yawangleborder+2)))
				if ((m_fDistanceOverall-m_fDistanceOverall_Start) > propDistLeftTurn)
				{
					m_iStateOfTurn=SOT_GOSTRAIGHT;
					m_fDistanceOverall_Start=m_fDistanceOverall;
				}
			}
			else
			{
				//if ((m_fDistanceOverall-m_fDistanceOverall_Start > propDistRightTurn)||(m_fYawAngle < yawangleborder))
				if ((m_fDistanceOverall-m_fDistanceOverall_Start) > propDistLeftTurn)
				{
					m_iStateOfTurn=SOT_GOSTRAIGHT;
					m_fDistanceOverall_Start=m_fDistanceOverall;
				}
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_straight;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > propDistGoStraightAfterTurn)
			{
				m_iStateOfTurn=SOT_FINISH;
			}
			break;
		}
    case SOT_FINISH:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//tbd
			m_iStateOfTurn=SOT_NOSTART;
			m_bFinishFlag=tTrue;
			m_bStart=tFalse;
			TransmitOutput();
			break;
		}
	}	
	RETURN_NOERROR;
}


tResult cCrossing::GoStraight(tBool bHaveToStop)
{
	if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Go Straight State of Turn %d", m_iStateOfTurn));

	switch(m_iStateOfTurn)
	{
    case SOT_PRIORITYINTRAFFIC:
		{
			// Beachtung der Vorfahrtsregeln

			// tbd

			// save start values
			m_fDistanceOverall_Start=m_fDistanceOverall;
			m_fYawAngleAtStart=m_fYawAngle;
			// change state of turn
			m_iStateOfTurn=SOT_TURN;
			break;
		}
    case SOT_TURN:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_straight;
			
			if ((m_fDistanceOverall-m_fDistanceOverall_Start) > propDistStraight)
			{
				m_iStateOfTurn=SOT_GOSTRAIGHT;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_straight;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > propDistGoStraightAfterTurn)
			{
				m_iStateOfTurn=SOT_FINISH;
			}
			break;
		}
    case SOT_FINISH:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//tbd
			m_iStateOfTurn=SOT_NOSTART;
			m_bFinishFlag=tTrue;
			m_bStart=tFalse;
			TransmitOutput();
			break;
		}
	}	
	RETURN_NOERROR;
}

/*
tResult cCrossing::TurnLeft(tBool bHaveToStop)
{
	if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Turn Left State of Turn %d", m_iStateOfTurn));

	switch(m_iStateOfTurn)
	{
	case SOT_NOSTART:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//reset finish flag
			m_bFinishFlag=tFalse;
			// change state of turn when StartSignal is true
			if(m_bStart)
			{
				m_iStateOfTurn=SOT_GOTOSTOP;
				//save the first distance and get started
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			break;
		}
	case SOT_GOTOSTOP:
		{
			// save new values about the stop line
			if(!m_bFlagNoStopLine)
			{
				m_fDist2Go=m_fDist2StopLine;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Dist %f    diff %f", m_fDist2Go, (m_fDistanceOverall-m_fDistanceOverall_Start)));
			//change state of turn if your [5 cm] before stop line
			if (m_fDistanceOverall-m_fDistanceOverall_Start > m_fDist2Go-5)
			{
				m_iStateOfTurn=SOT_WAIT;
				stop_time = _clock->GetStreamTime();
			}
			// stop line not reached --> continue going straight
			else
			{

				if(m_fOrientation2StopLine>95)
				{
					m_fSteeringOutput=-10;
				}
				else if(m_fOrientation2StopLine<85)
				{
					m_fSteeringOutput=+10;
				}
				else
				{
					m_fSteeringOutput=0;
				}
				m_fAccelerationOutput=speed2stopline;
			}
			break;
		}
    case SOT_WAIT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;

			if(bHaveToStop)
			{
				// waittime on stopping line
				if((_clock->GetStreamTime() - stop_time) > 3e6)
				{
					m_iStateOfTurn=SOT_PRIORITYINTRAFFIC;
				}
			}
			else
			{
				m_iStateOfTurn=SOT_PRIORITYINTRAFFIC;
			}
			break;
		}
    case SOT_PRIORITYINTRAFFIC:
		{
			// Beachtung der Vorfahrtsregeln

			// tbd

			m_iStateOfTurn=SOT_TURN;
			m_fDistanceOverall_Start=m_fDistanceOverall;
			break;
		}
    case SOT_TURN:
		{
			m_fSteeringOutput=propSteerLeft;
			m_fAccelerationOutput=speed_curve;

			if (m_fDistanceOverall-m_fDistanceOverall_Start > propDistLeftTurn)
			{
				m_iStateOfTurn=SOT_GOSTRAIGHT;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_straight;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > propDistGoStraightAfterTurn)
			{
				m_iStateOfTurn=SOT_FINISH;
			}
			break;
		}
    case SOT_FINISH:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=0;
			//tbd
			m_iStateOfTurn=SOT_NOSTART;
			m_bFinishFlag=tTrue;
			m_bStart=tFalse;
			TransmitOutput();
			break;
		}
	}	
	RETURN_NOERROR;
}
*/

tResult cCrossing::TransmitOutput()
{
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	cObjectPtr<IMediaSample> pMediaSamplesteer;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalLeftEnabled;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalRightEnabled;
	cObjectPtr<IMediaSample> pMediaSamplefinishflag;

	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	AllocMediaSample((tVoid**)&pMediaSamplesteer);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalLeftEnabled);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalRightEnabled);
	AllocMediaSample((tVoid**)&pMediaSamplefinishflag);

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
	
	// TurnSignalLeftEnabled
	cObjectPtr<IMediaSerializer> pSerializerTurnSignalLeftEnabled;
	m_pDescriptionOutputTurnSignalLeftEnabled->GetMediaSampleSerializer(&pSerializerTurnSignalLeftEnabled);
	tInt nSizeTurnSignalLeftEnabled = pSerializerTurnSignalLeftEnabled->GetDeserializedSize();
	pMediaSampleTurnSignalLeftEnabled->AllocBuffer(nSizeTurnSignalLeftEnabled);
	cObjectPtr<IMediaCoder> pCoderOutputTurnSignalLeftEnabled;
	m_pDescriptionOutputTurnSignalLeftEnabled->WriteLock(pMediaSampleTurnSignalLeftEnabled, &pCoderOutputTurnSignalLeftEnabled);
	pCoderOutputTurnSignalLeftEnabled->Set("bValue", (tVoid*)&(m_bTurnSignalLeftEnabled));
	pCoderOutputTurnSignalLeftEnabled->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputTurnSignalLeftEnabled->Unlock(pCoderOutputTurnSignalLeftEnabled);
	pMediaSampleTurnSignalLeftEnabled->SetTime(_clock->GetStreamTime());
	m_oOutputTurnSignalLeftEnabled.Transmit(pMediaSampleTurnSignalLeftEnabled);

	// TurnSignalRightEnabled
	cObjectPtr<IMediaSerializer> pSerializerTurnSignalRightEnabled;
	m_pDescriptionOutputTurnSignalRightEnabled->GetMediaSampleSerializer(&pSerializerTurnSignalRightEnabled);
	tInt nSizeTurnSignalRightEnabled = pSerializerTurnSignalRightEnabled->GetDeserializedSize();
	pMediaSampleTurnSignalRightEnabled->AllocBuffer(nSizeTurnSignalRightEnabled);
	cObjectPtr<IMediaCoder> pCoderOutputTurnSignalRightEnabled;
	m_pDescriptionOutputTurnSignalRightEnabled->WriteLock(pMediaSampleTurnSignalRightEnabled, &pCoderOutputTurnSignalRightEnabled);
	pCoderOutputTurnSignalRightEnabled->Set("bValue", (tVoid*)&(m_bTurnSignalRightEnabled));
	pCoderOutputTurnSignalRightEnabled->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputTurnSignalRightEnabled->Unlock(pCoderOutputTurnSignalRightEnabled);
	pMediaSampleTurnSignalRightEnabled->SetTime(_clock->GetStreamTime());
	m_oOutputTurnSignalRightEnabled.Transmit(pMediaSampleTurnSignalRightEnabled);

	// FinishFlag
    cObjectPtr<IMediaSerializer> pSerializerfinishflag;
	m_pDescFinishFlag->GetMediaSampleSerializer(&pSerializerfinishflag);
	tInt nSizefinishflag = pSerializerfinishflag->GetDeserializedSize();
	pMediaSamplefinishflag->AllocBuffer(nSizefinishflag);
	cObjectPtr<IMediaCoder> pCoderOutputfinishflag;
	m_pDescFinishFlag->WriteLock(pMediaSamplefinishflag, &pCoderOutputfinishflag);
	pCoderOutputfinishflag->Set("bValue", (tVoid*)&(m_bFinishFlag));//bValue
	pCoderOutputfinishflag->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescFinishFlag->Unlock(pCoderOutputfinishflag);
	pMediaSamplefinishflag->SetTime(_clock->GetStreamTime());
	m_oOutputFinishFlag.Transmit(pMediaSamplefinishflag);

	RETURN_NOERROR;
}
