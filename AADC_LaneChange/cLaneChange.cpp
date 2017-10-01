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
#include "cLaneChange.h"

// change part 1 left
#define PROP_DISTANCE_LEFT	"cLaneChange::prop_dist_left"
#define PROP_SPEED_LEFT		"cLaneChange::prop_speed_left"
#define PROP_STEER_LEFT		"cLaneChange::prop_steer_left"

//change part 2 right
#define PROP_DISTANCE_RIGHT		"cLaneChange::prop_dist_right"
#define PROP_SPEED_RIGHT		"cLaneChange::prop_speed_right"
#define PROP_STEER_RIGHT		"cLaneChange::prop_steer_right"

//change part 3 right back
#define PROP_DISTANCE_RIGHT_BACK	"cLaneChange::prop_dist_right_back"
#define PROP_SPEED_RIGHT_BACK		"cLaneChange::prop_speed_right_back"
#define PROP_STEER_RIGHT_BACK		"cLaneChange::prop_steer_right_back"

// change part 4 left back
#define PROP_DISTANCE_LEFT_BACK		"cLaneChange::prop_dist_left_back"
#define PROP_SPEED_LEFT_BACK		"cLaneChange::prop_speed_left_back"
#define PROP_STEER_LEFT_BACK		"cLaneChange::prop_steer_left_back"

#define PROP_SPEED_OVERTAKE		"cLaneChange::prop_speed_overtake"


/// Create filter shell
ADTF_FILTER_PLUGIN("LaneChange", OID_ADTF_LANECHANGE_FILTER, cLaneChange);

using namespace SensorDefinition;

cLaneChange::cLaneChange(const tChar* __info):cFilter(__info)
{
	// debug
	SetPropertyBool("Debug Output to Console",true);

	// lane change left 1
	SetPropertyFloat(PROP_DISTANCE_LEFT,0.7);
    SetPropertyBool(PROP_DISTANCE_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_DISTANCE_LEFT NSSUBPROP_DESCRIPTION, "the distance for lane change left 1");

	SetPropertyFloat(PROP_SPEED_LEFT,0.35);
    SetPropertyBool(PROP_SPEED_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED_LEFT NSSUBPROP_DESCRIPTION, "the speed for lane change left 1");

	SetPropertyFloat(PROP_STEER_LEFT,90);
    SetPropertyBool(PROP_STEER_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_STEER_LEFT NSSUBPROP_DESCRIPTION, "the steer for lane change left 1");

	// lane change right 2
	SetPropertyFloat(PROP_DISTANCE_RIGHT,0.6);
    SetPropertyBool(PROP_DISTANCE_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_DISTANCE_RIGHT NSSUBPROP_DESCRIPTION, "the distance for lane change right 2");

	SetPropertyFloat(PROP_SPEED_RIGHT, 0.35);
    SetPropertyBool(PROP_SPEED_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED_RIGHT NSSUBPROP_DESCRIPTION, "the speed for lane change right 2");

	SetPropertyFloat(PROP_STEER_RIGHT,90);
    SetPropertyBool(PROP_STEER_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_STEER_RIGHT NSSUBPROP_DESCRIPTION, "the steer for lane change right 2");

	// overtake
	SetPropertyFloat(PROP_SPEED_OVERTAKE,0.4);
    SetPropertyBool(PROP_SPEED_OVERTAKE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED_OVERTAKE NSSUBPROP_DESCRIPTION, "speed for overtaking");

	// lane change right back 3
	SetPropertyFloat(PROP_DISTANCE_RIGHT_BACK,0.6);
    SetPropertyBool(PROP_DISTANCE_RIGHT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_DISTANCE_RIGHT_BACK NSSUBPROP_DESCRIPTION, "the distance for lane change right back 3");

	SetPropertyFloat(PROP_SPEED_RIGHT_BACK, 0.35);
    SetPropertyBool(PROP_SPEED_RIGHT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED_RIGHT_BACK NSSUBPROP_DESCRIPTION, "the speed for lane change right back 3");

	SetPropertyFloat(PROP_STEER_RIGHT_BACK,90);
    SetPropertyBool(PROP_STEER_RIGHT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_STEER_RIGHT_BACK NSSUBPROP_DESCRIPTION, "the steer for lane change right back 3");

	// lane change left back 4
	SetPropertyFloat(PROP_DISTANCE_LEFT_BACK,0.7);
    SetPropertyBool(PROP_DISTANCE_LEFT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_DISTANCE_LEFT_BACK NSSUBPROP_DESCRIPTION, "the distance for lane change left back 4");

	SetPropertyFloat(PROP_SPEED_LEFT_BACK,0.35);
    SetPropertyBool(PROP_SPEED_LEFT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED_LEFT_BACK NSSUBPROP_DESCRIPTION, "the speed for lane change left back 4");

	SetPropertyFloat(PROP_STEER_LEFT_BACK,90);
    SetPropertyBool(PROP_STEER_LEFT_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_STEER_LEFT_BACK NSSUBPROP_DESCRIPTION, "the steer for lane change left back 4");
}

cLaneChange::~cLaneChange()
{

}

tResult cLaneChange::Init(tInitStage eStage, __exception)
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

		//create pin for Distance over all input
		tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
		cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
		RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));

		// creeate pin for start signal
		tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalstart);
		cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
		RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStart));

		// create pin for speed input
		tChar const * strDescSignalSpeed = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalSpeed);
		cObjectPtr<IMediaType> pTypeSignalSpeed = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSpeed, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputSpeed));
		RETURN_IF_FAILED(m_oInputSpeed.Create("Speed In", pTypeSignalSpeed, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSpeed));

		// create pin for steering input
		tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalSteering);
		cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputSteering));
		RETURN_IF_FAILED(m_oInputSteering.Create("Steering In", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSteering));

		// create pin for ultrasonic struct input
		tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strUltrasonicStruct);
		cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
		RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

		//create pin for steering signal output
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
        RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

		//create pin for acceleration signal output
		tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
		cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSpeed));
		RETURN_IF_FAILED(m_oOutputSpeed.Create("Acceleration", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeed));

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
    }
    else if (eStage == StageGraphReady)
    {
		// initial values for steering and acceleration
		m_fSpeedOutput=0;
		m_fSteeringOutput=0;

		// init states
		m_bStart= tFalse;
		m_iStateOfLaneChange=SOLG_NOSTART;

		// init US struct
		m_szIdsUsStructSet=tFalse;
		
		for(int i=0;i<=10;i++)
		{
			m_aUSSensors[i]=400;
		}

		// init output signals
		m_bFinishFlag=tFalse;
		m_bTurnSignalLeftEnabled=tFalse;
		m_bTurnSignalRightEnabled=tFalse;
    }

    RETURN_NOERROR;
}

tResult cLaneChange::Shutdown(tInitStage eStage, __exception)
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

tResult cLaneChange::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}
		
tResult cLaneChange::ReadProperties(const tChar* strPropertyName)
{
	//debug
	m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

	// lane change part left 1
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_DISTANCE_LEFT))
	{
		m_fPropDistLeft = static_cast<tFloat32> (GetPropertyFloat(PROP_DISTANCE_LEFT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED_LEFT))
	{
		m_fPropSpeedLeft = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED_LEFT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_STEER_LEFT))
	{
		m_fPropSteerLeft = static_cast<tFloat32> (GetPropertyFloat(PROP_STEER_LEFT));
	}

	// lane change part right 2
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_DISTANCE_RIGHT))
	{
		m_fPropDistRight = static_cast<tFloat32> (GetPropertyFloat(PROP_DISTANCE_RIGHT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED_RIGHT))
	{
		m_fPropSpeedRight = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED_RIGHT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_STEER_RIGHT))
	{
		m_fPropSteerRight = static_cast<tFloat32> (GetPropertyFloat(PROP_STEER_RIGHT));
	}

	// overtake
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED_OVERTAKE))
	{
		m_fPropSpeedOvertake = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED_OVERTAKE));
	}

	// lane change part right back 3
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_DISTANCE_RIGHT))
	{
		m_fPropDistRightBack = static_cast<tFloat32> (GetPropertyFloat(PROP_DISTANCE_RIGHT_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED_RIGHT_BACK))
	{
		m_fPropSpeedRightBack = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED_RIGHT_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_STEER_RIGHT_BACK))
	{
		m_fPropSteerRightBack = static_cast<tFloat32> (GetPropertyFloat(PROP_STEER_RIGHT_BACK));
	}

	// lane change part left back 4
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_DISTANCE_LEFT_BACK))
	{
		m_fPropDistLeftBack = static_cast<tFloat32> (GetPropertyFloat(PROP_DISTANCE_LEFT_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED_LEFT_BACK))
	{
		m_fPropSpeedLeftBack = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED_LEFT_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_STEER_LEFT))
	{
		m_fPropSteerLeftBack = static_cast<tFloat32> (GetPropertyFloat(PROP_STEER_LEFT));
	}




	RETURN_NOERROR;
}

tResult cLaneChange::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);
	{
		// Input signal at Start
		if (pSource == &m_oStart)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescStart->Unlock(pCoderInput);

			// stop lane change
			if(!m_bStart)
			{
				m_iStateOfLaneChange=SOLG_NOSTART;
				m_bTurnSignalLeftEnabled=tFalse;
				m_bTurnSignalRightEnabled=tFalse;
				m_fSteeringOutput=0;
				m_fSpeedOutput=0;
				TransmitOutput();
			}
		}

		else if(pSource == &m_oInputSpeed)
        {
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescriptionInputSpeed->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fSpeedInput);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescriptionInputSpeed->Unlock(pCoderInput);
        }
		else if (pSource == &m_oInputSteering)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescriptionInputSteering->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fSteeringInput);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescriptionInputSteering->Unlock(pCoderInput);

		}

		else if(pSource == &m_oInputUsStruct)
        {
            ProcessInputUS(pMediaSample);
        }

		// Input signal at Distance Overall
		else if (pSource == &m_oDistanceOverall)
		{
			//LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);

			if(m_bStart)
			{
				ProcessManeuver();
			}
		}

    }
    RETURN_NOERROR;
}
}

tResult cLaneChange::ProcessManeuver()
{
	switch(m_iStateOfLaneChange)
	{
	case SOLG_NOSTART:
		{
			m_fSteeringOutput=0;
			m_fSpeedOutput=0;
			// start lane change
			if(m_bStart)
			{
				m_iStateOfLaneChange=SOLG_CHANGELANE1;
				m_fDistStart=m_fDistanceOverall;
				m_bTurnSignalLeftEnabled=tTrue;
			}
			break;
		}
	case SOLG_CHANGELANE1:
		{
			TurnLeft();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDistLeft)
			{
				m_iStateOfLaneChange=SOLG_CHANGELANE2;
				m_fDistStart = m_fDistanceOverall;
			}
			break;
		}
	case SOLG_CHANGELANE2:
		{
			TurnRight();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDistRight)
			{
				m_iStateOfLaneChange=SOLG_WRONGLANE_CHECK_FR;
				m_fDistStart = m_fDistanceOverall;
			}
			break;
		}
	// check front right USsensor if it's free
	case SOLG_WRONGLANE_CHECK_FR:
		{
			// output = input of lane follower
			m_fSteeringOutput=m_fSteeringInput;
			m_fSpeedOutput=m_fPropSpeedOvertake;

			m_bTurnSignalLeftEnabled=tFalse;

			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("lane change state %i  dist %f", m_iStateOfLaneChange,m_aUSSensors[US_FRONTRIGHT]));

			// change state
			if ((m_aUSSensors[US_FRONTRIGHT]>100) && (m_fDistanceOverall-m_fDistStart>0.2))
			{
				m_iStateOfLaneChange=SOLG_WRONGLANE_CHECK_R;
				m_fDistStart = m_fDistanceOverall;
			}

			break;
		}
	// check right side USsensor if it's free
	case SOLG_WRONGLANE_CHECK_R:
		{
			// output = input of lane follower
			m_fSteeringOutput=m_fSteeringInput;
			m_fSpeedOutput=m_fPropSpeedOvertake;

			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("lane change state %i  dist %f", m_iStateOfLaneChange,m_aUSSensors[US_SIDERIGHT]));

			// change state
			if ((m_aUSSensors[US_SIDERIGHT]>100) && (m_fDistanceOverall-m_fDistStart>0.3))
			{
				m_iStateOfLaneChange=SOLG_WRONGLANE_CHECK_RR;
				m_fDistStart = m_fDistanceOverall;
			}

			break;
		}
	// check right rear USsensor if it's free
	case SOLG_WRONGLANE_CHECK_RR:
		{
			// output = input of lane follower
			m_fSteeringOutput=m_fSteeringInput;
			m_fSpeedOutput=m_fPropSpeedOvertake;

			// change state
			if ((m_fDistanceOverall-m_fDistStart>0.2))
			{
				m_iStateOfLaneChange=SOLG_CHANGEBACK1;
				m_fDistStart = m_fDistanceOverall;
			}
			// check in the next 20 cm if the right side is free
			else
			{
				if(m_aUSSensors[US_SIDERIGHT]<100)
					{
						m_fDistStart=m_fDistanceOverall;
					}
			}
			break;
		}
	case SOLG_CHANGEBACK1:
		{
			TurnRightBack();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDistRightBack)
			{
				m_iStateOfLaneChange=SOLG_CHANGEBACK2;
				m_fDistStart = m_fDistanceOverall;
			}
			break;
		}
	case SOLG_CHANGEBACK2:
		{
			TurnLeftBack();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDistLeftBack)
			{
				m_iStateOfLaneChange=SOLG_FINISH;
				m_fDistStart = m_fDistanceOverall;

				m_bTurnSignalRightEnabled=tFalse;
			}
			break;
		}
	case SOLG_FINISH:
		{
			//send finish flag
			m_bFinishFlag=tTrue;

			// reset values for steering and acceleration
			m_fSpeedOutput=0;
			m_fSteeringOutput=0;

			// reset states
			m_bStart= tFalse;
			m_iStateOfLaneChange=SOLG_NOSTART;

			// reset output signals
			m_bTurnSignalLeftEnabled=tFalse;
			m_bTurnSignalRightEnabled=tFalse;
			break;
		}
	}

	TransmitOutput();

	RETURN_NOERROR;
}

tResult cLaneChange::TurnRight()
{
	m_fSteeringOutput=m_fPropSteerRight;
	m_fSpeedOutput=m_fPropSpeedRight;
	RETURN_NOERROR;
}


tResult cLaneChange::TurnLeft()
{
	m_fSteeringOutput=(-1)*m_fPropSteerLeft;
	m_fSpeedOutput=m_fPropSpeedLeft;
	RETURN_NOERROR;
}

tResult cLaneChange::TurnRightBack()
{
	m_fSteeringOutput=m_fPropSteerRightBack;
	m_fSpeedOutput=m_fPropSpeedRightBack;
	RETURN_NOERROR;
}


tResult cLaneChange::TurnLeftBack()
{
	m_fSteeringOutput=(-1)*m_fPropSteerLeftBack;
	m_fSpeedOutput=m_fPropSpeedLeftBack;
	RETURN_NOERROR;
}


tResult cLaneChange::ProcessInputUS(IMediaSample* pMediaSample)
{
	// use mutex to access min US value
	__synchronized_obj(m_critSecMinimumUsValue);

    //read out incoming Media Samples
    __adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);

    if(!m_szIdsUsStructSet)
    {
        tBufferID idValue, idTimestamp;
        m_szIdUsStructValues.clear();
        m_szIdUsStructTss.clear();
		
	pCoderInput->GetID("tFrontLeft.f32Value", idValue);
	pCoderInput->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
	m_szIdUsStructValues.push_back(idValue);
	m_szIdUsStructTss.push_back(idTimestamp);

	pCoderInput->GetID("tFrontCenterLeft.f32Value", idValue);
	pCoderInput->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
	m_szIdUsStructValues.push_back(idValue);
	m_szIdUsStructTss.push_back(idTimestamp);

	pCoderInput->GetID("tFrontCenter.f32Value", idValue);
	pCoderInput->GetID("tFrontCenter.ui32ArduinoTimestamp", idTimestamp);
	m_szIdUsStructValues.push_back(idValue);
	m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tFrontCenterRight.f32Value", idValue);
        pCoderInput->GetID("tFrontCenterRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tFrontRight.f32Value", idValue);
        pCoderInput->GetID("tFrontRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tSideRight.f32Value", idValue);
        pCoderInput->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tSideLeft.f32Value", idValue);
        pCoderInput->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearLeft.f32Value", idValue);
        pCoderInput->GetID("tRearLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearCenter.f32Value", idValue);
        pCoderInput->GetID("tRearCenter.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearRight.f32Value", idValue);
        pCoderInput->GetID("tRearRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

		// m_szIdsUsStructSet = tTrue;
		m_szIdsUsStructSet = tTrue;

        }
	
	// iterate through all values and save them in m_aUSSensors
	for (int i=0;i<10;++i)
	{	
		tFloat32 buffer=-1;
		pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buffer);
		// Wrong Us-values are  0 or -1
		if (buffer>0)
		{
			pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buffer);
			m_aUSSensors[i]=buffer;
			//if(m_bDebugModeEnabled) LOG_INFO(cString::Format("buffer %f", buffer));
		}
	}
	RETURN_NOERROR;
}



tResult cLaneChange::TransmitOutput()
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
	m_pDescriptionOutputSpeed->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDescriptionOutputSpeed->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("f32Value", (tVoid*)&(m_fSpeedOutput));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputSpeed->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_oOutputSpeed.Transmit(pMediaSampleaccelerate);

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

