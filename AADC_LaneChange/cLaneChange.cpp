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

#define PROP_DISTANCE	"cLaneChange::prop_dist"
#define PROP_SPEED		"cLaneChange::prop_speed"
#define PROP_STEER		"cLaneChange::prop_steer"


/// Create filter shell
ADTF_FILTER_PLUGIN("LaneChange", OID_ADTF_LANECHANGE_FILTER, cLaneChange);

cLaneChange::cLaneChange(const tChar* __info):cFilter(__info)
{
	SetPropertyFloat(PROP_DISTANCE,0.5);
    SetPropertyBool(PROP_DISTANCE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_DISTANCE NSSUBPROP_DESCRIPTION, "the distance for lane change");

	SetPropertyFloat(PROP_SPEED,20);
    SetPropertyBool(PROP_SPEED NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_SPEED NSSUBPROP_DESCRIPTION, "the speed for lane change");

	SetPropertyFloat(PROP_STEER,60);
    SetPropertyBool(PROP_STEER NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(PROP_STEER NSSUBPROP_DESCRIPTION, "the steer for lane change");
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
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_DISTANCE))
	{
		m_fPropDist = static_cast<tFloat32> (GetPropertyFloat(PROP_DISTANCE));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_SPEED))
	{
		m_fPropSpeed = static_cast<tFloat32> (GetPropertyFloat(PROP_SPEED));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROP_STEER))
	{
		m_fPropSteer = static_cast<tFloat32> (GetPropertyFloat(PROP_STEER));
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

		// Input signal at Distance Overall
		if (pSource == &m_oDistanceOverall)
		{
			//LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);

			ProcessManeuver();
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
			if (m_fDistanceOverall-m_fDistStart > m_fPropDist)
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
			if (m_fDistanceOverall-m_fDistStart > m_fPropDist)
			{
				m_iStateOfLaneChange=SOLG_WRONGLANE;
				m_fDistStart = m_fDistanceOverall;
			}
			break;
		}
	case SOLG_WRONGLANE:
		{
			/*

			TO BE CHANGED

			speed_out = speed_in of lane follower

			steer_out = steer_in of lane follower

			Change back by signal or by USstruct

			*/

			m_fSteeringOutput=0;
			m_fSpeedOutput=-10;

			m_bTurnSignalLeftEnabled=tFalse;

			// change state
			if (m_fDistanceOverall-m_fDistStart > 1.0)
			{
				m_iStateOfLaneChange=SOLG_CHANGEBACK1;
				m_fDistStart = m_fDistanceOverall;
				
				m_bTurnSignalRightEnabled=tTrue;
			}

			break;
		}
	case SOLG_CHANGEBACK1:
		{
			TurnRight();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDist)
			{
				m_iStateOfLaneChange=SOLG_CHANGEBACK2;
				m_fDistStart = m_fDistanceOverall;
			}
			break;
		}
	case SOLG_CHANGEBACK2:
		{
			TurnLeft();

			// after prop_dist change state
			if (m_fDistanceOverall-m_fDistStart > m_fPropDist)
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
	m_fSteeringOutput=m_fPropSteer;
	m_fSpeedOutput=(-1)*m_fPropSpeed;
	RETURN_NOERROR;
}


tResult cLaneChange::TurnLeft()
{
	m_fSteeringOutput=(-1)*m_fPropSteer;
	m_fSpeedOutput=(-1)*m_fPropSpeed;
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