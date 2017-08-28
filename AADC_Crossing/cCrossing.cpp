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

#define DISTANCE_TRAJ1 			"cCrossing::distance_st1"
#define STEER_TRAJ1 			"cCrossing::st1_steer"
#define SPEED_TRAJ1			 	"cCrossing::st1_speed"

#define DISTANCE_TRAJ3 			"cCrossing::distance_st2"
#define STEER_TRAJ3 			"cCrossing::st2_steer"
#define SPEED_TRAJ3			 	"cCrossing::st2_speed"

/// Create filter shell
ADTF_FILTER_PLUGIN("Crossing", OID_ADTF_CROSSING_FILTER, cCrossing);

using namespace roadsignIDs;

cCrossing::cCrossing(const tChar* __info):cFilter(__info)
{
	SetPropertyFloat(DISTANCE_TRAJ1,0.0);
    SetPropertyBool(DISTANCE_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1 NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory");

	SetPropertyFloat(STEER_TRAJ1,90);
    SetPropertyBool(STEER_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ1 NSSUBPROP_DESCRIPTION, "the steer for the 1st trajectory");

	SetPropertyFloat(SPEED_TRAJ1,83.5);
    SetPropertyBool(SPEED_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ1 NSSUBPROP_DESCRIPTION, "the speed for the 1st trajectory");



	SetPropertyFloat(DISTANCE_TRAJ3,0.3);
    SetPropertyBool(DISTANCE_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ3 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

	SetPropertyFloat(STEER_TRAJ3,90);
    SetPropertyBool(STEER_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ3 NSSUBPROP_DESCRIPTION, "the steer for the straight after taking curve trajectory");

	SetPropertyFloat(SPEED_TRAJ3,83);
    SetPropertyBool(SPEED_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ3 NSSUBPROP_DESCRIPTION, "the speed for the straight after taking curve trajectory");

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
		
		//create pin for traffic sign input
		RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("TrafficSign", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));

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
    }
    else if (eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsRoadSignSet = tFalse;
		//m_bIDsSterringSet = tFalse;

		m_bStart= tFalse;

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
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1))
	{
		distance_st1 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ1))
	{
		steer_st1 = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ1));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ1))
	{
		speed_st1 = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ1));
	}


	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ3))
	{
		distance_st2 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ3));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ3))
	{
		steer_st2 = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ3));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ3))
	{
		speed_st2 = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ3));
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
        if (pMediaSample != NULL && m_pDescriptionInputTrafficSign != NULL)
	{
		// Input signal at Start
		if (pSource == &m_oStart)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescStart->Unlock(pCoderInput);

			// stop signal
			if(!m_bStart)
			{

				// reset
				// init values
			}
		}

		// Input signal at Traffic sign
		if (pSource == &m_oInputTrafficSign)
		{
			__adtf_sample_read_lock_mediadescription(m_pDescriptionInputTrafficSign,pMediaSample,pCoderInput);
			// get IDs
			if(!m_bIDsRoadSignSet)
			{
				pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
				pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
				m_bIDsRoadSignSet = tTrue;
			}

			pCoderInput->Get("i16Identifier", (tVoid*)&m_iTrafficSignID);
			pCoderInput->Get("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
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

			ProcessManeuver();
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
			break;
		}
	// give away
	case MARKER_ID_STOPANDGIVEWAY:
	case MARKER_ID_GIVEWAY:
		{
			TurnRight(tTrue);
			break;
		}
	// parking --> stop crossing
	case MARKER_ID_PARKINGAREA:
	case MARKER_ID_PEDESTRIANCROSSING:
		{
			TurnLeft(tTrue);
			break;
		}
	// have away
	case MARKER_ID_HAVEWAY:
		{
			TurnRight(tFalse);
			break;
		}
	// not implemented because not used in AADC 2017
	case MARKER_ID_AHEADONLY:
	case  MARKER_ID_ROUNDABOUT:
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

	TransmitOutput();

	//LOG_INFO(cString::Format("Johannes ID %d", m_iTrafficSignID));
	// LOG_INFO(cString::Format("Johannes steering %f", m_fSteeringOutput));

	RETURN_NOERROR;
}

tResult cCrossing::TurnRight(tBool bHaveToStop)
{
	LOG_INFO(cString::Format("Turn Right State of Turn %d", m_iStateOfTurn));

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
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_st1;
			
			//change state of turn if your at stop line
			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st1)
			{
				m_iStateOfTurn=SOT_WAIT;
				stop_time = _clock->GetStreamTime();
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
				if((_clock->GetStreamTime() - stop_time)/1000000 > 3)
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
			m_fSteeringOutput=steer_st1;
			m_fAccelerationOutput=speed_st1;

			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st1)
			{
				m_iStateOfTurn=SOT_GOSTRAIGHT;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_st1;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st1)
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
			break;
		}
	}	
	RETURN_NOERROR;
}


tResult cCrossing::TurnLeft(tBool bHaveToStop)
{
	LOG_INFO(cString::Format("Turn Left State of Turn %d", m_iStateOfTurn));

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
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_st2;
			
			//change state of turn if your at stop line
			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st2)
			{
				m_iStateOfTurn=SOT_WAIT;
				stop_time = _clock->GetStreamTime();
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
				if((_clock->GetStreamTime() - stop_time)/1000000 > 3)
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
			m_fSteeringOutput=steer_st2;
			m_fAccelerationOutput=speed_st2;

			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st2)
			{
				m_iStateOfTurn=SOT_GOSTRAIGHT;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
			break;
		}
    case SOT_GOSTRAIGHT:
		{
			m_fSteeringOutput=0;
			m_fAccelerationOutput=speed_st2;
			if (m_fDistanceOverall-m_fDistanceOverall_Start > distance_st2)
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
			break;
		}
	}	
	RETURN_NOERROR;
}

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