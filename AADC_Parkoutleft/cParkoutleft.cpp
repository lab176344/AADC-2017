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
#include "cParkoutleft.h"

#define DISTANCE_LANE_FOLLOW 			"cParkoutleft::DIST_LANEFOLLOW"
#define SPEED_LANE_FOLLOW 			"cParkoutleft::SPEED_LANEFOLLOW"

#define DISTANCE_OUT_FRONT			"cParkoutleft::DIST_OUTFRONT"
#define STEER_OUT_FRONT				"cParkoutleft::STEER_OUTFRONT"
#define SPEED_OUT_FRONT				"cParkoutleft::SPEED_OUTFRONT"

#define DISTANCE_IN_BACK			"cParkoutleft::DIST_INBACK"
#define STEER_IN_BACK				"cParkoutleft::STEER_INBACK"
#define SPEED_IN_BACK				"cParkoutleft::SPEED_INBACK"

#define DISTANCE_LANEFOLLOW_BACK		"cParkoutleft::DIST_LANEFOLLOWBACK"
#define SPEED_LANEFOLLOW_BACK			"cParkoutleft::SPEED_LANEFOLLOWBACK"

/// Create filter shell
ADTF_FILTER_PLUGIN("Parkoutleft", OID_ADTF_PARKOUTLEFT, cParkoutleft);

using namespace roadsignIDs;

cParkoutleft::cParkoutleft(const tChar* __info):cFilter(__info)
{
// set properties for lanefollow front
	SetPropertyFloat(DISTANCE_LANE_FOLLOW,0.7);
    SetPropertyBool(DISTANCE_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the distance for the lanefollow");

	SetPropertyFloat(SPEED_LANE_FOLLOW,-11);
    SetPropertyBool(SPEED_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the speed for the lanefollow");

// set properties for outsteer front	
	SetPropertyFloat(DISTANCE_OUT_FRONT,0.6);
    SetPropertyBool(DISTANCE_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_OUT_FRONT NSSUBPROP_DESCRIPTION, "the distance for the steer outside");

	SetPropertyFloat(STEER_OUT_FRONT,-75);
    SetPropertyBool(STEER_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_OUT_FRONT NSSUBPROP_DESCRIPTION, "the steer for the steer outside");

	SetPropertyFloat(SPEED_OUT_FRONT,-11);
    SetPropertyBool(SPEED_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_OUT_FRONT NSSUBPROP_DESCRIPTION, "the speed for the steer outside");

// set properties for insteer back
	SetPropertyFloat(DISTANCE_IN_BACK,0.9);
    SetPropertyBool(DISTANCE_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_IN_BACK NSSUBPROP_DESCRIPTION, "the distance for the steer outside");

	SetPropertyFloat(STEER_IN_BACK,100);
    SetPropertyBool(STEER_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_IN_BACK NSSUBPROP_DESCRIPTION, "the steer for the steer outside");

	SetPropertyFloat(SPEED_IN_BACK,11);
    SetPropertyBool(SPEED_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_IN_BACK NSSUBPROP_DESCRIPTION, "the speed for the steer outside");

// set property for lane follow back
	SetPropertyFloat(DISTANCE_LANEFOLLOW_BACK,0.3);
    SetPropertyBool(DISTANCE_LANEFOLLOW_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_LANEFOLLOW_BACK NSSUBPROP_DESCRIPTION, "the distance for the Lanefollow back");

	SetPropertyFloat(SPEED_LANEFOLLOW_BACK,10);
    SetPropertyBool(SPEED_LANEFOLLOW_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_LANEFOLLOW_BACK NSSUBPROP_DESCRIPTION, "the speed for the Lanefollow back");
}



cParkoutleft::~cParkoutleft()
{

}

tResult cParkoutleft::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // get a media type for the input pin
        //cObjectPtr<IMediaType> pInputType;
        //RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the input pin
        //RETURN_IF_FAILED(m_oInputPin.Create("input_template", pInputType, this));
        //RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        // get a media type for the output pin
        //cObjectPtr<IMediaType> pOutputType;
        //RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the output pin
        //RETURN_IF_FAILED(m_oOutputPin.Create("output_template", pOutputType, this));
        //RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));
        
	//get description for wheel sensors data pins
//        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
//        RETURN_IF_POINTER_NULL(strDescWheelData);
  //      cObjectPtr<IMediaDescriptionManager> pDescManager;

		cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

//create pin for start pin		
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



//create pin for Distance over all input
		tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
		cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
		RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));


//create pin for steering signal output
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
        	RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        	RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

//create pin for steering signal output
		tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
		cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
		RETURN_IF_FAILED(m_oOutputAcceleration.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
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


    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
	
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cParkoutleft::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
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
		m_bStart_step1=tFalse;
		m_bOutsteer_front=tFalse;
		m_bInsteer_back=tFalse;
		m_bBack_lanefollow=tFalse;
		m_bWait=tFalse;
		m_bFinished=tFalse;
		m_bTurnSignalLeftEnabled=tFalse;
		m_bTurnSignalRightEnabled=tFalse;
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

tResult cParkoutleft::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}

tResult cParkoutleft::ReadProperties(const tChar* strPropertyName)
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
//check properties for steer out front
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_OUT_FRONT))
	{
		DIST_OUTFRONT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_OUT_FRONT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_OUT_FRONT))
	{
		SPEED_OUTFRONT = static_cast<tFloat32> (GetPropertyFloat(SPEED_OUT_FRONT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_OUT_FRONT))
	{
		STEER_OUTFRONT = static_cast<tFloat32> (GetPropertyFloat(STEER_OUT_FRONT));
	}
//check property for steer in back
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_IN_BACK))
	{
		DIST_INBACK = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_IN_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_IN_BACK))
	{
		SPEED_INBACK = static_cast<tFloat32> (GetPropertyFloat(SPEED_IN_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_IN_BACK))
	{
		STEER_INBACK = static_cast<tFloat32> (GetPropertyFloat(STEER_IN_BACK));
//check property for backlanefollow
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_LANEFOLLOW_BACK))
	{
		DIST_LANEFOLLOWBACK = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_LANEFOLLOW_BACK));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_LANEFOLLOW_BACK))
	{
		SPEED_LANEFOLLOWBACK = static_cast<tFloat32> (GetPropertyFloat(SPEED_LANEFOLLOW_BACK));
	}
	}
	RETURN_NOERROR;
}

tResult cParkoutleft::OnPinEvent(IPin* pSource,
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
			LOG_INFO(cString::Format("Start bool false"));
				m_fAccelerationOutput=0;
				m_fSteeringOutput=0;
				m_bStart_step1=tFalse;
				m_bOutsteer_front=tFalse;
				m_bInsteer_back=tFalse;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tTrue;



			}
			else

			{
			LOG_INFO(cString::Format("Start bool true"));
				if (m_bFinished)
                                {				
				m_bFinished= tFalse;
				m_iTrafficSignID = MARKER_ID_NOMATCH ;
				m_bIDsRoadSignSet = tFalse;


				}

				/*
				// init process values
				m_bStarted=tFalse;
				m_bStoppedAtLine=tFalse;
				m_bWaited=tFalse;
				m_bTurned=tFalse;
				m_bFinished=tFalse;

				m_bTurnSignalLeftEnabled=tFalse;
				m_bTurnSignalRightEnabled=tFalse;
				*/
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


	
tResult cParkoutleft::ProcessManeuver()
{
	// StartSignal is true
	if(m_bStart)
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
			break;
		}
	// parking
	case MARKER_ID_PARKINGAREA:
		{
			stage_park();
		}
	case MARKER_ID_PEDESTRIANCROSSING:
		{
			break;
		}
	// have away
	case MARKER_ID_HAVEWAY:
		{
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
	case MARKER_ID_KMH50: 
	case MARKER_ID_KMH100: 
	case MARKER_ID_NOMATCH:
		{
			break;
		}
	}

	}
	else if(!m_bStart)
	{
		m_fSteeringOutput=0;  
		m_fAccelerationOutput=0;
		m_bStart_step1=tFalse;
		m_bOutsteer_front=tFalse;
		m_bInsteer_back=tFalse;
		m_bBack_lanefollow=tFalse;
		m_bWait=tFalse;
		m_bFinished=tTrue;
		m_bStart=tFalse;


	}
	
	TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput, 0);
	RETURN_NOERROR;
}
tResult cParkoutleft::stage_park()
{
	LOG_INFO(cString::Format("PARKING"));

	if(m_bFinished)
	{
	LOG_INFO(cString::Format("Step 5 : finished"));
		//stop
		m_fSteeringOutput=0;  
		m_fAccelerationOutput=0;

		m_bStart_step1=tFalse;
		m_bOutsteer_front=tFalse;
		m_bInsteer_back=tFalse;
		m_bBack_lanefollow=tFalse;
		m_bWait=tFalse;
		m_bFinished=tTrue;
		m_bStart=tFalse;
		m_iTrafficSignID= MARKER_ID_NOMATCH;
	}

	else if(m_bWait)
	{

		LOG_INFO(cString::Format("Step 5 : wait in parking"));
		if((_clock->GetStreamTime() - stop_time)/1000000 < 3) // time for wait
		{
			m_fSteeringOutput=0;        
			m_fAccelerationOutput=0; 

			m_bStart_step1=tFalse;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tFalse;
			m_bWait=tTrue;
			m_bFinished=tFalse;

		}
		else
		{

			m_bStart_step1=tFalse;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tFalse;
			m_bWait=tFalse;
			m_bFinished=tTrue;
			m_fDistanceOverall_Start=m_fDistanceOverall;				
		}
	}

	//step4: backside lanefollow
	else if(m_bBack_lanefollow)
	{

		LOG_INFO(cString::Format("Step 4 :backside lanefollow"));
		LOG_INFO(cString::Format("distance coverd = %f & set distance =%f" ,m_fDistanceOverall-m_fDistanceOverall_Start, 0.3));
		if(m_fDistanceOverall-m_fDistanceOverall_Start < 0.3) // distance for backside_lanefollow
		{
			LOG_INFO(cString::Format("if condition"));
			LOG_INFO(cString::Format("SPEED = %f & ",m_fAccelerationOutput));
			m_fSteeringOutput=0;               //accuator output for backside_lanefollow
			m_fAccelerationOutput=10; 

			m_bStart_step1=tFalse;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tTrue;
			m_bWait=tFalse;
			m_bFinished=tFalse;

		}
		else
		{
		LOG_INFO(cString::Format("else condition"));
			m_bStart_step1=tFalse;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tFalse;
			m_bWait=tTrue;
			m_bFinished=tFalse;
			m_fDistanceOverall_Start=m_fDistanceOverall;	
			stop_time = _clock->GetStreamTime();
					
		}
	}

	//step3: moved insteer backside
	else if(m_bInsteer_back)
	{

		LOG_INFO(cString::Format("Step 3 :Insteer backside"));
		if(m_fDistanceOverall-m_fDistanceOverall_Start < DIST_INBACK) // distance for insteer_back
		{
			m_fSteeringOutput=STEER_INBACK;		//accuator output for backside_lanefollow
			m_fAccelerationOutput=SPEED_INBACK;

			m_bStart_step1=tFalse;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tTrue;
			m_bBack_lanefollow=tFalse;
			m_bWait=tFalse;
			m_bFinished=tFalse;
			stop_time = _clock->GetStreamTime();

		}
		else
		{
			if((_clock->GetStreamTime() - stop_time)/1000000 < 1) // time for wait
			{
				m_fSteeringOutput=0;        
				m_fAccelerationOutput=0; 

				m_bStart_step1=tFalse;
				m_bOutsteer_front=tFalse;
				m_bInsteer_back=tTrue;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tFalse;

			}
			else
			{

				m_bStart_step1=tFalse;
				m_bOutsteer_front=tFalse;
				m_bInsteer_back=tFalse;
				m_bBack_lanefollow=tTrue;
				m_bWait=tFalse;
				m_bFinished=tFalse;
				m_fDistanceOverall_Start=m_fDistanceOverall;			
			}


		}
	}
	//step2: moved outsteer frontside
	else if(m_bOutsteer_front)
	{

		LOG_INFO(cString::Format("Step 2:outsteer frontside"));
		if(m_fDistanceOverall-m_fDistanceOverall_Start < DIST_OUTFRONT) // distance for outsteer_front
		{
			m_fSteeringOutput= STEER_OUTFRONT;		//accuator output for backside_lanefollow
			m_fAccelerationOutput=SPEED_OUTFRONT;

			m_bStart_step1=tFalse;
			m_bOutsteer_front=tTrue;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tFalse;
			m_bWait=tFalse;
			m_bFinished=tFalse;
			stop_time = _clock->GetStreamTime();
		}
		else
		{
			if((_clock->GetStreamTime() - stop_time)/1000000 < 1) // time for wait
			{
				m_fSteeringOutput=0;		//accuator output for backside_lanefollow
				m_fAccelerationOutput=0;

				m_bStart_step1=tFalse;
				m_bOutsteer_front=tTrue;
				m_bInsteer_back=tFalse;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tFalse;

		}
			else
			{
				m_bStart_step1=tFalse;
				m_bOutsteer_front=tFalse;
				m_bInsteer_back=tTrue;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tFalse;
				m_fDistanceOverall_Start=m_fDistanceOverall;
			}
		}
	}
	//step1: move to desired distance
	else if(m_bStart_step1)
	{

		LOG_INFO(cString::Format("Step 1:desired distance"));

		LOG_INFO(cString::Format("current dist %f     last dist %f",m_fDistanceOverall, m_fDistanceOverall_Start ));


		if(m_fDistanceOverall-m_fDistanceOverall_Start < DIST_LANEFOLLOW) // distance for first_lanefollow
		{
			m_fSteeringOutput=0;		//accuator output for backside_lanefollow
			m_fAccelerationOutput= SPEED_LANEFOLLOW; 

			m_bStart_step1=tTrue;
			m_bOutsteer_front=tFalse;
			m_bInsteer_back=tFalse;
			m_bBack_lanefollow=tFalse;
			m_bWait=tFalse;
			m_bFinished=tFalse;
			stop_time = _clock->GetStreamTime();
		}
		else
		{

			if((_clock->GetStreamTime() - stop_time)/1000000 < 1) //time for wait
			{
				m_fSteeringOutput=0;		//accuator output for backside_lanefollow
				m_fAccelerationOutput= 0; 

				m_bStart_step1=tTrue;
				m_bOutsteer_front=tFalse;
				m_bInsteer_back=tFalse;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tFalse;
			}
			else
			{

				m_bStart_step1=tFalse;
				m_bOutsteer_front=tTrue;
				m_bInsteer_back=tFalse;
				m_bBack_lanefollow=tFalse;
				m_bWait=tFalse;
				m_bFinished=tFalse;
				m_fDistanceOverall_Start=m_fDistanceOverall;				
			}				
		}
	}
	else
	{
		LOG_INFO(cString::Format("error in Algo or start"));
		m_fDistanceOverall_Start=m_fDistanceOverall;        //measure initial_distance and givestart command
		m_bStart_step1=tTrue;
		m_bOutsteer_front=tFalse;
		m_bInsteer_back=tFalse;
		m_bBack_lanefollow=tFalse;
		m_bWait=tFalse;
		m_bFinished=tFalse;
	}
	RETURN_NOERROR;
}

tResult cParkoutleft::TransmitOutput(tFloat32 speed,tFloat32 steering, tUInt32 timestamp)
{

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	cObjectPtr<IMediaSample> pMediaSamplesteer;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalLeftEnabled;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalRightEnabled;

	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	AllocMediaSample((tVoid**)&pMediaSamplesteer);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalLeftEnabled);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalRightEnabled);

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
/*	
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
	m_oOutputAcceleration.Transmit(pMediaSampleTurnSignalLeftEnabled);

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
	m_oOutputAcceleration.Transmit(pMediaSampleTurnSignalRightEnabled);
*/
	RETURN_NOERROR;
}


