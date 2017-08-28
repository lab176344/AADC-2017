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
#include "cACC.h"
/// Create filter shell


#define MAX_DIST		"cACC::max_dist"
#define MAX_DIST_CURVE	"cACC::max_dist_curve"
#define STEERING_SWITCH_US_FOCUS	"cACC::steering_switch_focus"

ADTF_FILTER_PLUGIN("ACC", OID_ADTF_ACC_FILTER, cACC);

using namespace SensorDefinition;

cACC::cACC(const tChar* __info):cFilter(__info)
{
	SetPropertyFloat(MAX_DIST,120);
    SetPropertyBool(MAX_DIST NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(MAX_DIST NSSUBPROP_DESCRIPTION, "max dist to go straight");

	SetPropertyFloat(MAX_DIST_CURVE,90);
    SetPropertyBool(MAX_DIST_CURVE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(MAX_DIST_CURVE NSSUBPROP_DESCRIPTION, "max dist in curves");

	SetPropertyFloat(STEERING_SWITCH_US_FOCUS,60);
    SetPropertyBool(STEERING_SWITCH_US_FOCUS NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEERING_SWITCH_US_FOCUS NSSUBPROP_DESCRIPTION, "at that steering switch the US focus");

}

cACC::~cACC()
{

}

tResult cACC::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
		// create description manager
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		
		// creeate pin for start signal input
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
		RETURN_IF_FAILED(pTypeSignalSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSpeed));
		RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed", pTypeSignalSpeed, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

		// create pin for steering input
		tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalSteering);
		cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSteering));
		RETURN_IF_FAILED(m_oInputSteering.Create("Steering", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSteering));

		// create pin for ultrasonic struct input
		tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strUltrasonicStruct);
		cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
		RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

		//create pin for steering signal output
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
        RETURN_IF_FAILED(m_oOutputSteering.Create("SteeringOutput", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
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

		//create pin for Overtake output
		tChar const * strDescSignalOvertake = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
		RETURN_IF_POINTER_NULL(strDescSignalOvertake);
		cObjectPtr<IMediaType> pTypeSignalOvertake = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalOvertake, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalOvertake->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOvertake));
		RETURN_IF_FAILED(m_oOutputOvertake.Create("Overtake", pTypeSignalOvertake, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputOvertake));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // init all member variables
		m_bStart=tFalse;
		m_szIdsUsStructSet=tFalse;

		m_bFlagTimeOvertake=tFalse;

		for(int i=0;i<=10;i++)
		{
			m_aUSSensors[i]=400;
		}

		// init output
		m_bOvertake=tFalse;
    }

    RETURN_NOERROR;
}

tResult cACC::Shutdown(tInitStage eStage, __exception)
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


tResult cACC::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}
		
tResult cACC::ReadProperties(const tChar* strPropertyName)
{
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MAX_DIST))
	{
		m_fMaxDist = static_cast<tFloat32> (GetPropertyFloat(MAX_DIST));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MAX_DIST_CURVE))
	{
		m_fMaxDistCurve = static_cast<tFloat32> (GetPropertyFloat(MAX_DIST_CURVE));
	}
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEERING_SWITCH_US_FOCUS))
	{
		m_fSteeringToSwitchFocus = static_cast<tFloat32> (GetPropertyFloat(STEERING_SWITCH_US_FOCUS));
	}
	RETURN_NOERROR;
}

tResult cACC::OnPinEvent(IPin* pSource,
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

		// Input signal at Start
		if (pSource == &m_oStart)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescStart->Unlock(pCoderInput);
		}
		else if(pSource == &m_oInputUsStruct)
        {
            ProcessInputUS(pMediaSample);
        }

		else if(pSource == &m_oInputSpeedController)
        {
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescSpeed->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fSpeedInput);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescSpeed->Unlock(pCoderInput);
        }
		else if (pSource == &m_oInputSteering)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescSteering->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fSteeringInput);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescSteering->Unlock(pCoderInput);

			// stop signal
			if(!m_bStart)
			{
				m_fSteeringOutput=0;
				m_fAccelerationOutput=0;
				m_bOvertake=tFalse;
			}
			else
			{
				m_fSteeringOutput=m_fSteeringInput;
				CalculateSpeed();
			}
			// Transmit Output
			TransmitOutput();
		}

    }

    RETURN_NOERROR;
}


tResult cACC::ProcessInputUS(IMediaSample* pMediaSample)
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

        pCoderInput->GetID("tSideLeft.f32Value", idValue);
        pCoderInput->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tSideRight.f32Value", idValue);
        pCoderInput->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
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
	
	// iterate through all values and find the minimum
	tFloat32 buf_UsSignal [10];
	for (int i=0;i<(int)m_szIdUsStructValues.size();++i)
	{
		pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buf_UsSignal);
		// Wrong Us-values are  0 or -1
		if (buf_UsSignal>0)
		{
			pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buf_UsSignal[i]);
			m_aUSSensors[i]=buf_UsSignal[i];
		}
	}
	/*
	m_oUSFrontLeft=buf_UsSignal[US_FRONTLEFT];
	m_oUSFrontCenterLeft=buf_UsSignal[US_FRONTCENTERLEFT];
	m_oUSFrontCenter=buf_UsSignal[US_FRONTCENTER];
	m_oUSFrontCenterRight=buf_UsSignal[US_FRONTCENTERRIGHT];
	m_oUSFrontRight=buf_UsSignal[US_FRONTRIGHT];
	*/
	//LOG_INFO(cString::Format("front left %f, frontcenterleft %f", m_oUSFrontLeft, m_oUSFrontCenterLeft));
	//LOG_INFO(cString::Format("front right %f, frontcenterright %f", m_oUSFrontRight, m_oUSFrontCenterRight));
	//LOG_INFO(cString::Format("ARRAY center %f   frontcenterleft %f  frontcenterright %f", m_aUSSensors[US_FRONTCENTER], m_aUSSensors[US_FRONTCENTERLEFT], m_aUSSensors[US_FRONTCENTERRIGHT]));

	RETURN_NOERROR;
}


tResult cACC::CalculateSpeed()
{

	// use mutex to access min US value
	__synchronized_obj(m_critSecMinimumUsValue);

	tFloat32 fAverageDist;

	// if sterring is bigger then a propertie, then focus on left or right US sensors
	if((m_fSteeringInput > m_fSteeringToSwitchFocus) || ((-1)*m_fSteeringInput > m_fSteeringToSwitchFocus) )
	{
		// focus on right US sensors
		if(m_fSteeringInput>0)
		{
			fAverageDist=(m_aUSSensors[US_FRONTCENTER] + 2 * m_aUSSensors[US_FRONTCENTERRIGHT] + m_aUSSensors[US_FRONTRIGHT])/4;
			// calculate ouput speed linear
			m_fAccelerationOutput= fAverageDist/m_fMaxDistCurve * m_fSpeedInput;
		}
		// focus on left US sensors
		else
		{
			fAverageDist=(m_aUSSensors[US_FRONTCENTER] + 2 * m_aUSSensors[US_FRONTCENTERLEFT] +  m_aUSSensors[US_FRONTLEFT])/4;
			// calculate ouput speed linear
			m_fAccelerationOutput= fAverageDist/m_fMaxDistCurve * m_fSpeedInput;
		}

		// // calculate ouput speed linear for curves
		if(fAverageDist < m_fMaxDistCurve)
		{
			m_fAccelerationOutput= fAverageDist/m_fMaxDist * m_fSpeedInput;
		}
		else
		{
			m_fAccelerationOutput=m_fSpeedInput;
		}
	}
	// focus on front US sensors
	else
	{
		fAverageDist=(4 * m_aUSSensors[US_FRONTCENTER] + m_aUSSensors[US_FRONTCENTERLEFT] + m_aUSSensors[US_FRONTCENTERRIGHT])/6;

		// calculate ouput speed linear
		if(fAverageDist < m_fMaxDist)
		{
			m_fAccelerationOutput= fAverageDist/m_fMaxDist * m_fSpeedInput;
		}
		else
		{
			m_fAccelerationOutput=m_fSpeedInput;
		}
	}
	// LOG_INFO(cString::Format("SPEED center %f  center left %f   center rigth %f", m_aUSSensors[US_FRONTCENTER], m_aUSSensors[US_FRONTCENTERLEFT], m_aUSSensors[US_FRONTCENTERRIGHT]));
	// LOG_INFO(cString::Format("SPEED fAverageDist %f", fAverageDist));
	// LOG_INFO(cString::Format(" SPEED output speed %f", m_fAccelerationOutput));

	// if speed is to low --> overtake
	if(m_fAccelerationOutput/m_fSpeedInput < 0.2)
	{
		if(!m_bFlagTimeOvertake)
		{
			timestampOvertake=_clock->GetStreamTime();
			m_bFlagTimeOvertake=tTrue;
		}
		else
		{
			if((_clock->GetStreamTime()-timestampOvertake)/1000000 > 3)
			{
				m_bStart=tFalse;
				m_bFlagTimeOvertake=tFalse;
				m_bOvertake=tTrue;
				TransmitOutputOvertake();
			}
		}
	}
	
	RETURN_NOERROR;
	
}



tResult cACC::TransmitOutput()
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



	RETURN_NOERROR;
}


tResult cACC::TransmitOutputOvertake()
{

	cObjectPtr<IMediaSample> pMediaSampleOvertake;
	AllocMediaSample((tVoid**)&pMediaSampleOvertake);

	// Overtake
	cObjectPtr<IMediaSerializer> pSerializerOvertake;
	m_pDescriptionOvertake->GetMediaSampleSerializer(&pSerializerOvertake);
	tInt nSizeOvertake = pSerializerOvertake->GetDeserializedSize();
	pMediaSampleOvertake->AllocBuffer(nSizeOvertake);
	cObjectPtr<IMediaCoder> pCoderOutputOvertake;
	m_pDescriptionOvertake->WriteLock(pMediaSampleOvertake, &pCoderOutputOvertake);
	pCoderOutputOvertake->Set("bValue", (tVoid*)&(m_bOvertake));
	pCoderOutputOvertake->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOvertake->Unlock(pCoderOutputOvertake);
	pMediaSampleOvertake->SetTime(_clock->GetStreamTime());
	m_oOutputOvertake.Transmit(pMediaSampleOvertake);

	RETURN_NOERROR;
}







/*
tResult cACC::CalculateSpeed()
{
	// if sterring is bigger then a propertie, then focus on left or right US sensors
	if((m_fSteeringInput > m_fSteeringToSwitchFocus) || ((-1)*m_fSteeringInput > m_fSteeringToSwitchFocus) )
	{
		// focus on right US sensors
		if(m_fSteeringInput>0)
		{
			tFloat32 fAverageDist=(m_oUSFrontCenter + 2 * m_oUSFrontCenterRight + m_oUSFrontRight)/4;
			// calculate ouput speed linear
			m_fAccelerationOutput= fAverageDist/m_fMaxDistCurve * m_fSpeedInput;
		}
		// focus on left US sensors
		else
		{
			tFloat32 fAverageDist=(m_oUSFrontCenter + 2 * m_oUSFrontCenterLeft + m_oUSFrontLeft)/4;
			// calculate ouput speed linear
			m_fAccelerationOutput= fAverageDist/m_fMaxDistCurve * m_fSpeedInput;
		}
	}
	// focus on front US sensors
	else
	{
		tFloat32 fAverageDist=(2 * m_oUSFrontCenter + m_oUSFrontCenterLeft + m_oUSFrontCenterRight)/4;

		LOG_INFO(cString::Format("SPEED center %f  center left %f   center rigth %f", m_oUSFrontCenter, m_oUSFrontCenterLeft, m_oUSFrontCenterRight));
		LOG_INFO(cString::Format("SPEED fAverageDist %f", fAverageDist));
		// calculate ouput speed linear
		m_fAccelerationOutput= fAverageDist/m_fMaxDist * m_fSpeedInput;
		LOG_INFO(cString::Format(" SPEED output speed %f", m_fAccelerationOutput));
	}
	RETURN_NOERROR;
	
}
*/