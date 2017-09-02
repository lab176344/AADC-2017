/**
 *
 * ADTF Template Project
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: belkera $
 * $Date: 2011-06-30 16:51:21 +0200 (Do, 30 Jun 2011) $
 * $Revision: 26514 $
 *
 * @remarks
 *
 */
#include "stdafx.h"
#include "lpf_us.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("LPF US Filter", OID_ADTF_LPF_US_FILTER, LPFUSFilter);


LPFUSFilter::LPFUSFilter(const tChar* __info) :cFilter(__info)
{
	//write values with zero
	inputSample = 0;
	for(int s=0; s<10; s++)
	{
		for (int i = 0 ; i<20 ; i++)
		{
			buffer[s][i] = 0;
			outputSample[i]=0;


		}
	}
        pointer = 0;
		timeStamp=0;

        SetPropertyInt("N", 4);
	SetPropertyBool("N" NSSUBPROP_ISCHANGEABLE, tTrue);
	
}

LPFUSFilter::~LPFUSFilter()
{

}

tResult LPFUSFilter::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		// create and register the input pin
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		// phase 1
		tChar const * strDescSignalValueInput = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strDescSignalValueInput);
		cObjectPtr<IMediaType> pTypeSignalValueInput = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescSignalValueInput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));

		// phase 2
		RETURN_IF_FAILED(m_oInputMeas.Create("tUltrasonicStruct", pTypeSignalValueInput, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputMeas));

		// create and register the output pin
		// phase 1
		tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutput);
		cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescSignalValueOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));
		// phase 2
		RETURN_IF_FAILED(m_oOutputFilter.Create("output_filter", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputFilter));
	}
	else if (eStage == StageNormal)
	{
		
                N = GetPropertyInt("N");
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

tResult LPFUSFilter::Shutdown(tInitStage eStage, __exception)
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

tResult LPFUSFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
		{
                        tUInt32 timeStamp = 0;

				// read input
				ProcessInputUS(pMediaSample);
                
				for(int s=0;s<10; s++)
				{
					for ( int i = 0; i< N; i++)
					{
						outputSample[s]+= (buffer[s][(pointer + i ) % N]/N);
					}
				}

				TransmitOutput();
		}

		//create new media sample
		cObjectPtr<IMediaSample> pMediaSampleOUT;
		AllocMediaSample((tVoid**)&pMediaSampleOUT);
		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		pMediaSampleOUT->AllocBuffer(nSize);
		//write date to the media sample with the coder of the descriptor
		cObjectPtr<IMediaCoder> pCoderOutput;
		m_pCoderDescSignalOutput->WriteLock(pMediaSampleOUT, &pCoderOutput);
		// ...
		pCoderOutput->Set("f32Value", (tVoid*)&(outputSample));
		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
		m_pCoderDescSignalOutput->Unlock(pCoderOutput);
		//transmit media sample over output pin
		pMediaSampleOUT->SetTime(_clock->GetStreamTime());
		m_oOutputFilter.Transmit(pMediaSampleOUT);

		}

	RETURN_NOERROR;
}


tResult LPFUSFilter::ProcessInputUS(IMediaSample* pMediaSample)
{
	// use mutex to access min US value
	__synchronized_obj(m_critSec);

    //read out incoming Media Samples
    __adtf_sample_read_lock_mediadescription(m_pCoderDescSignalInput, pMediaSample, pCoderInput);

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
			buffer[i][pointer]=buf_UsSignal[i];
		}
	}
	pointer = (pointer +1) % N;

	//LOG_INFO(cString::Format("front left %f, frontcenterleft %f", m_oUSFrontLeft, m_oUSFrontCenterLeft));
	//LOG_INFO(cString::Format("front right %f, frontcenterright %f", m_oUSFrontRight, m_oUSFrontCenterRight));
	//LOG_INFO(cString::Format("ARRAY center %f   frontcenterleft %f  frontcenterright %f", m_aUSSensors[US_FRONTCENTER], m_aUSSensors[US_FRONTCENTERLEFT], m_aUSSensors[US_FRONTCENTERRIGHT]));

	RETURN_NOERROR;
}


tResult LPFUSFilter::TransmitOutput()
{
	timeStamp=_clock->GetStreamTime();

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleOUT;
	AllocMediaSample((tVoid**)&pMediaSampleOUT);
	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	pMediaSampleOUT->AllocBuffer(nSize);
	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoderOutput;
	m_pCoderDescSignalOutput->WriteLock(pMediaSampleOUT, &pCoderOutput);
	// ...
	pCoderOutput->Set("tFrontLeft.f32Value", (tVoid*)&(outputSample[0]));
	pCoderOutput->Set("tFrontLeft.ui32ArduinoTimestamp", (tVoid*)&timeStamp);

	pCoderOutput->Set("tFrontCenterLeft.f32Value", (tVoid*)&(outputSample[1]));
	pCoderOutput->Set("tFrontLeft.ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalOutput->Unlock(pCoderOutput);
	//transmit media sample over output pin
	pMediaSampleOUT->SetTime(_clock->GetStreamTime());
	m_oOutputFilter.Transmit(pMediaSampleOUT);

	RETURN_NOERROR;
}