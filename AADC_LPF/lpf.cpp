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
#include "lpf.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("LPF Filter", OID_ADTF_LPF_FILTER, LPFFilter);


LPFFilter::LPFFilter(const tChar* __info) :cFilter(__info)
{
	//write values with zero
	inputSample = 0;
        for (int i = 0 ; i<1000 ; i++) { buffer[i] = 0; }
        pointer = 0;
	outputSample = 0;

        SetPropertyInt("N", 4);
	SetPropertyBool("N" NSSUBPROP_ISCHANGEABLE, tTrue);
	
}

LPFFilter::~LPFFilter()
{

}

tResult LPFFilter::Init(tInitStage eStage, __exception)
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
		tChar const * strDescSignalValueInput = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueInput);
		cObjectPtr<IMediaType> pTypeSignalValueInput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueInput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));

		// phase 2
		RETURN_IF_FAILED(m_oInputMeas.Create("input_sample", pTypeSignalValueInput, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputMeas));

		// create and register the output pin
		// phase 1
		tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutput);
		cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
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

tResult LPFFilter::Shutdown(tInitStage eStage, __exception)
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

tResult LPFFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
		{
                        tUInt32 timeStamp = 0;
			
				// read-out the incoming Media Sample
				cObjectPtr<IMediaCoder> pCoderInput;
				RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

				//get values from media sample        
				pCoderInput->Get("f32Value", (tVoid*)&inputSample);
				pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
				m_pCoderDescSignalInput->Unlock(pCoderInput);
		

		
                buffer[pointer] = inputSample;
                pointer = (pointer +1) % N;
                outputSample =0;
                for ( int i = 0; i< N; i++) {
                     outputSample+= (buffer[(pointer + i ) % N]/N);
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
	}

	RETURN_NOERROR;
}
