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
#include "lpfus.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("LPF US Filter", OID_ADTF_LPF_US_FILTER, LPFUSFilter);


LPFUSFilter::LPFUSFilter(const tChar* __info) :cFilter(__info)
{
        SetPropertyInt("N Average Number", 20);
        //SetPropertyBool("N Average Number" NSSUBPROP_ISCHANGEABLE, tTrue);


        for(tInt i = 0; i<19; i++){
            f32FrontLeft[i]=0;
            tsFrontLeft[i]=0;
            f32FrontCenterLeft[i]=0;
            tsFrontCenterLeft[i]=0;
            f32FrontCenter[i]=0;
            tsFrontCenter[i]=0;
            f32FrontCenterRight[i]=0;
            tsFrontCenterRight[i]=0;
            f32FrontRight[i]=0;
             tsFrontRight[i]=0;


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
            /*
                for(tInt i = 0; i<19; i++){
                    f32FrontLeft[i]=0;
                    tsFrontLeft[i]=0;
                    f32FrontCenterLeft[i]=0;
                    tsFrontCenterLeft[i]=0;
                    f32FrontCenter[i]=0;
                    tsFrontCenter[i]=0;
                    f32FrontCenterRight[i]=0;
                    tsFrontCenterRight[i]=0;
                    f32FrontRight[i]=0;
                     tsFrontRight[i]=0;
                }
                */
                //iAverageNumber = GetPropertyInt("N Average Number");
                // In this stage you would do further initialisation and/or create your dynamic pins.
		// Please take a look at the demo_dynamicpin example for further reference.
	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
		m_szIdsUsStructSet=tFalse;
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
                        //tUInt32 timeStamp = 0;

                        // read input
                        ProcessInputUS(pMediaSample);

                        // Set all values one back so always the last 20 values are processed
                        for(tInt i = 19; i<0;i--){
                                    f32FrontLeft[i]         = f32FrontLeft[i-1];
                                    tsFrontLeft[i]          = tsFrontLeft[i-1];
                                    f32FrontCenterLeft[i]   = f32FrontCenterLeft[i-1];
                                    tsFrontCenterLeft[i]    = tsFrontCenterLeft[i-1];
                                    f32FrontCenter[i]       = f32FrontCenter[i-1];
                                    tsFrontCenter[i]        = tsFrontCenter[i-1];
                                    f32FrontCenterRight[i]  = f32FrontCenterRight[i-1];
                                    tsFrontCenterRight[i]   = tsFrontCenterRight[i-1];
                                    f32FrontRight[i]        = f32FrontRight[i-1];
                                    tsFrontRight[i]         = tsFrontRight[i-1];
                        }
                                // Store the current values
                                f32FrontLeft[0]        = m_szIdUsStructValues[0];
                                tsFrontLeft[0]         = m_szIdUsStructTss[0];
                                f32FrontCenterLeft[0]  = m_szIdUsStructValues[1];
                                tsFrontCenterLeft[0]   = m_szIdUsStructTss[1];
                                f32FrontCenter[0]      = m_szIdUsStructValues[2];
                                tsFrontCenter[0]       = m_szIdUsStructTss[2];
                                f32FrontCenterRight[0] = m_szIdUsStructValues[3];
                                tsFrontCenterRight[0]  = m_szIdUsStructTss[3];
                                f32FrontRight[0]       = m_szIdUsStructValues[4];
                                tsFrontRight[0]        = m_szIdUsStructTss[5];

                               // LOG_INFO(cString::Format("front left %f", m_szIdUsStructValues[0]));

                                tFloat32 f32MinFrontLeft = 400;
                                tTimeStamp tsMinFrontLeft = 0;
                                tFloat32 f32MinFrontCenterLeft = 400;
                                tTimeStamp tsMinFrontCenterLeft = 0;
                                tFloat32 f32MinFrontCenter = 400;
                                tTimeStamp tsMinFrontCenter = 0;
                                tFloat32 f32MinFrontCenterRight = 400;
                                tTimeStamp tsMinFrontCenterRight = 0;
                                tFloat32 f32MinFrontRight = 400;
                                tTimeStamp tsMinFrontRight = 0;
                                for (tInt i=0;i<20;i++)
                                {
                                   // LEFT          Wrong Us-values are  0 or -1
                                   if (f32FrontLeft[i]<=f32MinFrontLeft && f32FrontLeft[i]>0)
                                   {
                                       f32MinFrontLeft = f32FrontLeft[i];
                                       tsMinFrontLeft = tsFrontLeft[i];
                                    }
                                   // CENTER-LEFT   Wrong Us-values are  0 or -1
                                   if (f32FrontCenterLeft[i]<=f32MinFrontCenterLeft && f32FrontCenterLeft[i]>0)
                                   {
                                       f32MinFrontCenterLeft = f32FrontCenterLeft[i];
                                       tsMinFrontCenterLeft = tsFrontCenterLeft[i];
                                    }
                                   // CENTER        Wrong Us-values are  0 or -1
                                   if (f32FrontCenter[i]<=f32MinFrontCenter && f32FrontCenter[i]>0)
                                   {
                                       f32MinFrontCenter = f32FrontCenter[i];
                                       tsMinFrontCenter = tsFrontCenter[i];
                                    }
                                   // CENTER-RIGHT  Wrong Us-values are  0 or -1
                                   if (f32FrontCenterRight[i]<=f32MinFrontCenterRight && f32FrontCenterRight[i]>0)
                                   {
                                       f32MinFrontCenterRight = f32FrontCenterRight[i];
                                       tsMinFrontCenterRight = tsFrontCenterRight[i];
                                    }
                                   // RIGHT         Wrong Us-values are  0 or -1
                                   if (f32FrontRight[i]<=f32MinFrontRight && f32FrontRight[i]>0)
                                   {
                                       f32MinFrontRight = f32FrontRight[i];
                                       tsMinFrontRight = tsFrontRight[i];
                                    }
                                }



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

                                //LOG_INFO(cString::Format("out %f", outputSample[0]));

                                pCoderOutput->Set("tFrontLeft.f32Value", (tVoid*)&(f32MinFrontLeft));
                                pCoderOutput->Set("tFrontLeft.ui32ArduinoTimestamp", (tVoid*)&tsMinFrontLeft);

                                pCoderOutput->Set("tFrontCenterLeft.f32Value", (tVoid*)&(f32MinFrontCenterLeft));
                                pCoderOutput->Set("tFrontLeft.ui32ArduinoTimestamp", (tVoid*)&tsMinFrontCenterLeft);

                                pCoderOutput->Set("tFrontCenter.f32Value", (tVoid*)&(f32MinFrontCenter));
                                pCoderOutput->Set("tFrontCenter.ui32ArduinoTimestamp", (tVoid*)&tsMinFrontCenter);

                                pCoderOutput->Set("tFrontCenterRight.f32Value", (tVoid*)&(f32MinFrontCenterRight));
                                pCoderOutput->Set("tFrontCenterRight.ui32ArduinoTimestamp", (tVoid*)&tsMinFrontCenterRight);

                                pCoderOutput->Set("tFrontRight.f32Value", (tVoid*)&(f32MinFrontRight));
                                pCoderOutput->Set("tFrontRight.ui32ArduinoTimestamp", (tVoid*)&tsMinFrontRight);
                                /*
                                pCoderOutput->Set("tSideLeft.f32Value", (tVoid*)&(outputSample[5]));
                                pCoderOutput->Set("tSideLeft.ui32ArduinoTimestamp", (tVoid*)&timeStamp);

                                pCoderOutput->Set("tSideRight.f32Value", (tVoid*)&(outputSample[6]));
                                pCoderOutput->Set("tSideRight.ui32ArduinoTimestamp", (tVoid*)&timeStamp);

                                pCoderOutput->Set("tRearLeft.f32Value", (tVoid*)&(outputSample[7]));
                                pCoderOutput->Set("tRearLeft.ui32ArduinoTimestamp", (tVoid*)&timeStamp);

                                pCoderOutput->Set("tRearCenter.f32Value", (tVoid*)&(outputSample[8]));
                                pCoderOutput->Set("tRearCenter.ui32ArduinoTimestamp", (tVoid*)&timeStamp);

                                pCoderOutput->Set("tRearRight.f32Value", (tVoid*)&(outputSample[9]));
                                pCoderOutput->Set("tRearRight.ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                                */

                                m_pCoderDescSignalOutput->Unlock(pCoderOutput);

                                //transmit media sample over output pin
                                pMediaSampleOUT->SetTime(_clock->GetStreamTime());
                                m_oOutputFilter.Transmit(pMediaSampleOUT);








                }
/*
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
*/
		}

	RETURN_NOERROR;
}


tResult LPFUSFilter::ProcessInputUS(IMediaSample* pMediaSample)
{
	// use mutex to access min US value
	__synchronized_obj(m_critSec);

    //read out incoming Media Samples
    __adtf_sample_read_lock_mediadescription(m_pCoderDescSignalInput, pMediaSample, pCoderInput);

    if(true)
    {
        tBufferID idValue, idTimestamp;
        m_szIdUsStructValues.clear();
        m_szIdUsStructTss.clear();
		
        pCoderInput->GetID("tFrontLeft.f32Value", idValue);
	pCoderInput->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
	m_szIdUsStructValues.push_back(idValue);
	m_szIdUsStructTss.push_back(idTimestamp);

        LOG_INFO(cString::Format("idValue: front left %f", idValue));
        LOG_INFO(cString::Format("m_szIdUsStructValues[0] front left %f", m_szIdUsStructValues[0]));
        tFloat32 buf_UsSignal[0];
        pCoderInput->Get(m_szIdUsStructValues[0],(tVoid*)&buf_UsSignal[0]);
        tFloat32 buffer[0] = buf_UsSignal[0];
        LOG_INFO(cString::Format("buffer[0]: front left %f", buffer[0]));

	pCoderInput->GetID("tFrontCenterLeft.f32Value", idValue);
	pCoderInput->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
	m_szIdUsStructValues.push_back(idValue);
	m_szIdUsStructTss.push_back(idTimestamp);
        LOG_INFO(cString::Format("idValue: front center %f", idValue));
        LOG_INFO(cString::Format("m_szIdUsStructValues[0] front center %f", m_szIdUsStructValues[0]));

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


        LOG_INFO(cString::Format("m_szIdUsStructValues [4]: %f", m_szIdUsStructValues[4]));

	// m_szIdsUsStructSet = tTrue;
	m_szIdsUsStructSet = tTrue;

        }
	//LOG_INFO(cString::Format("pointer %i", pointer));

	//LOG_INFO(cString::Format("front left %f, frontcenterleft %f", m_oUSFrontLeft, m_oUSFrontCenterLeft));
	//LOG_INFO(cString::Format("front right %f, frontcenterright %f", m_oUSFrontRight, m_oUSFrontCenterRight));
	//LOG_INFO(cString::Format("ARRAY center %f   frontcenterleft %f  frontcenterright %f", m_aUSSensors[US_FRONTCENTER], m_aUSSensors[US_FRONTCENTERLEFT], m_aUSSensors[US_FRONTCENTERRIGHT]));

        RETURN_NOERROR;
}
