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
#include "cSituationDetection.h"
//#include "template_data.h"
/// Create filter shell
ADTF_FILTER_PLUGIN("SituationDetection", OID_ADTF_SITUATION_DETECTION, cSituationDetection);

using namespace roadsignIDs;


// Detecting what instance of the filter is running currently
cSituationDetection::cSituationDetection(const tChar* __info):cFilter(__info)
{

}

cSituationDetection::~cSituationDetection()
{

}

tResult cSituationDetection::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
	

	// Registration of the input and output pins
	/*
        // get a media type for the input pin
	cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
	//
        // create and register the video input pin
	RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
	*/

	/*
	// get a media type for the input pin
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the input pin
        RETURN_IF_FAILED(m_oInputPin.Create("input_template", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPin));	

        // get a media type for the output pin
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the output pin
        RETURN_IF_FAILED(m_oOutputPin.Create("output_template", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));

	*/





	cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // Input - Dirverstruct
        tChar const * strInputDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strInputDriverStruct);
        cObjectPtr<IMediaType> pTypeInputDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strInputDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeInputDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputDriverStruct));
        RETURN_IF_FAILED(m_oInputDriverStruct.Create("DriverStruct", pTypeInputDriverStruct, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputDriverStruct));

        // Input - RoadSign
        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

	RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("TrafficSign", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));

        // Input - ParkingFeedback
        tChar const * strParkingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strParkingFinished);
        cObjectPtr<IMediaType> pTypeSignalParkingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strParkingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalParkingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputParkingFinished));
        RETURN_IF_FAILED(m_oInputParkingFinished.Create("Parking Finished", pTypeSignalParkingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputParkingFinished));

        // Input - PullOutLeftFeedback
        tChar const * strPullOutLeftFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strPullOutLeftFinished);
        cObjectPtr<IMediaType> pTypeSignalPullOutLeftFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strPullOutLeftFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutLeftFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputPullOutLeftFinished));
        RETURN_IF_FAILED(m_oInputPullOutLeftFinished.Create("PullOutLeft Finished", pTypeSignalPullOutLeftFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPullOutLeftFinished));

        // Input - PullOutRightFeedback
        tChar const * strPullOutRightFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strPullOutRightFinished);
        cObjectPtr<IMediaType> pTypeSignalPullOutRightFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strPullOutRightFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutRightFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputPullOutRightFinished));
        RETURN_IF_FAILED(m_oInputPullOutRightFinished.Create("PullOutRight Finished", pTypeSignalPullOutRightFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPullOutRightFinished));

        // Input - CrossingFeedback
        tChar const * strCrossingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strCrossingFinished);
        cObjectPtr<IMediaType> pTypeSignalCrossingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strCrossingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalCrossingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputCrossingFinished));
        RETURN_IF_FAILED(m_oInputCrossingFinished.Create("Crossing Finished", pTypeSignalCrossingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputCrossingFinished));



        // Output - JuryStruct
        tChar const * strOutputJuryStruct = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strOutputJuryStruct);
        cObjectPtr<IMediaType> pTypeOutputJuryStruct = new cMediaType(0, 0, 0, "tJuryStruct", strOutputJuryStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputJuryStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputJuryStruct));
        RETURN_IF_FAILED(m_oOutputJuryStruct.Create("JuryStruct", pTypeOutputJuryStruct, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputJuryStruct));



        // Output - LaneFollowerStart
        tChar const * strDescSignalLaneFollowerStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalLaneFollowerStart);
        cObjectPtr<IMediaType> pTypeSignalLaneFollowerStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalLaneFollowerStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalLaneFollowerStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputLaneFollowerStart));
        RETURN_IF_FAILED(m_oOutputLaneFollowerStart.Create("LaneFollower Start", pTypeSignalLaneFollowerStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputLaneFollowerStart));

        // Output - ParkingStart
        tChar const * strDescSignalParkingStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalParkingStart);
        cObjectPtr<IMediaType> pTypeSignalParkingStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalParkingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalParkingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputParkingStart));
        RETURN_IF_FAILED(m_oOutputParkingStart.Create("Parking Start", pTypeSignalParkingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputParkingStart));

        // Output - PullOutLeft
        tChar const * strDescSignalPullOutLeftStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalPullOutLeftStart);
        cObjectPtr<IMediaType> pTypeSignalPullOutLeftStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalPullOutLeftStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutLeftStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputPullOutLeftStart));
        RETURN_IF_FAILED(m_oOutputPullOutLeftStart.Create("PullOutLeft Start", pTypeSignalPullOutLeftStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPullOutLeftStart));

        // Output - PullOutRight
        tChar const * strDescSignalPullOutRightStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalPullOutRightStart);
        cObjectPtr<IMediaType> pTypeSignalPullOutRightStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalPullOutRightStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutRightStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputPullOutRightStart));
        RETURN_IF_FAILED(m_oOutputPullOutRightStart.Create("PullOutRight Start", pTypeSignalPullOutRightStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPullOutRightStart));

        // Output - CrossingStart
        tChar const * strDescSignalCrossingStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalCrossingStart);
        cObjectPtr<IMediaType> pTypeSignalCrossingStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalCrossingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalCrossingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputCrossingStart));
        RETURN_IF_FAILED(m_oOutputCrossingStart.Create("Crossing Start", pTypeSignalCrossingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputCrossingStart));





    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.


        // Inputs
        m_bInputParkingFinished = false;
        m_bInputPullOutLeftFinished = false;
        m_bInputPullOutRightFinished =false;
        m_bInputCrossingFinished =false;


        // Outputs
        m_bOutputParkingStart = false;
        m_bOutputCrossingStart = false;
        m_bOutputPullOutLeftStart = false;
        m_bOutputPullOutRightStart = false;
        m_bOutputLaneFollowerStart = false;

        m_iCurrentTrafficSignID = 0;
        m_iPreviousTrafficSignID = 0;
        m_iRoadSignDetectorCounter = 0;
        m_bManeuverInProcess = false;
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cSituationDetection::Shutdown(tInitStage eStage, __exception)
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



tResult cSituationDetection::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample


        // Driver Struct received
        if(pSource == &m_oInputDriverStruct){
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputParkingFinished->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("i8ActionID", (tVoid*)&i8StateID);
            pCoderInput->Get("i16ManeuverEntry", (tVoid*)&i16ManeuverEntry);
            m_pDescriptionInputParkingFinished->Unlock(pCoderInput);

            LOG_INFO(adtf_util::cString::Format("Struct received"));
            LOG_INFO(adtf_util::cString::Format("i8ActionID",i8StateID));
            LOG_INFO(adtf_util::cString::Format("i16ManeuverEntry",i16ManeuverEntry));

        }

        // Road sign detected
        else if (pSource == &m_oInputTrafficSign)
        {
            LOG_INFO(adtf_util::cString::Format("Traffic Sign"));
            __adtf_sample_read_lock_mediadescription(m_pDescriptionInputTrafficSign,pMediaSample,pCoderInput);

            // During a Maneuver no road sign should be processed
            if(!m_bManeuverInProcess){
                // get IDs
                if(!m_bIDsRoadSignSet)
                {
                        pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
                        pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
                        m_bIDsRoadSignSet = tTrue;
                }
                pCoderInput->Get("i16Identifier", (tVoid*)&m_iCurrentTrafficSignID);
                pCoderInput->Get("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);

                //trafficSignNumb = m_szIDRoadSignI16Identifier;

                // Only process a road sign if the same Road sign was detected 3 times in a row
                //LOG_INFO(adtf_util::cString::Format("current Road sign ID %i",m_iCurrentTrafficSignID));
                //LOG_INFO(adtf_util::cString::Format("Previous Road sign ID %i",m_iPreviousTrafficSignID));
                if(m_iCurrentTrafficSignID == m_iPreviousTrafficSignID){
                    // Same road sign
                    if (m_iRoadSignDetectorCounter < 3){
                        m_iRoadSignDetectorCounter++;
                        //LOG_INFO(adtf_util::cString::Format("Counter++: Counter = %i", m_iRoadSignDetectorCounter));
                    }else {
                        // Process the road sign
                        EvaluateRoadSign();
                        // Road sign is processed -> Reset the counter
                        m_iRoadSignDetectorCounter = 0;
                        //LOG_INFO(adtf_util::cString::Format("Road sign is processed: Counter = %i", m_iRoadSignDetectorCounter));
                    }
                }else {
                    // Not the same road sign anymore -> reset the counter
                    m_iRoadSignDetectorCounter = 0;
                    //LOG_INFO(adtf_util::cString::Format("Different Road sign: Counter = %i", m_iRoadSignDetectorCounter));
                }
                m_iPreviousTrafficSignID = m_iCurrentTrafficSignID;
            }
        }

        // Received Feedpack from Parking filter
        else if(pSource == &m_oInputParkingFinished){
            // Check what is the value
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputParkingFinished->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bInputParkingFinished);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputParkingFinished->Unlock(pCoderInput);

            // positive Feedback = parking is finished
            if(m_bInputParkingFinished){
                LOG_INFO(adtf_util::cString::Format("PARKING FINISHED"));

                // Check the xml-file and see to pull out left or right

                // PullOutLeft
                if(false){
                    // Send start signal to PullOutLeft-filter
                    m_bOutputPullOutLeftStart = true;

                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSamplePullOutLeftStart;
                    AllocMediaSample((tVoid**)&pMediaSamplePullOutLeftStart);

                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerPullOutLeftStart;
                    m_pDescriptionOutputPullOutLeftStart->GetMediaSampleSerializer(&pSerializerPullOutLeftStart);
                    tInt nSizePullOutLeftStart = pSerializerPullOutLeftStart->GetDeserializedSize();
                    pMediaSamplePullOutLeftStart->AllocBuffer(nSizePullOutLeftStart);
                    cObjectPtr<IMediaCoder> pCoderOutputPullOutLeftStart;
                    m_pDescriptionOutputPullOutLeftStart->WriteLock(pMediaSamplePullOutLeftStart, &pCoderOutputPullOutLeftStart);
                    pCoderOutputPullOutLeftStart->Set("bValue", (tVoid*)&(m_bOutputPullOutLeftStart));
                    pCoderOutputPullOutLeftStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescriptionOutputPullOutLeftStart->Unlock(pCoderOutputPullOutLeftStart);
                    pMediaSamplePullOutLeftStart->SetTime(_clock->GetStreamTime());
                    m_oOutputPullOutLeftStart.Transmit(pMediaSamplePullOutLeftStart);
                }


                // PullOutRight
                if(true){
                    // Send start signal to PullOutRight-filter
                    m_bOutputPullOutRightStart = true;

                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSamplePullOutRightStart;
                    AllocMediaSample((tVoid**)&pMediaSamplePullOutRightStart);

                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerPullOutRightStart;
                    m_pDescriptionOutputPullOutRightStart->GetMediaSampleSerializer(&pSerializerPullOutRightStart);
                    tInt nSizePullOutRightStart = pSerializerPullOutRightStart->GetDeserializedSize();
                    pMediaSamplePullOutRightStart->AllocBuffer(nSizePullOutRightStart);
                    cObjectPtr<IMediaCoder> pCoderOutputPullOutRightStart;
                    m_pDescriptionOutputPullOutRightStart->WriteLock(pMediaSamplePullOutRightStart, &pCoderOutputPullOutRightStart);
                    pCoderOutputPullOutRightStart->Set("bValue", (tVoid*)&(m_bOutputPullOutRightStart));
                    pCoderOutputPullOutRightStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescriptionOutputPullOutRightStart->Unlock(pCoderOutputPullOutRightStart);
                    pMediaSamplePullOutRightStart->SetTime(_clock->GetStreamTime());
                    m_oOutputPullOutRightStart.Transmit(pMediaSamplePullOutRightStart);
                }

            }
            m_bInputParkingFinished =false;
        }

        // Received Feedpack from PullOutLeft filter
        else if(pSource == &m_oInputPullOutLeftFinished){
            // Check what is the value
            LOG_INFO(adtf_util::cString::Format("PULLOULEFT FINISHED answered"));
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputPullOutLeftFinished->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bInputPullOutLeftFinished);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputPullOutLeftFinished->Unlock(pCoderInput);

            // positive Feedback = Crossing is finished
            if(m_bInputPullOutLeftFinished){
                LOG_INFO(adtf_util::cString::Format("PULLOUTLEFT FINISHED"));
                // The maneuver is finished so enable the road sign detection again
                m_bManeuverInProcess = false;
                // Send start signal to LaneFollower-filter
                StartLaneFollower();
            }
            m_bInputPullOutLeftFinished = false;
        }

        // Received Feedpack from PullOutRight filter
        else if(pSource == &m_oInputPullOutRightFinished){
            // Check what is the value
            LOG_INFO(adtf_util::cString::Format("PULLOURIGHT FINISHED answered"));
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputPullOutRightFinished->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bInputPullOutRightFinished);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputPullOutRightFinished->Unlock(pCoderInput);
            // positive Feedback = Crossing is finished
            if(m_bInputPullOutRightFinished){
                LOG_INFO(adtf_util::cString::Format("PULLOURIGHT FINISHED"));
                // The maneuver is finished so enable the road sign detection again
                m_bManeuverInProcess = false;
                // Send start signal to LaneFollower-filter
                StartLaneFollower();
            }
            m_bInputPullOutRightFinished = false;
        }

        // Received Feedpack from Crossing filter
        else if(pSource == &m_oInputCrossingFinished){
            // Check what is the value
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputCrossingFinished->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bInputCrossingFinished);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputCrossingFinished->Unlock(pCoderInput);

            // positive Feedback = Crossing is finished
            if(m_bInputCrossingFinished){
                LOG_INFO(adtf_util::cString::Format("CROSSING FINISHED"));
                // The maneuver is finished so enable the road sign detection again
                m_bManeuverInProcess = false;
                // Send start signal to LaneFollower-filter
                StartLaneFollower();
            }
            m_bInputCrossingFinished =false;
        }

        // Template example
        /*
             // Templete data


            // this will store the value for our new sample
            tTemplateData fNewValue;

            // now lets access the data in the sample,
            // the Lock method gives you access to the buffer of the sample.
            // we use a scoped sample lock to ensure that the lock is released in all circumstances.

            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tTemplateData, pData);
                // now we can access the sample data through the pointer
                fNewValue = *pData + 1.0;
                // the read lock on the sample will be released when leaving this scope
            }

            // now we need a new media sample to forward the data.
            cObjectPtr<IMediaSample> pNewSample;
            if (IS_OK(AllocMediaSample(&pNewSample)))
            {
                // now set its data
                // we reuse the timestamp from the incoming media sample. Please see the api documentation
                // (ADTF Extreme Programmers -> The ADTF Streamtime) for further reference on how sample times are handled in ADTF
                pNewSample->Update(pMediaSample->GetTime(), &fNewValue, sizeof(tTemplateData), 0);

                // and now we can transmit it
                m_oOutputPin.Transmit(pNewSample);
            }
         */

    }

    RETURN_NOERROR;
}


// Warning: control reaches end of non-void function [-Wreturn-type]
tResult cSituationDetection::EvaluateRoadSign() {
    LOG_INFO(adtf_util::cString::Format("Traffic Sign# %i",m_iCurrentTrafficSignID));

    // LOG_INFO(adtf_util::cString::Format("m_szIDRoadSignI16Identifier %i",i16Identifier));
    //LOG_INFO(adtf_util::cString::Format("m_szIDRoadSignF32Imagesize %f",m_szIDRoadSignF32Imagesize));
    switch(m_iCurrentTrafficSignID) {
        // MARKER_ID_UNMARKEDINTERSECTION
        case 0 : LOG_INFO(adtf_util::cString::Format("UNMARKEDINTERSECTION"));             // Check if any manouver is required
                            // --> Signal to manouver (bool, ID and direction based on XML-file)
            // Check what is the next order in the xml-file


        break;

        // MARKER_ID_STOPANDGIVEWAY
        case 1 : LOG_INFO(adtf_util::cString::Format("STOPANDGIVEWAY"));
            // Check if any manouver is required
            // --> Signal to manouver (bool, ID and direction based on XML-file)

            // Set this true so no other road sign can be processed until the current maneuver
            // m_bManeuverInProcess = true;
        break;


        // MARKER_ID_PARKINGAREA
        case 2 : LOG_INFO(adtf_util::cString::Format("PARKINAREA"));  // --> Signal to Parking

            // if(XML-file says PARKNING)
            if(true)
            {
                // Set this true so no other road sign can be processed until the current maneuver
                m_bManeuverInProcess = true;

                // Send start signal to parking-filter

                m_bOutputParkingStart = true;

                // Create a new MediaSmaple
                cObjectPtr<IMediaSample> pMediaSampleParkingStart;
                AllocMediaSample((tVoid**)&pMediaSampleParkingStart);


                // Send the Media Sample
                cObjectPtr<IMediaSerializer> pSerializerParkingStart;
                m_pDescriptionOutputParkingStart->GetMediaSampleSerializer(&pSerializerParkingStart);
                tInt nSizeParkingStart = pSerializerParkingStart->GetDeserializedSize();
                pMediaSampleParkingStart->AllocBuffer(nSizeParkingStart);
                cObjectPtr<IMediaCoder> pCoderOutputParkingStart;
                m_pDescriptionOutputParkingStart->WriteLock(pMediaSampleParkingStart, &pCoderOutputParkingStart);
                pCoderOutputParkingStart->Set("bValue", (tVoid*)&(m_bOutputParkingStart));
                pCoderOutputParkingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                m_pDescriptionOutputParkingStart->Unlock(pCoderOutputParkingStart);
                pMediaSampleParkingStart->SetTime(_clock->GetStreamTime());
                m_oOutputParkingStart.Transmit(pMediaSampleParkingStart);
            }

        break;


        // MARKER_ID_HAVEWAY
        case 3 : LOG_INFO(adtf_util::cString::Format("HAVEWAY"));
            // Check if any manouver is required
            // --> Signal to manouver (bool, ID and direction based on XML-file)


            // Set this true so no other road sign can be processed until the current maneuver
            // m_bManeuverInProcess = true;
        break;


        // MARKER_ID_AHEADONLY
        case 4 : LOG_INFO(adtf_util::cString::Format("AHEADONLY"));
            // DO NOTHING (bool, ID and direction based on XML-file)

            // Set this true so no other road sign can be processed until the current maneuver
            // m_bManeuverInProcess = true;
        break;


        // MARKER_ID_GIVEWAY
        case 5 : LOG_INFO(adtf_util::cString::Format("GIVEWAY"));
            // Check if any manouver is required
            // --> Signal to manouver (bool, ID and direction based on XML-file)

            // Set this true so no other road sign can be processed until the current maneuver
            // m_bManeuverInProcess = true;
        break;



        // MARKER_ID_PEDESTRIANCROSSING
        case 6 :   LOG_INFO(adtf_util::cString::Format("PEDESTRIANCROSSING"));
            // Check for predestrians and drive slow
            // --> Bool to zebracrossing

            // Set this true so no other road sign can be processed until the current maneuver
            // m_bManeuverInProcess = true;
        break;


        // MARKER_ID_ROUNDABOUT
        case 7 :   LOG_INFO(adtf_util::cString::Format("ROUNDABOUT"));
            // DO NOTHING
        break;


        // MARKER_ID_NOOVERTAKING
        case 8 :   LOG_INFO(adtf_util::cString::Format("NOOVERTAKING"));
            // DO NOTHING
        break;


        // MARKER_ID_NOENTRYVEHICULARTRAFFIC
        case 9 :   LOG_INFO(adtf_util::cString::Format("NOENTRYVEHICULARTRAFFIC"));
            // DO NOTHING
        break;


        // MARKER_ID_TESTCOURSEA9
        case 10 :  LOG_INFO(adtf_util::cString::Format("TESTCOURSEA9"));
            // DO NOTHING
        break;


        // MARKER_ID_ONEWAYSTREET
        case 11 :   LOG_INFO(adtf_util::cString::Format("ONEWAYSTREET"));
            // DO NOTHING
        break;


        // MARKER_ID_ROADWORKS
        case 12 :    LOG_INFO(adtf_util::cString::Format("ROADWORKS"));
            // Check for obsteckels and drive slow
        break;


        // MARKER_ID_KMH50
        case 13 :  LOG_INFO(adtf_util::cString::Format("KMH50"));
            // Set max valocity at 50 km/h
        break;


        // MARKER_ID_KMH100
        case 14 :  LOG_INFO(adtf_util::cString::Format("KMH100"));
            // Set max valocity at 100 km/h
        break;


        // MARKER_ID_NOMATCH
        case 99 :  LOG_INFO(adtf_util::cString::Format("NOMATCH"));
            // Do nothing
        break;


        RETURN_NOERROR;
    }

}


tResult cSituationDetection::StartLaneFollower(){
    // Send start signal to LaneFollower-filter
    m_bOutputLaneFollowerStart = true;

    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
    AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);

    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
    tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
    pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
    cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
    pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(m_bOutputLaneFollowerStart));
    pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
    pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
    m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);
}



