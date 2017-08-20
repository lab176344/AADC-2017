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
#include "cAEB.h"
/// Create filter shell
ADTF_FILTER_PLUGIN("AEB", OID_ADTF_TEMPLATE_FILTER, cAEB);

// Constructor get called at first
// EDIT
cAEB::cAEB(const tChar* __info):cFilter(__info)
{

}

// Deconstructor 
cAEB::~cAEB()
{

}


// Init-Fkt is called after Constructor
// EDIT
tResult cAEB::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

	    cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));

            // media type for Signal Values
            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);



	    // Creation of the first Input of the AEB filter
            // media type for Struct Pins
            tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
            RETURN_IF_POINTER_NULL(strUltrasonicStruct);
            cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));




/*
            //get mediatype description for ultrasonic sensor data type
            RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));

            //get mediatype description for Signal data type
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

            //create pins for ultrasonic sensor data
            RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedControllerIn", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

            RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedControllerOut", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

*/








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




// Shutdown is the reverse function of the Init-Fct
// EDIT
tResult cAEB::Shutdown(tInitStage eStage, __exception)
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




tResult cAEB::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

/*
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_oInputPin)
        {
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
        }
*/
    }

    RETURN_NOERROR;
}
