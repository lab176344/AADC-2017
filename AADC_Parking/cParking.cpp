 /**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cParking.h"

#define DISTANCE_LANE_FOLLOW 			"cParking::DIST_LANEFOLLOW"
#define SPEED_LANE_FOLLOW 			"cParking::SPEED_LANEFOLLOW"

#define DISTANCE_OUT_FRONT			"cParking::DIST_OUTFRONT"
#define STEER_OUT_FRONT				"cParking::STEER_OUTFRONT"
#define SPEED_OUT_FRONT				"cParking::SPEED_OUTFRONT"

#define DISTANCE_IN_BACK			"cParking::DIST_INBACK"
#define STEER_IN_BACK				"cParking::STEER_INBACK"
#define SPEED_IN_BACK				"cParking::SPEED_INBACK"

#define DISTANCE_LANEFOLLOW_BACK		"cParking::DIST_LANEFOLLOWBACK"
#define SPEED_LANEFOLLOW_BACK			"cParking::SPEED_LANEFOLLOWBACK"

#define DISTANCE_SCAN_SLOT_LENGTH 			"cParking::DIST_SCAN_SLOTLENGHT"
#define DISTANCE_SCAN_SLOT_DEPTH 			"cParking::DIST_SCAN_SLOTDEPTH"
#define DISTANCE_SCAN_SPACE 			"cParking::DIST_SCANSPACE"
#define KPSTEERING				"cParking::kpsteering"


/// Create filter shell
ADTF_FILTER_PLUGIN("Parking", OID_ADTF_PARKING, cParking);

using namespace roadsignIDs;

cParking::cParking(const tChar* __info):cFilter(__info)
{
// set properties for Dibug
    // m_bDebugModeEnabled = tTrue;
    SetPropertyBool("Debug Output to Console",false);
// set properties for lanefollow front
        SetPropertyFloat(DISTANCE_LANE_FOLLOW,1.5);
    SetPropertyBool(DISTANCE_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the distance for the lanefollow");

        SetPropertyFloat(SPEED_LANE_FOLLOW,0.35);
    SetPropertyBool(SPEED_LANE_FOLLOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_LANE_FOLLOW NSSUBPROP_DESCRIPTION, "the speed for the lanefollow");

// set properties for outsteer front
        SetPropertyFloat(DISTANCE_OUT_FRONT,0.7);
    SetPropertyBool(DISTANCE_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_OUT_FRONT NSSUBPROP_DESCRIPTION, "the distance for the steer outside");

        SetPropertyFloat(STEER_OUT_FRONT,-95);
    SetPropertyBool(STEER_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_OUT_FRONT NSSUBPROP_DESCRIPTION, "the steer for the steer outside");

        SetPropertyFloat(SPEED_OUT_FRONT,0.4);
    SetPropertyBool(SPEED_OUT_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_OUT_FRONT NSSUBPROP_DESCRIPTION, "the speed for the steer outside");

// set properties for insteer back
        SetPropertyFloat(DISTANCE_IN_BACK,0.45);
    SetPropertyBool(DISTANCE_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_IN_BACK NSSUBPROP_DESCRIPTION, "the distance for the steer outside");

        SetPropertyFloat(STEER_IN_BACK,80);
    SetPropertyBool(STEER_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_IN_BACK NSSUBPROP_DESCRIPTION, "the steer for the steer outside");

        SetPropertyFloat(SPEED_IN_BACK,-0.3);
    SetPropertyBool(SPEED_IN_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_IN_BACK NSSUBPROP_DESCRIPTION, "the speed for the steer outside");

// set property for lane follow back
        SetPropertyFloat(DISTANCE_LANEFOLLOW_BACK,0.6);
    SetPropertyBool(DISTANCE_LANEFOLLOW_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_LANEFOLLOW_BACK NSSUBPROP_DESCRIPTION, "the distance for the Lanefollow back");

        SetPropertyFloat(SPEED_LANEFOLLOW_BACK,-0.3);
    SetPropertyBool(SPEED_LANEFOLLOW_BACK NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_LANEFOLLOW_BACK NSSUBPROP_DESCRIPTION, "the speed for the Lanefollow back");

// set property for distance for scanning slot lenght
        SetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH,0.47);
    SetPropertyBool(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_DESCRIPTION, "the distance for One slot Area");

// set property for distance for scanning slot depth
        SetPropertyFloat(DISTANCE_SCAN_SLOT_DEPTH,0.75);
    SetPropertyBool(DISTANCE_SCAN_SLOT_DEPTH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SLOT_DEPTH NSSUBPROP_DESCRIPTION, "the distance for One slot Area");

// set property for distance for scanning desired space for parking
        SetPropertyFloat(DISTANCE_SCAN_SPACE,0.4);
    SetPropertyBool(DISTANCE_SCAN_SPACE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SPACE NSSUBPROP_DESCRIPTION, "the distance for desired space for parking");

    //set property for back sterring controller value
        SetPropertyFloat(KPSTEERING,15);
    SetPropertyBool(KPSTEERING NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(KPSTEERING NSSUBPROP_DESCRIPTION, "Kp value for steering control until stop line");
}



cParking::~cParking()
{

}

tResult cParking::Init(tInitStage eStage, __exception)
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


//create pin for start pin input
                tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalstart);
                cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
                RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oStart));

//create pin for Distance over all input
                tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
                cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
                RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));

//create pin for lanefollower steering
                tChar const * strDescSignalLane_steer = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalLane_steer);
                cObjectPtr<IMediaType> pTypeSignalLane_steer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalLane_steer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalLane_steer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescLane_steer));
                RETURN_IF_FAILED(m_oLane_steer.Create("Lane_steer", pTypeSignalLane_steer, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oLane_steer));

// Input - US Struct
                tChar const * strUSStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
                RETURN_IF_POINTER_NULL(strUSStruct);
                cObjectPtr<IMediaType> pTypeUSStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUSStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeUSStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputUSStruct));
                RETURN_IF_FAILED(m_oInputUSStruct.Create("US Struct", pTypeUSStruct, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputUSStruct));

//create pin for Yaw input
                tChar const * strDescSignalYaw = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalYaw);
                cObjectPtr<IMediaType> pTypeSignalYaw = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalYaw, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalYaw->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescYaw));
                RETURN_IF_FAILED(m_oYaw.Create("Yaw", pTypeSignalYaw, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oYaw));


// create pin for Infos about Edges
//                tChar const * strDescEdges = pDescManager->GetMediaDescription("tEdgeStruct");
//                RETURN_IF_POINTER_NULL(strDescEdges);
//                cObjectPtr<IMediaType> pTypeSignalEdges = new cMediaType(0, 0, 0, "tEdgeStruct", strDescEdges, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
//                RETURN_IF_FAILED(pTypeSignalEdges->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescEdges));
//                RETURN_IF_FAILED(m_oEdges.Create("Edges", pTypeSignalEdges, static_cast<IPinEventSink*> (this)));
//                RETURN_IF_FAILED(RegisterPin(&m_oEdges));
                tChar const * strDescEdges = pDescManager->GetMediaDescription("tStoplineStruct");
                RETURN_IF_POINTER_NULL(strDescEdges);
                cObjectPtr<IMediaType> pTypeSignalEdges = new cMediaType(0, 0, 0, "tStoplineStruct", strDescEdges, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalEdges->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescEdges));
                RETURN_IF_FAILED(m_oEdges.Create("Edges", pTypeSignalEdges, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oEdges));

// Output - Parking_Finish
                tChar const * strDescSignalParking_Finish = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalParking_Finish);
                cObjectPtr<IMediaType> pTypeSignalParking_Finish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalParking_Finish, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalParking_Finish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputParking_Finish));
                RETURN_IF_FAILED(m_oOutputParking_Finish.Create("Parking Finish", pTypeSignalParking_Finish, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputParking_Finish));

//create pin for steering signal output
                tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSteering);
                cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
                RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

//create pin for speed signal output
                tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
                cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
                RETURN_IF_FAILED(m_oOutputAcceleration.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration));
// Output - Hazard_Light
                tChar const * strDescSignalHazard_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalHazard_Light);
                cObjectPtr<IMediaType> pTypeSignalHazard_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalHazard_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalHazard_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputHazard_Light));
                RETURN_IF_FAILED(m_oOutputHazard_Light.Create("Hazard Light", pTypeSignalHazard_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputHazard_Light));
// Output - Back_Light
                tChar const * strDescSignalBack_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalBack_Light);
                cObjectPtr<IMediaType> pTypeSignalBack_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBack_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalBack_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBack_Light));
                RETURN_IF_FAILED(m_oOutputBack_Light.Create("Back Light", pTypeSignalBack_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputBack_Light));
// Output - Head_Light
                tChar const * strDescSignalHead_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalHead_Light);
                cObjectPtr<IMediaType> pTypeSignalHead_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalHead_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalHead_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputHead_Light));
                RETURN_IF_FAILED(m_oOutputHead_Light.Create("Head Light", pTypeSignalHead_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputHead_Light));
// Output - TurnLeft_Light
                tChar const * strDescSignalTurnLeft_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnLeft_Light);
                cObjectPtr<IMediaType> pTypeSignalTurnLeft_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnLeft_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnLeft_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnLeft_Light));
                RETURN_IF_FAILED(m_oOutputTurnLeft_Light.Create("TurnLeft Light", pTypeSignalTurnLeft_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft_Light));
// Output - TurnRight_Light
                tChar const * strDescSignalTurnRight_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnRight_Light);
                cObjectPtr<IMediaType> pTypeSignalTurnRight_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnRight_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnRight_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnRight_Light));
                RETURN_IF_FAILED(m_oOutputTurnRight_Light.Create("TurnRight Light", pTypeSignalTurnRight_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight_Light));
// Output - Break_Light
                tChar const * strDescSignalBreak_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalBreak_Light);
                cObjectPtr<IMediaType> pTypeSignalBreak_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBreak_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalBreak_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBreak_Light));
                RETURN_IF_FAILED(m_oOutputBreak_Light.Create("Break Light", pTypeSignalBreak_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputBreak_Light));
// create pin for Infos about Stop Line
                tChar const * strDescStopLine = pDescManager->GetMediaDescription("tStoplineStruct");
                RETURN_IF_POINTER_NULL(strDescStopLine);
                cObjectPtr<IMediaType> pTypeSignalStopLine = new cMediaType(0, 0, 0, "tStoplineStruct", strDescStopLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalStopLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStopLine));
                RETURN_IF_FAILED(m_oStopLine.Create("StopLine", pTypeSignalStopLine, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oStopLine));


				// Input pin for position input ADD IN Scanning
				tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
				RETURN_IF_POINTER_NULL(strDescPos);
				cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
				RETURN_IF_FAILED(m_InputPostion.Create("Position", pTypePos, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_InputPostion));
				// create pin for heading angle
				tChar const * strDescSignalHeadingAngle = pDescManager->GetMediaDescription("tSignalValue");
				RETURN_IF_POINTER_NULL(strDescSignalHeadingAngle);
				cObjectPtr<IMediaType> pTypeSignalHeadingAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalHeadingAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeSignalHeadingAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescHeadingAngle));
				RETURN_IF_FAILED(m_oInputHeadingAngle.Create("Heading Angle", pTypeSignalHeadingAngle, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oInputHeadingAngle));
				//output pin for parking space update
				tChar const * strDescParkingSpace = pDescManager->GetMediaDescription("tParkingSpace");
				RETURN_IF_POINTER_NULL(strDescParkingSpace);
				cObjectPtr<IMediaType> pTypeParkingSpace = new cMediaType(0, 0, 0, "tParkingSpace", strDescParkingSpace, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeParkingSpace->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingSpace));
				RETURN_IF_FAILED(m_InputParkingSpace.Create("ParkingSpace", pTypeParkingSpace, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_InputParkingSpace));

    }

    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
	m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
		m_bStart = tFalse;
		m_bFinished = tFalse;
		m_bTransmitout = tFalse;
		m_bTransmitstop = tFalse;
        m_bHazard_Light= tFalse;
        m_bBack_Light= tFalse;
        m_bHead_Light= tFalse;
        m_bTurnLeft_Light= tFalse;
        m_bTurnRight_Light= tFalse;
        m_bImage_used = tFalse;
        m_bBreak_Light= tFalse;
		m_iStateOfPark = SOP_NOSTART;
		m_iStateOfScan = SOP_NOSTART;
		m_bFront_Slot_Detected = tFalse;  
		m_fLane_steer = 0;
		m_fcover_dist = 0;
		m_iParking_slot = 0;
		m_iScanning_slot = 0;
		m_fslot_space_start = 0;
		m_fnextslot_dist_start = 0;
		m_fImage_detection_distance = 0;
		m_fmax_threshold = 110;
		m_fmin_threshold = 45;
		stop_time  = 0;
		// Variables for input position of car
		m_szF32X = 0;			// input of car position to be added with the image distance 			
		m_szF32Y = 0;
		m_szF32Radius = 0;
		m_szF32Speed = 0;
		m_szF32Heading = 0;
		// Variables forparking update
		m_parkingI16Id = 0;		// parking slot ID	
		m_parkingF32X = 0;		// Parking spot position in X  (=m_szF32X+image distance)
		m_parkingF32Y = 0;		// Parking spot position in Y
		m_parkingUI16Status = 0;		// (Parking status free or occupied)
            m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");


        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cParking::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {



                // initial values for steering and acceleration
                m_fAccelerationOutput=0;
                m_fSteeringOutput=0;

        // init process values
        m_bStart=tFalse;
        m_bFinished=tFalse;
        m_bTransmitout= tFalse;
        m_bTransmitstop = tFalse;
        m_bHazard_Light= tFalse;
        m_bBack_Light= tFalse;
        m_bHead_Light= tFalse;
        m_bTurnLeft_Light= tFalse;
        m_bTurnRight_Light= tFalse;
        m_bBreak_Light= tFalse;
        m_bImage_used = tFalse;
        m_iStateOfPark=SOP_NOSTART;
        m_iStateOfScan=SOP_NOSTART;
        m_bFront_Slot_Detected=tFalse;
        m_fLane_steer = 0;
        m_fcover_dist = 0;
        m_iParking_slot=0;
        m_iScanning_slot=0;
        m_fslot_space_start= 0;
        m_fnextslot_dist_start=0;
        m_fImage_detection_distance=0;
		m_fmax_threshold = 110;
		m_fmin_threshold = 45;
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

tResult cParking::PropertyChanged(const char* strProperty)
{
        ReadProperties(strProperty);
        RETURN_NOERROR;
}

tResult cParking::ReadProperties(const tChar* strPropertyName)
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
        }
//check property for backlanefollow
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_LANEFOLLOW_BACK))
        {
                DIST_LANEFOLLOWBACK = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_LANEFOLLOW_BACK));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_LANEFOLLOW_BACK))
        {
                SPEED_LANEFOLLOWBACK = static_cast<tFloat32> (GetPropertyFloat(SPEED_LANEFOLLOW_BACK));
        }
//check property for Scanning
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SLOT_LENGTH))
        {
                DIST_SCAN_SLOTLENGHT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH));
        }
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SLOT_DEPTH))
        {
                DIST_SCAN_SLOTDEPTH = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SLOT_DEPTH));
        }
		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SPACE))
        {
                DIST_SCANSPACE = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SPACE));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, KPSTEERING))
        {
                propKpSteering = static_cast<tFloat32> (GetPropertyFloat(KPSTEERING));
        }
        RETURN_NOERROR;
}
tResult cParking::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{

	cObjectPtr<IMediaCoder> pCoderInput;
	RETURN_IF_FAILED(m_pDescriptionPos->Lock(pMediaSampleIn, &pCoderInput));
	pCoderInput->Get("f32x", (tVoid*)&m_szF32X);
	pCoderInput->Get("f32y", (tVoid*)&m_szF32Y);
	pCoderInput->Get("f32radius", (tVoid*)&m_szF32Radius);
	pCoderInput->Get("f32speed", (tVoid*)&m_szF32Speed);
	pCoderInput->Get("f32heading", (tVoid*)&m_szF32Heading);
	m_pDescriptionPos->Unlock(pCoderInput);

	RETURN_NOERROR;
}

tResult cParking::computepose()
{

	tFloat32 m_angle_change = m_fFirstHeadingAngle - m_szF32Heading;
	m_parkingF32X = m_fposx*std::cos(m_angle_change) - m_fposy*std::sin(m_angle_change);
	m_parkingF32Y = m_fposy*std::cos(m_angle_change) + m_fposx*std::sin(m_angle_change);

	LOG_INFO(cString::Format("Park X %f Park Y %f Angle change %f", m_parkingF32X, m_parkingF32Y, m_angle_change));
	RETURN_NOERROR;
}
tResult cParking::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        if (pMediaSample != NULL)
        {
                // Input signal at Start
                if (pSource == &m_oStart)
                {
		if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool input"));
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_bStart);
                    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescStart->Unlock(pCoderInput);
                    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool input %d", m_bStart));
                    if(!m_bStart)
          	    {	
               		stop_time = _clock->GetStreamTime();		
					m_bTransmitstop=tTrue;
                                        if (m_bTransmitstop)
                                        {
                                            m_bHazard_Light= tFalse;
                                            m_bBack_Light= tFalse;
                                            m_bHead_Light= tFalse;
                                            m_bTurnLeft_Light= tFalse;
                                            m_bTurnRight_Light= tFalse;
                                            m_bBreak_Light= tFalse;
                                            TransmitLight();

                                        }

		    }
                }

                // Input signal at Distance Overall
                else if (pSource == &m_oDistanceOverall)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescdistanceoverall->Unlock(pCoderInput);

                }
				else if (pSource == &m_oInputHeadingAngle)
				{
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pDescHeadingAngle->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&m_fFirstHeadingAngle);
					m_pDescHeadingAngle->Unlock(pCoderInput);
				}
				else  if (pSource == &m_InputPostion)
				{
					tTimeStamp tsInputTime;
					tsInputTime = pMediaSample->GetTime();
					//Process Sample
					RETURN_IF_FAILED(ProcessInputPosition(pMediaSample, tsInputTime));
				}
                else if (pSource == &m_oLane_steer)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescLane_steer->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fLane_steer);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescLane_steer->Unlock(pCoderInput);

                }
                // Input signal at Yaw
                else if (pSource == &m_oYaw)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescYaw->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fYaw);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescYaw->Unlock(pCoderInput);

                }
				else if (pSource == &m_oInputUSStruct)
				{
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pDescriptionInputUSStruct->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("tFrontRight.f32Value", (tVoid*)&m_fUSFrontRightside);
					pCoderInput->Get("tSideRight.f32Value", (tVoid*)&m_fUSRightside);
					pCoderInput->Get("tFrontCenterRight.f32Value", (tVoid*)&m_fUSCenterRight);
					pCoderInput->Get("tRearRight.f32Value", (tVoid*)&m_fUSBackright);
					pCoderInput->Get("tRearLeft.f32Value", (tVoid*)&m_fUSBackleft);

					m_pDescriptionInputUSStruct->Unlock(pCoderInput);

				}

                // infos about Edges
                else if (pSource == &m_oEdges)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescEdges->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("bValue", (tVoid*)&m_bFront_Slot_Detected);
                        pCoderInput->Get("f32Distance", (tVoid*)&m_fImage_detection_distance);
                        pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2parkLine);
                        m_pDescEdges->Unlock(pCoderInput);

                }
                else if (pSource == &m_oStopLine)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescStopLine->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("bValue", (tVoid*)&m_bLine_detection);
                        pCoderInput->Get("f32Distance", (tVoid*)&m_fLine_distance);
                        pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2StopLine);
                        m_pDescStopLine->Unlock(pCoderInput);

                }

	  // stop signal
                if(!m_bStart)
                {
                 // LOG_INFO(cString::Format("Start bool false"));
                        m_fAccelerationOutput=0;
                        m_fSteeringOutput=0;
                        m_iStateOfPark=SOP_NOSTART;
                        m_iStateOfScan=SOS_NOSTART;

                        if(m_bTransmitstop)
                        {
                        TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput, 0);
                        m_bTransmitstop= tFalse;
                        }


                }
            else if (m_bStart)
            {


                        stage_park();
                        TransmitLight();
                        TransmitOutput(m_fSteeringOutput, m_fAccelerationOutput, 0);
            }
                else
                {
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("no bool"));
                }
    }


}
        RETURN_NOERROR;
}

tResult cParking::stage_park()
{

	switch(m_iStateOfPark)
		{
                LOG_INFO(cString::Format("PARKING"));
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Parking case: %d",m_iStateOfPark));
		case SOP_NOSTART:
			{

				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 0:no start"));

				//reset finish flag
				m_bFinished=tFalse;
				m_bFront_Slot_Detected=tFalse;
				//reset Values
				m_iParking_slot=0;
				m_iScanning_slot=0;
				m_fmax_threshold = 105;
				m_fmin_threshold = 45;
                                m_fnextslot_dist_start = m_fDistanceOverall;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
				m_fImage_detection_distance=0;
				m_fcover_dist = DIST_LANEFOLLOW;
				m_bnext_slot_freespace = tTrue;

				// change state of turn when StartSignal is true
				if(m_bStart)
					{
                    		m_iStateOfPark=SOP_Start_step1;
						//save the first distance and get started
					m_iScanning_slot=0;
                    		if(m_bFront_Slot_Detected && m_fImage_detection_distance < 60)
						{
                            m_iStateOfPark=SOP_Reach_slot;
                            m_fcover_dist = (m_fImage_detection_distance/100)+0.23;
                            m_bFront_Slot_Detected=tFalse;
						}
					}
				break;
			}
		case SOP_Start_step1:
			{
				//if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1:desired distance"));
                                            if (m_fDistanceOverall - m_fDistanceOverall_Start < m_fcover_dist) // distance for first_lanefollow
						{
                            if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Step 1:desired distance"));
							m_fSteeringOutput = m_fLane_steer;		//accuator output
                            m_fAccelerationOutput= SPEED_LANEFOLLOW ;

							stop_time = _clock->GetStreamTime();
                                if (m_bFront_Slot_Detected && m_fImage_detection_distance < 60 )  // to check with laxman
								{
                                                                m_fposx = (m_fImage_detection_distance / 100) ; //parking spot update
                                                                m_fposy = 0;
                                                                computepose();
                                                                m_parkingF32X = m_szF32X + m_parkingF32X;
                                                                m_parkingF32Y = m_szF32Y + m_parkingF32Y;
								m_fnextslot_dist_start=m_fDistanceOverall;
								m_iScanning_slot=0;
                                                                m_fcover_dist = (m_fImage_detection_distance / 100) + 0.23; //Image detection distance is in cm
								m_bFront_Slot_Detected=tFalse;
                                                                m_iStateOfPark=SOP_Reach_slot;
                                                                m_fDistanceOverall_Start=m_fDistanceOverall;

								}

						}
						else
						{

							if((_clock->GetStreamTime() - stop_time)/1000000 < 3) //time for wait
							{

								m_fSteeringOutput=0;		//accuator output
								m_fAccelerationOutput= 0;
							}
							else
							{	
                                                        m_fposx = 0; //parking spot update if no spot detection
                                                        m_fposy = 0;
                                                        computepose();
                                                        m_parkingF32X = m_szF32X + m_parkingF32X;
                                                        m_parkingF32Y = m_szF32Y + m_parkingF32Y;
                                                        m_iStateOfScan = SOS_NOSTART;
                                                        m_iStateOfPark = SOP_Scan;
                                                        stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

							}
						
						}
				break;
			}
		case SOP_Reach_slot:
                    {
                               if (m_fDistanceOverall - m_fDistanceOverall_Start < m_fcover_dist) // distance for first_lanefollow
                                    {
                                       //if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Step 1:desired distance"));

                                       m_fSteeringOutput = m_fLane_steer;		//accuator output
                                       m_fAccelerationOutput = SPEED_LANEFOLLOW;
                                       m_bHazard_Light=tTrue;
                                       LOG_INFO(cString::Format("Step 2:reach distance"));
                                       stop_time = _clock->GetStreamTime();
									   if ((m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start)) <= 0.5)
									   {
                                                                                   m_fmax_threshold = 30 + (102 * (m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start))); // detection range decresing as per going furture
                                                                                   m_fmin_threshold = 3 + (102 * (m_fcover_dist - (m_fDistanceOverall - m_fDistanceOverall_Start)));
										   LOG_INFO(cString::Format("1st slot scanning bz front max threshold = %f & min threshold = %f", m_fmax_threshold, m_fmin_threshold));
										   if ((m_fUSFrontRightside <= m_fmax_threshold && m_fUSFrontRightside >= m_fmin_threshold) || (m_fUSCenterRight <= m_fmax_threshold && m_fUSCenterRight >= m_fmin_threshold))
										   {
											   m_bnext_slot_freespace = tFalse;
										   }
									   }


                                    }
                                    else
                                    {
                                        m_bFront_Slot_Detected = tFalse;
                                        m_iStateOfPark = SOP_Scan;
                                        m_iStateOfScan = SOS_NOSTART;
                                        stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                        m_bBreak_Light = tFalse;
                                                                                m_bHazard_Light = tTrue;
									}
                     break;
                    }
		case SOP_Scan:
			{
				stage_scan();
				break;
			}

		case SOP_Outsteer_front:
			{
				 if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 2:outsteer frontside"));
				 if(m_fDistanceOverall-m_fDistanceOverall_Start < DIST_OUTFRONT) // distance for outsteer_front
					{
						m_fSteeringOutput= STEER_OUTFRONT;		//accuator output for outsteer frontside
						m_fAccelerationOutput=SPEED_OUTFRONT;
						stop_time = _clock->GetStreamTime();
                                                m_bTurnLeft_Light = tTrue;
					}
				 else
					{
						if((_clock->GetStreamTime() - stop_time)/1000000 < 1) // time for wait
							{
                                                                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 2:outsteer frontside wait"));
                                                                m_fSteeringOutput=STEER_INBACK;		//accuator output
								m_fAccelerationOutput=0;
                                                                m_bBreak_Light = tTrue;
                                                                m_bTurnLeft_Light = tFalse;
							}
						else
							{
								m_iStateOfPark=SOP_Insteer_back;  // change parking manuver
								m_fDistanceOverall_Start=m_fDistanceOverall;
                                                                m_bBreak_Light = tFalse;
                                                                m_fSteeringOutput=STEER_INBACK;		//accuator output for backside_lanefollow


							}
					}
				 break;
			}
		case SOP_Insteer_back:
			{
				 if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 3:Insteer back"));
				 if(m_fDistanceOverall-m_fDistanceOverall_Start < DIST_INBACK) // distance for insteer_back
					{
						m_fSteeringOutput=STEER_INBACK;		//accuator output for backside_lanefollow
						m_fAccelerationOutput=SPEED_INBACK;
						stop_time = _clock->GetStreamTime();

					}
				 else
					{

								m_iStateOfPark=SOP_Back_lanefollow;  // change parking manuver
								m_fDistanceOverall_Start=m_fDistanceOverall;
                                                                m_fback_coverdist=DIST_LANEFOLLOWBACK;

                                                                m_bBack_Light = tTrue; // back light
                                                                m_bImage_used = tFalse;
                                                if (m_bLine_detection && m_fLine_distance < 60)
                                                {

                                                        m_fback_coverdist = (m_fLine_distance / 100) + 0.28;
                                                        m_bLine_detection = tFalse;
							LOG_INFO(cString::Format("go back with image detection"));
                                                        m_fDistanceOverall_Start = m_fDistanceOverall;
                                                        
                                                }

					}
				 break;
			}

		case SOP_Back_lanefollow:
			{
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 4:backside lanefollow"));
                                 if(m_fDistanceOverall-m_fDistanceOverall_Start < m_fback_coverdist) // distance for backside_lanefollow
					{

                                                //m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                                m_fAccelerationOutput= SPEED_LANEFOLLOWBACK;
                                                //-------------------------------------------steering decision--------------------------
                                                if (m_fUSBackleft <=30 && m_fUSBackright<=30)
                                                {
                                                    m_fSteeringOutput=2*propKpSteering*(m_fUSBackright-m_fUSBackleft);
                                                }
                                                else if(m_fUSBackright<=15 && m_fUSBackleft >=30)        // only one car at right side
                                                {
                                                    m_fSteeringOutput=2*propKpSteering*(m_fUSBackright-15);
                                                }
                                                else if(m_fUSBackleft<=15 && m_fUSBackright >=15)          // only one care at left side
                                                {
                                                    m_fSteeringOutput=propKpSteering*(15-m_fUSBackleft);
                                                }
                                                else
                                                {
                                                   if (m_bLine_detection)
                                                   {
                                                        m_fSteeringOutput = propKpSteering*(90 - m_fOrientation2StopLine);
                                                   }
                                                   else
                                                   {
                                                       m_fSteeringOutput=0;
                                                   }
                                                }
                                                //----------------------------------------------------------------------------------------
						stop_time = _clock->GetStreamTime();

					}
				 //-------------------------------------------------------------------------------------------------------------
				 else
					{

                                                m_iStateOfPark=SOP_Wait;  // change parking manuver
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                stop_time = _clock->GetStreamTime();
                                                m_bHazard_Light = tTrue;
                                                m_bImage_used = tFalse;
                                                m_bBack_Light = tFalse; // back light
					}
				 break;
			}

		case SOP_Wait:
			{
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 5:Wait"));
                                 if((_clock->GetStreamTime() - stop_time)/1000000 < 3) // wait for backside_lanefollow
					{
						m_fSteeringOutput=0;               //accuator output for backside_lanefollow
						m_fAccelerationOutput=0;

					}
				 else
                                 {
                                         m_iStateOfPark=SOP_Finished;  // change parking manuver
                                         m_fDistanceOverall_Start=m_fDistanceOverall;
                                         m_bHazard_Light = tFalse;
                                 }
                                 break;
			}

		case SOP_Finished:
			{
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 5 : finished"));
					//stop
				m_bFinished=tTrue;
                                TransmitFinish();
                                m_bStart=tFalse;
                                m_bHazard_Light=tFalse;
                                m_bBack_Light=tFalse;
                                m_bHead_Light=tFalse;
                                m_bTurnLeft_Light=tFalse;
                                m_bTurnRight_Light=tFalse;
                                m_bBreak_Light=tFalse;
				stop_time = _clock->GetStreamTime();  
                                m_fposx=0; //parking spot update
                                m_fposy=0;
                                computepose();
                                m_parkingF32X = 0;
                                m_parkingF32Y = 0;
                                m_parkingI16Id = 0;
                                m_parkingUI16Status = 0;
                                SendParkingData();
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 5 : sent transmit finish for parking= %d ",m_bFinished));
                                m_iStateOfPark=SOP_NOSTART;
				break;
			}
                default :
                        {
                                m_iStateOfPark=SOP_NOSTART;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("PARKING default case"));
                        }
		}
	RETURN_NOERROR;
}

tResult cParking::stage_scan()
{
//scanning code
	if(m_bDebugModeEnabled) LOG_INFO(cString::Format("in side scan"));
	switch(m_iStateOfScan)
		{
		case SOS_NOSTART:
			{
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 0:no scanning start"));
				m_fScan_dist_start= m_fDistanceOverall;
                                m_fnextslot_dist_start= m_fDistanceOverall;
				m_fslot_space_start=m_fDistanceOverall;
				m_fScan_distance= 0;
				m_iStateOfScan=SOS_Start_currentslot;
 				stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
				m_iScanning_slot++; //increment of scanning slot
				m_fSlot_Length=DIST_SCAN_SLOTLENGHT;
                                m_bFree_space_by_sideUS = tFalse;
				m_bFree_space_by_frontUS = m_bnext_slot_freespace;
				if (m_bFront_Slot_Detected && m_fImage_detection_distance < 60)
					{
						if(m_bDebugModeEnabled) LOG_INFO(cString::Format("image detect"));
						m_fnextslot_dist_start=m_fDistanceOverall;
						m_fSlot_Length = (m_fImage_detection_distance / 100)-DIST_SCAN_SLOTLENGHT + 0.23; //Image detection distance is in cm
						m_bFront_Slot_Detected=tFalse;
					}

				break;
			}
		case SOS_Start_currentslot:
			{
				if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step :current slot"));
                                //----------------------------------------check image detection-------------------------------
                                if (m_bFront_Slot_Detected && m_fImage_detection_distance < 60)
                                {
                                        if(m_bDebugModeEnabled)LOG_INFO(cString::Format("image detect"));
                                        m_fnextslot_dist_start=m_fDistanceOverall;
                                        m_fSlot_Length = (m_fImage_detection_distance / 100)-DIST_SCAN_SLOTLENGHT + 0.23;  //Image detection distance is in cm
                                        m_bFront_Slot_Detected=tFalse;
										if (m_iScanning_slot <= 3)
										{
											m_fposx = (m_fImage_detection_distance / 100) - DIST_SCAN_SLOTLENGHT; //parking spot update
											m_fposy = 0;
											computepose();
											m_parkingF32X = m_szF32X + m_parkingF32X;
											m_parkingF32Y = m_szF32Y + m_parkingF32Y;
										}

										else
										{
											m_fposx = m_fposx + 0.46; //parking spot update
											m_fposy = 0;
											computepose();
											m_parkingF32X = m_szF32X + m_parkingF32X;
											m_parkingF32Y = m_szF32Y + m_parkingF32Y;
										}
                                }
                                //--------------------------------------------------------------------------------------------

                               if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Scanning slot =%d",m_iScanning_slot));
                                if(m_fDistanceOverall-m_fnextslot_dist_start < m_fSlot_Length) // distance for scanning area in meter
                                        {

											m_fSteeringOutput = m_fLane_steer;		//accuator output
                                            m_fAccelerationOutput = SPEED_LANEFOLLOW;
                                            m_fmax_threshold = 30 + (102 * (m_fSlot_Length - (m_fDistanceOverall - m_fnextslot_dist_start))); // detection range decrtesing as per going furture
                                            m_fmin_threshold = 3 + (102 * (m_fSlot_Length - (m_fDistanceOverall - m_fnextslot_dist_start)));
											
                                            //----------------------------------------check object detection in curret parking slot--------------
                                            if (m_fUSRightside < ((DIST_SCAN_SLOTDEPTH)*100)) //scan slot depth
                                                    {
                                                            //LOG_INFO(cString::Format(" object detect at = %f ",m_fUSRightside));
                                                            m_fslot_space_start= m_fDistanceOverall;  // if object in parking slot detected  ---> no space distance update
                                                            m_fScan_distance= m_fDistanceOverall-m_fslot_space_start; // =0
															m_bFree_space_by_sideUS = tFalse;
                                                    }
                                            else
                                                    {
                                                            m_fScan_distance= m_fDistanceOverall-m_fslot_space_start;  // no object dected ----> space distance calculated
                                                            //LOG_INFO(cString::Format("no object : scan distance = %f",m_fScan_distance));


                                                    }
                                            if (m_fScan_distance >= DIST_SCANSPACE) m_bFree_space_by_sideUS = tTrue;

                                            else m_bFree_space_by_sideUS = tFalse;
                                            //-------------------------------------------------------------------------------------------
                                            //--------------------------------------- object detection in next parking slot-----------------
                                            if ((m_fUSFrontRightside <= m_fmax_threshold && m_fUSFrontRightside >= m_fmin_threshold) || (m_fUSCenterRight <= m_fmax_threshold && m_fUSCenterRight >= m_fmin_threshold))
                                            {
                                                    m_bnext_slot_freespace = tFalse;
                                                    if(m_bDebugModeEnabled)LOG_INFO(cString::Format(" : scan distance = %f", m_fScan_distance));

                                            }
                                            //----------------------------------------------------------------------------------------------

                                            //----------------------------------------decision on above detections-----------------------
                                            if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS)  // if desired space is available
                                                    {

                                                                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("space_available : scan distance = %f",m_fScan_distance));
                                                                        m_bBreak_Light = tTrue;
                                                                        m_fScan_distance=m_fScan_distance-0.03;
                                                                        m_bBack_Light = tTrue;
                                                                        m_iStateOfScan=SOS_Back_for_manuver; // park_space found so going for manuve
                                                                        m_fScan_dist_start=m_fDistanceOverall;
                                                                        stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                                                         LOG_INFO(cString::Format("scanned slot =%d ,by sidesensor = %d, by frontsensor = %d ,freespace = %d", m_iScanning_slot, m_bFree_space_by_sideUS, m_bFree_space_by_frontUS, m_bFree_space_by_sideUS && m_bFree_space_by_frontUS));
																		 m_parkingI16Id = m_iScanning_slot;
																		 if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS) m_parkingUI16Status = 1;
																		 else m_parkingUI16Status = 0;
																		 SendParkingData();
                                                    }
                                           else
                                                    {
                                                            stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
															
                                                    }
											//------------------------------------------------------------------------------------------------
                                        }

                                else
                                        {
											
						if (m_fScan_distance >= DIST_SCANSPACE)  // if desired space is available
						{
							m_bFree_space_by_sideUS = tTrue;
						}
						if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS)  // decision
                                                    {

                                                        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("space_available : scan distance = %f",m_fScan_distance));
                                                        m_fSteeringOutput = m_fLane_steer;  //accuator output
                                                        m_fAccelerationOutput = SPEED_LANEFOLLOW;
														m_bBreak_Light = tTrue;
                                                        m_fScan_distance=m_fScan_distance-0.03;
														m_bBack_Light = tTrue;
                                                        m_iStateOfScan=SOS_Back_for_manuver; // park_space found so going for manuve
                                                        LOG_INFO(cString::Format("scanned slot =%d ,by sidesensor = %d, by frontsensor = %d ,freespace = %d", m_iScanning_slot, m_bFree_space_by_sideUS, m_bFree_space_by_frontUS, m_bFree_space_by_sideUS && m_bFree_space_by_frontUS));
														m_parkingI16Id = m_iScanning_slot;
														if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS) m_parkingUI16Status = 1;
														else m_parkingUI16Status = 0;
														SendParkingData();
														m_fScan_dist_start=m_fDistanceOverall;
                                                        stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                                    }
                                            else if (m_iScanning_slot <= 3) // if parking available slot are 4
                                                   {
                                                       if (m_bDebugModeEnabled) LOG_INFO(cString::Format("going to next slot"));
                                                       m_iStateOfScan = SOS_go_nextslot;  //if no space found so going for next slot
                                                       m_fSlot_Length = DIST_SCAN_SLOTLENGHT;
													   m_bFree_space_by_sideUS = tFalse;

                                                   }
                                                   else
                                                   {
                                                        LOG_INFO(cString::Format("scanned slot =%d ,by sidesensor = %d, by frontsensor = %d ,freespace = %d", m_iScanning_slot, m_bFree_space_by_sideUS, m_bFree_space_by_frontUS, m_bFree_space_by_sideUS && m_bFree_space_by_frontUS));
                                                        m_iStateOfScan=SOS_Finished;
														m_bFree_space_by_sideUS = tFalse;
                                                   }
                                        }
				break;
			}
                case SOS_go_nextslot:
			{
                                    m_fposx = m_parkingF32X + 0.46; //parking spot update if no spot detection
                                    m_fposy = 0;
                                    computepose();
                                    m_parkingF32X = m_szF32X + m_parkingF32X;
                                    m_parkingF32Y = m_szF32Y + m_parkingF32Y;
                                    m_parkingI16Id = m_iScanning_slot;
                                    if (m_bFree_space_by_sideUS && m_bFree_space_by_frontUS) m_parkingUI16Status = 1;
                                    else m_parkingUI16Status = 0;
                                    SendParkingData();
                                    LOG_INFO(cString::Format("scanned slot =%d ,by sidesensor = %d, by frontsensor = %d ,freespace = %d", m_iScanning_slot, m_bFree_space_by_sideUS, m_bFree_space_by_frontUS, m_bFree_space_by_sideUS && m_bFree_space_by_frontUS));
                                m_iScanning_slot++;
                                m_fScan_dist_start = m_fDistanceOverall;
                                m_fslot_space_start = m_fDistanceOverall;
                                m_fScan_distance = 0;
                                m_fnextslot_dist_start = m_fDistanceOverall;
                                stop_time = _clock->GetStreamTime();
                                m_iStateOfScan = SOS_Start_currentslot;
                                m_fSlot_Length = DIST_SCAN_SLOTLENGHT;
                                m_bFree_space_by_sideUS = tTrue;
								m_bFree_space_by_frontUS = m_bnext_slot_freespace; // transfer the detection of next slot to current slot
                                m_bnext_slot_freespace = tTrue;

				break;
			}
		case SOS_Back_for_manuver:
			{
                                if((_clock->GetStreamTime() - stop_time)/1000000 < 0.5) // distance for backside_lanefollow
                                        {
                                               if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Wait before going to back"));

                                                m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                                m_fAccelerationOutput=0;

                                        }
                                else
                                        {
                                               if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step: Back for manuver"));
                                                if(m_fDistanceOverall-m_fScan_dist_start < (m_fScan_distance) && m_fUSRightside >40) // add offset distancevalue for parking manuver
                                                        {

					
								        m_fSteeringOutput = -m_fLane_steer;		//accuator output
								   
								
                                                                m_fAccelerationOutput= SPEED_LANEFOLLOWBACK;
								m_bBreak_Light = tFalse;

                                                        }
                                                else
                                                        {
                                                                if(m_fUSRightside<40)
                                                                {
                                                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                                                    m_iStateOfScan=SOS_Set_for_manuver;
                                                                    m_iParking_slot=m_iScanning_slot;
                                                                    m_bBack_Light = tFalse;
                                                                }
                                                                else
                                                                {
                                                                    m_iStateOfScan=SOS_Finished; // so scan is finished now !!
                                                                    m_iParking_slot=m_iScanning_slot;
                                                                    stop_time = _clock->GetStreamTime();
                                                                    m_bBack_Light = tFalse;
                                                                }
                                                        }
                                         }
                               break;
			}
                case SOS_Set_for_manuver:
                        {
                        if(m_fDistanceOverall-m_fDistanceOverall_Start < 0.05 )
                        {
                                m_fSteeringOutput = m_fLane_steer;		//accuator output
                                m_fAccelerationOutput = SPEED_LANEFOLLOW;
                        }
                        else
                        {
                        m_iStateOfScan=SOS_Finished; // so scan is finished now !!
                        stop_time = _clock->GetStreamTime();
                        }

                        break;
                        }
                case SOS_Back_for_scan:
                        {
                        if(m_fDistanceOverall-m_fDistanceOverall_Start < 3 ) // going back 3 meter
                        {

                                m_fSteeringOutput = -0.5*m_fLane_steer;		//accuator output
		                m_fAccelerationOutput = SPEED_LANEFOLLOWBACK;
				stop_time = _clock->GetStreamTime();
                        }
                        else
                        {	
                            if((_clock->GetStreamTime() - stop_time)/1000000 < 0.5)
                                    {
                                           if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Wait before stop"));
                                            m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                            m_fAccelerationOutput=0;
                                    }
			    else
				{
				m_iStateOfScan=SOS_NOSTART; // so scan we have to start it again!!
				m_iStateOfPark=SOP_NOSTART;  //parking start again
				m_iParking_slot=0;
				m_iScanning_slot=0;
				stop_time = _clock->GetStreamTime();
				m_fDistanceOverall_Start=m_fDistanceOverall;
				}
                        }

                        break;
                        }
		case SOS_Finished:
			{

				if(m_iParking_slot!=0) // if parking slot Found
					{
                                                if((_clock->GetStreamTime() - stop_time)/1000000 < 1) // distance for backside_lanefollow
                                                        {
                                                                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Wait before handover to parking manuver"));

                                                                m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                                                m_fAccelerationOutput=0;
                                                        }
                                                else
                                                        {
                                                                m_iStateOfScan=SOS_NOSTART; // scan swich to initial value
                                                                m_iStateOfPark=SOP_Outsteer_front; // parking manuver change
                                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                                m_iScanning_slot=0;
                                                                stop_time = _clock->GetStreamTime();
                                                        }
                                        }
				else
					{
                                    if((_clock->GetStreamTime() - stop_time)/1000000 < 0.5)
                                            {
                                                   if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Wait before stop"));
                                                    m_fSteeringOutput=0;               //accuator output for backside_lanefollow
                                                    m_fAccelerationOutput=0;
                                            }

                                    else
                                            {
                                                LOG_INFO(cString::Format("no free space go back for scan again"));
						m_iStateOfScan=SOS_Back_for_scan;
						m_iParking_slot=0;
						m_iScanning_slot=0;
                                                stop_time = _clock->GetStreamTime();
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                            	m_bBack_Light = tTrue;
						m_bHead_Light=tFalse;
                                                m_fposx=0; //parking spot update
                                                m_fposy=0;
                                                computepose();
                                                m_parkingF32X = 0;
                                                m_parkingF32Y = 0;
                                                m_parkingI16Id = 0;
                                                m_parkingUI16Status = 0;
                                                SendParkingData();

                                            }
					}

				break;
			}
		default :
                    {
                            m_iStateOfScan=SOS_NOSTART;
                            m_fScan_dist_start=m_fDistanceOverall;
                           if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Scanning default case"));
                    }
	}
	RETURN_NOERROR;
}





tResult cParking::TransmitFinish()
{
	// Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleParking_Finish;
        AllocMediaSample((tVoid**)&pMediaSampleParking_Finish);

        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerParking_Finish;
        m_pDescriptionOutputParking_Finish->GetMediaSampleSerializer(&pSerializerParking_Finish);
        tInt nSizeParking_Finish = pSerializerParking_Finish->GetDeserializedSize();
        pMediaSampleParking_Finish->AllocBuffer(nSizeParking_Finish);
        cObjectPtr<IMediaCoder> pCoderOutputParking_Finish;
        m_pDescriptionOutputParking_Finish->WriteLock(pMediaSampleParking_Finish, &pCoderOutputParking_Finish);
        pCoderOutputParking_Finish->Set("bValue", (tVoid*)&(m_bFinished));
        pCoderOutputParking_Finish->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputParking_Finish->Unlock(pCoderOutputParking_Finish);
        pMediaSampleParking_Finish->SetTime(_clock->GetStreamTime());
        m_oOutputParking_Finish.Transmit(pMediaSampleParking_Finish);
        RETURN_NOERROR;
}

tResult cParking::TransmitLight()
{
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleHead_Light;
        cObjectPtr<IMediaSample> pMediaSampleBack_Light;
        cObjectPtr<IMediaSample> pMediaSampleHazard_Light;
        cObjectPtr<IMediaSample> pMediaSampleTurnLeft_Light;
        cObjectPtr<IMediaSample> pMediaSampleTurnRight_Light;
        cObjectPtr<IMediaSample> pMediaSampleBreak_Light;

        AllocMediaSample((tVoid**)&pMediaSampleHead_Light);
        AllocMediaSample((tVoid**)&pMediaSampleBack_Light);
        AllocMediaSample((tVoid**)&pMediaSampleHazard_Light);
        AllocMediaSample((tVoid**)&pMediaSampleTurnLeft_Light);
        AllocMediaSample((tVoid**)&pMediaSampleTurnRight_Light);
        AllocMediaSample((tVoid**)&pMediaSampleBreak_Light);





        // Send the Media Sample Head_Light
        cObjectPtr<IMediaSerializer> pSerializerHead_Light;
        m_pDescriptionOutputHead_Light->GetMediaSampleSerializer(&pSerializerHead_Light);
        tInt nSizeHead_Light = pSerializerHead_Light->GetDeserializedSize();
        pMediaSampleHead_Light->AllocBuffer(nSizeHead_Light);
        cObjectPtr<IMediaCoder> pCoderOutputHead_Light;
        m_pDescriptionOutputHead_Light->WriteLock(pMediaSampleHead_Light, &pCoderOutputHead_Light);
        pCoderOutputHead_Light->Set("bValue", (tVoid*)&(m_bHead_Light));
        pCoderOutputHead_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputHead_Light->Unlock(pCoderOutputHead_Light);
        pMediaSampleHead_Light->SetTime(_clock->GetStreamTime());
        m_oOutputHead_Light.Transmit(pMediaSampleHead_Light);


        // Send the Media Sample Back_Light
        cObjectPtr<IMediaSerializer> pSerializerBack_Light;
        m_pDescriptionOutputBack_Light->GetMediaSampleSerializer(&pSerializerBack_Light);
        tInt nSizeBack_Light = pSerializerBack_Light->GetDeserializedSize();
        pMediaSampleBack_Light->AllocBuffer(nSizeBack_Light);
        cObjectPtr<IMediaCoder> pCoderOutputBack_Light;
        m_pDescriptionOutputBack_Light->WriteLock(pMediaSampleBack_Light, &pCoderOutputBack_Light);
        pCoderOutputBack_Light->Set("bValue", (tVoid*)&(m_bBack_Light));
        pCoderOutputBack_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputBack_Light->Unlock(pCoderOutputBack_Light);
        pMediaSampleBack_Light->SetTime(_clock->GetStreamTime());
        m_oOutputBack_Light.Transmit(pMediaSampleBack_Light);


        // Send the Media Sample Hazard_Light
        cObjectPtr<IMediaSerializer> pSerializerHazard_Light;
        m_pDescriptionOutputHazard_Light->GetMediaSampleSerializer(&pSerializerHazard_Light);
        tInt nSizeHazard_Light = pSerializerHazard_Light->GetDeserializedSize();
        pMediaSampleHazard_Light->AllocBuffer(nSizeHazard_Light);
        cObjectPtr<IMediaCoder> pCoderOutputHazard_Light;
        m_pDescriptionOutputHazard_Light->WriteLock(pMediaSampleHazard_Light, &pCoderOutputHazard_Light);
        pCoderOutputHazard_Light->Set("bValue", (tVoid*)&(m_bHazard_Light));
        pCoderOutputHazard_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputHazard_Light->Unlock(pCoderOutputHazard_Light);
        pMediaSampleHazard_Light->SetTime(_clock->GetStreamTime());
        m_oOutputHazard_Light.Transmit(pMediaSampleHazard_Light);

        // Send the Media Sample TurnLeft_Light
        cObjectPtr<IMediaSerializer> pSerializerTurnLeft_Light;
        m_pDescriptionOutputTurnLeft_Light->GetMediaSampleSerializer(&pSerializerTurnLeft_Light);
        tInt nSizeTurnLeft_Light = pSerializerTurnLeft_Light->GetDeserializedSize();
        pMediaSampleTurnLeft_Light->AllocBuffer(nSizeTurnLeft_Light);
        cObjectPtr<IMediaCoder> pCoderOutputTurnLeft_Light;
        m_pDescriptionOutputTurnLeft_Light->WriteLock(pMediaSampleTurnLeft_Light, &pCoderOutputTurnLeft_Light);
        pCoderOutputTurnLeft_Light->Set("bValue", (tVoid*)&(m_bTurnLeft_Light));
        pCoderOutputTurnLeft_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputTurnLeft_Light->Unlock(pCoderOutputTurnLeft_Light);
        pMediaSampleTurnLeft_Light->SetTime(_clock->GetStreamTime());
        m_oOutputTurnLeft_Light.Transmit(pMediaSampleTurnLeft_Light);


        // Send the Media Sample TurnRight_Light
        cObjectPtr<IMediaSerializer> pSerializerTurnRight_Light;
        m_pDescriptionOutputTurnRight_Light->GetMediaSampleSerializer(&pSerializerTurnRight_Light);
        tInt nSizeTurnRight_Light = pSerializerTurnRight_Light->GetDeserializedSize();
        pMediaSampleTurnRight_Light->AllocBuffer(nSizeTurnRight_Light);
        cObjectPtr<IMediaCoder> pCoderOutputTurnRight_Light;
        m_pDescriptionOutputTurnRight_Light->WriteLock(pMediaSampleTurnRight_Light, &pCoderOutputTurnRight_Light);
        pCoderOutputTurnRight_Light->Set("bValue", (tVoid*)&(m_bTurnRight_Light));
        pCoderOutputTurnRight_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputTurnRight_Light->Unlock(pCoderOutputTurnRight_Light);
        pMediaSampleTurnRight_Light->SetTime(_clock->GetStreamTime());
        m_oOutputTurnRight_Light.Transmit(pMediaSampleTurnRight_Light);

		// Send the Media Sample Break_Light
		cObjectPtr<IMediaSerializer> pSerializerBreak_Light;
		m_pDescriptionOutputBreak_Light->GetMediaSampleSerializer(&pSerializerBreak_Light);
		tInt nSizeBreak_Light = pSerializerBreak_Light->GetDeserializedSize();
		pMediaSampleBreak_Light->AllocBuffer(nSizeBreak_Light);
		cObjectPtr<IMediaCoder> pCoderOutputBreak_Light;
		m_pDescriptionOutputBreak_Light->WriteLock(pMediaSampleBreak_Light, &pCoderOutputBreak_Light);
		pCoderOutputBreak_Light->Set("bValue", (tVoid*)&(m_bBreak_Light));
		pCoderOutputBreak_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
		m_pDescriptionOutputBreak_Light->Unlock(pCoderOutputBreak_Light);
		pMediaSampleBreak_Light->SetTime(_clock->GetStreamTime());
		m_oOutputBreak_Light.Transmit(pMediaSampleBreak_Light);

        RETURN_NOERROR;
}
tResult cParking::TransmitOutput(tFloat32 speed,tFloat32 steering, tUInt32 timestamp)
{

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleaccelerate;
        cObjectPtr<IMediaSample> pMediaSamplesteer;



        AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
        AllocMediaSample((tVoid**)&pMediaSamplesteer);


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

        RETURN_NOERROR;
}


tResult cParking::SendParkingData() // add this in Scanning Code

{
	cObjectPtr<IMediaSample> pMediaSampleTraffic;
	AllocMediaSample((tVoid**)&pMediaSampleTraffic);
	// Parking space data
	cObjectPtr<IMediaSerializer> pSerializerTraffic;
	m_pDescriptionParkingSpace->GetMediaSampleSerializer(&pSerializerTraffic);
	tInt nSizesteer = pSerializerTraffic->GetDeserializedSize();
	pMediaSampleTraffic->AllocBuffer(nSizesteer);
	cObjectPtr<IMediaCoder> pCoderOutputTraffic;
	m_pDescriptionParkingSpace->WriteLock(pMediaSampleTraffic, &pCoderOutputTraffic);
	pCoderOutputTraffic->Set("i16Identifier", (tVoid*)&(m_parkingI16Id));
	pCoderOutputTraffic->Set("f32x", (tVoid*)&(m_parkingF32X));
	pCoderOutputTraffic->Set("f32y", (tVoid*)&(m_parkingF32Y));
	pCoderOutputTraffic->Set("ui16Status", (tVoid*)&(m_parkingUI16Status));
	m_pDescriptionOutputSteering->Unlock(pCoderOutputTraffic);
	pMediaSampleTraffic->SetTime(_clock->GetStreamTime());
	m_InputParkingSpace.Transmit(pMediaSampleTraffic);

	RETURN_NOERROR;
}
