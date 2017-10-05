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
#ifndef _PARKING_H_
#define _PARKING_H_

#define OID_ADTF_PARKING "adtf.example.parking_filter"


/*! @defgroup TemplateFilter
*  @{
*
*  \image html User_Template.PNG "Plugin Template Filter"
*
* This is a small template which can be used by the AADC teams for their own filter implementations.
* \b Dependencies \n
* This plugin needs the following libraries:
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
* <tr><td>Filename<td>user_templateFilter.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*
*/

//!  Template filter for AADC Teams
/*!
* This is a example filter for the AADC
*/
class cParking : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_PARKING, "Parking", adtf::OBJCAT_DataFilter);

protected:
    /*! input pins  */
        // Start
        cInputPin m_oStart;
        cObjectPtr<IMediaTypeDescription> m_pDescStart;
        cInputPin    m_oInputTrafficSign;

        // input pin Diclaration yaw
        cInputPin m_oYaw;
        cObjectPtr<IMediaTypeDescription> m_pDescYaw;
        // mamber pin Diclaration yaw
        tFloat32 m_fYaw;
        tFloat32 m_fYaw_Start;
		
        // Front_Image_Slot
        cInputPin m_oFront_Image_Slot;
        cObjectPtr<IMediaTypeDescription> m_pDescFront_Image_Slot;

        // Front_Slot_Distance
        cInputPin m_oFront_Slot_Distance;
        cObjectPtr<IMediaTypeDescription> m_pDescFront_Slot_Distance;

        // Distance over all
        cInputPin m_oDistanceOverall;
        cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;

        // Lane steer
        cInputPin m_oLane_steer;
        cObjectPtr<IMediaTypeDescription> m_pDescLane_steer;

        // us Struct
        cInputPin   m_oInputUSStruct;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionInputUSStruct;

        // Infos about Edges
        cInputPin m_oEdges;
        cObjectPtr<IMediaTypeDescription> m_pDescEdges;
        // Infos about Stop Line
        cInputPin m_oStopLine;
        cObjectPtr<IMediaTypeDescription> m_pDescStopLine;

    /*! output pins */
        // output steering
        cOutputPin    m_oOutputSteering;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteering;

        // output acceleration
        cOutputPin m_oOutputAcceleration;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputAcceleration;

        // output Parking_finish
        cOutputPin m_oOutputParking_Finish;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputParking_Finish;
        // output Back_Light
        cOutputPin m_oOutputBack_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputBack_Light;
        // output Hazard_Light
        cOutputPin m_oOutputHazard_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputHazard_Light;
        // output Head_Light
        cOutputPin m_oOutputHead_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputHead_Light;
        // output TurnLeft_Light
        cOutputPin m_oOutputTurnLeft_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnLeft_Light;
        // output TurnRight_Light
        cOutputPin m_oOutputTurnRight_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnRight_Light;
        // output Break_Light
        cOutputPin m_oOutputBreak_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputBreak_Light;


        /* MEMBER VARIABLES INPUT*/

        tBool m_bStart;
        tFloat32 m_fLane_steer;


        /* MEMBER VARIABLES PROCESS*/



        tTimeStamp stop_time;
        tBool m_bFinished;
        tBool m_bTurnSignalLeftEnabled;
        tBool m_bTurnSignalRightEnabled;
        tBool m_bTransmitout;
        tBool m_bTransmitstop;
        tBool m_bImage_used;
        //scanning veriable
		tBool m_bLine_detection;
		tBool m_bnext_slot_freespace;
		tBool m_bFree_space_by_frontUS;
		tBool m_bFree_space_by_sideUS;
        tBool m_bFront_Slot_Detected;
		tBool m_bFree_space;
        tFloat32 m_fScan_distance;
        tFloat32 m_fScan_dist_start;
        tFloat32 m_fUSRightside;
        tFloat32 m_fUSBackright;
        tFloat32 m_fUSBackleft;
		tFloat32 m_fUSFrontRightside;
		tFloat32 m_fUSCenterRight;
        tFloat32 m_fnextslot_dist_start;
        tFloat32 m_fslot_space_start;
        tFloat32 m_bBack_Slot_Detected;
		tFloat32 m_fcover_dist;
        tFloat32 m_fImage_detection_distance;
        tFloat32 m_fSlot_Length;
		tFloat32 m_fmax_threshold;
		tFloat32 m_fmin_threshold;
        tInt16 m_iScanning_slot;
        tInt16 m_iParking_slot;
        tInt16 m_iStateOfPark;
        tInt16 m_iStateOfScan;
        tFloat32 m_fLine_distance;

        tFloat32 m_fback_coverdist;
        tFloat32 m_fOrientation2StopLine;

	enum StateOfParkEnums{
        SOP_NOSTART = 0,
        SOP_Start_step1,
		SOP_Reach_slot,
		SOP_Scan,
        SOP_Outsteer_front,
        SOP_Insteer_back,
        SOP_Back_lanefollow,
        SOP_Wait,
        SOP_Finished      
    };
	enum StateOfScanEnums{
        SOS_NOSTART = 0,
        SOS_Start_currentslot,
        SOS_go_nextslot,
        SOS_Back_for_manuver,
        SOS_Set_for_manuver,
        SOS_Finished      
    };
	// time stamp
        tUInt32 timestamp;

	// debug mode
	tBool m_bDebugModeEnabled;

        
	// distance over all
        tFloat32 m_fDistanceOverall;
        tFloat32 m_fDistanceOverall_Start;

        /* MEMBER VARIABLES OUTPUT*/
        tFloat32 m_fSteeringOutput;
        tFloat32 m_fAccelerationOutput;		
        tBool m_bHazard_Light;
        tBool m_bBack_Light;
        tBool m_bHead_Light;
        tBool m_bTurnLeft_Light;
        tBool m_bTurnRight_Light;
        tBool m_bBreak_Light;



public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cParking(const tChar* __info);

    /*! default destructor */
    virtual ~cParking();
        tResult PropertyChanged(const char* strProperty);

protected:
    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
        tResult ProcessManeuver();
        tResult stage_park();
		tResult stage_scan();
                tResult ReadProperties(const tChar* strPropertyName);
                tResult TransmitOutput(tFloat32 speed,tFloat32 steering, tUInt32 timestamp);
                tResult TransmitFinish();
                tResult TransmitLight();
        private:

                // Properties
                tFloat32    DIST_LANEFOLLOW,SPEED_LANEFOLLOW,DIST_OUTFRONT,STEER_OUTFRONT,SPEED_OUTFRONT ;
                tFloat32    DIST_INBACK,STEER_INBACK,SPEED_INBACK,DIST_LANEFOLLOWBACK,SPEED_LANEFOLLOWBACK,DIST_SCAN_SLOTLENGHT,DIST_SCAN_SLOTDEPTH,DIST_SCANSPACE ;
                tFloat32    propKpSteering;
    /*! Property - whether output to console is enabled or not*/

};

//*************************************************************************************************
#endif // _PARKING_H_

/*!
*@}
*/
