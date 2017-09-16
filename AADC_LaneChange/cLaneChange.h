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
#ifndef _LANECHANGE_H_
#define _LANECHANGE_H_

#define OID_ADTF_LANECHANGE_FILTER "adtf.aadc.LaneChange"

class cLaneChange : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_LANECHANGE_FILTER, "LaneChange", adtf::OBJCAT_DataFilter);

protected:
    /* INPUT PINS */

	// Start
	cInputPin m_oStart;
	cObjectPtr<IMediaTypeDescription> m_pDescStart;

	// Distance over all
	cInputPin m_oDistanceOverall;
	cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;

	// input steering
	cInputPin    m_oInputSteering;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInputSteering;

	// input speed
	cInputPin m_oInputSpeed;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInputSpeed;

	// input ultrasonic struct
	cInputPin    m_oInputUsStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;

	cCriticalSection m_critSecMinimumUsValue;

    std::vector<tBufferID> m_szIdUsStructValues;
    std::vector<tBufferID> m_szIdUsStructTss;
    tBool	 m_szIdsUsStructSet;


    /* OUTPUT PINS */

	// output steering
	cOutputPin    m_oOutputSteering;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteering;

	// output acceleration
	cOutputPin m_oOutputSpeed;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSpeed;

	// output turnSignalLeftEnabled
	cOutputPin m_oOutputTurnSignalLeftEnabled;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnSignalLeftEnabled;

	// output turnSignalRightEnabled
	cOutputPin m_oOutputTurnSignalRightEnabled;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnSignalRightEnabled;

	// output FinishFlag
	cOutputPin m_oOutputFinishFlag;
	cObjectPtr<IMediaTypeDescription> m_pDescFinishFlag;

	/* MEMBER VARIABLES INPUT*/

	tBool m_bStart;
	tFloat32 m_fSpeedInput;
	tFloat32 m_fSteeringInput;

	/* DEBUG */
	tBool m_bDebugModeEnabled;

	/* MEMBER VARIABLES PROCESS*/

	tTimeStamp timestamp;

	// US struct
	tFloat32 m_aUSSensors[10];

	// distance over all
	tFloat32 m_fDistanceOverall;
	tFloat32 m_fDistStart;

	/* MEMBER VARIABLES OUTPUT*/
	tFloat32 m_fSteeringOutput;
	tFloat32 m_fSpeedOutput;
	tBool m_bTurnSignalLeftEnabled;
	tBool m_bTurnSignalRightEnabled;
	tBool m_bFinishFlag;

	// state of turn
	tInt16 m_iStateOfLaneChange;

	enum StateOfLaneChange{
        SOLG_NOSTART = 0,
        SOLG_CHANGELANE1,
        SOLG_CHANGELANE2,
        SOLG_WRONGLANE_CHECK_FR,
	SOLG_WRONGLANE_CHECK_R,
	SOLG_WRONGLANE_CHECK_RR,
        SOLG_CHANGEBACK1,
        SOLG_CHANGEBACK2,
        SOLG_FINISH      
    };

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cLaneChange(const tChar* __info);

    /*! default destructor */
    virtual ~cLaneChange();

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

	// read US struct
	tResult ProcessInputUS(IMediaSample* pMediaSample);

	// function to decide the maneuver
	tResult ProcessManeuver();

	// function to turn right or left
	tResult TurnRight();
	tResult TurnLeft();
	tResult TurnRightBack();
	tResult TurnLeftBack();

	tResult ReadProperties(const tChar* strPropertyName);

	tResult TransmitOutput();

	private:
		
		// Properties for change part 1
		tFloat32    m_fPropDist1,m_fPropSpeed1,m_fPropSteer1;

		// Properties for change part 2
		tFloat32    m_fPropDist2,m_fPropSpeed2,m_fPropSteer2;
};

//*************************************************************************************************
#endif // _LANECHANGE_H_

/*!
*@}
*/
