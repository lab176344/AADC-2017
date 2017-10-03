/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _XMLWRITER_FILTER_HEADER_
#define _XMLWRITER_FILTER_HEADER_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"


#define OID_ADTF_FILTER_DEF "adtf.user_xmlwriter" //unique for a filter
#define ADTF_FILTER_DESC "AADC User XML Writer"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "OpenCVTemplateFilter"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small OpenCV Template Filter \n$Rev:: 62948"

/*! Storage structure for the road sign data */
typedef struct _roadSign
    {
        /*! road sign */
        tInt16 u16Id;

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;

        /*! sign search radius */
        tFloat32 f32Radius;

        /*! direction (heading) of the road sign */
        tFloat32 f32Direction;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } roadSign;
	
/*! Storage structure for the Parking sign data */
typedef struct _parking
    {
        /*! Parking sign */
        tInt16 u16Id_parking;

        /*! location */
        tFloat32 f32X_parking;
        tFloat32 f32Y_parking;

        /*! sign search radius */
        tFloat32 f32status;

        /*! direction (heading) of the Parking sign */
        tFloat32 f32Direction_parking;

        tInt u16Cnt_parking;

        tTimeStamp u32ticks_parking;/*! measurement ticks*/

    } parking;
/*! Storage structure for the Obstacle sign data */
typedef struct _obstacle
    {
        
        /*! location */
        tFloat32 f32X_obstacle;
        tFloat32 f32Y_obstacle;

       
        tInt u16Cnt_obstacle;

        tTimeStamp u32ticks_obstacle;/*! measurement ticks*/

    } obstacle;
#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)

class cXMLWriter : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

protected:

	// Input pins
    cInputPin m_InputTrafficSign;
    cInputPin m_InputObstacle;
    cInputPin m_InputParkingSpace;
	
	//Traffic Sign input
    tBool m_TrafficSignInputSet;
    tBufferID m_tsI16id,m_tsF32X,m_tsF32Y,m_tsF32Angle;

    //Parking Input
    tBool m_ParkingInputSet;
    tBufferID m_parkingI16Id,m_parkingF32X,m_parkingF32Y,m_parkingUI16Status;

    //Obstacle Input
    tBool m_ObstacleInputSet;
    tBufferID m_obstacleF32X,m_obstacleF32Y;

    //Media Description
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionObstacle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionParkingSpace;

	tResult CreateInputPins(__exception = NULL);

	tResult ProcessInputTrafficSign(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);
	tResult ProcessInputParkingSpace(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);
	tResult ProcessInputObstacle(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);

	/* DOM Member Variables*/

	


	/* DOM Functions*/

	tResult CheckIfTrafficSignExistInXML(QDomDocument &io_domDoc, QDomElement &io_domRootElement, tInt16 i_iID, tFloat32 i_fX, tFloat32 i_fY, tFloat32 i_fRadius, tFloat32 i_fDirection);

	tResult DomAddElement(QDomDocument &io_domDoc, QDomElement &io_domRootElement, QString i_qstrString, tInt16 i_iID, tFloat32 i_fX, tFloat32 i_fY, tFloat32 i_fRadius, tFloat32 i_fDirection);
	
	tResult DomDeleteElement(QDomDocument &io_domDoc, tInt16 i_iID, tFloat32 i_fX, tFloat32 i_fY, tFloat32 i_fRadius, tFloat32 i_fDirection);

	tResult ReadXML(QDomDocument &o_docDocument);

	tResult WriteToXML(QDomDocument &i_docDocument);





public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    cXMLWriter(const tChar* __info);

    /*! default destructor */
    virtual ~cXMLWriter();

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
    *   \result Returns a standard result code.
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

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);


private: // private methods


};

/** @} */ // end of group

#endif  //_XMLWRITER_FILTER_HEADER_
