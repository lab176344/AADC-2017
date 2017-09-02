/**
 *
 * ADTF Template Project Filter.
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
#ifndef _LPF_US_FILTER_H_
#define _LPF_US_FILTER_H_

#define OID_ADTF_LPF_US_FILTER "adtf.aadc.lpf_us"


//*************************************************************************************************
class LPFUSFilter : public adtf::cFilter {

    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LPF_US_FILTER, "LPF US Filter", OBJCAT_DataFilter, "LPF US Filter", 1, 0, 0, "Beta Version");

    cInputPin     m_oInputMeas;
    cOutputPin    m_oOutputFilter;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;


	cCriticalSection m_critSec;

    std::vector<tBufferID> m_szIdUsStructValues;
    std::vector<tBufferID> m_szIdUsStructTss;
    tBool	 m_szIdsUsStructSet;

public:
    LPFUSFilter(const tChar* __info);
    virtual ~LPFUSFilter();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

	tResult ProcessInputUS(IMediaSample* pMediaSample);

	tResult LPFUSFilter::TransmitOutput();

private:
    tFloat32 inputSample;
    tFloat32 buffer[10][20];
    tInt pointer;
	tFloat32 outputSample[10];
    tInt N;
	tTimeStamp timeStamp;
};

//*************************************************************************************************
#endif // _ESCAPER_FILTER_H_
