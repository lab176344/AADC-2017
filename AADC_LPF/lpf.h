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
#ifndef _LPF_FILTER_H_
#define _LPF_FILTER_H_

#define OID_ADTF_LPF_FILTER "adtf.aadc.lpf"


//*************************************************************************************************
class LPFFilter : public adtf::cFilter {

    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LPF_FILTER, "LPF Filter", OBJCAT_DataFilter, "LPF Filter", 1, 0, 0, "Beta Version");

    cInputPin     m_oInputMeas;
    cOutputPin    m_oOutputFilter;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;

public:
    LPFFilter(const tChar* __info);
    virtual ~LPFFilter();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:
    tFloat32 inputSample;
    tFloat32 buffer[1000];
    tInt pointer;
    tFloat32 outputSample;
    tInt N;
};

//*************************************************************************************************
#endif // _ESCAPER_FILTER_H_
