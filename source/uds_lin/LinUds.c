/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_lin\LinUds.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:17:02
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 23:34:07
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "LinUds.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define TP_MTYPE_SF         0U
#define TP_MTYPE_FF         1U
#define TP_MTYPE_CF         2U
#define TP_MTYPE_MAX        3U

#define TP_FRM_DLC_MIN      2U
#define TP_FRM_DLC_MAX      8U

#define TP_SF_DLC_MIN       2U
#define TP_FF_DLC_MIN       3U
#define TP_CF_DLC_MIN       2U
#define TP_FC_DLC_MIN       3U

#define TP_SF_DL_MAX        6U
#define TP_FF_DL_MIN        7U
#define TP_FF_DL_EFF        5U
#define TP_FF_DL_MAX        4096U

#define TP_CF_SN_INIT       0U

#define TP_ERR_TIMEOUT_A    1U
#define TP_ERR_TIMEOUT_BS   2U
#define TP_ERR_TIMEOUT_CR   3U
#define TP_ERR_WRONG_SN     4U
#define TP_ERR_INVALID_FS   5U
#define TP_ERR_UNEXP_PDU    6U
#define TP_ERR_WFT_OVRN     7U
#define TP_ERR_BUF_OVFLW    8U

#define TP_PDU_PAD_DATA     0xFFU
#define TP_BLOCK_SIZE       0U
#define TP_WFT_MAX          8U

#define TP_TMG_STmin        20U
#define TP_TMG_N_AS         70U
#define TP_TMG_N_AR         70U
#define TP_TMG_N_BS         150U
#define TP_TMG_N_BR         10U
#define TP_TMG_N_CS         10U
#define TP_TMG_N_CR         150U

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/
typedef union lin_tp_pdu
{
    tU8 Byte[8];
    union {
        struct {
            tU8 NAD;
            struct {
                tU8 SF_DL : 4;
                tU8 Mtype : 4;
            } PCI;
            tU8 Data[6];
        } SF;
        struct {
            tU8 NAD;
            struct {
                tU8 FF_DL_H : 4;
                tU8 Mtype   : 4;
                tU8 FF_DL_L : 8;
            } PCI;
            tU8 Data[5];
        } FF;
        struct {
            tU8 NAD;
            struct {
                tU8 SN : 4;
                tU8 Mtype : 4;
            } PCI;
            tU8 Data[6];
        } CF;
    } Type;
} lin_tp_pdu_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 LIN_TP_Receive(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _timeout_ms);

static tS16 LIN_TP_SFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len);
static tS16 LIN_TP_FFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len);
static tS16 LIN_TP_CFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len, tU16 _stmin);

static tS16 LIN_TP_SFReceive(lin_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen);
static tS16 LIN_TP_FFReceive(lin_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen);
static tS16 LIN_TP_CFReceive(lin_tp_pdu_t* _ppdu, tU8* _pData, tU16 _len);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 LIN_TP_Receive(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _timeout_ms)
{
    lin_frame_msg_t frame = {0};
    DWORD now_time, last_time;
    tS16 retval = 0;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        frame.id = 0x3DU;
        frame.dlc = 8;
        frame.crc_check_status = 0;
        retval = lin_frame_master_read(&frame);
        if (retval != 0) return retval;
        now_time = GetTickCount();
        if (frame.dlc != 8) return LIN_UDS_FORMAT_ERROR;
        if (now_time - last_time > _timeout_ms) return LIN_UDS_REV_TIMEOUT;
        if (LIN_DATA_STANDARD_CHECKSUM != frame.crc_check_status || 0x3DU != frame.id) continue;
        if (frame.data[0] == _nad) break;
    }

    /* Copy the data to the transmission character. */
    memcpy(_pdata, frame.data, 8);

    return retval;
}

static tS16 LIN_TP_SFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len)
{
    lin_frame_msg_t txframe = {0};
    lin_tp_pdu_t* ptxpdu = NULL;

    txframe.id = 0x3CU;
    txframe.dlc = 8U;
    txframe.crc_check_status = _type ? LIN_DATA_ENHANCED_CHECKSUM : LIN_DATA_STANDARD_CHECKSUM;
    ptxpdu = (lin_tp_pdu_t*)txframe.data;

    /* Configure LIN NAD. */
    ptxpdu->Type.SF.NAD = _nad;
    
    /* Configure message type. */
    ptxpdu->Type.SF.PCI.Mtype = TP_MTYPE_SF;
    /* Configure SF_DL. */
    ptxpdu->Type.SF.PCI.SF_DL = _len;
    /* Copy the data to the transmission PDU. */
    memcpy(ptxpdu->Type.SF.Data, _pdata, _len);
    /* Filling unused data. */
    for (tU16 idx = _len; idx < 6U; idx ++)
    {
        ptxpdu->Type.SF.Data[idx] = TP_PDU_PAD_DATA;
    }

    return lin_frame_master_write(&txframe);
}

static tS16 LIN_TP_FFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len)
{
    lin_frame_msg_t txframe = {0};
    lin_tp_pdu_t* ptxpdu = NULL;

    txframe.id = 0x3CU;
    txframe.dlc = 8U;
    txframe.crc_check_status = _type ? LIN_DATA_ENHANCED_CHECKSUM : LIN_DATA_STANDARD_CHECKSUM;
    ptxpdu = (lin_tp_pdu_t*)txframe.data;

    /* Configure LIN NAD. */
    ptxpdu->Type.SF.NAD = _nad;

    /* Configure message type. */
    ptxpdu->Type.FF.PCI.Mtype = TP_MTYPE_FF;
    /* Configure FF_DL. */
    ptxpdu->Type.FF.PCI.FF_DL_H = WORD_HI(_len);
    ptxpdu->Type.FF.PCI.FF_DL_L = WORD_LO(_len);
    /* Copy the data to the transmission PDU. */
    memcpy(ptxpdu->Type.FF.Data, _pdata, TP_FF_DL_EFF);
    
    return lin_frame_master_write(&txframe);
}

static tS16 LIN_TP_CFTrans(tU8 _nad, tU8 _type, tU8* _pdata, tU16 _len, tU16 _stmin)
{
    lin_frame_msg_t txframe = {0};
    lin_tp_pdu_t* ptxpdu = NULL;
    tS16 retval = 0, sendpos = 0;

    txframe.id = 0x3CU;
    txframe.dlc = 8U;
    txframe.crc_check_status = _type ? LIN_DATA_ENHANCED_CHECKSUM : LIN_DATA_STANDARD_CHECKSUM;
    ptxpdu = (lin_tp_pdu_t*)txframe.data;

    /* Configure LIN NAD. */
    ptxpdu->Type.SF.NAD = _nad;

    /* Configure message type. */
    ptxpdu->Type.CF.PCI.Mtype = TP_MTYPE_CF;
    ptxpdu->Type.CF.PCI.SN = TP_CF_SN_INIT;
    for (;;)
    {
        /* Configure SN. */
        ptxpdu->Type.CF.PCI.SN ++;
        if (_len - sendpos >= TP_FF_DL_MIN) {
            /* Copy the data to the transmission PDU. */
            memcpy(ptxpdu->Type.CF.Data, _pdata + sendpos, 6U);
            sendpos += 6U;
        } else {
            /* Copy the data to the transmission PDU. */
            memcpy(ptxpdu->Type.CF.Data, _pdata + sendpos, _len - sendpos);
            /* Filling unused data. */
            for (tU16 idx = _len - sendpos; idx < 6; idx ++)
            {
                ptxpdu->Type.CF.Data[idx] = TP_PDU_PAD_DATA;
            }
            sendpos = _len;
        }
        
        /* Copy the data to the transmission character. */
        retval = lin_frame_master_write(&txframe);
        if (retval != 0) return retval;
        if (sendpos >= _len) {
            retval = wait_lin_frame_send_complete();
            break;
        }
        // Sleep(_stmin);
    }

    return retval;
}

static tS16 LIN_TP_SFReceive(lin_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen)
{
    if (_ppdu->Type.SF.PCI.SF_DL > *_plen) return LIN_UDS_BUFFER_OVFLW;
    if (_ppdu->Type.SF.PCI.SF_DL > TP_SF_DL_MAX) return LIN_UDS_FORMAT_ERROR;
    memcpy(_pdata, _ppdu->Type.SF.Data, _ppdu->Type.SF.PCI.SF_DL);
    *_plen = _ppdu->Type.SF.PCI.SF_DL;

    return 0;
}

static tS16 LIN_TP_FFReceive(lin_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen)
{
    *_plen = (tU16)(_ppdu->Type.FF.PCI.FF_DL_H << 8U) + (tU16)_ppdu->Type.FF.PCI.FF_DL_L;
    if (*_plen < TP_FF_DL_MIN) return LIN_UDS_FORMAT_ERROR;
    memcpy(_pdata, _ppdu->Type.FF.Data, TP_FF_DL_EFF);

    return 0;
}

static tS16 LIN_TP_CFReceive(lin_tp_pdu_t* _ppdu, tU8* _pData, tU16 _len)
{
    static tU8 blockSN = 0;

    if (++ blockSN != _ppdu->Type.CF.PCI.SN) return LIN_UDS_WRONG_SN;
    memcpy(_pData, _ppdu->Type.CF.Data, _len);

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 lin_uds_request(uds_data_t* _pdata)
{
    tU16 stmin = TP_TMG_STmin;
    tS16 retval = 0;

    if (_pdata->pData == NULL) return LIN_UDS_BUFFER_INVAILD;
    if (_pdata->DataLen >= TP_FF_DL_MAX) {
        return LIN_UDS_BUFFER_OVFLW;
    } else if (_pdata->DataLen > TP_SF_DL_MAX) {
        retval = LIN_TP_FFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData, _pdata->DataLen);
        if (retval != 0) return retval;
        retval = LIN_TP_CFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData + TP_FF_DL_EFF,
            _pdata->DataLen - TP_FF_DL_EFF, stmin);
        if (retval != 0) return retval;
    } else {
        return LIN_TP_SFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData, _pdata->DataLen);
    }

    return 0;
}

tS16 lin_uds_response(uds_data_t* _pdata, tU16 _timeout_ms)
{
    tU8 laststate = TP_MTYPE_SF;
    lin_tp_pdu_t rxpdu = {0};
    tU16 datalen = 0;
    tS16 retval = 0;

    if (_pdata->pData == NULL) return LIN_UDS_BUFFER_INVAILD;
    for (;;)
    {
        retval = LIN_TP_Receive(_pdata->ResID, _pdata->flag, (tU8*)&rxpdu, _timeout_ms);
        if (retval != 0) return retval;
        switch (rxpdu.Byte[1] >> 4U)
        {
        case TP_MTYPE_SF:
            if (laststate != TP_MTYPE_SF) return LIN_UDS_SEQUENCE_ERROR;
            return LIN_TP_SFReceive(&rxpdu, _pdata->pData, (tU16*)&_pdata->DataLen);
        break;
        case TP_MTYPE_FF:
            if (laststate != TP_MTYPE_SF) return LIN_UDS_SEQUENCE_ERROR;
            retval = LIN_TP_FFReceive(&rxpdu, _pdata->pData, &datalen);
            if (retval != 0) return retval;
            laststate = TP_MTYPE_FF;
        break;
        case TP_MTYPE_CF:
            if ((laststate == TP_MTYPE_FF && _pdata->DataLen == TP_FF_DL_EFF) || 
                (laststate == TP_MTYPE_CF && _pdata->DataLen > TP_FF_DL_EFF)) {
                if (datalen - _pdata->DataLen > TP_SF_DL_MAX) {
                    retval = LIN_TP_CFReceive(&rxpdu, _pdata->pData + _pdata->DataLen, TP_SF_DL_MAX);
                    _pdata->DataLen += TP_SF_DL_MAX;
                } else {
                    retval = LIN_TP_CFReceive(&rxpdu, _pdata->pData + _pdata->DataLen, datalen - _pdata->DataLen);
                    _pdata->DataLen = datalen;
                }
                laststate = TP_MTYPE_CF;
                if (retval != 0) return retval;
                if (_pdata->DataLen == datalen) return 0;
            } else {
                return LIN_UDS_ERROR;
            }
        break;
            return LIN_UDS_ERROR;
        break;
        }
    }

    return 0;
}

/* End of file */
