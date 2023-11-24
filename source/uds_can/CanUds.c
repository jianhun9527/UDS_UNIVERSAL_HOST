/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_can\CanUds.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:15:23
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 00:07:08
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "CanUds.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define TP_MTYPE_SF         0U
#define TP_MTYPE_FF         1U
#define TP_MTYPE_CF         2U
#define TP_MTYPE_FC         3U
#define TP_MTYPE_MAX        3U

#define TP_FRM_DLC_MIN      2U
#define TP_FRM_DLC_MAX      8U

#define TP_SF_DLC_MIN       2U
#define TP_FF_DLC_MIN       3U
#define TP_CF_DLC_MIN       2U
#define TP_FC_DLC_MIN       3U

#define TP_SF_DL_MAX        7U
#define TP_FF_DL_MIN        8U
#define TP_FF_DL_EFF        6U
#define TP_FF_DL_MAX        4096U

#define TP_FC_FS_CTS        0U
#define TP_FC_FS_WAIT       1U
#define TP_FC_FS_OVFLW      2U

#define TP_CF_SN_INIT       0U

#define TP_RX_STAT_IDLE 0U
#define TP_RX_STAT_FC_TRANS 1U
#define TP_RX_STAT_FC_CONF  2U
#define TP_RX_STAT_CF_RECV  3U

#define TP_TX_STAT_IDLE     0U
#define TP_TX_STAT_SF_CONF  1U
#define TP_TX_STAT_FF_CONF  2U
#define TP_TX_STAT_FC_RECV  3U
#define TP_TX_STAT_CF_TRANS 4U
#define TP_TX_STAT_CF_CONF  5U

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
typedef union can_tp_pdu
{
    tU8 Byte[8];
    union {
        struct {
            struct {
                tU8 SF_DL : 4;
                tU8 Mtype : 4;
            } PCI;
            tU8 Data[7];
        } SF;
        struct {
            struct {
                tU8 FF_DL_H : 4;
                tU8 Mtype   : 4;
                tU8 FF_DL_L : 8;
            } PCI;
            tU8 Data[6];
        } FF;
        struct {
            struct {
                tU8 SN : 4;
                tU8 Mtype : 4;
            } PCI;
            tU8 Data[7];
        } CF;
        struct {
            struct {
                tU8 FS : 4;
                tU8 Mtype : 4;
            } PCI;
            tU8 BS;
            tU8 STmin;
            tU8 Data[5];
        } FC;
    } Type;
} can_tp_pdu_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 CAN_TP_Receive(tU32 _id, tU8 _type, tU8* _pdata, tU16 _timeout_ms);
static tS16 CAN_TP_FCReceive(tU32 _id, tU8 _type, tU16* _pstmin, tU16 _timeout_ms);

static tS16 CAN_TP_SFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len);
static tS16 CAN_TP_FFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len);
static tS16 CAN_TP_CFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len, tU16 _stmin);
static tS16 CAN_TP_FCTrans(tU32 _id, tU8 _type, tU32* _pbufflen, tU16 _len);

static tS16 CAN_TP_SFReceive(can_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen);
static tS16 CAN_TP_FFReceive(can_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen);
static tS16 CAN_TP_CFReceive(can_tp_pdu_t* _ppdu, tU8* _pData, tU16 _len);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 CAN_TP_Receive(tU32 _id, tU8 _type, tU8* _pdata, tU16 _timeout_ms)
{
    can_frame_msg_t frame = {0};
    DWORD now_time, last_time;
    tS16 retval = 0;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        retval = can_frame_read(&frame);
        if (retval != 0) return retval;
        now_time = GetTickCount();
        if (frame.dlc != 8) return CAN_UDS_FORMAT_ERROR;
        if (now_time - last_time > _timeout_ms) return CAN_UDS_REV_TIMEOUT;
        if (((_type != 1U && frame.id_type == CAN_ID_TYPE_STANDARD) ||
             (_type == 1U && frame.id_type == CAN_ID_TYPE_EXTENDED)) &&
            (_id == frame.id)) break;
    }

    /* Copy the data to the transmission character. */
    memcpy(_pdata, frame.data, 8);

    return retval;
}

static tS16 CAN_TP_SFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len)
{
    can_frame_msg_t txframe = {0};
    can_tp_pdu_t* ptxpdu = NULL;

    txframe.id = _id;
    txframe.dlc = 8U;
    txframe.frame_type = CAN_FRAME_TYPE_DATA;
    txframe.id_type = _type ? CAN_ID_TYPE_EXTENDED : CAN_ID_TYPE_STANDARD;
    ptxpdu = (can_tp_pdu_t*)txframe.data;

    /* Configure message type. */
    ptxpdu->Type.SF.PCI.Mtype = TP_MTYPE_SF;
    /* Configure SF_DL. */
    ptxpdu->Type.SF.PCI.SF_DL = _len;
    /* Copy the data to the transmission PDU. */
    memcpy(ptxpdu->Type.SF.Data, _pdata, _len);
    /* Filling unused data. */
    for (tU16 idx = _len; idx < 7U; idx ++)
    {
        ptxpdu->Type.SF.Data[idx] = TP_PDU_PAD_DATA;
    }

    return can_frame_send(&txframe);
}

static tS16 CAN_TP_FFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len)
{
    can_frame_msg_t txframe = {0};
    can_tp_pdu_t* ptxpdu = NULL;

    txframe.id = _id;
    txframe.dlc = 8U;
    txframe.frame_type = CAN_FRAME_TYPE_DATA;
    txframe.id_type = _type ? CAN_ID_TYPE_EXTENDED : CAN_ID_TYPE_STANDARD;
    ptxpdu = (can_tp_pdu_t*)txframe.data;

    /* Configure message type. */
    ptxpdu->Type.FF.PCI.Mtype = TP_MTYPE_FF;
    /* Configure FF_DL. */
    ptxpdu->Type.FF.PCI.FF_DL_H = WORD_HI(_len);
    ptxpdu->Type.FF.PCI.FF_DL_L = WORD_LO(_len);
    /* Copy the data to the transmission PDU. */
    memcpy(ptxpdu->Type.FF.Data, _pdata, TP_FF_DL_EFF);
    
    return can_frame_send(&txframe);
}

static tS16 CAN_TP_CFTrans(tU32 _id, tU8 _type, tU8* _pdata, tU16 _len, tU16 _stmin)
{
    can_frame_msg_t txframe = {0};
    can_tp_pdu_t* ptxpdu = NULL;
    tS16 retval = 0, sendpos = 0;

    txframe.id = _id;
    txframe.dlc = 8U;
    txframe.frame_type = CAN_FRAME_TYPE_DATA;
    txframe.id_type = _type ? CAN_ID_TYPE_EXTENDED : CAN_ID_TYPE_STANDARD;
    ptxpdu = (can_tp_pdu_t*)txframe.data;

    /* Configure message type. */
    ptxpdu->Type.CF.PCI.Mtype = TP_MTYPE_CF;
    ptxpdu->Type.CF.PCI.SN = TP_CF_SN_INIT;
    for (;;)
    {
        /* Configure SN. */
        ptxpdu->Type.CF.PCI.SN ++;
        if (_len - sendpos >= TP_FF_DL_MIN) {
            /* Copy the data to the transmission PDU. */
            memcpy(ptxpdu->Type.CF.Data, _pdata + sendpos, 7U);
            sendpos += 7U;
        } else {
            /* Copy the data to the transmission PDU. */
            memcpy(ptxpdu->Type.CF.Data, _pdata + sendpos, _len - sendpos);
            /* Filling unused data. */
            for (tU16 idx = _len - sendpos; idx < 7; idx ++)
            {
                ptxpdu->Type.CF.Data[idx] = TP_PDU_PAD_DATA;
            }
            sendpos = _len;
        }
        
        /* Copy the data to the transmission character. */
        retval = can_frame_send(&txframe);
        if (retval != 0) return retval;
        if (sendpos >= _len) {
            retval = wait_can_frame_send_complete();
            break;
        }
        // Sleep(_stmin);
    }

    return retval;
}

static tS16 CAN_TP_FCTrans(tU32 _id, tU8 _type, tU32* _pbufflen, tU16 _len)
{
    can_frame_msg_t txframe = {0};
    can_tp_pdu_t* ptxpdu = NULL;

    txframe.id = _id;
    txframe.dlc = 8U;
    txframe.frame_type = CAN_FRAME_TYPE_DATA;
    txframe.id_type = _type ? CAN_ID_TYPE_EXTENDED : CAN_ID_TYPE_STANDARD;
    ptxpdu = (can_tp_pdu_t*)txframe.data;

    /* Configure message type. */
    ptxpdu->Type.FC.PCI.Mtype = TP_MTYPE_FC;
    /* Configure flow status. */
    if (_len > *_pbufflen) {
        ptxpdu->Type.FC.PCI.FS = TP_FC_FS_OVFLW;
        return CAN_UDS_BUFFER_OVFLW;
    } else {
        ptxpdu->Type.FC.PCI.FS = TP_FC_FS_CTS;
    }

    *_pbufflen = TP_FF_DL_EFF;
    /* Configure block size. */
    ptxpdu->Type.FC.BS = TP_BLOCK_SIZE;
    /* Configure stmin. */
    ptxpdu->Type.FC.STmin = TP_TMG_STmin;
    /* Unused bytes padding. */
    memset(ptxpdu->Type.FC.Data, TP_PDU_PAD_DATA, 5);

    /* Flow control frame transmission. */
    return can_frame_send(&txframe);
}

static tS16 CAN_TP_SFReceive(can_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen)
{
    if (_ppdu->Type.SF.PCI.SF_DL > *_plen) return CAN_UDS_BUFFER_OVFLW;
    if (_ppdu->Type.SF.PCI.SF_DL > TP_SF_DL_MAX) return CAN_UDS_FORMAT_ERROR;
    memcpy(_pdata, _ppdu->Type.SF.Data, _ppdu->Type.SF.PCI.SF_DL);
    *_plen = _ppdu->Type.SF.PCI.SF_DL;

    return 0;
}

static tS16 CAN_TP_FFReceive(can_tp_pdu_t* _ppdu, tU8* _pdata, tU16* _plen)
{
    *_plen = (tU16)(_ppdu->Type.FF.PCI.FF_DL_H << 8U) + (tU16)_ppdu->Type.FF.PCI.FF_DL_L;
    if (*_plen < TP_FF_DL_MIN) return CAN_UDS_FORMAT_ERROR;
    memcpy(_pdata, _ppdu->Type.FF.Data, TP_FF_DL_EFF);

    return 0;
}

static tS16 CAN_TP_CFReceive(can_tp_pdu_t* _ppdu, tU8* _pData, tU16 _len)
{
    static tU8 blockSN = 0;

    if (++ blockSN != _ppdu->Type.CF.PCI.SN) return CAN_UDS_WRONG_SN;
    memcpy(_pData, _ppdu->Type.CF.Data, _len);

    return 0;
}

static tS16 CAN_TP_FCReceive(tU32 _id, tU8 _type, tU16* _pstmin, tU16 _timeout_ms)
{
    can_tp_pdu_t tprxpdu = {0};
    tS16 retval = 0;

    for (;;)
    {
        retval = CAN_TP_Receive(_id, _type, (tU8*)&tprxpdu, _timeout_ms);
        if (retval != 0) return retval;
        if (tprxpdu.Type.FC.PCI.Mtype != TP_MTYPE_FC) return CAN_UDS_INVALID_FC;
        switch (tprxpdu.Type.FC.PCI.FS)
        {
        case TP_FC_FS_CTS:
            *_pstmin = tprxpdu.Type.FC.STmin;
            return 0;
        break;
        case TP_FC_FS_WAIT:
            _timeout_ms <<= 1U;
        break;
        case TP_FC_FS_OVFLW:
            return CAN_UDS_BUFFER_OVFLW;
        break;
        default:
            return CAN_UDS_INVALID_FC;
        break;
        }
    }

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 can_uds_request(uds_data_t* _pdata)
{
    tU16 stmin = TP_TMG_STmin;
    tS16 retval = 0;

    if (_pdata->pData == NULL) return CAN_UDS_BUFFER_INVAILD;
    if (_pdata->DataLen >= TP_FF_DL_MAX) {
        return CAN_UDS_BUFFER_OVFLW;
    } else if (_pdata->DataLen > TP_SF_DL_MAX) {
        retval = CAN_TP_FFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData, _pdata->DataLen);
        if (retval != 0) return retval;
        retval = CAN_TP_FCReceive(_pdata->ResID, _pdata->flag, &stmin, TP_TMG_N_CR);
        if (retval != 0) return retval;
        retval = CAN_TP_CFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData + TP_FF_DL_EFF,
            _pdata->DataLen - TP_FF_DL_EFF, stmin);
        if (retval != 0) return retval;
    } else {
        return CAN_TP_SFTrans(_pdata->ReqID, _pdata->flag, _pdata->pData, _pdata->DataLen);
    }

    return 0;
}

tS16 can_uds_response(uds_data_t* _pdata, tU16 _timeout_ms)
{
    tU8 laststate = TP_MTYPE_SF;
    can_tp_pdu_t rxpdu = {0};
    tU16 datalen = 0;
    tS16 retval = 0;

    if (_pdata->pData == NULL) return CAN_UDS_BUFFER_INVAILD;
    for (;;)
    {
        retval = CAN_TP_Receive(_pdata->ResID, _pdata->flag, (tU8*)&rxpdu, _timeout_ms);
        if (retval != 0) return retval;
        switch (rxpdu.Byte[0] >> 4U)
        {
        case TP_MTYPE_SF:
            if (laststate != TP_MTYPE_SF) return CAN_UDS_SEQUENCE_ERROR;
            return CAN_TP_SFReceive(&rxpdu, _pdata->pData, (tU16*)&_pdata->DataLen);
        break;
        case TP_MTYPE_FF:
            if (laststate != TP_MTYPE_SF) return CAN_UDS_SEQUENCE_ERROR;
            retval = CAN_TP_FFReceive(&rxpdu, _pdata->pData, &datalen);
            if (retval != 0) return retval;
            retval = CAN_TP_FCTrans(_pdata->ReqID, _pdata->flag, &_pdata->DataLen, datalen);
            if (retval != 0) return retval;
            laststate = TP_MTYPE_FF;
        break;
        case TP_MTYPE_CF:
            if ((laststate == TP_MTYPE_FF && _pdata->DataLen == TP_FF_DL_EFF) || 
                (laststate == TP_MTYPE_CF && _pdata->DataLen > TP_FF_DL_EFF)) {
                if (datalen - _pdata->DataLen > TP_SF_DL_MAX) {
                    retval = CAN_TP_CFReceive(&rxpdu, _pdata->pData + _pdata->DataLen, TP_SF_DL_MAX);
                    _pdata->DataLen += TP_SF_DL_MAX;
                } else {
                    retval = CAN_TP_CFReceive(&rxpdu, _pdata->pData + _pdata->DataLen, datalen - _pdata->DataLen);
                    _pdata->DataLen = datalen;
                }
                laststate = TP_MTYPE_CF;
                if (retval != 0) return retval;
                if (_pdata->DataLen == datalen) return 0;
            } else {
                return CAN_UDS_ERROR;
            }
        break;
        case TP_MTYPE_FC:
        default:
            return CAN_UDS_ERROR;
        break;
        }
    }

    return 0;
}

/* End of file */
