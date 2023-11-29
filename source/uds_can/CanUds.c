/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\uds_can\CanUds.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:15:23
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-29 12:17:19
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

#define TP_RX_STAT_IDLE     0U
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

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 can_uds_request(uds_data_t* _pdata)
{
    extern comm_tool_t CommToolCnt;
    CAN_UDS_ADDR canuds;

    memset(&canuds, 0, sizeof(CAN_UDS_ADDR));
    canuds.ReqID = _pdata->ReqID;
    canuds.ResID = _pdata->ResID;
    canuds.Flag = _pdata->flag;
    canuds.AddrFormats = 0;
    canuds.AddrExt = 0;
    canuds.MaxDLC = 8;

    return CAN_UDS_Request(CommToolCnt.mHand, 0, &canuds, _pdata->pData, _pdata->DataLen);
}

tS16 can_uds_response(uds_data_t* _pdata, tU16 _timeout_ms)
{
    extern comm_tool_t CommToolCnt;
    CAN_UDS_ADDR canuds;
    tS16 retval;

    memset(&canuds, 0, sizeof(CAN_UDS_ADDR));
    canuds.ReqID = _pdata->ReqID;
    canuds.ResID = _pdata->ResID;
    canuds.Flag = _pdata->flag;
    canuds.AddrFormats = 0;
    canuds.AddrExt = 0;
    canuds.MaxDLC = 8;

    retval = CAN_UDS_Response(CommToolCnt.mHand, 0, &canuds, _pdata->pData, _timeout_ms);
    if (retval > 0) {
        if (retval > _pdata->DataLen) {
            return CAN_UDS_BUFFER_OVFLW;
        } else {
            _pdata->DataLen = retval;
            retval = 0;
        }
    }

    return retval;
}

/* End of file */
