/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\uds_lin\LinUds.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:17:02
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-29 18:33:20
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

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 lin_uds_request(uds_data_t* _pdata)
{
    LIN_UDS_ADDR linuds;

    memset(&linuds, 0, sizeof(LIN_UDS_ADDR));
    linuds.ReqID = 0x3C;
    linuds.ResID = 0x3D;
    linuds.NAD = _pdata->ReqID;
    linuds.STmin = _pdata->STmin;
    linuds.CheckType = _pdata->flag;

    return LIN_UDS_Request(CommToolCnt.mHand, 0, &linuds, _pdata->pData, _pdata->DataLen);
}

tS16 lin_uds_response(uds_data_t* _pdata, tU16 _timeout_ms)
{
    LIN_UDS_ADDR linuds;
    tS16 retval;

    memset(&linuds, 0, sizeof(LIN_UDS_ADDR));
    linuds.ReqID = 0x3C;
    linuds.ResID = 0x3D;
    linuds.NAD = _pdata->ResID;
    linuds.STmin = _pdata->STmin;
    linuds.CheckType = _pdata->flag;

    retval = LIN_UDS_Response(CommToolCnt.mHand, 0, &linuds, _pdata->pData, _timeout_ms);
    if (retval > 0) {
        if (retval > _pdata->DataLen) {
            return LIN_UDS_BUFFER_OVFLW;
        } else {
            _pdata->DataLen = retval;
            retval = 0;
        }
    }

    return retval;
}

/* End of file */
