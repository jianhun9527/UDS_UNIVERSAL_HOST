/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\uds_can\CanUds.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:15:35
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-29 12:31:56
 * @Description  : 
 ******************************************************************************/

#ifndef __CANUDS_H__
#define __CANUDS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "CanDrive.h"
#include "can_uds.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define CAN_UDS_BUFFER_INVAILD      -40
// #define CAN_UDS_BUFFER_OVFLW        CAN_UDS_BUFFER_OVFLW
#define CAN_UDS_FORMAT_ERROR        CAN_UDS_UNEXP_PDU
// #define CAN_UDS_WRONG_SN            CAN_UDS_WRONG_SN
#define CAN_UDS_REV_TIMEOUT         CAN_UDS_TIMEOUT_Bs
#define CAN_UDS_INVALID_FC          CAN_UDS_INVALID_FS
#define CAN_UDS_SEQUENCE_ERROR      CAN_UDS_WFT_OVRN
// #define CAN_UDS_ERROR               CAN_UDS_ERROR

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 can_uds_request(uds_data_t* _pdata);
extern tS16 can_uds_response(uds_data_t* _pdata, tU16 _timeout_ms);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __CANUDS_H__
