/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_can\CanUds.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:15:35
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-12 23:10:08
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

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define CAN_UDS_BUFFER_INVAILD      -40
#define CAN_UDS_BUFFER_OVFLW        -41
#define CAN_UDS_FORMAT_ERROR        -42
#define CAN_UDS_WRONG_SN            -43
#define CAN_UDS_REV_TIMEOUT         -44
#define CAN_UDS_INVALID_FC          -45
#define CAN_UDS_SEQUENCE_ERROR      -46
#define CAN_UDS_ERROR               -47

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
