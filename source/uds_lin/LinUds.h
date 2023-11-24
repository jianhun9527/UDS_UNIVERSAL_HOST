/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_lin\LinUds.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:16:52
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 00:52:23
 * @Description  : 
 ******************************************************************************/

#ifndef __LINUDS_H__
#define __LINUDS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "LinDrive.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define LIN_UDS_BUFFER_INVAILD      -50
#define LIN_UDS_BUFFER_OVFLW        -51
#define LIN_UDS_FORMAT_ERROR        -52
#define LIN_UDS_WRONG_SN            -53
#define LIN_UDS_REV_TIMEOUT         -54
#define LIN_UDS_INVALID_FC          -55
#define LIN_UDS_SEQUENCE_ERROR      -56
#define LIN_UDS_ERROR               -57

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 lin_uds_request(uds_data_t* _pdata);
extern tS16 lin_uds_response(uds_data_t* _pdata, tU16 _timeout_ms);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __LINUDS_H__
