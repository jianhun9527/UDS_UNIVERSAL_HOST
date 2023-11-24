/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_lin\LinDrive.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:16:23
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 00:11:53
 * @Description  : 
 ******************************************************************************/

#ifndef __LINDRIVE_H__
#define __LINDRIVE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "UartDrive.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define LIN_DATA_STANDARD_CHECKSUM      1
#define LIN_DATA_ENHANCED_CHECKSUM      2
#define LIN_SLAVE_WRITE_DISABLE         0
#define LIN_SLAVE_WRITE_ENABLE          1

#define LIN_SEND_FAIL                   -21
#define LIN_READ_FAIL                   -22
#define LIN_SEND_BUFFER_OVERFLOW        -23
#define LIN_ID_EXCEEDS_VAILD_RANGE      -24
#define LIN_READ_TIMEOUT                -25
#define LIN_READ_FRAME_FORMAT_ERR       -26
#define LIN_DERIVE_INITIAL_FAIL         -27
#define LIN_READ_CHECKSUM_ERR           -28
#define LIN_UNKNOW_ERR                  -29
#define LIN_ERROR_CHECKSUM_METHOD       -30

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef struct lin_data
{
   tU8 id;
   tU8 dlc;
   tU8 crc_check_status;
   tU8 pid;
   tU8 data[8];
} lin_frame_msg_t;

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 set_lin_device_init(const tS8* _pcanname, tU16 _baudrate);
extern tS16 set_lin_device_deinit(void);
extern tS16 lin_frame_master_write(lin_frame_msg_t* _frame);
extern tS16 lin_frame_master_read(lin_frame_msg_t* _frame);
extern tS16 lin_frame_slave_write(lin_frame_msg_t* _frame, tU8 _state);
extern tS16 lin_frame_slave_read(lin_frame_msg_t* _frame);
extern tS16 wait_lin_frame_send_complete(void);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __LINDRIVE_H__
