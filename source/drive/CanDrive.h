/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\drive\CanDrive.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:14:02
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-24 22:35:39
 * @Description  : 
 ******************************************************************************/

#ifndef __CANDRIVE_H__
#define __CANDRIVE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "FileParsing.h"
#include "ToolDrive.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define CAN_ID_TYPE_STANDARD            0
#define CAN_ID_TYPE_EXTENDED            1
#define CAN_FRAME_TYPE_DATA             0
#define CAN_FRAME_TYPE_REMOTE           1
#define CAN_TERMIAL_RESISTOR_DISABLE    0
#define CAN_TERMIAL_RESISTOR_ENABLE     1

#define CAN_SEND_FAIL                   -1
#define CAN_READ_FAIL                   -2
#define CAN_SEND_BUFFER_OVERFLOW        -3
#define CAN_SEND_REMOTE_FUN_NS          -4
#define CAN_READ_TIMEOUT                -5
#define CAN_READ_FRAME_FORMAT_ERR       -6
#define CAN_DERIVE_INITIAL_FAIL         -7

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef struct can_frame_msg
{
    tU32 id;
    tU8 dlc;
    tU8 id_type;
    tU8 frame_type;
    tU8 flags;
    tU8 data[8];
} can_frame_msg_t;

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 set_can_device_init(const config_file_t* _pCfg);
extern tS16 set_can_device_deinit(void);
extern tS16 can_frame_send(can_frame_msg_t* frame);
extern tS16 can_frame_read(can_frame_msg_t* frame);
extern tS16 wait_can_frame_send_complete(void);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __CANDRIVE_H__
