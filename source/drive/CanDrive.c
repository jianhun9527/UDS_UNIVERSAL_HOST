/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\drive\CanDrive.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:13:48
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-28 00:48:26
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "CanDrive.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/
typedef enum tr_status
{
    Disable = 0,
    Enable = 1
} tr_status_t;

#pragma pack(1)
typedef union can_frame
{
    tU8 msgData[20];
    struct 
    {
        tU8 command;
        tU32 standard_id;
        tU32 extender_id;
        tU8 id_type;        // 0x00: standard frame, 0x01: extended frame
        tU8 frame_type;     // 0x00: Data Frame, 0x01: remote frame
        tU8 dlc;
        tU8 data[8];
    } msg;
} can_frame_t;
#pragma pack()

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
tS16 set_can_device_init(const config_file_t* _pCfg)
{
    (void)_pCfg;
    
    LOG_ERR("This device does not support the current communication method!");
    return TOOL_FUNCTION_NOT_SUPPORT;
}

tS16 set_can_device_deinit(void)
{
    LOG_ERR("This device does not support the current communication method!");
    return TOOL_FUNCTION_NOT_SUPPORT;
}

tS16 can_frame_send(can_frame_msg_t* frame)
{
    (void)frame;

    LOG_ERR("This device does not support the current communication method!");
    return TOOL_FUNCTION_NOT_SUPPORT;
}

tS16 can_frame_read(can_frame_msg_t* frame)
{
    (void)frame;

    LOG_ERR("This device does not support the current communication method!");
    return TOOL_FUNCTION_NOT_SUPPORT;
}

tS16 wait_can_frame_send_complete(void)
{
    LOG_ERR("This device does not support the current communication method!");
    return TOOL_FUNCTION_NOT_SUPPORT;
}

/* End of file */
