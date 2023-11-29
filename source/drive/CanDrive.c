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
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-29 16:31:34
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "usb2canfd.h"
#include "CanDrive.h"
#include "usb2can.h"

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
static tS16 set_can_baudrate(tS32 _mhand, tU16 _baudrate);
static tS16 set_can_recv_filter(tS32 _mhand, tU32 _id);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 set_can_baudrate(tS32 _mhand, tU16 _baudrate)
{
    if ((_mhand & 0xFF000000) == 0x53000000) {
        CANFD_INIT_CONFIG canfd_config;

        if (CANFD_SUCCESS != CANFD_GetCANSpeedArg(_mhand, &canfd_config, (tU32)_baudrate * 1000, 2000000)) {
            return -1;
        }
        canfd_config.Mode = 0;          // 正常模式
        canfd_config.RetrySend = 1;     // 无限制重发
        canfd_config.ISOCRCEnable = 1;  // 使能ISO CRC
        canfd_config.ResEnable = 1;     // 接入内部120欧终端电阻
        if (CANFD_SUCCESS != CANFD_Init(_mhand, 0, &canfd_config)) {
            return -2;
        }
    } else {
        CAN_INIT_CONFIG can_config;
        
        if (CAN_SUCCESS != CAN_GetCANSpeedArg(_mhand, &can_config, (tU32)_baudrate * 1000)) {
            return -3;
        }
        can_config.CAN_Mode = 0x80;     // 正常模式，接入终端电阻
        can_config.CAN_ABOM = 0;        // 禁止自动离线
        can_config.CAN_NART = 0;        // 使能报文重传
        can_config.CAN_RFLM = 0;        // FIFO满之后覆盖旧报文
        can_config.CAN_TXFP = 1;        // 发送请求决定发送顺序
        if (CAN_SUCCESS != CAN_Init(_mhand , 0, &can_config)) {
            return -4;
        }
    }

    LOG_INF("Configure can terminal resistor -> Enable");
    LOG_INF("Configure can baud rate         -> %dk", _baudrate);

    return 0;
}

static tS16 set_can_recv_filter(tS32 _mhand, tU32 _id)
{
    if ((_mhand & 0xFF000000) == 0x53000000) {
        CANFD_FILTER_CONFIG canfd_filter;

        canfd_filter.Enable = 1;
        canfd_filter.Index = 0;
        canfd_filter.ID_Accept = _id;
        canfd_filter.ID_Mask = 0xFFFFFFFF;
        if (CANFD_SUCCESS != CANFD_SetFilter(_mhand, 0, &canfd_filter, 1)) {
            return -1;
        }
    } else {
        CAN_FILTER_CONFIG can_filter;

        can_filter.Enable = 1;
        can_filter.ExtFrame = 0;
        can_filter.FilterIndex = 0;
        can_filter.FilterMode = 0;
        can_filter.ID_IDE = 0;
        can_filter.ID_RTR = 0;
        can_filter.ID_Std_Ext = _id;
        can_filter.MASK_IDE = 1;
        can_filter.MASK_RTR = 1;
        can_filter.MASK_Std_Ext = 0xFFFFFFFF;
        if (CAN_SUCCESS != CAN_Filter_Init(_mhand, 0, &can_filter)) {
            return -2;
        }
    }

    LOG_INF("Configure can id filtering      -> Enable");

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_can_device_init(const config_file_t* _pCfg)
{
    memset(&CommToolCnt, 0, sizeof(CommToolCnt));

    CommToolCnt.CommToolNo = _pCfg->setCommToolNo;
    if (comm_tool_scan(&CommToolCnt, _pCfg->pDevicePort)) return -1;
    if (comm_tool_open(&CommToolCnt, 0)) return -2;

    LOG_INF("Start Configure CAN device!");
    if (set_can_baudrate(CommToolCnt.mHand, _pCfg->comInfo.comBaud.value)) {
        LOG_ERR("Failed to configure can baud rate!");
        return -4;
    }

    if (set_can_recv_filter(CommToolCnt.mHand, _pCfg->udsCfg.resDiagID)) {
        LOG_ERR("Failed to configure can id filtering!");
        return -5;
    }
    
    LOG_INF("TCANLINPro Configure completed!");
    printf("====================================================================\r\n");

    return 0;
}

tS16 set_can_device_deinit(void)
{
    LOG_INF("Successfully stop CAN communication!");
    return comm_tool_close(&CommToolCnt);
}

tS16 can_frame_send(can_frame_msg_t* _frame)
{
    (void)_frame;

    return 0;
}

tS16 can_frame_read(can_frame_msg_t* _frame)
{
    (void)_frame;

    return 0;
}

tS16 wait_can_frame_send_complete(void)
{
    return 0;
}

/* End of file */
