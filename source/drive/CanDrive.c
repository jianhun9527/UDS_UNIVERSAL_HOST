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
 * @LastEditTime : 2023-11-29 02:06:57
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
comm_tool_t CommToolCnt = {0};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 set_can_baudrate(tS16 _mhand, tU16 _baudrate);
static tS16 set_can_recv_filter(tU32 _id);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 set_can_baudrate(tS16 _mhand, tU16 _baudrate)
{
    if ((_mhand & 0xFF000000) == 0x53000000) {
        CANFD_INIT_CONFIG canfd_config;

        if (CANFD_SUCCESS != CANFD_GetCANSpeedArg(_mhand, &canfd_config, _baudrate * 1000, )) {
            return -1;
        }
        canfd_config.Mode = 0;
        canfd_config.RetrySend = 1;  
        canfd_config.ISOCRCEnable = 1;
        canfd_config.ResEnable = 1;   
        canfd_config.NBT_BRP = 1;
        canfd_config.NBT_SEG1 = 63;
        canfd_config.NBT_SEG2 = 16;
        canfd_config.NBT_SJW = 16;
        canfd_config.DBT_BRP = 1;
        canfd_config.DBT_SEG1 = 15;
        canfd_config.DBT_SEG2 = 4;
        canfd_config.DBT_SJW = 4;
        if (CANFD_SUCCESS != CANFD_Init(_mhand, 0, &canfd_config)) {
            return -1;
        }
    } else {
        CAN_INIT_CONFIG can_config;
        
        if (CAN_SUCCESS != CAN_GetCANSpeedArg(_mhand, &can_config, _baudrate * 1000)) {
            return -2;
        }
        CAN_GetCANSpeedArg(DevHandle[0],&CANConfig,500000);
        can_config.CAN_Mode = 0x80;//正常模式，接入终端电阻
        can_config.CAN_ABOM = 0;//禁止自动离线
        can_config.CAN_NART = 0;//使能报文重传
        can_config.CAN_RFLM = 0;//FIFO满之后覆盖旧报文
        can_config.CAN_TXFP = 1;//发送请求决定发送顺序
        //配置波特率，配置为500K
        can_config.CAN_BRP = 4;
        can_config.CAN_BS1 = 15;
        can_config.CAN_BS2 = 5;
        can_config.CAN_SJW = 2;
        ret = CAN_Init(device->handle[device->indexSlave],device->indexMaster,&can_config);
        if(ret != CAN_SUCCESS){
            return 0;
        }else{
        }
    }

    return comm_tool_send_data(&CommToolCnt, buff, 11);
}

static tS16 set_can_recv_filter(tU32 _id)
{
    tU8 filtertab[10] = {0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    tU8 idx = 0;
    
    for (idx = 0; idx < 14; idx ++)
    {
        filtertab[3] = idx;
        if (0 == idx) {
            filtertab[4] = 1;
            filtertab[6] = (tU8)(_id >> 0);
            filtertab[7] = (tU8)(_id >> 8);
            filtertab[8] = (tU8)(_id >> 16);
            filtertab[9] = (tU8)(_id >> 24);
        } else {
            filtertab[4] = 0;
            filtertab[6] = 0;
            filtertab[7] = 0;
            filtertab[8] = 0;
            filtertab[9] = 0;
        }
        if (comm_tool_send_data(&CommToolCnt, filtertab, 10)) break;
    }
    if (idx != 14) return -1;
    filtertab[2] = 0x06;

    return comm_tool_send_data(&CommToolCnt, filtertab, 3);
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
    if (set_can_recv_filter(_pCfg->udsCfg.resDiagID)) {
        LOG_ERR("Failed to configure can id filtering!");
        return -5;
    }
    LOG_INF("Configure can id filtering      -> Enable");
    LOG_INF("Configure can terminal resistor -> Enable");
    LOG_INF("Configure can baud rate         -> %dk", _pCfg->comInfo.comBaud.value);
    LOG_INF("TCANLINPro Configure completed!");
    printf("====================================================================\r\n");

    return 0;
}

tS16 set_can_device_deinit(void)
{
    CommToolCnt.ComStateCtl.ReceEnable = FALSE;
    CommToolCnt.ComStateCtl.SendEnable = FALSE;
    LOG_INF("Successfully stop CAN communication!");
    return comm_tool_close(&CommToolCnt);
}

tS16 can_frame_send(can_frame_msg_t* frame)
{
    can_frame_t canframe;

    if (INVALID_HANDLE_VALUE != CommToolCnt.mHand) {
        memset(canframe.msgData, 0, sizeof(canframe));
        canframe.msg.command = 1;

        if (frame->dlc > 8) {
            LOG_ERR("CAN sends more than 8 bytes in a single frame: %d", frame->dlc);
            return CAN_SEND_BUFFER_OVERFLOW;
        }
        canframe.msg.dlc = frame->dlc;

        if (CAN_ID_TYPE_EXTENDED == frame->id_type) {
            canframe.msg.extender_id = frame->id;
            canframe.msg.id_type = 1;
        } else {
            canframe.msg.standard_id = frame->id;
            canframe.msg.id_type = 0;
        }

        if (CAN_FRAME_TYPE_REMOTE == frame->frame_type) {
            LOG_ERR("Remote frames are temporarily not supported!");
            return CAN_SEND_REMOTE_FUN_NS;
        } else {
            canframe.msg.frame_type = 0;
        }

        memcpy(canframe.msg.data, frame->data, frame->dlc);
        
        return comm_tool_send_data(&CommToolCnt, canframe.msgData, 20);
    } else {
        LOG_ERR("Can device is not initialized or initialization failed!");
        return CAN_DERIVE_INITIAL_FAIL;
    }
}

tS16 can_frame_read(can_frame_msg_t* frame)
{
    can_frame_t canframe;
    tU8 recvsize = 20;

    if (INVALID_HANDLE_VALUE != CommToolCnt.mHand) {
        memset(&canframe, 0, sizeof(canframe));
        if (!comm_tool_rece_data(&CommToolCnt, canframe.msgData, &recvsize) && recvsize == 20) {
            if (canframe.msg.command == 1) {
                if (canframe.msg.id_type == CAN_ID_TYPE_STANDARD) {
                    frame->id_type = CAN_ID_TYPE_STANDARD;
                    frame->id = canframe.msg.standard_id;
                } else {
                    frame->id_type = CAN_ID_TYPE_EXTENDED;
                    frame->id = canframe.msg.extender_id;
                }

                if (canframe.msg.frame_type == CAN_FRAME_TYPE_DATA) {
                    frame->frame_type = CAN_FRAME_TYPE_DATA;
                } else {
                    frame->frame_type = CAN_FRAME_TYPE_REMOTE;
                }

                frame->dlc = canframe.msg.dlc;
                memcpy(frame->data, canframe.msg.data, frame->dlc);
            } else {
                LOG_ERR("CAN read frame format error!");
                return CAN_READ_FRAME_FORMAT_ERR;
            }
        } else {
            frame->dlc = 8U;
        }
    } else {
        LOG_ERR("Can device is not initialized or initialization failed!");
        return CAN_DERIVE_INITIAL_FAIL;
    }

    return 0;
}

tS16 wait_can_frame_send_complete(void)
{
    while (FALSE == CommToolCnt.ComDataFifo.TXEmpty) Sleep(0);
    
    return 0;
}

/* End of file */
