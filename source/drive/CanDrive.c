/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_can\CanDrive.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:13:48
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-19 17:54:01
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
serial_port_t serialPort = {0};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 set_can_terminal_resistor(tr_status_t _state);
static tS16 set_can_baudrate(tU16 _baudrate);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 set_can_terminal_resistor(tr_status_t _state)
{
    tU8 buff[2] = {0x06, 0x00};

    buff[1] = _state == Enable ? 0x01 : 0x02;

    return serial_port_send_data(&serialPort, buff, 2);
}

static tS16 set_can_baudrate(tU16 _baudrate)
{
    tU8 buff[11] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00};

    switch (_baudrate)
    {
    case 500:
        buff[2] = 0xF4, buff[3] = 0x01, buff[6] = 0x12;
    break;
    case 1000:
        buff[2] = 0xE8, buff[3] = 0x03, buff[6] = 0x09;
    break;
    case 100:
        buff[2] = 0x64, buff[6] = 0x5A;
    break;
    case 125:
        buff[2] = 0x7D, buff[6] = 0x48;
    break;
    case 200:
        buff[2] = 0xC8, buff[6] = 0x2D;
    break;
    case 250:
        buff[2] = 0xFA, buff[6] = 0x24;
    break;
    case 400:
        buff[2] = 0x90, buff[3] = 0x01, buff[6] = 0x12, buff[9] = 0x07;
    break;
    case 666:
        buff[2] = 0x9A, buff[3] = 0x02, buff[6] = 0x0B, buff[9] = 0x07;
    break;
    case 800:
        buff[2] = 0x20, buff[3] = 0x03, buff[6] = 0x09, buff[9] = 0x07;
    break;
    case 20:
        buff[2] = 0x14, buff[6] = 0xC2, buff[7] = 0x01;
    break;
    case 40:
        buff[2] = 0x28, buff[6] = 0xE1;
    break;
    case 50:
        buff[2] = 0x32, buff[6] = 0xB4;
    break;
    case 80:
        buff[2] = 0x50, buff[6] = 0x71;
    break;
    default:
        LOG_ERR("The baud rate configuration is wrong and the current baud rate cannot be set!");
        return -2;
    break;
    }

    return serial_port_send_data(&serialPort, buff, 11);
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_can_device_init(const tS8* pcanname, tS8 _trstate, tU16 _baudrate)
{
    memset(&serialPort, 0, sizeof(serialPort));

    if (serial_port_scan(&serialPort, pcanname)) return -1;
    if (serial_port_open(&serialPort, 20)) return -2;

    LOG_INF("Start Configure CAN device!");
    if (set_can_terminal_resistor((tr_status_t)_trstate)) {
        LOG_ERR("Failed to configure CAN terminal resistor!");
        return -3;
    }
    if (set_can_baudrate(_baudrate)) {
        LOG_ERR("Failed to configure can baud rate!");
        return -4;
    }
    if ((tr_status_t)_trstate == Enable) {
        LOG_INF("Configure can terminal resistor -> Enable");
    } else {
        LOG_INF("Configure can terminal resistor -> Disable");
    }
    LOG_INF("Configure can baud rate         -> %dk", _baudrate);
    printf("====================================================================\r\n");

    return 0;
}

tS16 set_can_device_deinit(void)
{
    serialPort.ComStateCtl.ReceEnable = FALSE;
    serialPort.ComStateCtl.SendEnable = FALSE;
    LOG_INF("Successfully stop CAN communication!");
    return serial_port_close(&serialPort);
}

tS16 can_frame_send(can_frame_msg_t* frame)
{
    can_frame_t canframe;

    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
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
        
        return serial_port_send_data(&serialPort, canframe.msgData, 20);
    } else {
        LOG_ERR("Can device is not initialized or initialization failed!");
        return CAN_DERIVE_INITIAL_FAIL;
    }
}

tS16 can_frame_read(can_frame_msg_t* frame)
{
    can_frame_t canframe;
    tU8 recvsize = 20;

    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
        memset(&canframe, 0, sizeof(canframe));
        if (!serial_port_rece_data(&serialPort, canframe.msgData, &recvsize) && recvsize == 20) {
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
    while (FALSE == serialPort.ComDataFifo.TXEmpty) Sleep(0);
    
    return 0;
}

/* End of file */
