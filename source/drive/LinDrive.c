/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uds_lin\LinDrive.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:16:36
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-14 13:10:05
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "LinDrive.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/
typedef enum lin_status
{
    Lin_Master = 0,
    Lin_Slave = 1,
} lin_status_t;

#pragma pack(1)
typedef union lin_data_list
{
    tU8 msgData[13];
    struct
    {
        tU8 command;
        tU8 dlc;
        tU8 id;   
        tU8 data[8];
        tU8 crc;
        tU8 crc_check_status;   // 0X01标准校验,0x02增强校验,如单片机返回0xFF,则校验失败
    } msg;
} lin_frame_t;

typedef union lin_slaver_write
{
    tU8 msgData[16];
    struct 
    {
        tU8 command;                // 固定为0x04
        tU8 sub_commond_01;         // 固定为0x02
        tU8 sub_commond_02;         // 固定为0x00
        tU8 lin_slave_write_enable; // 64个id写是独立的 如果要开启某一个从机反馈数据  0x01   关闭0x00
        tU8 dlc;                    // 数据区长度 一般8字节
        tU8 id;                     // 设备对应id  0-63（并非PID）
        tU8 data[8];                // 8字节数据
        tU8 crc;                    // CRC可以用我提供的上位机输入id+数据  选好校验模式得到
        tU8 crc_check_status;       // 0X01标准校验   0x02增强校验
    } msg;
} lin_slaver_write_t;
#pragma pack()

/*
bit7 = ~(bit1 ^ bit3 ^ bit4 ^ bit5);
bit6 = (bit0 ^ bit1 ^ bit2 ^ bit4);
*/
typedef union lin_pid
{
    tU8 pid;
    struct
    {
        tU8  bit0 : 1;
        tU8  bit1 : 1;
        tU8  bit2 : 1;
        tU8  bit3 : 1;
        tU8  bit4 : 1;
        tU8  bit5 : 1;
        tU8  bit6 : 1;
        tU8  bit7 : 1;
    } bit;
} lin_pid_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static const tU8 LinPidTab[64] = {
    0x80, 0xc1, 0x42, 0x03, 0xc4, 0x85, 0x06, 0x47, 0x08, 0x49, 0xca, 0x8b,
    0x4c, 0x0d, 0x8e, 0xcf, 0x50, 0x11, 0x92, 0xd3, 0x14, 0x55, 0xd6, 0x97,
    0xd8, 0x99, 0x1a, 0x5b, 0x9c, 0xdd, 0x5e, 0x1f, 0x20, 0x61, 0xe2, 0xa3,
    0x64, 0x25, 0xa6, 0xe7, 0xa8, 0xe9, 0x6a, 0x2b, 0xec, 0xad, 0x2e, 0x6f,
    0xf0, 0xb1, 0x32, 0x73, 0xb4, 0xf5, 0x76, 0x37, 0x78, 0x39, 0xba, 0xfb, 
    0x3c, 0x7d, 0xfe, 0xbf, 
};
static serial_port_t serialPort = {0};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 set_lin_master_slave_mode(lin_status_t _state);
static tS16 set_lin_baudrate(tU16 _baudrate);
static tU8 get_lin_pid(tU8 _id);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 set_lin_master_slave_mode(lin_status_t _state)
{
    tU8 buff[2] = {0x06, 0x00};

    buff[1] = _state == Lin_Master ? 0x03 : 0x04;

    return serial_port_send_data(&serialPort, buff, 2);
}

static tS16 set_lin_baudrate(tU16 _baudrate)
{
    tU8 buff[6] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00};

    switch (_baudrate)
    {
    case 19200:
        buff[2] = 0x00, buff[3] = 0x4B;
    break;
    case 14400:
        buff[2] = 0x40, buff[3] = 0x38;
    break;
    case 10417:
        buff[2] = 0xB1, buff[3] = 0x28;
    break;
    case 9600:
        buff[2] = 0x80, buff[3] = 0x25;
    break;
    case 4800:
        buff[2] = 0xC0, buff[3] = 0x12;
    break;
    case 2400:
        buff[2] = 0x60, buff[3] = 0x09;
    break;
    default:
        LOG_ERR("The baud rate configuration is wrong and the current baud rate cannot be set!");
        return -2;
    break;
    }

    return serial_port_send_data(&serialPort, buff, 6);
}

static tU8 get_lin_pid(tU8 _id)
{
    lin_pid_t pid_data;

    pid_data.pid = _id & 0x3F;
    pid_data.bit.bit7 = ~(pid_data.bit.bit1 ^ pid_data.bit.bit3 ^ pid_data.bit.bit4 ^ pid_data.bit.bit5);
    pid_data.bit.bit6 = (pid_data.bit.bit0 ^ pid_data.bit.bit1 ^ pid_data.bit.bit2 ^ pid_data.bit.bit4);

    return pid_data.pid;
}

static tU8 get_lin_checksum(tU8* _pdata, tU8 _nsize)
{
    tU16 wsum = 0;
 
    for (tU8 idx = 0; idx < _nsize; idx++)
    {
        wsum += _pdata[idx];
        if (wsum > 255)
        {
            wsum -= 255;
        }
    }

    return ~(tU8)(wsum & 0xff);
}
 
/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_lin_device_init(const tS8* _pcanname, tU16 _baudrate)
{
    memset(&serialPort, 0, sizeof(serialPort));

    if (serial_port_scan(&serialPort, _pcanname)) return -1;
    if (serial_port_open(&serialPort, 13)) return -2;

    LOG_INF("Start Configure lin device!");
    if (set_lin_master_slave_mode(Lin_Master)) {
        LOG_ERR("Failed to configure lin running mode!");
        return -3;
    }
    if (set_lin_baudrate(_baudrate)) {
        LOG_ERR("Failed to configure lin baud rate!");
        return -4;
    }
    LOG_INF("Configure lin running mode -> Master");
    LOG_INF("Configure lin baud rate    -> %d", _baudrate);
    printf("====================================================================\r\n");

    return 0;
}

tS16 set_lin_device_deinit(void)
{
    serialPort.ComStateCtl.ReceEnable = FALSE;
    serialPort.ComStateCtl.SendEnable = FALSE;
    LOG_INF("Successfully stop LIN communication!");
    return serial_port_close(&serialPort);
}

tS16 lin_frame_master_write(lin_frame_msg_t* _frame)
{
    lin_frame_t linframe;

    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
        memset(&linframe, 0, sizeof(linframe));
        linframe.msg.command = 2;
        if (_frame->id >= 0x40) {
            LOG_ERR("LIN ID exceeds valid range: %d", _frame->id);
            return LIN_ID_EXCEEDS_VAILD_RANGE;
        }
        if (_frame->dlc > 8) {
            LOG_ERR("LIN sends more than 8 bytes in a single frame: %d", _frame->dlc);
            return LIN_SEND_BUFFER_OVERFLOW;
        }
        linframe.msg.id = _frame->id;
        linframe.msg.dlc = _frame->dlc;
        memcpy(linframe.msg.data, _frame->data, _frame->dlc);
        if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
            linframe.msg.crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
        } else {
            linframe.msg.crc_check_status = LIN_DATA_ENHANCED_CHECKSUM;
        }
        if (linframe.msg.id == 0x3C || linframe.msg.id == 0x3D) {
            linframe.msg.crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
        }
        _frame->pid = LinPidTab[_frame->id];

        if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
            linframe.msg.crc = get_lin_checksum(_frame->data, _frame->dlc);
        } else {
            linframe.msg.crc = get_lin_checksum(&_frame->pid, _frame->dlc + 1);
        }

        return serial_port_send_data(&serialPort, linframe.msgData, 13);
    } else {
        LOG_ERR("Lin device is not initialized or initialization failed!");
        return LIN_DERIVE_INITIAL_FAIL;
    }
}

tS16 lin_frame_master_read(lin_frame_msg_t* _frame)
{
    lin_frame_t linframe;
    tU8 recvsize = 13;

    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
        memset(&linframe, 0, sizeof(linframe));
        linframe.msg.command = 2;
        linframe.msg.id = _frame->id;
        linframe.msg.dlc = 0;
        linframe.msg.crc_check_status = _frame->crc_check_status;
        if (serial_port_send_data(&serialPort , linframe.msgData, 13)) {
            LOG_ERR("LIN data reading failed!");
            return LIN_READ_FAIL;
        }

        memset(&linframe, 0, sizeof(linframe));
        while(serial_port_rece_data(&serialPort, linframe.msgData, &recvsize));
        if (recvsize != 13 || linframe.msg.command != 2) {
            LOG_ERR("LIN data format error!");
            return LIN_READ_FRAME_FORMAT_ERR;
        }
        switch (linframe.msg.crc_check_status)
        {
        case 0x01:
            _frame->crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
        break;
        case 0x02:
            _frame->crc_check_status = LIN_DATA_ENHANCED_CHECKSUM;
            if (_frame->id == 0x3C || _frame->id == 0x3D) {
                LOG_ERR("Enhanced verification is not supported for 3C or 3D!");
                return LIN_ERROR_CHECKSUM_METHOD;
            }
        break;
        case 0x03:
            linframe.msg.dlc = 8U;
        break;
        case 0xff:
            LOG_ERR("LIN data checksum failed!");
            return LIN_READ_CHECKSUM_ERR;
        break;
        default:
            return LIN_UNKNOW_ERR;
        break;
        }
    } else {
        LOG_ERR("Lin device is not initialized or initialization failed!");
        return LIN_DERIVE_INITIAL_FAIL;
    }

    _frame->id = linframe.msg.id;
    _frame->dlc = linframe.msg.dlc;
    memcpy(_frame->data, linframe.msg.data, _frame->dlc);
    _frame->pid = LinPidTab[_frame->id];

    return 0;
}

tS16 lin_frame_slave_write(lin_frame_msg_t* _frame, tU8 _state)
{
    lin_slaver_write_t linframe;

    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
        memset(&linframe, 0, sizeof(linframe));
        if (_frame->id >= 0x40) {
            LOG_ERR("LIN ID exceeds valid range: %d", _frame->id);
            return LIN_ID_EXCEEDS_VAILD_RANGE;
        }
        if (_frame->dlc > 8) {
            LOG_ERR("LIN sends more than 8 bytes in a single frame: %d", _frame->dlc);
            return LIN_SEND_BUFFER_OVERFLOW;
        }
        linframe.msg.lin_slave_write_enable = (_state == LIN_SLAVE_WRITE_ENABLE) ? 0x01 : 0x00;
        linframe.msg.command = 0x04;
        linframe.msg.sub_commond_01 = 0x02;
        linframe.msg.sub_commond_02 = 0x00;
        linframe.msg.dlc = _frame->dlc;
        linframe.msg.id = _frame->id;
        memcpy(linframe.msg.data, _frame->data, _frame->dlc);
        _frame->pid = LinPidTab[_frame->id];
        if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
            linframe.msg.crc = get_lin_checksum(_frame->data, _frame->dlc);
        } else {
            linframe.msg.crc = get_lin_checksum(&_frame->pid, _frame->dlc + 1);
        }

        return serial_port_send_data(&serialPort, linframe.msgData, 16);
    } else {
        LOG_ERR("Lin device is not initialized or initialization failed!");
        return LIN_DERIVE_INITIAL_FAIL;
    }
}

tS16 lin_frame_slave_read(lin_frame_msg_t* _frame)
{
    lin_frame_t linframe;
    tU8 recvsize = 13;

    memset(_frame, 0 , sizeof(lin_frame_msg_t));
    if (INVALID_HANDLE_VALUE != serialPort.mHand) {
        memset(&linframe, 0, sizeof(linframe));
        if (serial_port_rece_data(&serialPort, linframe.msgData, &recvsize)) {
            return LIN_READ_TIMEOUT;
        }
        if (recvsize == 0 && linframe.msg.command == 0) {
            return LIN_READ_FAIL;
        } else if (recvsize != 13 || linframe.msg.command != 2) {
            LOG_ERR("LIN data format error! %d", linframe.msg.command);
            return LIN_READ_FRAME_FORMAT_ERR;
        }
        switch (linframe.msg.crc_check_status)
        {
        case 0x01:
            _frame->crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
        break;
        case 0x02:
            _frame->crc_check_status = LIN_DATA_ENHANCED_CHECKSUM;
            if (_frame->id == 0x3C || _frame->id == 0x3D) {
                LOG_ERR("Enhanced verification is not supported for 3C or 3D!");
                return LIN_ERROR_CHECKSUM_METHOD;
            }
        break;
        case 0x03:
            LOG_ERR("LIN data acceptance timeout!");
            return LIN_READ_TIMEOUT;
        break;
        case 0xff:
            LOG_ERR("LIN data checksum failed!");
            return LIN_READ_CHECKSUM_ERR;
        break;
        default:
            return LIN_UNKNOW_ERR;
        break;
        }
    } else {
        LOG_ERR("Lin device is not initialized or initialization failed!");
        return LIN_DERIVE_INITIAL_FAIL;
    }

    _frame->id = linframe.msg.id;
    _frame->dlc = linframe.msg.dlc;
    memcpy(_frame->data, linframe.msg.data, _frame->dlc);
    _frame->pid = LinPidTab[_frame->id];

    return 0;
}

tS16 wait_lin_frame_send_complete(void)
{
    while(FALSE == serialPort.ComDataFifo.TXEmpty) Sleep(0);

    return 0;
}

/* End of file */
