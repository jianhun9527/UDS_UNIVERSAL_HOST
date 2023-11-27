/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\drive\LinDrive.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-21 23:16:36
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-28 01:00:31
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "LinDrive.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define DEFAULT_DATA_TRANSM_TIMEOUT         50
#define COMMAND_TYPE_NONE                   0x00
#define COMMAND_TYPE_START                  0x01
#define COMMAND_TYPE_BASE_DATA              0x02
#define COMMAND_TYPE_FILTER                 0x03
#define COMMAND_TYPE_DATA                   0x10
#define COMMAND_TYPE_RESET                  0x20

#define USB_STATE_HID_CLOSE                 0x00
#define USB_STATE_HID_OPEN                  0x01
#define USB_STATE_CDC_OPEN                  0x02
#define USB_STATE_CDC_CLOSE                 0x03

#define DEVICE_TYPE_BASIC_VER               0x00
#define DEVICE_TYPE_PRO_VER                 0x01

#define CMD_MEAN_HEARTBEAT                  0x00
#define CMD_MEAN_FORCE_UPDATE               0x01

#define LIN_STATE_MASTER                    0x00
#define LIN_STATE_SLAVE                     0x01

#define ERR_FRAME_UNMASK                    0x00
#define ERR_FRAME_MASK                      0x01

#define SEND_DISPLAY_CLOSE                  0x00
#define SEND_DISPLAY_OPEN                   0x01

#define RECV_DISPLAY_CLOSE                  0x00
#define RECV_DISPLAY_OPEN                   0x01

#define DEL_USB_CONFIG                      0x01

#define SET_FILTER_CONFIG                   0x00
#define GET_FILTER_CONFIG                   0x01

#define SET_LIN_FRAME_STATE                 0x00
#define GET_LIN_FRAME_STATE                 0x01

#define LIN_STATE_MASTER_CMD                0x00
#define LIN_STATE_SLAVE_SEND                0x10
#define LIN_STATE_SLAVE_RECV                0x20

#define LIN_MASTER_STATE_SYNC               0x00
#define LIN_MASTER_STATE_SEND               0x01
#define LIN_MASTER_STATE_RECV               0x02

#define LIN_SLAVE_SEND_DISABLE              0x00
#define LIN_SLAVE_SEND_ENABLE               0x01

#define LIN_CHECK_MODE_STANDARD             0x00
#define LIN_CEECK_MODE_ENHANCE              0x01
#define LIN_CHECK_MODE_CUSTOM               0x02
#define LIN_CHECK_MODE_SALVE                0xFF

#define LIN_MASTER_SEND_SUCCESS             0x00
#define LIN_MASTER_SEND_STANDARD_CHECKSUM   0x01
#define LIN_MASTER_SEND_ENHANCE_CHECKSUM    0x02
#define LIN_MASTER_SEND_CUSTOM_CHECKSUM     0x03
#define LIN_MASTER_SEND_BREAK_ERROR         0x04
#define LIN_MASTER_SEND_SYNC_ERROR          0x05
#define LIN_MASTER_SEND_DECT_ERROR          0x06
#define LIN_MASTER_SEND_NCK_ERROR           0x07
#define LIN_MASTER_SEND_LEN_ERROR           0x08

#define LIN_MASTER_RECV_SUCCESS             0x00
#define LIN_MASTER_RECV_STANDARD_CHECKSUM   0x01
#define LIN_MASTER_RECV_ENHANCE_CHECKSUM    0x02
#define LIN_MASTER_RECV_CUSTOM_CHECKSUM     0x03
#define LIN_MASTER_RECV_CHECKSUM_ERROR      0x04
#define LIN_MASTER_RECV_BREAK_ERROR         0x05
#define LIN_MASTER_RECV_SYNC_ERROR          0x06
#define LIN_MASTER_RECV_TIMEOUT             0x07
#define LIN_MASTER_RECV_LEN_ERROR           0x08

#define LIN_SLAVE_SEND_SUCCESS              0x20
#define LIN_SLAVE_SEND_STANDARD_CHECKSUM    0x21
#define LIN_SLAVE_SEND_ENHANCE_CHECKSUM     0x22
#define LIN_SLAVE_SEND_CUSTOM_CHECKSUM      0x23
#define LIN_SLAVE_SEND_NO_CHECKSUM          0x24
#define LIN_SLAVE_SEND_COLLISION_ERROR      0x25
#define LIN_SLAVE_SEND_NCK_ERROR            0x26
#define LIN_SLAVE_SEND_ID_ERROR             0x05

#define LIN_SLAVE_RECV_NCK_ERROR            0x01
#define LIN_SLAVE_RECV_ONLY_SYNC            0x02
#define LIN_SLAVE_RECV_DATA_ERROR           0x03
#define LIN_SLAVE_RECV_SYNC_INTERVAL        0x04
#define LIN_SLAVE_RECV_ID_ERROR             0x05
#define LIN_SLAVE_RECV_LEN_ERROR            0x06
#define LIN_SLAVE_RECV_COSTOM_CHECKOUT      0x10
#define LIN_SLAVE_RECV_STANDARD_CHECKSUM    0x11
#define LIN_SLAVE_RECV_ENHANCE_CHECKSUM     0x12
#define LIN_SLAVE_RECV_CHECKSUM_ERROR       0x13

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
    tU8 msgData[28];
    struct {
        tU8 command;
        tU8 reserve0;
        tU8 usbstate;
        tU8 devicetype;
        tU8 firmwarever;
    } Start;
    struct {
        tU8 command;
        tU8 cmdmean;
        tU8 linstate;
        tU8 errmask;
        tU8 senddisp;
        tU8 recvdisp;
        tU32 setbaudrate;
        tU32 heartbeat;
        tU32 measbaudrate;
        tU8 deletecfg;
    } basedata;
    struct {
        tU8 command;
        tU8 config;
        tU8 mask[8];
    } filter;
    struct {
        tU8 command;
        tU8 frame_state;
        tU8 register_cnt;
        tU8 lin_state;
        tU8 master_state;
        tU8 slave_state;
        tU8 id;
        tU8 crc_check_status;
        tU8 dlc;
        tU8 data[8];
        tU8 crc;
        tU32 cycle_ms;
        tU8 cycle_status;
        tU8 master_result;
        tU8 slave_result;
    } datacmd;
    struct {
        tU8 command;
    } reset;
} lin_frame_t;
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
static comm_tool_t CommToolCnt = {0};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 set_device_baud(lin_status_t _state, tU16 _baudrate);
static tS16 set_lin_state(lin_status_t _state, tU16 _baudrate);
static tS16 set_device_hid_comm(void);
static tS16 set_device_filter(void);
static tU8 get_lin_checksum(tU8* _pdata, tU8 _nsize);
static tU8 get_lin_pid(tU8 _id);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
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
        wsum = (wsum & 0x00FF) + (wsum >> 8);
    }

    return ~(tU8)(wsum & 0xff);
}

static tS16 set_device_hid_comm(void)
{
    lin_frame_t sendframe, recvframe;
    DWORD now_time, last_time;
    tU8 len;

    memset(&sendframe, 0, sizeof(lin_frame_t));
    memset(&recvframe, 0, sizeof(lin_frame_t));

    len = CommToolCnt.ComDataFifo.Size;
    sendframe.Start.command = COMMAND_TYPE_START;
    sendframe.Start.usbstate = USB_STATE_HID_OPEN;
    sendframe.Start.devicetype = DEVICE_TYPE_BASIC_VER;
    if (comm_tool_send_data(&CommToolCnt, sendframe.msgData, len)) goto __FIAL_CFG_HID;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) goto __FIAL_CFG_HID;
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, recvframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_START == recvframe.Start.command && USB_STATE_HID_OPEN == recvframe.Start.usbstate) {
            if (DEVICE_TYPE_PRO_VER == recvframe.Start.devicetype) {
                LOG_INF("Device version: ZM_USB_LIN Pro V%d", recvframe.Start.firmwarever);
            } else {
                LOG_INF("Device version: ZM_USB_LIN V%d", recvframe.Start.firmwarever);
            }
            break;
        } else if (COMMAND_TYPE_BASE_DATA == recvframe.Start.command) {
            Sleep(0);
            continue;
        } else {
            goto __FIAL_CFG_HID;
        }
    }
    LOG_INF("Successfully enabled HID communication!");

    return 0;

__FIAL_CFG_HID:
    LOG_ERR("Failed to enable HID communication!");
    return -1;
}

static tS16 set_device_baud(lin_status_t _state, tU16 _baudrate)
{
    lin_frame_t sendframe, recvframe;
    DWORD now_time, last_time;
    tU8 len;

    memset(&sendframe, 0, sizeof(lin_frame_msg_t));
    memset(&recvframe, 0, sizeof(lin_frame_msg_t));

    len = CommToolCnt.ComDataFifo.Size;
    sendframe.basedata.command = COMMAND_TYPE_BASE_DATA;
    sendframe.basedata.cmdmean = CMD_MEAN_HEARTBEAT;
    sendframe.basedata.linstate = _state == Lin_Master ? LIN_STATE_MASTER : LIN_STATE_SLAVE;
    sendframe.basedata.errmask = ERR_FRAME_UNMASK;
    sendframe.basedata.senddisp = SEND_DISPLAY_OPEN;
    sendframe.basedata.recvdisp = RECV_DISPLAY_OPEN;
    sendframe.basedata.setbaudrate = _baudrate;
    sendframe.basedata.heartbeat = 1000;
    sendframe.basedata.measbaudrate = 0;
    sendframe.basedata.deletecfg = 0;
    if (comm_tool_send_data(&CommToolCnt, sendframe.msgData, len)) goto __FAIL_CFG_BAUD;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) goto __FAIL_CFG_BAUD;
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, recvframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_BASE_DATA == recvframe.basedata.command) {
            for (len = 1; len < 14; len ++)
            {
                if (sendframe.msgData[len] != recvframe.msgData[len]) break;
            }
            if (len == 14) break;
        } else {
            goto __FAIL_CFG_BAUD;
        }
    }
    LOG_INF("Successfully configuring LIN status and baud rate!");

    return 0;

__FAIL_CFG_BAUD:
    LOG_ERR("Failed to configure LIN status and baud rate!");
    return -1;
}

static tS16 set_device_filter(void)
{
    lin_frame_t sendframe, recvframe;
    DWORD now_time, last_time;
    tU8 len;

    memset(&sendframe, 0, sizeof(lin_frame_msg_t));
    memset(&recvframe, 0, sizeof(lin_frame_msg_t));

    len = CommToolCnt.ComDataFifo.Size;
    sendframe.filter.command = COMMAND_TYPE_FILTER;
    sendframe.filter.config = SET_FILTER_CONFIG;
    // mask 3C -> 0x3C >> 3 = 7, 1 << (0x3C & 0x7) = 0x10;
    // mask 3C -> 0x3D >> 3 = 7, 1 << (0x3D & 0x7) = 0x20;
    sendframe.filter.mask[0] = 0x00;
    sendframe.filter.mask[1] = 0x00;
    sendframe.filter.mask[2] = 0x00;
    sendframe.filter.mask[3] = 0x00;
    sendframe.filter.mask[4] = 0x00;
    sendframe.filter.mask[5] = 0x00;
    sendframe.filter.mask[6] = 0x00;
    sendframe.filter.mask[7] = 0x30;
    if (comm_tool_send_data(&CommToolCnt, sendframe.msgData, len)) goto __FAIL_CFG_FLT;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) goto __FAIL_CFG_FLT;
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, recvframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_FILTER == recvframe.filter.command) {
            for (len = 1; len < 10; len ++)
            {
                if (sendframe.msgData[len] != recvframe.msgData[len]) break;
            }
            if (len == 10) break;
        } else if (COMMAND_TYPE_BASE_DATA == recvframe.filter.command) {
            Sleep(0);
            continue;
        } else {
            goto __FAIL_CFG_FLT;
        }
    }

    LOG_INF("Successfully configuring acceptance mask!");

    return 0;

__FAIL_CFG_FLT:
    LOG_ERR("Failed to configure acceptance mask!");
    return -3;
}

static tS16 set_lin_state(lin_status_t _state, tU16 _baudrate)
{
    if (set_device_hid_comm()) return -1;
    if (set_device_baud(_state, _baudrate)) return -2;
    if (set_device_filter()) return -3;

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_lin_device_init(const config_file_t* _pCfg)
{
    memset(&CommToolCnt, 0, sizeof(CommToolCnt));

    CommToolCnt.CommToolNo = _pCfg->setCommToolNo;
    if (comm_tool_scan(&CommToolCnt, _pCfg->pDevicePort)) return -1;
    if (comm_tool_open(&CommToolCnt, 28)) return -2;

    LOG_INF("Start Configure lin device!");
    if (set_lin_state(Lin_Master, _pCfg->comInfo.comBaud.value)) return -3;
    printf("====================================================================\r\n");

    return 0;
}

tS16 set_lin_device_deinit(void)
{
    CommToolCnt.ComStateCtl.ReceEnable = FALSE;
    CommToolCnt.ComStateCtl.SendEnable = FALSE;
    LOG_INF("Successfully stop LIN communication!");
    return comm_tool_close(&CommToolCnt);
}

tS16 lin_frame_master_write(lin_frame_msg_t* _frame)
{
    lin_frame_t sendframe, recvframe;
    DWORD now_time, last_time;
    tU8 len;

    memset(&sendframe, 0, sizeof(lin_frame_t));
    memset(&recvframe, 0, sizeof(lin_frame_t));

    if (_frame->id >= 0x40) {
        LOG_ERR("LIN ID exceeds valid range: %d", _frame->id);
        return LIN_ID_EXCEEDS_VAILD_RANGE;
    }
    if (_frame->dlc > 8) {
        LOG_ERR("LIN sends more than 8 bytes in a single frame: %d", _frame->dlc);
        return LIN_SEND_BUFFER_OVERFLOW;
    }
    sendframe.datacmd.command = COMMAND_TYPE_DATA;
    sendframe.datacmd.lin_state = LIN_STATE_MASTER_CMD;
    sendframe.datacmd.frame_state = SET_LIN_FRAME_STATE;
    sendframe.datacmd.slave_state = LIN_SLAVE_SEND_DISABLE;
    sendframe.datacmd.master_state = LIN_MASTER_STATE_SEND;
    sendframe.datacmd.cycle_ms = 0;
    sendframe.datacmd.register_cnt = 0;
    sendframe.datacmd.cycle_status = 0;
    sendframe.datacmd.slave_result = 0;
    sendframe.datacmd.master_result = 0;

    sendframe.datacmd.id = _frame->id;
    sendframe.datacmd.dlc = _frame->dlc;
    memcpy(sendframe.datacmd.data, _frame->data, _frame->dlc);
    if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
        sendframe.datacmd.crc_check_status = LIN_CHECK_MODE_STANDARD;
    } else {
        sendframe.datacmd.crc_check_status = LIN_CEECK_MODE_ENHANCE;
    }
    if (sendframe.datacmd.id == 0x3C || sendframe.datacmd.id == 0x3D) {
        sendframe.datacmd.crc_check_status = LIN_CHECK_MODE_STANDARD;
    }
    _frame->pid = LinPidTab[_frame->id];

    if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
        sendframe.datacmd.crc = get_lin_checksum(_frame->data, _frame->dlc);
    } else {
        sendframe.datacmd.crc = get_lin_checksum(&_frame->pid, _frame->dlc + 1);
    }
    if (comm_tool_send_data(&CommToolCnt, sendframe.msgData, 28)) goto __FAIL_SEND_DATA;
    
    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) goto __FAIL_SEND_DATA;
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, recvframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_DATA == recvframe.datacmd.command) {
            if (sendframe.datacmd.id == recvframe.datacmd.id && GET_LIN_FRAME_STATE == recvframe.datacmd.frame_state &&
                sendframe.datacmd.register_cnt == recvframe.datacmd.register_cnt) {
                if (sendframe.datacmd.dlc == recvframe.datacmd.dlc && 
                    LIN_MASTER_SEND_SUCCESS == recvframe.datacmd.master_result &&
                    LIN_STATE_MASTER_CMD == recvframe.datacmd.lin_state &&
                    LIN_MASTER_STATE_SEND == recvframe.datacmd.master_state) {
                    break;
                } else {
                    goto __FAIL_SEND_DATA;
                }
            }
        } else if (COMMAND_TYPE_BASE_DATA == recvframe.datacmd.command) {
            Sleep(0);
            continue;
        } else {
            goto __FAIL_SEND_DATA;
        }
    }

    return 0;

__FAIL_SEND_DATA:
    return LIN_SEND_FAIL;
}

tS16 lin_frame_master_read(lin_frame_msg_t* _frame)
{
    lin_frame_t sendframe, recvframe;
    DWORD now_time, last_time;
    tU8 len;

    memset(&sendframe, 0, sizeof(sendframe));
    memset(&recvframe, 0, sizeof(recvframe));

    if (_frame->id >= 0x40) {
        LOG_ERR("LIN ID exceeds valid range: %d", _frame->id);
        return LIN_ID_EXCEEDS_VAILD_RANGE;
    }
    sendframe.datacmd.command = COMMAND_TYPE_DATA;
    sendframe.datacmd.lin_state = LIN_STATE_MASTER_CMD;
    sendframe.datacmd.frame_state = SET_LIN_FRAME_STATE;
    sendframe.datacmd.slave_state = LIN_SLAVE_SEND_DISABLE;
    sendframe.datacmd.master_state = LIN_MASTER_STATE_RECV;
    sendframe.datacmd.cycle_ms = 0;
    sendframe.datacmd.register_cnt = 0;
    sendframe.datacmd.cycle_status = 0;
    sendframe.datacmd.slave_result = 0;
    sendframe.datacmd.master_result = 0;

    sendframe.datacmd.id = _frame->id;
    sendframe.datacmd.dlc = 8;
    sendframe.datacmd.crc_check_status = LIN_CHECK_MODE_SALVE;
    _frame->pid = LinPidTab[_frame->id];
    if (comm_tool_send_data(&CommToolCnt, sendframe.msgData, 28)) goto __FAIL_RECV_DATA;

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) {
            recvframe.datacmd.dlc = 8;
            recvframe.datacmd.id = 0;
            break;
        }
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, recvframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_DATA == recvframe.datacmd.command) {
            if (sendframe.datacmd.id == recvframe.datacmd.id && GET_LIN_FRAME_STATE == recvframe.datacmd.frame_state &&
                sendframe.datacmd.register_cnt == recvframe.datacmd.register_cnt) {
                if (LIN_MASTER_STATE_RECV == recvframe.datacmd.master_state &&
                    LIN_STATE_MASTER_CMD == recvframe.datacmd.lin_state &&
                    (LIN_MASTER_RECV_STANDARD_CHECKSUM == recvframe.datacmd.master_result || 
                     LIN_MASTER_RECV_ENHANCE_CHECKSUM == recvframe.datacmd.master_result)) {
                    break;
                } else {
                    if (recvframe.datacmd.master_result == LIN_MASTER_RECV_TIMEOUT) continue;
                    goto __FAIL_RECV_DATA;
                }
            }
        } else if (COMMAND_TYPE_BASE_DATA == recvframe.datacmd.command) {
            Sleep(0);
            continue;
        } else {
            goto __FAIL_RECV_DATA;
        }
    }
    
    switch(recvframe.datacmd.master_result)
    {
    case LIN_MASTER_RECV_SUCCESS:
    case LIN_MASTER_RECV_TIMEOUT:
    break;
    case LIN_MASTER_RECV_STANDARD_CHECKSUM:
        _frame->crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
    break;
    case LIN_MASTER_RECV_ENHANCE_CHECKSUM:
        _frame->crc_check_status = LIN_DATA_ENHANCED_CHECKSUM;
    break;
    case LIN_MASTER_RECV_CUSTOM_CHECKSUM:
    case LIN_MASTER_RECV_CHECKSUM_ERROR:
        LOG_ERR("LIN data checksum failed!");
        return LIN_READ_CHECKSUM_ERR;
    break;
    default:
        return LIN_UNKNOW_ERR;
    break;
    }

    _frame->id = recvframe.datacmd.id;
    _frame->dlc = recvframe.datacmd.dlc;
    memcpy(_frame->data, recvframe.datacmd.data, _frame->dlc);
    _frame->pid = LinPidTab[_frame->id];

    return 0;

__FAIL_RECV_DATA:
    return LIN_READ_FAIL;
}

tS16 lin_frame_slave_write(lin_frame_msg_t* _frame, tU8 _state)
{
    lin_frame_t linframe;

    memset(&linframe, 0, sizeof(linframe));

    if (_frame->id >= 0x40) {
        LOG_ERR("LIN ID exceeds valid range: %d", _frame->id);
        return LIN_ID_EXCEEDS_VAILD_RANGE;
    }
    if (_frame->dlc > 8) {
        LOG_ERR("LIN sends more than 8 bytes in a single frame: %d", _frame->dlc);
        return LIN_SEND_BUFFER_OVERFLOW;
    }

    linframe.datacmd.command = COMMAND_TYPE_DATA;
    linframe.datacmd.lin_state = LIN_STATE_SLAVE_SEND;
    linframe.datacmd.frame_state = SET_LIN_FRAME_STATE;
    linframe.datacmd.slave_state = (_state == LIN_SLAVE_WRITE_ENABLE) ? 0x01 : 0x00;
    linframe.datacmd.master_state = LIN_MASTER_STATE_SYNC;
    linframe.datacmd.cycle_ms = 0;
    linframe.datacmd.register_cnt = 0;
    linframe.datacmd.cycle_status = 0;
    linframe.datacmd.slave_result = 0;
    linframe.datacmd.master_result = 0;

    linframe.datacmd.id = _frame->id;
    linframe.datacmd.dlc = _frame->dlc;
    memcpy(linframe.datacmd.data, _frame->data, _frame->dlc);
    if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
        linframe.datacmd.crc_check_status = LIN_CHECK_MODE_STANDARD;
    } else {
        linframe.datacmd.crc_check_status = LIN_CEECK_MODE_ENHANCE;
    }
    if (linframe.datacmd.id == 0x3C || linframe.datacmd.id == 0x3D) {
        linframe.datacmd.crc_check_status = LIN_CHECK_MODE_STANDARD;
    }
    _frame->pid = LinPidTab[_frame->id];

    if (_frame->crc_check_status == LIN_DATA_STANDARD_CHECKSUM) {
        linframe.datacmd.crc = get_lin_checksum(_frame->data, _frame->dlc);
    } else {
        linframe.datacmd.crc = get_lin_checksum(&_frame->pid, _frame->dlc + 1);
    }

    return comm_tool_send_data(&CommToolCnt, linframe.msgData, 28);
}

tS16 lin_frame_slave_read(lin_frame_msg_t* _frame)
{
    DWORD now_time, last_time;
    lin_frame_t linframe;
    tU8 len;

    memset(&linframe, 0, sizeof(lin_frame_t));

    last_time = GetTickCount();
    now_time = last_time;
    for (;;)
    {
        now_time = GetTickCount();
        if (now_time - last_time > 50) goto __FAIL_RECV_DATA;
        len = CommToolCnt.ComDataFifo.Size;
        if (comm_tool_rece_data(&CommToolCnt, linframe.msgData, &len)) {
            Sleep(0);
            continue;
        }
        if (COMMAND_TYPE_DATA == linframe.datacmd.command) {
            if (GET_LIN_FRAME_STATE == linframe.datacmd.frame_state && linframe.datacmd.register_cnt < 5) {
                if (LIN_STATE_SLAVE_RECV == linframe.datacmd.lin_state &&
                    (LIN_SLAVE_RECV_STANDARD_CHECKSUM == linframe.datacmd.slave_result || 
                     LIN_SLAVE_RECV_ENHANCE_CHECKSUM == linframe.datacmd.slave_result)) {
                    break;
                } else {
                    goto __FAIL_RECV_DATA;
                }
            }
        } else if (COMMAND_TYPE_BASE_DATA == linframe.datacmd.command) {
            Sleep(0);
            continue;
        } else {
            goto __FAIL_RECV_DATA;
        }
    }
    
    switch(linframe.datacmd.slave_result)
    {
    case LIN_SLAVE_RECV_STANDARD_CHECKSUM:
        _frame->crc_check_status = LIN_DATA_STANDARD_CHECKSUM;
    break;
    case LIN_SLAVE_RECV_ENHANCE_CHECKSUM:
        _frame->crc_check_status = LIN_DATA_ENHANCED_CHECKSUM;
    break;
    case LIN_MASTER_RECV_CUSTOM_CHECKSUM:
    case LIN_MASTER_RECV_CHECKSUM_ERROR:
        LOG_ERR("LIN data checksum failed!");
        return LIN_READ_CHECKSUM_ERR;
    break;
    default:
       return LIN_UNKNOW_ERR;
    break;
    }

    _frame->id = linframe.datacmd.id;
    _frame->dlc = linframe.datacmd.dlc;
    memcpy(_frame->data, linframe.datacmd.data, _frame->dlc);
    _frame->pid = LinPidTab[_frame->id];

    return 0;

__FAIL_RECV_DATA:
    return LIN_READ_FAIL;
}

tS16 wait_lin_frame_send_complete(void)
{
    while(FALSE == CommToolCnt.ComDataFifo.TXEmpty) Sleep(0);

    return 0;
}

/* End of file */
