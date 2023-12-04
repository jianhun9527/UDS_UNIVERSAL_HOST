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
 * @LastEditTime : 2023-11-30 20:29:49
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
typedef union can_frame
{
    tU8 msgData[sizeof(cando_frame_t)];
    cando_frame_t msg;
} can_frame_t;

typedef enum can_baud_no
{
    can_baud_5000,
    can_baud_10000,
    can_baud_20000,
    can_baud_50000,
    can_baud_100000,
    can_baud_125000,
    can_baud_250000,
    can_baud_500000,
    can_baud_800000,
    can_baud_1000000,
    can_baud_list_max
} can_baud_no_t;

typedef struct cando_timing_stru {
    tU16 baudrate;
    can_baud_no_t no;
    cando_bittiming_t timing;
} cando_timing_stru_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static cando_timing_stru_t candotiming[can_baud_list_max] = {
    {5,  can_baud_5000,  {1, 12, 2, 1, 600}},
    {10, can_baud_10000, {1, 12, 2, 1, 300}},
    {20, can_baud_20000, {1, 12, 2, 1, 150}},
    {50, can_baud_50000,   {1, 12, 2, 1, 60}},
    {100, can_baud_100000, {1, 12, 2, 1, 30}},
    {125, can_baud_125000, {1, 12, 2, 1, 24}},
    {250, can_baud_250000, {1, 12, 2, 1, 12}},
    {500, can_baud_500000, {1, 12, 2, 1, 6}},
    {800, can_baud_800000, {1, 13, 5, 1, 3}},
    {1000, can_baud_1000000, {1, 10, 4, 1, 3}},
};
static comm_tool_t CommToolDev = {0};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tS16 set_can_baudrate(cando_handle _mhand, tU16 _baudrate);
static tS16 set_can_start(cando_handle _mhand);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tS16 set_can_baudrate(cando_handle _mhand, tU16 _baudrate)
{
    tU8 idx;

    for (idx = 0; idx < can_baud_list_max; idx ++)
    {
        if (candotiming[idx].baudrate == _baudrate) {
            if (false == cando_set_timing(_mhand, &candotiming[idx].timing)) {
                return TOOL_CONFIG_BAUD_RATE_FAIL;
            }
            break;
        }
    }
    if (can_baud_list_max == idx) return TOOL_BAUD_RATE_NOT_SUPPORT;
    LOG_INF("Configure can baud rate         -> %dk", _baudrate);

    return 0;
}

static tS16 set_can_start(cando_handle _mhand)
{
    tU32 fw_version, hw_version;
    wchar_t* pcando_serial_num;

    if (false == cando_start(_mhand, CANDO_MODE_NORMAL)) {
        return TOOL_CANDO_START_FAIL;
    }
    LOG_INF("Successfully to start the Cando device!");

    pcando_serial_num = cando_get_serial_number_str(_mhand);
    if (false == cando_get_dev_info(_mhand, &fw_version, &hw_version)) {
        LOG_ERR("Failed to get Cando device information!");
        return TOOL_GET_CANDO_INFO_FAIL;
    }
    LOG_INF("Cando Firmware Info:");
    LOG_INF("Cando Firmware serial number: %S", pcando_serial_num);
    LOG_INF("Cando Firmware Version: v%d.%d", (fw_version / 10), (fw_version % 10));
    LOG_INF("Cando Hardware Version: v%d.%d", (hw_version / 10), (hw_version % 10));

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_can_device_init(const config_file_t* _pCfg)
{
    memset(&CommToolDev, 0, sizeof(CommToolDev));

    CommToolDev.CommToolNo = _pCfg->setCommToolNo;
    if (comm_tool_scan(&CommToolDev, _pCfg->pDevicePort)) goto __DEVICE_INIT_FAIL;
    if (comm_tool_open(&CommToolDev, sizeof(can_frame_t))) goto __DEVICE_INIT_FAIL;

    LOG_INF("Start Configure CAN device!");
    if (set_can_baudrate(CommToolDev.mHand, _pCfg->comInfo.comBaud.value)) {
        LOG_ERR("Failed to configure can baud rate!");
        goto __DEVICE_INIT_FAIL;
    }
    if (set_can_start(CommToolDev.mHand)) {
        LOG_ERR("Failed to start Cando device!");
        goto __DEVICE_INIT_FAIL;
    }

    LOG_INF("Cando device Configure completed!");
    printf("====================================================================\r\n");

    return 0;

__DEVICE_INIT_FAIL:
    return CAN_DERIVE_INITIAL_FAIL;
}

tS16 set_can_device_deinit(void)
{
    CommToolDev.ComStateCtl.ReceEnable = FALSE;
    CommToolDev.ComStateCtl.SendEnable = FALSE;
    CommToolDev.ComStateCtl.RxErrState = 0;
    CommToolDev.ComStateCtl.TxErrState = 0;
    LOG_INF("Successfully stop CAN communication!");
    return comm_tool_close(&CommToolDev);
}

tS16 can_frame_send(can_frame_msg_t* _exframe)
{
    can_frame_t intframe;

    memset(&intframe, 0 , sizeof(can_frame_t));
    intframe.msg.can_id = _exframe->id;
    if (_exframe->id_type == CAN_ID_TYPE_EXTENDED) {
        intframe.msg.can_id |= CANDO_ID_EXTENDED;
    }
    if (_exframe->frame_type == CAN_FRAME_TYPE_REMOTE) {
        LOG_ERR("Remote frames are temporarily not supported!");
        return CAN_SEND_REMOTE_FUN_NS;
    }
    if (_exframe->dlc > 8) {
        LOG_ERR("CAN sends more than 8 bytes in a single frame: %d", _exframe->dlc);
        return CAN_SEND_BUFFER_OVERFLOW;
    }
    intframe.msg.can_dlc = _exframe->dlc;
    intframe.msg.flags = _exframe->flags;
    memcpy(intframe.msg.data, _exframe->data, _exframe->dlc);

    return comm_tool_send_data(&CommToolDev.ComDataFifo.TxMsgBuff, intframe.msgData, CommToolDev.ComDataFifo.Size);
}

tS16 can_frame_read(can_frame_msg_t* _exframe)
{
    can_frame_t intframe;

    memset(&intframe, 0, sizeof(can_frame_t));

    if (!comm_tool_rece_data(&CommToolDev.ComDataFifo.RxMsgBuff, intframe.msgData, CommToolDev.ComDataFifo.Size)) {
        if (0 != CommToolDev.ComStateCtl.RxErrState) {
            CommToolDev.ComStateCtl.RxErrState = 0;
            return CAN_RECV_FAIL;
        }
        _exframe->id = intframe.msg.can_id & CANDO_ID_MASK;
        _exframe->dlc = intframe.msg.can_dlc;
        _exframe->flags = intframe.msg.flags;
        memcpy(_exframe->data, intframe.msg.data, _exframe->dlc);
        if (0 != (intframe.msg.can_id & CANDO_ID_EXTENDED)) {
            _exframe->id_type = CAN_ID_TYPE_EXTENDED;
        } else {
            _exframe->id_type = CAN_ID_TYPE_STANDARD;
        }
        if (0 != (intframe.msg.can_id & CANDO_ID_RTR)) {
            _exframe->frame_type = CAN_FRAME_TYPE_REMOTE;
        } else {
            _exframe->frame_type = CAN_FRAME_TYPE_DATA;
        }
    } else {
        _exframe->id = 0;
        _exframe->dlc = 8;
        _exframe->id_type = CAN_ID_TYPE_STANDARD;
    }

    return 0;
}

tS16 wait_can_frame_send_complete(void)
{
    for(;; Sleep(1))
    {
        if (CommToolDev.ComStateCtl.TxErrState != 0) {
            CommToolDev.ComStateCtl.TxErrState = 0;
            return CAN_SEND_FAIL;
        } else {
            if (TRUE == CommToolDev.ComDataFifo.TXEmpty) break;
        }
    }
    
    return 0;
}

/* End of file */
