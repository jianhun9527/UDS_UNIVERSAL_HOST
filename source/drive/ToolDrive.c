/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\drive\ToolDrive.c
 * @Author       : jianhun
 * @CreationTime : 2023-11-24 22:31:02
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-30 21:57:15
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "ToolDrive.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static unsigned __stdcall ComRecvThread(void* lpparam);
static unsigned __stdcall ComSendThread(void* lpparam);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static unsigned __stdcall ComRecvThread(void* lpparam)
{
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;

    for(;pCtrl->ComStateCtl.ReceEnable;Sleep(1))
    {
        cando_frame_t frame = {0U, 0U, 0U, 0U, 0U, 0U, {0U}, 0U};

        if (false == cando_frame_read(pCtrl->mHand, &frame, 10)) continue;
        if (frame.can_id & CANDO_ID_ERR) {
            uint32_t err_code;
            uint8_t err_tx, err_rx;

            if (cando_parse_err_frame(&frame, &err_code, &err_tx, &err_rx)) {
                if (err_code & (CAN_ERR_BUSOFF | CAN_ERR_RX_TX_WARNING | CAN_ERR_RX_TX_PASSIVE)) {
                    pCtrl->ComStateCtl.RxErrState = TOOL_BUS_FAULT;
                }
                if (err_code & CAN_ERR_OVERLOAD) {
                    pCtrl->ComStateCtl.RxErrState = TOOL_BUS_OVERLOAD;
                }
                if (err_code & (CAN_ERR_STUFF | CAN_ERR_BIT_RECESSIVE | CAN_ERR_BIT_DOMINANT)) {
                    pCtrl->ComStateCtl.RxErrState = TOOL_BIT_ERR;
                }
                if (err_code & (CAN_ERR_FORM | CAN_ERR_CRC)) {
                    pCtrl->ComStateCtl.RxErrState = TOOL_BIT_ERR;
                }
                if (err_code & CAN_ERR_ACK) {
                    pCtrl->ComStateCtl.RxErrState = TOOL_BUS_ACK_ERR;
                }
            } else {
                pCtrl->ComStateCtl.RxErrState = TOOL_FAULT_PASING_ERR;
            }
            continue;
        }

        if (frame.echo_id != 0xFFFFFFFF) continue;
        if (set_fifo_write(&pCtrl->ComDataFifo.RxMsgBuff, (tU8*)&frame, pCtrl->ComDataFifo.Size)) {
            set_fifo_reset(&pCtrl->ComDataFifo.RxMsgBuff);
        }
    }

    return 0;
}

static unsigned __stdcall ComSendThread(void* lpparam)
{
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;

    for(;pCtrl->ComStateCtl.SendEnable;Sleep(1))
    {
        for(;;)
        {
            cando_frame_t frame = {0U, 0U, 0U, 0U, 0U, 0U, {0U}, 0U};

            if (get_fifo_is_empty(&pCtrl->ComDataFifo.TxMsgBuff)) {
                pCtrl->ComDataFifo.TXEmpty = TRUE;
                break;
            } else {
                pCtrl->ComDataFifo.TXEmpty = FALSE;
                if (get_fifo_read(&pCtrl->ComDataFifo.TxMsgBuff, (tU8*)&frame, pCtrl->ComDataFifo.Size)) {
                    set_fifo_reset(&pCtrl->ComDataFifo.TxMsgBuff);
                } else {
                    if (false == cando_frame_send(pCtrl->mHand, &frame)) {
                        pCtrl->ComStateCtl.TxErrState = TOOL_SEND_DATA_FAIL;
                    }
                }
            }
        }
    }

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 comm_tool_scan(comm_tool_t* _pctrl, const tS8* _pDevicePort)
{
    // 创建Cando设备接受列表句柄
    if (false == cando_list_malloc(&_pctrl->DeviceList)) {
        LOG_ERR("Failed to create Cando list handle!");
        return TOOL_MALLOC_MEMORY_FAIL;
    }
    // 扫描接入的Cando设备
    if (false == cando_list_scan(_pctrl->DeviceList)) {
        LOG_ERR("Failed to scan Cando device!");
        return TOOL_SCAN_DEVICE_FAIL;
    }
    // 获取接入的Cando设备数量
    if (false == cando_list_num(_pctrl->DeviceList, (tU8*)&_pctrl->CommToolCnt)) {
        LOG_ERR("Failed to get the number of connected Cando devices!");
        return TOOL_GET_DEVICE_NUM_FAIL;
    }

    LOG_INF("Cando device search completed!");
    if (_pctrl->CommToolCnt <= 0) {
        _pctrl->mHand = NULL;
        LOG_ERR("No Cando device connected!");
        return TOOL_NOT_CONNECTED;
    } else {
        LOG_INF("Cando Device List:");
        for (tU16 idx = 0; idx < _pctrl->CommToolCnt; idx++)
            LOG_INF("No: %d - Cando_device_%d", idx, idx);
        printf("====================================================================\r\n");
    }

	return 0;
}

tS16 comm_tool_open(comm_tool_t* _pctrl, tU8 _size)
{
    // 创建待使用Cando设备句柄
    if (false == cando_malloc(_pctrl->DeviceList, _pctrl->CommToolNo, &_pctrl->mHand)) {
        LOG_ERR("Failed to get Cando_device_%d handle of the Cando list handle!", _pctrl->CommToolNo);
        return TOOL_GET_DEVICE_HANDLE_FAIL;
    }
    // 打开cando设备
    if (false == cando_open(_pctrl->mHand)) {
        LOG_ERR("Failed opened the Cando_device_%d!", _pctrl->CommToolNo);
        return TOOL_OPEN_CANDO_DEVICE_FAIL;
    }

    LOG_INF("Successfully opened the Cando_device_%d!", _pctrl->CommToolNo);

    _pctrl->ComDataFifo.Size = (tU16)_size;
    if (set_fifo_init(&_pctrl->ComDataFifo.RxMsgBuff, _pctrl->ComDataFifo.Size, UART_RECV_BUFF_MAX)) {
        LOG_ERR("Failed to initialize serial port receiving buffer area!");
        return -8;
    }
    if (set_fifo_init(&_pctrl->ComDataFifo.TxMsgBuff, _pctrl->ComDataFifo.Size, UART_SEND_BUFF_MAX)) {
        LOG_ERR("Failed to initialize serial port receiving buffer area!");
        return -8;
    }
    LOG_INF("Successfully created communication buffer!");

    _pctrl->ComDataFifo.RxMsgBuff.MutexSign = CreateSemaphore(NULL, 1, 1, NULL);
    _pctrl->ComDataFifo.TxMsgBuff.MutexSign = CreateSemaphore(NULL, 1, 1, NULL);
    if (NULL == _pctrl->ComDataFifo.RxMsgBuff.MutexSign ||
        NULL == _pctrl->ComDataFifo.TxMsgBuff.MutexSign) {
        LOG_ERR("Failed to create communication mutex semaphore!");
        return -11;
    }

    _pctrl->ComStateCtl.ReceEnable = TRUE;
    _pctrl->ComStateCtl.SendEnable = TRUE;
    _pctrl->ComStateCtl.RxErrState = 0;
    _pctrl->ComStateCtl.TxErrState = 0;
    HANDLE mThRecv = (HANDLE)_beginthreadex(NULL, 0, ComRecvThread, _pctrl, 0, NULL);
    if (NULL == mThRecv) {
        LOG_ERR("Failed to create accepting function thread!");
        return -12;
    }
    CloseHandle(mThRecv);
    HANDLE mThSend = (HANDLE)_beginthreadex(NULL, 0, ComSendThread, _pctrl, 0, NULL);
    if (NULL == mThSend) {
        LOG_ERR("Failed to create accepting function thread!");
        return -12;
    }
    CloseHandle(mThSend);
    LOG_INF("Successfully created Cando receiving and sending threads!");

    return 0;
}

tS16 comm_tool_close(comm_tool_t* _pctrl)
{
    set_fifo_destroy(&_pctrl->ComDataFifo.RxMsgBuff);
    set_fifo_destroy(&_pctrl->ComDataFifo.TxMsgBuff);
    if (_pctrl->ComDataFifo.RxMsgBuff.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.RxMsgBuff.MutexSign);
    if (_pctrl->ComDataFifo.TxMsgBuff.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.TxMsgBuff.MutexSign);
    if (_pctrl->mHand != NULL) {
        cando_stop(_pctrl->mHand);
        cando_close(_pctrl->mHand);
        cando_free(_pctrl->mHand);
    }
    if (_pctrl->DeviceList != NULL) cando_list_free(_pctrl->DeviceList);

    LOG_INF("Successfully closed the Cando device!");

    return 0;
}

tS16 comm_tool_send_data(msg_fifo_t* _pbuff, tU8* _pdata, tU16 _size)
{
    return set_fifo_write(_pbuff, _pdata, _size);
}

tS16 comm_tool_rece_data(msg_fifo_t* _pbuff, tU8* _pdata, tU16 _size)
{
    return get_fifo_read(_pbuff, _pdata, _size);
}

/* End of file */
