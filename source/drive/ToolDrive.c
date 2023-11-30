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
 * @LastEditTime : 2023-11-30 00:19:02
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

    for(;pCtrl->ComStateCtl.ReceEnable;)
    {
        if (set_fifo_write(&pCtrl->ComDataFifo.UartRxMsg, NULL, pCtrl->ComDataFifo.Size)) {
            set_fifo_reset(&pCtrl->ComDataFifo.UartRxMsg);
        }
    }

    return 0;
}

static unsigned __stdcall ComSendThread(void* lpparam)
{
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;
    tU16 len = 0;

    for(;pCtrl->ComStateCtl.SendEnable;)
    {
        len = pCtrl->ComDataFifo.Size;
        if (get_fifo_read(&pCtrl->ComDataFifo.UartTxMsg, NULL, &len)) {
            pCtrl->ComDataFifo.TXEmpty = TRUE;
            Sleep(1);
        } else {
            pCtrl->ComDataFifo.TXEmpty = FALSE;
            set_fifo_reset(&pCtrl->ComDataFifo.UartTxMsg);
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
    if (false == cando_list_scan(&_pctrl->DeviceList)) {
        LOG_ERR("Failed to scan Cando device!");
        return TOOL_SCAN_DEVICE_FAIL;
    }
    // 获取接入的Cando设备数量
    if (false == cando_list_num(&_pctrl->DeviceList, &_pctrl->CommToolCnt)) {
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

    _pctrl->ComDataFifo.Size = _size + 1U;
    if (set_fifo_init(&_pctrl->ComDataFifo.UartRxMsg, _pctrl->ComDataFifo.Size, 128U)) {
        LOG_ERR("Failed to initialize serial port receiving buffer area!");
        return -8;
    }
    if (set_fifo_init(&_pctrl->ComDataFifo.UartTxMsg, _pctrl->ComDataFifo.Size, 128U)) {
        LOG_ERR("Failed to initialize serial port receiving buffer area!");
        return -8;
    }
    LOG_INF("Successfully created communication buffer!");

    _pctrl->ComDataFifo.UartRxMsg.MutexSign = CreateSemaphore(NULL, 1, 1, NULL);
    _pctrl->ComDataFifo.UartTxMsg.MutexSign = CreateSemaphore(NULL, 1, 1, NULL);
    if (NULL == _pctrl->ComDataFifo.UartRxMsg.MutexSign ||
        NULL == _pctrl->ComDataFifo.UartTxMsg.MutexSign) {
        LOG_ERR("Failed to create communication mutex semaphore!");
        return -11;
    }

    _pctrl->ComStateCtl.ReceEnable = TRUE;
    _pctrl->ComStateCtl.SendEnable = TRUE;
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

    LOG_INF("Cando Configure completed!");

    return 0;
}

tS16 comm_tool_close(comm_tool_t* _pctrl)
{
    set_fifo_destroy(&_pctrl->ComDataFifo.UartRxMsg);
    set_fifo_destroy(&_pctrl->ComDataFifo.UartTxMsg);
    if (_pctrl->ComDataFifo.UartRxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartRxMsg.MutexSign);
    if (_pctrl->ComDataFifo.UartTxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartTxMsg.MutexSign);
    if (_pctrl->mHand != NULL) {
        cando_stop(_pctrl->mHand);
        cando_close(_pctrl->mHand);
        cando_free(_pctrl->mHand);
    }
    if (_pctrl->DeviceList != NULL) cando_list_free(_pctrl->DeviceList);

    LOG_INF("Successfully closed the Cando device!");

    return 0;
}

tS16 comm_tool_send_data(comm_tool_t* _pctrl, tU8* _pdata, tU8 _size)
{
    tU8 data[32] = {0};
    
    memset(data, 0, 32);
    data[0] = _size;
    
    memcpy(data + 1, _pdata, _size);
    if (set_fifo_write(&_pctrl->ComDataFifo.UartTxMsg, data, _pctrl->ComDataFifo.Size))
        return -2;
    
    return 0;
}

tS16 comm_tool_rece_data(comm_tool_t* _pctrl, tU8* _pdata, tU8* _size)
{
    tU16 len = _pctrl->ComDataFifo.Size;
    tU8 data[32] = {0};

    memset(data, 0, 32);
    if (get_fifo_read(&_pctrl->ComDataFifo.UartRxMsg, data, &len))
        return -2;
    if (len != _pctrl->ComDataFifo.Size || *_size < data[0]) return -3;
    *_size = data[0];
    memcpy(_pdata, data + 1, *_size);

    return 0;
}

/* End of file */
