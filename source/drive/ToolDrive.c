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
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-27 00:08:20
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "ToolDrive.h"
#include <hidapi.h>

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define USB_VID     0x1993
#define USB_PID     0x2021

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
static tU8 getdatachecksum(tU8* pdata, tU8 len);
static unsigned __stdcall ComRecvThread(void* lpparam);
static unsigned __stdcall ComSendThread(void* lpparam);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tU8 getdatachecksum(tU8* pdata, tU8 len)
{
    tU8 retval = 0;

    for (tU8 idx = 0U; idx < len; idx ++)
    {
        retval += pdata[idx];
    }

    return retval;
}

static unsigned __stdcall ComRecvThread(void* lpparam)
{
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;
    tU8 recvbuff[64];

    for(;pCtrl->ComStateCtl.ReceEnable;)
    {
        int retval = hid_read(pCtrl->mHand, recvbuff, 64);

        if (retval > 0) {
            for (tU8 idx = 0; idx < 64; idx ++)
            {
                if (recvbuff[idx] == 0x55 && recvbuff[idx + 1] == 0xAA && recvbuff[idx + 31] == 0x88 && 
                    recvbuff[idx + 30] == getdatachecksum(recvbuff + idx, 30)) {
                    if (recvbuff[idx + 2] != 0x00) {
                        if (set_fifo_write(&pCtrl->ComDataFifo.UartRxMsg, recvbuff + 2, pCtrl->ComDataFifo.Size)) {
                            set_fifo_reset(&pCtrl->ComDataFifo.UartRxMsg);
                        }
                    }
                    idx += 31U;
                }
            }
        } else {
            if (retval < 0) LOG_ERR("Recv data error: %ls!", hid_error(pCtrl->mHand));
            Sleep(0);
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
        tU8 data[] = { 0x00,
            0x55, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x88,
            0x55, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x88,
        };
        len = pCtrl->ComDataFifo.Size;
        if (get_fifo_read(&pCtrl->ComDataFifo.UartTxMsg, data + 3, &len)) {
            pCtrl->ComDataFifo.TXEmpty = TRUE;
            Sleep(0);
        } else {
            data[31] = getdatachecksum(data + 1, 30);
            len = pCtrl->ComDataFifo.Size;
            if (get_fifo_read(&pCtrl->ComDataFifo.UartTxMsg, data + 35, &len)) {
                pCtrl->ComDataFifo.TXEmpty = TRUE;
            } else {
                data[63] = getdatachecksum(data + 33, 30);
                pCtrl->ComDataFifo.TXEmpty = FALSE;
            }

            if (hid_write(pCtrl->mHand, data, 65) < 0) {
                LOG_ERR("Send data error: %ls!", hid_error(pCtrl->mHand));
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
    struct hid_device_info* usbinfo;

    (void)_pDevicePort;
    usbinfo = hid_enumerate(USB_VID, USB_PID);
    /*遍历所有信息并打印*/
    for(; usbinfo != NULL; usbinfo = usbinfo->next){
        sprintf(_pctrl->DeviceName[_pctrl->CommToolCnt], "%ls", usbinfo->serial_number);
        sprintf(_pctrl->DevicePort[_pctrl->CommToolCnt], "%s", usbinfo->path);
        _pctrl->CommToolCnt ++;
    }
    /*释放链表*/
    hid_free_enumeration(usbinfo);
    LOG_INF("ZM_USB_LIN device search completed!");
    if (_pctrl->CommToolCnt == 0) {
        _pctrl->mHand = INVALID_HANDLE_VALUE;
        LOG_ERR("No ZM_USB_LIN device connected!");
        return -1;
    } else {
        LOG_INF("ZM_USB_LIN Device List:");
        for (tU16 idx = 0; idx < _pctrl->CommToolCnt; idx++)
            LOG_INF("No: %d - %s", idx, _pctrl->DeviceName[idx]);
        printf("====================================================================\r\n");
    }

	return 0;
}

tS16 comm_tool_open(comm_tool_t* _pctrl, tU8 _size)
{
     _pctrl->mHand = hid_open_path(_pctrl->DevicePort[_pctrl->SetCommToolNo]);
    if (INVALID_HANDLE_VALUE == _pctrl->mHand) {
        LOG_ERR("Failed opened the ZM_USB_LIN device!");
        return -1;
    }
    LOG_INF("Successfully opened the ZM_USB_LIN device!");

    if (hid_set_nonblocking(_pctrl->mHand, 1)) {
        LOG_ERR("Failed to configure non-blocking mode!");
        return -2;
    }
    LOG_INF("Successfully configuring non-blocking mode!");

    _pctrl->ComDataFifo.Size = _size;
    if (set_fifo_init(&_pctrl->ComDataFifo.UartRxMsg, _pctrl->ComDataFifo.Size, 128U)) {
        LOG_ERR("Failed to initialize device receiving buffer area!");
        return -8;
    }
    if (set_fifo_init(&_pctrl->ComDataFifo.UartTxMsg, _pctrl->ComDataFifo.Size, 128U)) {
        LOG_ERR("Failed to initialize device receiving buffer area!");
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
    LOG_INF("Successfully created communication tool receiving and sending threads!");

    LOG_INF("Communication Tool configure completed!");

    return 0;
}

tS16 comm_tool_close(comm_tool_t* _pctrl)
{
    set_fifo_destroy(&_pctrl->ComDataFifo.UartRxMsg);
    set_fifo_destroy(&_pctrl->ComDataFifo.UartTxMsg);
    if (_pctrl->ComDataFifo.UartRxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartRxMsg.MutexSign);
    if (_pctrl->ComDataFifo.UartTxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartTxMsg.MutexSign);
    if (_pctrl->mHand != INVALID_HANDLE_VALUE) hid_close(_pctrl->mHand);
    (void)hid_exit();

    LOG_INF("Successfully closed the communication tool!");

    return 0;
}

tS16 comm_tool_send_data(comm_tool_t* _pctrl, tU8* _pdata, tU8 _size)
{
    tU16 len = _size;

    if (set_fifo_write(&_pctrl->ComDataFifo.UartTxMsg, _pdata, len))
        return -2;
    
    return 0;
}

tS16 comm_tool_rece_data(comm_tool_t* _pctrl, tU8* _pdata, tU8* _psize)
{
    tU16 len = *_psize;

    if (get_fifo_read(&_pctrl->ComDataFifo.UartRxMsg, _pdata, &len))
        return -2;
    *_psize = len;

    return 0;
}

/* End of file */
