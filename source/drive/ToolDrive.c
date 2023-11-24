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
 * @LastEditTime : 2023-11-24 22:31:23
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "ToolDrive.h"
#include <setupapi.h>
#include <tchar.h>
#include <conio.h>

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
    DWORD WaitEventMask = 0, Bytes = 0, dwError = 0;
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;
    WINBOOL Status = FALSE;
    COMSTAT cs;

    tU8* pBuff = (tU8*)malloc(sizeof(tU8) * pCtrl->ComDataFifo.Size);
    if (pBuff == NULL) return -1;
    for(;pCtrl->ComStateCtl.ReceEnable;)
    {
        WaitEventMask = 0;
        Status = FALSE;
        pCtrl->ComOvlpEvent.pOvWaitEvent->Offset = 0;
        /* WaitCommEvent等待串口通信事件的发生
         * 用途: 用来判断用SetCommMask()函数设置的串口通信事件是否已发生。
         * 原型: BOOL WaitCommEvent(HANDLE hFile,LPDWORD lpEvtMask,LPOVERLAPPED lpOverlapped);
         * 参数说明：
         *    -hFile: 串口句柄
         *    -lpEvtMask: 函数执行完后如果检测到串口通信事件的话就将其写入该参数中。
         *    -lpOverlapped: 异步结构，用来保存异步操作结果。
        */
        if (FALSE == WaitCommEvent(pCtrl->mHand, &WaitEventMask, pCtrl->ComOvlpEvent.pOvWaitEvent)) {
            // GetLastError()函数返回ERROR_IO_PENDING,表明串口正在进行读操作
            if (ERROR_IO_PENDING == GetLastError())
                Status = GetOverlappedResult(pCtrl->mHand, pCtrl->ComOvlpEvent.pOvWaitEvent, &Bytes, TRUE);
        }

        // 在使用ReadFile 函数进行读操作前，应先使用ClearCommError函数清除错误。
        ClearCommError(pCtrl->mHand, &dwError, &cs);
        if ((FALSE != Status) && (WaitEventMask & EV_RXCHAR) && (cs.cbInQue > 0)) {
            pCtrl->ComOvlpEvent.pOvReadEvent->Offset = 0;
            memset(pBuff, 0, pCtrl->ComDataFifo.Size);
            if (0 != ReadFile(pCtrl->mHand, pBuff + 1, pCtrl->ComDataFifo.Size - 1U, &Bytes, pCtrl->ComOvlpEvent.pOvReadEvent)) {
                if (pCtrl->ComDataFifo.Size == Bytes + 1U) {
                    pBuff[0] = Bytes;
                    if (set_fifo_write(&pCtrl->ComDataFifo.UartRxMsg, pBuff, pCtrl->ComDataFifo.Size)) {
                        set_fifo_reset(&pCtrl->ComDataFifo.UartRxMsg);
                    }
                }
            }
        }
        PurgeComm(pCtrl->mHand, PURGE_RXCLEAR | PURGE_RXABORT);
    }
    free(pBuff);

    return 0;
}

static unsigned __stdcall ComSendThread(void* lpparam)
{
    comm_tool_t* pCtrl = (comm_tool_t*)lpparam;
    DWORD Bytes = 0, dwError = 0;
    tU16 len = 0;

    tU8* pBuff = (tU8*)malloc(sizeof(tU8) * pCtrl->ComDataFifo.Size);
    if (pBuff == NULL) return -1;
    for(;pCtrl->ComStateCtl.SendEnable;)
    {
        len = pCtrl->ComDataFifo.Size;
        if (get_fifo_read(&pCtrl->ComDataFifo.UartTxMsg, pBuff, &len)) {
            pCtrl->ComDataFifo.TXEmpty = TRUE;
            Sleep(1);
        } else {
            pCtrl->ComDataFifo.TXEmpty = FALSE;
            PurgeComm(pCtrl->mHand, PURGE_TXCLEAR | PURGE_TXABORT);
            pCtrl->ComOvlpEvent.pOvWriteEvent->Offset = 0;
            if (FALSE == WriteFile(pCtrl->mHand, pBuff + 1, pBuff[0], &Bytes, pCtrl->ComOvlpEvent.pOvWriteEvent) &&
                ERROR_IO_PENDING == GetLastError() &&
                FALSE != GetOverlappedResult(pCtrl->mHand, pCtrl->ComOvlpEvent.pOvWriteEvent, &Bytes, TRUE) &&
                NO_ERROR == GetLastError()) {
            } else {
                ClearCommError(pCtrl->mHand, &dwError, NULL);
                set_fifo_reset(&pCtrl->ComDataFifo.UartTxMsg);
            }
        }
    }
    free(pBuff);

    return 0;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 comm_tool_scan(comm_tool_t* _pctrl, const tS8* _pDevicePort)
{
    // 返回设备集句柄
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(NULL, NULL, NULL, DIGCF_PRESENT|DIGCF_ALLCLASSES);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        LOG_ERR("SetupDiGetClassDevs function Error: %ld!", GetLastError());
		return -2;
	};

    SP_DEVINFO_DATA spDevInfoData = {.cbSize = sizeof(SP_DEVINFO_DATA)};
    for (DWORD idx = 0; SetupDiEnumDeviceInfo(hDevInfo, idx, &spDevInfoData); idx++)
	{
        tU8 szBuf[MAX_PATH];

        // 读取设备类型，Ports
        memset(szBuf, 0, sizeof(szBuf));
        if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &spDevInfoData, SPDRP_CLASS, NULL, szBuf, MAX_PATH, 0))
            continue;
        if (strcmp((const tS8 *)szBuf, "Ports") != 0)
            continue;

        // 读取设备描述符
        tS8* szName = NULL;
        tS8 numbuf[4] = {0};
        
        memset(szBuf, 0, sizeof(szBuf));
        if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &spDevInfoData, SPDRP_FRIENDLYNAME, NULL, szBuf, MAX_PATH, 0))
            continue;

        // 对比设备描述符
        szName = strstr((const tS8*)szBuf, _pDevicePort);
        if (szName == NULL)
            continue;
        szName += strlen(_pDevicePort) + 1;

        // 提取串口号
        sscanf_s(szName, "(COM%[0-9])", numbuf, 4);
        memcpy(&_pctrl->DevicePort[_pctrl->CommToolCnt][0], "\\\\.\\COM", 7);
        memcpy(&_pctrl->DevicePort[_pctrl->CommToolCnt][7], numbuf, 4);
        memcpy(&_pctrl->DeviceName[_pctrl->CommToolCnt][0], szBuf, SP_NAME_LEN_MAX);
        _pctrl->CommToolCnt++;
    }

    // 释放设备集句柄
    SetupDiDestroyDeviceInfoList(hDevInfo);

    LOG_INF("Serial CAN_LIN_Tool device search completed!");
    if (_pctrl->CommToolCnt == 0) {
        _pctrl->mHand = INVALID_HANDLE_VALUE;
        LOG_ERR("No CAN_LIN_Tool device connected!");
        return -1;
    } else {
        LOG_INF("CAN_LIN_Tool Device List:");
        for (tU16 idx = 0; idx < _pctrl->CommToolCnt; idx++)
            LOG_INF("No: %d - %s", idx, _pctrl->DeviceName[idx]);
        printf("====================================================================\r\n");
    }

	return 0;
}

tS16 comm_tool_open(comm_tool_t* _pctrl, tU8 _size)
{
    // Use the CreateFile function to open the serial port
    _pctrl->mHand = CreateFile(
        (LPCSTR)_pctrl->DevicePort[_pctrl->SetCommToolNo],    // 串口名称
        GENERIC_READ | GENERIC_WRITE,                       // 读写权限
        0,                                                  // 共享模式（0表示不共享）
        NULL,                                               // 安全属性（默认）
        OPEN_EXISTING,                                      // 打开已存在的串口
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,       // 异步标志
        NULL);                                              // 模板文件句柄（串口不需要）
    
    if (INVALID_HANDLE_VALUE == _pctrl->mHand) {
        LOG_ERR("Failed opened the serial port!");
        return -1;
    }
    
    LOG_INF("Successfully opened the serial port!");
    
    COMMTIMEOUTS timeouts;  // Data timeout configuration structure

    // read timeout
    timeouts.ReadIntervalTimeout = MAXDWORD;          // The interval between two characters timed out during the read operation
    timeouts.ReadTotalTimeoutMultiplier = 0;    // Timeout for read operations on each character read
    timeouts.ReadTotalTimeoutConstant = 0;     // Fixed timeout for read operations
    // write timeout
    timeouts.WriteTotalTimeoutMultiplier = 100;   // Timeout for write operations on each character written
    timeouts.WriteTotalTimeoutConstant = 500;    // Fixed timeout for write operations

    if (!SetCommTimeouts(_pctrl->mHand, &timeouts)) {
        LOG_ERR("Serial port timeout configuration failed!");
        return -2;
    }

    // Set the input and output buffer size
    if (!SetupComm(_pctrl->mHand, 1024, 4096)) {
        LOG_ERR("Serial port buffer configuration failed!");
        return -3;
    }

    // Configure serial port parameters
    DCB dcb; // Set serial port parameters, such as baud rate

    if (!GetCommState(_pctrl->mHand, &dcb)) {
        LOG_ERR("Failed to obtain serial port configuration information!");
        return -4;
    }

    dcb.BaudRate = CBR_19200;       // baud rate
    dcb.ByteSize = 8;               // Data bits
    dcb.Parity = NOPARITY;          // Check bits
    dcb.StopBits = ONESTOPBIT;      // Stop bits

    if (!SetCommState(_pctrl->mHand, &dcb)) {
        LOG_ERR("Failed to configure serial port configuration information!");
        return -5;
    }

    if (!PurgeComm(_pctrl->mHand, PURGE_TXCLEAR|PURGE_RXCLEAR|PURGE_TXABORT|PURGE_RXABORT)) {
        LOG_ERR("Failed to clear the serial port buffer!");
        return -6;
    }

    if (!SetCommMask(_pctrl->mHand, EV_ERR | EV_RXCHAR)) {
        LOG_ERR("Failed to configure serial port events!");
        return -7;
    }

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

    // Apply for asynchronous communication structure
    _pctrl->ComOvlpEvent.pOvReadEvent = (LPOVERLAPPED)malloc(sizeof(OVERLAPPED));
    _pctrl->ComOvlpEvent.pOvWaitEvent = (LPOVERLAPPED)malloc(sizeof(OVERLAPPED));
    _pctrl->ComOvlpEvent.pOvWriteEvent = (LPOVERLAPPED)malloc(sizeof(OVERLAPPED));
    if (NULL == _pctrl->ComOvlpEvent.pOvReadEvent ||
        NULL == _pctrl->ComOvlpEvent.pOvWaitEvent ||
        NULL == _pctrl->ComOvlpEvent.pOvWriteEvent) {
        LOG_ERR("Failed to create serial port OVERLAPPED structure!");
        return -9;
    }

    // Initialize asynchronous communication events
    memset(_pctrl->ComOvlpEvent.pOvReadEvent, 0, sizeof(OVERLAPPED));
    memset(_pctrl->ComOvlpEvent.pOvWaitEvent, 0, sizeof(OVERLAPPED));
    memset(_pctrl->ComOvlpEvent.pOvWriteEvent, 0, sizeof(OVERLAPPED));

    // Create asynchronous communication events
    _pctrl->ComOvlpEvent.pOvReadEvent->hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    _pctrl->ComOvlpEvent.pOvWaitEvent->hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    _pctrl->ComOvlpEvent.pOvWriteEvent->hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (NULL == _pctrl->ComOvlpEvent.pOvReadEvent->hEvent ||
        NULL == _pctrl->ComOvlpEvent.pOvWaitEvent->hEvent ||
        NULL == _pctrl->ComOvlpEvent.pOvWriteEvent->hEvent) {
        LOG_ERR("Failed to create communication related event!");
        return -10;
    }
    LOG_INF("Successfully created communication related event!");

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
    LOG_INF("Successfully created serial port receiving and sending threads!");

    LOG_INF("Serial port Configure completed!");

    return 0;
}

tS16 comm_tool_close(comm_tool_t* _pctrl)
{
    set_fifo_destroy(&_pctrl->ComDataFifo.UartRxMsg);
    set_fifo_destroy(&_pctrl->ComDataFifo.UartTxMsg);
    if (_pctrl->ComDataFifo.UartRxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartRxMsg.MutexSign);
    if (_pctrl->ComDataFifo.UartTxMsg.MutexSign != NULL) CloseHandle(_pctrl->ComDataFifo.UartTxMsg.MutexSign);
    if (_pctrl->ComOvlpEvent.pOvReadEvent != NULL) {
        if (_pctrl->ComOvlpEvent.pOvReadEvent->hEvent != NULL) CloseHandle(_pctrl->ComOvlpEvent.pOvReadEvent->hEvent);
        free(_pctrl->ComOvlpEvent.pOvReadEvent);
    }
    if (_pctrl->ComOvlpEvent.pOvWaitEvent != NULL) {
        if (_pctrl->ComOvlpEvent.pOvWaitEvent->hEvent != NULL) CloseHandle(_pctrl->ComOvlpEvent.pOvWaitEvent->hEvent);
        free(_pctrl->ComOvlpEvent.pOvWaitEvent);
    }
    if (_pctrl->ComOvlpEvent.pOvWriteEvent != NULL) {
        if (_pctrl->ComOvlpEvent.pOvWriteEvent->hEvent != NULL) CloseHandle(_pctrl->ComOvlpEvent.pOvWriteEvent->hEvent);
        free(_pctrl->ComOvlpEvent.pOvWriteEvent);
    }
    if (_pctrl->mHand != INVALID_HANDLE_VALUE) CloseHandle(_pctrl->mHand);

    LOG_INF("Successfully closed the serial port!");

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
