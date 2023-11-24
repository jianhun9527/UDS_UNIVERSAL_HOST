/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\uart_drive\UartDrive.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-15 14:47:49
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-20 00:21:43
 * @Description  : Serial driver header file
 ******************************************************************************/

#ifndef __UARTDRIVE_H__
#define __UARTDRIVE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "commondef.h"
#include "fifo.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define SP_NUM_MAX          5
#define SP_NAME_LEN_MAX     50
#define UART_SEND_BUFF_MAX  256
#define UART_RECE_BUFF_MAX  32

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef struct com_over_lap
{
    LPOVERLAPPED pOvReadEvent;
    LPOVERLAPPED pOvWaitEvent;
    LPOVERLAPPED pOvWriteEvent;
} com_over_lap_t;

typedef struct com_fifo_data
{
    msg_fifo_t UartRxMsg;
    msg_fifo_t UartTxMsg;
    WINBOOL RxEmpty;
    WINBOOL TXEmpty;
    tU8 Size;
} com_fifo_data_t;

typedef struct com_status_ctl
{
    WINBOOL ReceEnable;
    WINBOOL SendEnable;
} com_status_ctl_t;

typedef struct serial_port
{
    HANDLE mHand;
    tS16 SerialCnt;
    tS16 SetSerialNo;
    tU8 SerialName[SP_NUM_MAX][SP_NAME_LEN_MAX];
    tU8 DeviceName[SP_NUM_MAX][SP_NAME_LEN_MAX];
    com_over_lap_t ComOvlpEvent;
    com_fifo_data_t ComDataFifo;
    com_status_ctl_t ComStateCtl;
} serial_port_t;

// UDS地址定义
typedef struct uds_data
{
    tU32 ReqID;         // 请求报文ID
    tU32 ResID;         // 应答报文ID
    tU8* pData;         // 数据空间
    tU32 DataLen;       // 数据长度
    tU8 flag;           // can: 0-标准帧, 1-扩展帧 or lin: 0-标准，1-增强，一般为标准校验
    tU8 STmin;          // 连续帧时间间隔，单位为毫秒
    tU16 __Res;
} uds_data_t;

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 serial_port_scan(serial_port_t* _pctrl, const tS8* _pserialname);
extern tS16 serial_port_open(serial_port_t* _pctrl, tU8 _size);
extern tS16 serial_port_close(serial_port_t* _pCtrl);
extern tS16 serial_port_send_data(serial_port_t* _pctrl, tU8* _pdata, tU8 _size);
extern tS16 serial_port_rece_data(serial_port_t* _pctrl, tU8* _pdata, tU8* _size);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __UARTDRIVE_H__
