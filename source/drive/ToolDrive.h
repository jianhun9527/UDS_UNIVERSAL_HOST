/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\drive\ToolDrive.h
 * @Author       : jianhun
 * @CreationTime : 2023-11-24 22:30:49
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-25 23:35:45
 * @Description  : 
 ******************************************************************************/

#ifndef __TOOLDRIVE_H__
#define __TOOLDRIVE_H__

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
#define SP_NUM_MAX                  5
#define SP_NAME_LEN_MAX             256
#define UART_SEND_BUFF_MAX          256
#define UART_RECE_BUFF_MAX          32

#define TOOL_FUNCTION_NOT_SUPPORT   -255

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
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

typedef struct comm_tool
{
    HANDLE mHand;
    tS16 CommToolCnt;
    tS16 SetCommToolNo;
    tS8 DevicePort[SP_NUM_MAX][SP_NAME_LEN_MAX];
    tS8 DeviceName[SP_NUM_MAX][SP_NAME_LEN_MAX];
    com_fifo_data_t ComDataFifo;
    com_status_ctl_t ComStateCtl;
} comm_tool_t;

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
extern tS16 comm_tool_scan(comm_tool_t* _pctrl, const tS8* _pDevicePort);
extern tS16 comm_tool_open(comm_tool_t* _pctrl, tU8 _size);
extern tS16 comm_tool_close(comm_tool_t* _pCtrl);
extern tS16 comm_tool_send_data(comm_tool_t* _pctrl, tU8* _pdata, tU8 _size);
extern tS16 comm_tool_rece_data(comm_tool_t* _pctrl, tU8* _pdata, tU8* _psize);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __TOOLDRIVE_H__
