/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\fifo\fifo.h
 * @Author       : jianhun
 * @CreationTime : 2023-11-03 23:41:07
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-09 01:17:27
 * @Description  : 
 ******************************************************************************/

#ifndef __FIFO_H__
#define __FIFO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "commondef.h"

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef enum fifo_state
{
    FIFO_EMPTY = 0U,
    FIFO_VALID,
    FIFO_FULL
} fifo_state_t;

typedef struct msg_fifo
{
    HANDLE MutexSign;
    tU8 *Buff;
    tU8 Head;
    tU8 Rear;
    tU8 Size;
    tU8 Count;
    fifo_state_t State;
} msg_fifo_t;

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 set_fifo_init(msg_fifo_t* _pmf, tU8 _size, tU8 _count);
extern tS16 set_fifo_reset(msg_fifo_t* _pmf);
extern tS16 set_fifo_destroy(msg_fifo_t* _pmf);
extern tS16 set_fifo_write(msg_fifo_t* _pmf, tU8* _pdata, tU16 _len);
extern tS16 get_fifo_read(msg_fifo_t* _pmf, tU8* _pdata, tU16* _plen);
extern tS16 get_fifo_is_empty(msg_fifo_t* _pmf);
extern tS16 get_fifo_is_full(msg_fifo_t* _pmf);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__
