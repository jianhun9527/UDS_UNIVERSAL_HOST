/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\fifo\fifo.c
 * @Author       : jianhun
 * @CreationTime : 2023-11-03 23:41:19
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-26 22:31:25
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "fifo.h"

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

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 set_fifo_init(msg_fifo_t* _pmf, tU8 _size, tU8 _count)
{
    if (_pmf == NULL) return -1;
    _pmf->Head = 0;
    _pmf->Rear = 0;
    _pmf->Size = _size;
    _pmf->Count = _count;
    _pmf->State = FIFO_EMPTY;
    _pmf->Buff = (tU8*)malloc(sizeof(tU8) * (tU16)_size * (tU16)_count);

    if (_pmf->Buff == NULL) return -2;

    return 0;
}

tS16 set_fifo_reset(msg_fifo_t* _pmf)
{
    _pmf->Head = 0;
    _pmf->Rear = 0;
    _pmf->State = FIFO_EMPTY;

    return 0;
}

tS16 set_fifo_destroy(msg_fifo_t* _pmf)
{
    if (_pmf->Buff != NULL) free(_pmf->Buff);

    return 0;
}

tS16 set_fifo_write(msg_fifo_t* _pmf, tU8* _pdata, tU16 _len)
{
    tU16 headaddr ,rearaddr ,endaddr ,residuesize;
    
    WaitForSingleObject(_pmf->MutexSign, INFINITE);
    
    headaddr = (tU16)_pmf->Head * _pmf->Size;
    rearaddr = (tU16)_pmf->Rear * _pmf->Size;
    endaddr = (tU16)_pmf->Count * _pmf->Size;
    residuesize = (headaddr + endaddr - rearaddr) % endaddr;

    if (get_fifo_is_full(_pmf)) goto __ERROR_STATE;
    if (_len == 0U || _len > endaddr) goto __ERROR_STATE;
    if (_len % _pmf->Size != 0) goto __ERROR_STATE;
    if (get_fifo_is_empty(_pmf)) residuesize = endaddr;
    if (_len > residuesize) goto __ERROR_STATE;

    if (_len == residuesize) {
        _pmf->Rear = _pmf->Head;
        _pmf->State = FIFO_FULL;
    } else {
        _pmf->Rear = (_pmf->Rear + (_len / _pmf->Size)) % _pmf->Count;
        _pmf->State = FIFO_VALID;
    }

    if (_len + rearaddr <= endaddr) {
        memcpy(_pmf->Buff + rearaddr, _pdata, _len);
    } else {
        memcpy(_pmf->Buff + rearaddr, _pdata, endaddr - rearaddr);
        memcpy(_pmf->Buff, _pdata + endaddr - rearaddr, _len + rearaddr - endaddr);
    }

    ReleaseSemaphore(_pmf->MutexSign, 1, NULL);
    return 0;

__ERROR_STATE:
    ReleaseSemaphore(_pmf->MutexSign, 1, NULL);
    return -1;
}

tS16 get_fifo_read(msg_fifo_t* _pmf, tU8* _pdata, tU16* _plen)
{
    tU16 headaddr ,rearaddr ,endaddr , residuesize;

    WaitForSingleObject(_pmf->MutexSign, INFINITE);

    headaddr = (tU16)_pmf->Head * _pmf->Size;
    rearaddr = (tU16)_pmf->Rear * _pmf->Size;
    endaddr = (tU16)_pmf->Count * _pmf->Size;
    residuesize = (rearaddr + endaddr - headaddr) % endaddr;

    if (get_fifo_is_empty(_pmf)) goto __ERROR_STATE;
    if (*_plen == 0) goto __ERROR_STATE;
    if ((*_plen % _pmf->Size) != 0) goto __ERROR_STATE;

    if (get_fifo_is_full(_pmf)) residuesize = endaddr;
    if (*_plen > residuesize) *_plen = residuesize;
    
    if (*_plen == residuesize) {
        _pmf->Head = _pmf->Rear;
        _pmf->State = FIFO_EMPTY;
    } else {
        _pmf->Head = (_pmf->Head + (*_plen / _pmf->Size)) % _pmf->Count;
        _pmf->State = FIFO_VALID;
    }

    if (*_plen + headaddr <= endaddr) {
        memcpy(_pdata, _pmf->Buff + headaddr, *_plen);
    } else {
        memcpy(_pdata, _pmf->Buff + headaddr, endaddr - headaddr);
        memcpy(_pdata + endaddr - headaddr, _pmf->Buff, *_plen + headaddr - endaddr);
    }

    ReleaseSemaphore(_pmf->MutexSign, 1, NULL);
    return 0;

__ERROR_STATE:
    ReleaseSemaphore(_pmf->MutexSign, 1, NULL);
    return -1;
}

tS16 get_fifo_is_empty(msg_fifo_t* _pmf)
{
    return _pmf->State == FIFO_EMPTY ? 1 : 0;
}

tS16 get_fifo_is_full(msg_fifo_t* _pmf)
{
    return _pmf->State == FIFO_FULL ? 1 : 0;
}

/* End of file */
