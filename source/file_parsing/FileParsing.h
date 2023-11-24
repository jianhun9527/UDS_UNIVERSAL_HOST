/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\file_parsing\FileParsing.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-17 23:10:22
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 21:12:22
 * @Description  : File parsing header file definition
 ******************************************************************************/

#ifndef __FILEPARSING_H__
#define __FILEPARSING_H__

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
#define SEGMENT_COUNT_MAX       8

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef enum com_type
{
    COM_CAN,
    COM_LIN,
    COM_PWM,
    COM_UART,
    COM_SPI,
    COM_TYPE_MAX
} com_type_t;

typedef struct com_baud_t
{
    tU16 value;
    tS8* feature;
} com_baud_t;

typedef enum file_type
{
    FILE_S19,
    FILE_HEX,
    FILE_BIN,
    FILE_TYPE_MAX
} file_type_t;

typedef struct uds_cfg
{
    tU32 phyReqID;
    tU32 funReqID;
    tU32 resDiagID;
} uds_cfg_t;

typedef struct com_info
{
    com_type_t comType;
    com_baud_t comBaud;
} com_info_t;

typedef struct config_file
{
    tS8* pSerialName;
    tS8* pAppPath;
    tS8* pDerivePath;
    tS8 setSerialNo;
    com_info_t comInfo;
    file_type_t fileType;
    uds_cfg_t udsCfg;
} config_file_t;

typedef struct seg_info
{
    tU32 startAddr;
    tU32 buffIndex;
    tU32 dataLength;
    tU32 crc32;
} seg_info_t;

typedef struct FileInfo
{
    tU8* buffer;
    tU32 buffLength;
    tU32 crc32;
    tU16 segCount;
    seg_info_t segInfo[SEGMENT_COUNT_MAX];
} file_info_t;

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern tS16 parsing_config_file(const tS8* _pininame, config_file_t* const _pctrl);
extern void destroy_config_file(void);
extern tS16 parsing_compile_file(tS8* const _pname, file_type_t _type, tU32 _filesize, file_info_t* _pfileinfo);
extern void destroy_file_buffer(file_info_t* _pfile);

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __FILEPARSING_H__
