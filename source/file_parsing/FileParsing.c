/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\file_parsing\FileParsing.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-17 23:11:23
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-13 22:58:53
 * @Description  : Read the project file and extract valid data
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "GenerateKeyEx.h"
#include "FileParsing.h"
#include "iniparser.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define LINE_MAX_LEN        (400) // 单行最大400字符

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/
typedef enum fps_line_result
{
    HEADER_LINE,
    DATA_LINE,
    EXT_SEG_ADDR_LINE,
    EXT_LIN_ADDR_LINE,
    START_SEG_ADDR_LINE,
    START_LIN_ADDR_LINE,
    COUNT_LINE,
    END_LINE,
    INVALID_LINE
} fps_line_result_t;

typedef struct data_mapping
{
    union
    {
       com_type_t comtype;
       file_type_t filetype;
    } index;
    tS8* feature;
} data_mapping_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static dictionary* iniFileDir = NULL;
static const data_mapping_t comDataDict[COM_TYPE_MAX] = {
    {{COM_CAN}, "CAN"},
    {{COM_LIN}, "LIN"},
    {{COM_PWM}, "PWM"},
    {{COM_UART}, "UART"},
    {{COM_SPI}, "SPI"},
};
static const data_mapping_t fileDataDict[FILE_TYPE_MAX] = {
    {{FILE_S19}, "S19"},
    {{FILE_HEX}, "HEX"},
    {{FILE_BIN}, "BIN"},
};

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tU32 data_conver_ascii2hex(tU8* _pbuff, tU16 _begin, tU16 _count);
static fps_line_result_t parsing_s19_single_line_file(tU8* _pdest, tU32* _paddr, tU32* _plen, tU8* _psource);
static fps_line_result_t parsing_hex_single_line_file(tU8* _pdest, tU32* _paddr, tU32* _plen, tU8* _psource);
static tS16 parsing_file_info(file_info_t* _pdestfile, file_type_t _type, FILE* _pf);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tU32 data_conver_ascii2hex(tU8* _pbuff, tU16 _begin, tU16 _count)
{
    tU32 retval = 0;

    for (tU16 idx = 0; idx < _count; idx ++)
    {
        retval <<= 4U;

        tU8 tmpval = _pbuff[_begin + idx];
        if (tmpval >= '0' && tmpval <= '9') {
            retval += tmpval - '0';
        } else if (tmpval >= 'A' && tmpval <= 'F') {
            retval += tmpval - 'A' + 10;
        } else if (tmpval >= 'a' && tmpval <= 'f') {
            retval += tmpval - 'a' + 10;
        } else {
            retval >>= 4U;
            break;
        }
    }

    return retval;
}

static fps_line_result_t parsing_s19_single_line_file(tU8* _pdest, tU32* _paddr, tU32* _plen, tU8* _psource)
{
    fps_line_result_t retval = INVALID_LINE;
    tU8 checksum = 0;

    if (_psource[0] != 'S') return retval;
    switch (_psource[1])
    {
    case '0':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 3;
        *_paddr = data_conver_ascii2hex(_psource, 4, 4);
        for (tU32 idx = 0; idx < *_plen; idx ++)
        {
            _pdest[idx] = (tU8)data_conver_ascii2hex(_psource, (8 + 2 * idx), 2);
            checksum += _pdest[idx];
        }
        checksum += *_plen + 3;
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, (*_plen * 2 + 8), 2);
        if (0xFF == checksum) {
            retval = HEADER_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '1':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 3;
        *_paddr = data_conver_ascii2hex(_psource, 4, 4);
        for (tU32 idx = 0; idx < *_plen; idx ++)
        {
            _pdest[idx] = (tU8)(data_conver_ascii2hex(_psource, (8 + 2 * idx), 2));
            checksum += _pdest[idx];
        }
        checksum += *_plen + 3;
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, (*_plen * 2 + 8), 2);
        if (0xFF == checksum) {
            retval = DATA_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '2':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 4;
        *_paddr = data_conver_ascii2hex(_psource, 4, 6);
        for (tU16 idx = 0; idx < *_plen; idx ++)
        {
            _pdest[idx] = (tU8)(data_conver_ascii2hex(_psource, (10 + 2 * idx), 2));
            checksum += _pdest[idx];
        }
        checksum += *_plen + 4;
        checksum += (tU8)(*_paddr >> 16);
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, (*_plen * 2 + 10), 2);
        if (0xFF == checksum) {
            retval = DATA_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '3':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 5;
        *_paddr = data_conver_ascii2hex(_psource, 4, 8);
        for (tU16 idx = 0; idx < *_plen; idx++)
        {
            _pdest[idx] = (tU8)(data_conver_ascii2hex(_psource, (12 + 2 * idx), 2));
            checksum += _pdest[idx];
        }
        checksum += *_plen + 5;
        checksum += (tU8)(*_paddr >> 24);
        checksum += (tU8)(*_paddr >> 16);
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, (*_plen * 2 + 12), 2);
        if (0xFF == checksum) {
            retval = DATA_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '5':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 3;
        *_paddr = data_conver_ascii2hex(_psource, 4, 4);
        checksum += *_plen + 3;
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, 8, 2);
        if (0xFF == checksum) {
            retval = COUNT_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '6':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 4;
        *_paddr = data_conver_ascii2hex(_psource, 4, 6);
        checksum += *_plen + 4;
        checksum += (tU8)(*_paddr >> 16);
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, 10, 2);
        if (0xFF == checksum) {
            retval = COUNT_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '7':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 5;
        *_paddr = data_conver_ascii2hex(_psource, 4, 8);
        checksum += *_plen + 5;
        checksum += (tU8)(*_paddr >> 24);
        checksum += (tU8)(*_paddr >> 16);
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, 12, 2);
        if (0xFF == checksum) {
            retval = END_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '8':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 4;
        *_paddr = data_conver_ascii2hex(_psource, 4, 6);
        checksum += *_plen + 4;
        checksum += (tU8)(*_paddr >> 16);
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, 10, 2);
        if (0xFF == checksum) {
            retval = END_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    case '9':
        *_plen = data_conver_ascii2hex(_psource, 2, 2) - 3;
        *_paddr = data_conver_ascii2hex(_psource, 4, 4);
        checksum += *_plen + 3;
        checksum += (tU8)(*_paddr >> 8);
        checksum += (tU8)*_paddr;
        checksum += (tU8)data_conver_ascii2hex(_psource, 8, 2);
        if (0xFF == checksum) {
            retval = END_LINE;
        } else {
            LOG_ERR("LINE data checksum error!!!");
        }
        break;
    default:
        retval = INVALID_LINE;
        break;
    }

    return retval;
}

static fps_line_result_t parsing_hex_single_line_file(tU8* _pdest, tU32* _paddr, tU32* _plen, tU8* _psource)
{
    fps_line_result_t retval = INVALID_LINE;
    tU8 checksum = 0U;

    if (_psource[0] != ':' && _psource[7] == '0') return retval;
    *_plen = data_conver_ascii2hex(_psource, 1, 2);
    *_paddr = data_conver_ascii2hex(_psource, 3, 4);
    for (tU32 idx = 0; idx < *_plen; idx ++)
    {
        _pdest[idx] = (tU8)data_conver_ascii2hex(_psource, (9 + 2 * idx), 2);
        checksum += _pdest[idx];
    }
    checksum += *_plen;
    checksum += (tU8)(*_paddr >> 8);
    checksum += *_paddr;
    checksum += (tU8)data_conver_ascii2hex(_psource, (*_plen * 2 + 9), 2);
    switch (_psource[8])
    {
    case '0':
        checksum += 0;
        retval = DATA_LINE;
        break;
    case '1':
        checksum += 1;
        if (0 == *_plen && 0 == *_paddr) {
            retval = END_LINE;
        } else {
            LOG_ERR("End of file format error!");
            retval = INVALID_LINE;
        }
        break;
    case '2':
        checksum += 2;
        if (2 == *_plen && 0 == *_paddr) {
            retval = EXT_SEG_ADDR_LINE;
        } else {
            LOG_ERR("The record identifying the extended segment address is in an incorrect format.!");
            retval = INVALID_LINE;
        }
        break;
    case '3':
        checksum += 3;
        if (4 == *_plen && 0 == *_paddr) {
            retval = START_SEG_ADDR_LINE;
        } else {
            LOG_ERR("Start segment address record format error!");
            retval = INVALID_LINE;
        }
        break;
    case '4':
        checksum += 4;
        if (2 == *_plen && 0 == *_paddr) {
            retval = EXT_LIN_ADDR_LINE;
        } else {
            LOG_ERR("Record format identifying extended linear address is malformed!");
            retval = INVALID_LINE;
        }
        break;
    case '5':
        checksum += 5;
        if (4 == *_plen && 0 == *_paddr) {
            retval = EXT_LIN_ADDR_LINE;
        } else {
            LOG_ERR("Start linear address record format error!");
            retval = INVALID_LINE;
        }
        retval = START_LIN_ADDR_LINE;
        break;
    default:
        retval = INVALID_LINE;
        break;
    }

    if (0x00 != checksum) {
        LOG_ERR("LINE data checksum error!!!");
        retval = INVALID_LINE;
    }

    return retval;
}

static tS16 parsing_file_info(file_info_t* _pdestfile, file_type_t _type, FILE* _pf)
{
    tS16 retval = 0;

    tS8 asciiBuff[LINE_MAX_LEN]; // Used to record the ASCII code of a line in the S19 file
    tU8 hexBuff[LINE_MAX_LEN];   // HEX data for recording ASCII conversion
    tU32 addressTemp = 0, lengthTemp = 0, addressLast = 0;

    _pdestfile->buffLength = 0;
    _pdestfile->segCount = 0;

    switch (_type)
    {
    case FILE_S19:
    {
        for (asciiBuff[0] = 'S'; 'S' == asciiBuff[0];)
        {
            if (NULL == fgets(asciiBuff, LINE_MAX_LEN - 1, _pf)) break;
            if (DATA_LINE == parsing_s19_single_line_file(hexBuff, &addressTemp, &lengthTemp, (tU8*)asciiBuff)) {
                if (0 == _pdestfile->buffLength || addressLast != addressTemp) {
                    _pdestfile->segInfo[_pdestfile->segCount].buffIndex = _pdestfile->buffLength;
                    _pdestfile->segInfo[_pdestfile->segCount].startAddr = addressTemp;
                    _pdestfile->segInfo[_pdestfile->segCount].dataLength = 0;
                    addressLast = addressTemp;
                    _pdestfile->segCount ++;
                }

                memcpy(_pdestfile->buffer + _pdestfile->buffLength, hexBuff, lengthTemp);
                _pdestfile->segInfo[_pdestfile->segCount - 1].dataLength += lengthTemp;
                _pdestfile->buffLength += lengthTemp;
                addressLast += lengthTemp;
            }
        }
    }
    break;
    case FILE_HEX:
    {
        tU32 lineraddr = 0, segmentaddr = 0, appentryaddr = 0;

        for (asciiBuff[0] = ':'; ':' == asciiBuff[0];)
        {
            if (NULL == fgets(asciiBuff, LINE_MAX_LEN - 1, _pf)) break;
            switch (parsing_hex_single_line_file(hexBuff, &addressTemp, &lengthTemp, (tU8*)asciiBuff))
            {
            case DATA_LINE:
            {
              if (0 == _pdestfile->buffLength || addressLast != addressTemp) {
                    _pdestfile->segInfo[_pdestfile->segCount].buffIndex = _pdestfile->buffLength;
                    _pdestfile->segInfo[_pdestfile->segCount].startAddr = addressTemp + segmentaddr + lineraddr;
                    _pdestfile->segInfo[_pdestfile->segCount].dataLength = 0;
                    addressLast = addressTemp;
                    _pdestfile->segCount ++;
                }

                memcpy(_pdestfile->buffer + _pdestfile->buffLength, hexBuff, lengthTemp);
                _pdestfile->segInfo[_pdestfile->segCount - 1].dataLength += lengthTemp;
                _pdestfile->buffLength += lengthTemp;
                addressLast += lengthTemp;
            }
            break;
            case EXT_SEG_ADDR_LINE:
                segmentaddr = (((tU32)hexBuff[0] << 8) | ((tU32)hexBuff[1] << 0)) << 4;
            break;
            case EXT_LIN_ADDR_LINE:
                lineraddr = (((tU32)hexBuff[0] << 8) | ((tU32)hexBuff[1] << 0)) << 16;
            break;
            case START_SEG_ADDR_LINE:
                appentryaddr = ((tU32)hexBuff[0] << 24) | ((tU32)hexBuff[1] << 16) | ((tU32)hexBuff[2] << 8) | ((tU32)hexBuff[3] << 0);
            break;
            case START_LIN_ADDR_LINE:
                appentryaddr = ((tU32)hexBuff[0] << 24) | ((tU32)hexBuff[1] << 16) | ((tU32)hexBuff[2] << 8) | ((tU32)hexBuff[3] << 0);
            break;
            case END_LINE:
                asciiBuff[0] = 'S';
            break;
            case INVALID_LINE:
            case HEADER_LINE:
            case COUNT_LINE:
            break;
            }
        }
        (void)appentryaddr;
    }
    break;
    case FILE_BIN:
    {
        fseek(_pf, 0, SEEK_END);
        _pdestfile->buffLength = ftell(_pf);
        fseek(_pf, 0, SEEK_SET);
        fread(_pdestfile->buffer, sizeof(tU8), _pdestfile->buffLength, _pf);
        _pdestfile->segInfo->dataLength = _pdestfile->buffLength;
        _pdestfile->segCount = 1;
    }
    break;
    default:
        retval = -1;
        break;
    }

    // 计算CRC校验值
    _pdestfile->crc32 = GetFileCRC32(_pdestfile->buffer, _pdestfile->buffLength);
    for (tU16 idx = 0; idx < _pdestfile->segCount; idx ++)
    {
        _pdestfile->segInfo[idx].crc32 = GetFileCRC32(
            _pdestfile->buffer + _pdestfile->segInfo[idx].buffIndex,
            _pdestfile->segInfo[idx].dataLength);
    }

    for (tU16 idx = 0; idx < _pdestfile->segCount; idx ++)
    {
        for (tU16 jdx = idx + 1; jdx < _pdestfile->segCount; jdx ++)
        {
            if (_pdestfile->segInfo[idx].startAddr > _pdestfile->segInfo[jdx].startAddr) {
                seg_info_t segInfoTemp = _pdestfile->segInfo[idx];
                _pdestfile->segInfo[idx] = _pdestfile->segInfo[jdx];
                _pdestfile->segInfo[jdx] = segInfoTemp;
            }
        }
    }

    return retval;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 parsing_config_file(const tS8* _pininame, config_file_t* const _pctrl)
{
    tS16 idx = 0;

    iniFileDir = iniparser_load(_pininame);
    if (iniFileDir == NULL) {
        LOG_ERR("Cannot parse file: %s", _pininame);
        return -1;
    }
    
    _pctrl->pDevicePort = (tS8*)iniparser_getstring(iniFileDir, "SYS_CONFIG:SERIAL_NAME", NULL);
    if (_pctrl->pDevicePort == NULL) {
        LOG_ERR("Device identifier is not defined in the configuration file!");
        return -2;
    }

    tS8* pStrTmp = (tS8*)iniparser_getstring(iniFileDir, "SYS_CONFIG:SERIAL_NO", NULL);
    if (pStrTmp == NULL) {
        LOG_ERR("The device number currently being used is not defined in the configuration file!");
        return -3;
    }
    _pctrl->SetToolNo = atoi(pStrTmp);

    pStrTmp = (tS8*)iniparser_getstring(iniFileDir, "SYS_CONFIG:COM_TYPE", NULL);
    if (pStrTmp == NULL) {
        LOG_ERR("The current communication method is not defined in the configuration file!");
        return -4;
    } else {
        for (idx = 0; idx < COM_TYPE_MAX; idx ++)
        {
            if (!strcmp(pStrTmp, comDataDict[idx].feature)) {
                _pctrl->comInfo.comType = comDataDict[idx].index.comtype;
                break;
            }
        }
        if (idx == COM_TYPE_MAX) {
            LOG_ERR("Please configure a valid communication method!");
            return -5;
        } else if (idx >= COM_PWM) {
            LOG_ERR("The toolbox does not currently support this communication method -> %s", comDataDict[idx].feature);
            return -6;
        }
    }

    _pctrl->comInfo.comBaud.feature = (tS8*)iniparser_getstring(iniFileDir, "SYS_CONFIG:COM_BAUD", NULL);
    if (_pctrl->comInfo.comBaud.feature == NULL) {
        LOG_ERR("The current communication baud rate is not defined in the configuration file!");
        return -7;
    }
    _pctrl->comInfo.comBaud.value = atoi(_pctrl->comInfo.comBaud.feature);
    if (0 == _pctrl->comInfo.comBaud.value) {
        LOG_ERR("Baud rate character field contains non-decimal characters!");
        return -8;
    }

    pStrTmp = (tS8*)iniparser_getstring(iniFileDir, "BOOT_CONFIG:FILE_TYPE", NULL);
    if (pStrTmp == NULL) {
        LOG_ERR("The current file type is not defined in the configuration file!");
        return -7;
    } else {
        for (idx = 0; idx < FILE_TYPE_MAX; idx ++)
        {
            if (!strcmp(pStrTmp, fileDataDict[idx].feature)) {
                _pctrl->fileType = fileDataDict[idx].index.filetype;
                break;
            }
        }
        if (idx == FILE_TYPE_MAX) {
            LOG_ERR("Please configure a valid file type!");
            return -8;
        }
    }

    _pctrl->pAppPath = (tS8*)iniparser_getstring(iniFileDir, "BOOT_CONFIG:APP_PATH", NULL);
    if (_pctrl->pAppPath == NULL) {
        LOG_ERR("The application file path is not defined in the configuration file!");
        return -9;
    }

    _pctrl->pDerivePath = (tS8*)iniparser_getstring(iniFileDir, "BOOT_CONFIG:DRIVER_PATH", NULL);
    if (_pctrl->pDerivePath == NULL) {
        LOG_ERR("The driver file path is not defined in the configuration file!");
        return -10;
    }

    tS8 *phyID, *funID, *resID;
    phyID = (tS8*)iniparser_getstring(iniFileDir, "UDS_CONFIG:UDS_PHYSICAL_ID", NULL);
    funID = (tS8*)iniparser_getstring(iniFileDir, "UDS_CONFIG:UDS_FUNCTIOl_ID", NULL);
    resID = (tS8*)iniparser_getstring(iniFileDir, "UDS_CONFIG:UDS_RESPONSE_ID", NULL);
    if (phyID == NULL || funID == NULL || resID == NULL) {
        LOG_ERR("UDS related ID is not defined in the configuration file!");
        return -11;
    }
    if (phyID[0] != '0' || phyID[1] != 'x' || funID[0] != '0' || funID[1] != 'x' || resID[0] != '0' || resID[1] != 'x') {
        LOG_ERR("UDS ID configuration format error, please add 0x at the beginning!");
        return -12;
    }
    _pctrl->udsCfg.phyReqID = data_conver_ascii2hex((tU8*)phyID, 2, 3);
    _pctrl->udsCfg.funReqID = data_conver_ascii2hex((tU8*)funID, 2, 3);
    _pctrl->udsCfg.resDiagID = data_conver_ascii2hex((tU8*)resID, 2, 3);
    
    LOG_INF("[INI Config] Parsing configuration file successfully!");
    LOG_INF("[SYS Config] Communication method   -> %s", comDataDict[_pctrl->comInfo.comType].feature);
    LOG_INF("[SYS Config] Communication baudrate -> %s", _pctrl->comInfo.comBaud.feature);
    LOG_INF("[SYS Config] Parse file types       -> %s", fileDataDict[_pctrl->fileType].feature);
    LOG_INF("[UDS Config] Physical Addressing ID -> 0x%X", _pctrl->udsCfg.phyReqID);
    LOG_INF("[UDS Config] Function addressing ID -> 0x%X", _pctrl->udsCfg.funReqID);
    LOG_INF("[UDS Config] Response Addressing ID -> 0x%X", _pctrl->udsCfg.resDiagID);
    
    printf("====================================================================\r\n");

    return 0;
}

void destroy_config_file(void)
{
    if (iniFileDir != NULL) iniparser_freedict(iniFileDir);
}

tS16 parsing_compile_file(tS8* const _pname, file_type_t _type, tU32 _filesize, file_info_t* _pfileinfo)
{
    tS16 retval = 0;

    LOG_INF("[FILE] %s", _pname);
    _pfileinfo->buffer = (tU8*)malloc(sizeof(tU8) * _filesize);
    if (NULL == _pfileinfo->buffer) {
        LOG_ERR("Insufficient memory, Malloc failed to apply for memory!");
        return -5;
    }

    FILE* fp = (_type == FILE_BIN) ? fopen(_pname, "rb") : fopen(_pname, "r");
    if (NULL == fp) {
        LOG_ERR("Open %s failed!", _pname);
        return -6;
    }

    retval = parsing_file_info(_pfileinfo, _type, fp);

    printf("-------------------------------------------------------\r\n");
    for (tU8 idx = 0; idx < _pfileinfo->segCount; idx ++)
    {
        LOG_INF("Segment <%d> start address:    0x%08X", idx, _pfileinfo->segInfo[idx].startAddr);
        LOG_INF("Segment <%d> data length:      0x%08X", idx, _pfileinfo->segInfo[idx].dataLength);
        LOG_INF("Segment <%d> crc32 result:     0x%08X", idx, _pfileinfo->segInfo[idx].crc32);
        printf("-------------------------------------------------------\r\n");
    }

    LOG_INF("File total data length:       0x%08X", _pfileinfo->buffLength);
    LOG_INF("File crc32:                   0x%08X", _pfileinfo->crc32);
    printf("====================================================================\r\n");

    return retval;
}

void destroy_file_buffer(file_info_t* _pfile)
{
    for (tU8 idx = 0U; idx < 2U; idx ++)
    {
        if (_pfile[idx].buffer != NULL) free(_pfile[idx].buffer);
    }
}

/* End of file */
