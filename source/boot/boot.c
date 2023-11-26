/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_UNIVERSAL_HOST\source\boot\boot.c
 * @Author       : jianhun
 * @CreationTime : 2023-10-27 03:21:52
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-27 01:21:13
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "GenerateKeyEx.h"
#include "DataLog.h"
#include "CanUds.h"
#include "LinUds.h"
#include "boot.h"
#include <time.h>

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define UDS_DATA_SIZE_MAX   300
#define FLASH_SECTOR_SIZE   512

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/
typedef tS16 (*uds_fun_req_t)(uds_data_t* );
typedef tS16 (*uds_fun_res_t)(uds_data_t*,tU16);

typedef struct thread_info
{
    file_info_t* pFileInfo;
    uds_cfg_t* pUdsCfg;
    uds_fun_req_t pComUdsRequest;
    uds_fun_res_t pComUdsresponse;
} thread_info_t;

typedef enum com_dir
{
    dSend,
    dRecv,
} com_dir_t;

typedef enum uds_step_name
{
    defSession,
    extSession,
    reqSeed_L1,
    submKey_L1,
    preProgDet,
    extSessECU,
    disDTCRecd,
    closNetMsg,
    proSession,
    reqSeed_FL,
    submKey_FL,
    wFingerInf,
    reqDrvDown,
    reqDrvTran,
    reqDrvExit,
    chkDrvEffe,
    eraseAppFs,
    reqAppDown,
    reqAppTran,
    reqAppExit,
    chkAppEffe,
    openNetMsg,
    reqMcuRest,
    usComplete,
} uds_step_name_t;

typedef struct fun_step
{
    char* stepname;
    uds_step_name_t nowstep;
    uds_step_name_t passstep;
    uds_step_name_t repestep;
    uds_step_name_t failstep;
    tU32 allowRepeat;
    tU32 allowRece;
    void* runFun;
} fun_step_t;

typedef struct com_data
{
    file_info_t* pFileInfo;
    uds_cfg_t* pUdsCfg;
} com_data_t;

typedef tS16 (*uds_step_fun_t)(com_dir_t, uds_step_name_t, uds_data_t*, com_data_t*);

typedef struct down_seg
{
    tU32    dataAddress;    // segment data start address
    tU32    dataLength;     // segment date length
    tU32    dataCrc32;      // segment data crc32
    tU32    dataOffset;     // segment data offset in data buffer
    tU32    dataCursor;     // segment data length has been downloaded
    tU32    maxDataLen;     // Maximum download length allowed by the client
    tU8     segCount;       // Downloaded segment
    tU8     blkCount;       // current downloading block
} down_seg_t;

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static tU16 P2ServerMax = 500;
static tU16 P21ServerMax = 5000;

/*******************************************************************************
* Global variable definitions   (scope: module-exported)
*******************************************************************************/

/*******************************************************************************
* Function prototypes           (scope: module-local)
*******************************************************************************/
static tU32 WINAPI boot_thread(LPVOID _pdata);
static tU8 addressInSameSector(tU32 lastaddr, tU32 nowaddr);

static tS16 ctl_Session_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 ctl_access_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 ctl_Rout_Ser_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 ctl_Dtc_Record_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 ctl_Net_Msg_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 wri_Cfg_Info_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 req_File_Down_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 req_File_Tran_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 req_File_Exit_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);
static tS16 req_App_Reset_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata);

/*******************************************************************************
* Function prototypes           (scope: module-exported)
*******************************************************************************/
static fun_step_t funStepList[] = {
    // stepname  nowstep  passstep  repestep  failstep  allowRepeat  allowRece  runFun
    {"defSession", defSession, extSession, defSession, usComplete, 1, 1, ctl_Session_fun},
    {"extSession", extSession, reqSeed_L1, extSession, usComplete, 1, 1, ctl_Session_fun},
    {"reqSeed_L1", reqSeed_L1, submKey_L1, extSession, usComplete, 0, 1, ctl_access_fun},
    {"submKey_L1", submKey_L1, preProgDet, extSession, usComplete, 0, 1, ctl_access_fun},
    {"preProgDet", preProgDet, extSessECU, extSession, usComplete, 0, 1, ctl_Rout_Ser_fun},
    {"extSessECU", extSessECU, disDTCRecd, extSession, usComplete, 0, 0, ctl_Session_fun},
    {"disDTCRecd", disDTCRecd, closNetMsg, disDTCRecd, usComplete, 0, 0, ctl_Dtc_Record_fun},
    {"closNetMsg", closNetMsg, proSession, closNetMsg, usComplete, 0, 0, ctl_Net_Msg_fun},
    {"proSession", proSession, reqSeed_FL, extSession, usComplete, 0, 1, ctl_Session_fun},
    {"reqSeed_FL", reqSeed_FL, submKey_FL, extSession, usComplete, 0, 1, ctl_access_fun},
    {"submKey_FL", submKey_FL, wFingerInf, extSession, usComplete, 0, 1, ctl_access_fun},
    {"wFingerInf", wFingerInf, reqDrvDown, extSession, usComplete, 0, 1, wri_Cfg_Info_fun},
    {"reqDrvDown", reqDrvDown, reqDrvTran, extSession, usComplete, 0, 1, req_File_Down_fun},
    {"reqDrvTran", reqDrvTran, reqDrvExit, reqDrvTran, usComplete, 0, 1, req_File_Tran_fun},
    {"reqDrvExit", reqDrvExit, chkDrvEffe, reqDrvDown, usComplete, 0, 1, req_File_Exit_fun},
    {"chkDrvEffe", chkDrvEffe, eraseAppFs, extSession, usComplete, 0, 1, ctl_Rout_Ser_fun},
    {"eraseAppFs", eraseAppFs, reqAppDown, extSession, usComplete, 0, 1, ctl_Rout_Ser_fun},
    {"reqAppDown", reqAppDown, reqAppTran, extSession, usComplete, 0, 1, req_File_Down_fun},
    {"reqAppTran", reqAppTran, reqAppExit, reqAppTran, usComplete, 0, 1, req_File_Tran_fun},
    {"reqAppExit", reqAppExit, chkAppEffe, eraseAppFs, usComplete, 0, 1, req_File_Exit_fun},
    {"chkAppEffe", chkAppEffe, openNetMsg, extSession, usComplete, 0, 1, ctl_Rout_Ser_fun},
    {"openNetMsg", openNetMsg, reqMcuRest, openNetMsg, usComplete, 0, 0, ctl_Net_Msg_fun},
    {"reqMcuRest", reqMcuRest, usComplete, usComplete, usComplete, 0, 1, req_App_Reset_fun},
};

static down_seg_t downSegData = {0};

/*******************************************************************************
* Function implementations      (scope: module-local)
*******************************************************************************/
static tU32 WINAPI boot_thread(LPVOID _pdata)
{
    thread_info_t* pthreadinfo = (thread_info_t*)_pdata;
    uds_step_name_t step = defSession;
    com_data_t comdata;
    uds_data_t udsdata;
    tS16 state = 0;
    tS16 timeout;

    LOG_INF("Start Boot Communcation...");

    memset(&udsdata, 0, sizeof(udsdata));
    memset(&comdata, 0, sizeof(comdata));
    memset(&downSegData, 0, sizeof(down_seg_t));

    udsdata.pData = (tU8*)malloc(sizeof(tU8) * UDS_DATA_SIZE_MAX);
    memset(udsdata.pData, 0, UDS_DATA_SIZE_MAX);
    udsdata.DataLen = UDS_DATA_SIZE_MAX;
    udsdata.flag = 0;
    udsdata.ReqID = pthreadinfo->pUdsCfg->phyReqID;
    udsdata.ResID = pthreadinfo->pUdsCfg->resDiagID;
    
    comdata.pFileInfo = pthreadinfo->pFileInfo;
    comdata.pUdsCfg = pthreadinfo->pUdsCfg;

    LOG_INF("Waiting for connecting...");

    start_data_log();

    for (;step != usComplete;)
    {
        if (((uds_step_fun_t)funStepList[step].runFun)(dSend, step, &udsdata, &comdata)) break;
        write_data_log(udsdata.ReqID, udsdata.pData, udsdata.DataLen, DATA_SEND_PROCESS);

        // uds send message
        state = pthreadinfo->pComUdsRequest(&udsdata);
        if (state) {
            LOG_ERR("UDS request failed, error code: %d", state);
            write_data_log(udsdata.ReqID, (tU8*)(&state), 1, DATA_ERROR_PROCESS);
            break;
#ifdef DBG_TX_STREAM
        } else {
            printf("[log I] Send Data:");
            for (tU16 idx = 0; idx < udsdata.DataLen; idx ++)
            {
                printf(" %02X", udsdata.pData[idx]);
            }
            printf("\r\n");
#endif
        }
        timeout = P2ServerMax;
        for (;;)
        {
            if (!funStepList[step].allowRece) {
                step = funStepList[step].passstep;
                Sleep(10);
                break;
            }
            // uds receive message
            udsdata.DataLen = UDS_DATA_SIZE_MAX;
            state = pthreadinfo->pComUdsresponse(&udsdata, timeout);
            if (state) {
                if ((CAN_UDS_REV_TIMEOUT == state || LIN_UDS_REV_TIMEOUT == state) && funStepList[step].allowRepeat) {
                    step = funStepList[step].repestep;
                } else {
                    LOG_ERR("Step: <%s> Response Error! Code: %d", funStepList[step].stepname, state);
                    step = funStepList[step].failstep;
                }
                write_data_log(udsdata.ResID, (tU8*)(&state), 1, DATA_ERROR_PROCESS);
                break;
            } else {
#ifdef DBG_RX_STREAM
                printf("[log I] Recv Data:");
                for (tU16 idx = 0; idx < udsdata.DataLen; idx ++)
                {
                    printf(" %02X", udsdata.pData[idx]);
                }
                printf("\r\n");
#endif
                write_data_log(udsdata.ResID, udsdata.pData, udsdata.DataLen, DATA_RECV_PROCESS);
                if (0x7F == udsdata.pData[0]) {
                    //nagetive response received
                    if (0x78 == udsdata.pData[2]) {
                        timeout = P21ServerMax;
                    } else {
                        LOG_ERR("Service: <%02X> NRC [%02X] Received!", udsdata.pData[1], udsdata.pData[2]);
                        step = funStepList[step].failstep;
                        state = -1;
                        break;
                    }
                } else {
                    //positive response received
                    state = ((uds_step_fun_t)funStepList[step].runFun)(dRecv, step, &udsdata, &comdata);
                    switch (state)
                    {
                    case -1:
                        step = funStepList[step].failstep;
                    break;
                    case 0:
                        step = funStepList[step].passstep;
                    break;
                    case 1:
                        step = funStepList[step].repestep;
                    break;
                    default:
                        LOG_ERR("Abnormal status, undefined status!");
                        step = funStepList[step].failstep;
                    break;
                    }
                    break;
                }
            }
        }
    }

    end_data_log();

    free(udsdata.pData);
    if (step == usComplete && state == 0) {
        LOG_DBG("Download flash file successfully!");
    } else {
        LOG_ERR("Flash file is downloaded failed!");
    }

    return 0;
}

static tU8 addressInSameSector(tU32 lastaddr, tU32 nowaddr)
{
    return (((lastaddr - 1) / FLASH_SECTOR_SIZE) == (nowaddr / FLASH_SECTOR_SIZE)) ? 1 : 0;
}

static tS16 ctl_Session_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case defSession:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x10;
            _pdata->pData[1] = 0x01;
            _pdata->DataLen = 2;
        break;
        case extSession:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x10;
            _pdata->pData[1] = 0x03;
            _pdata->DataLen = 2;
        break;
        case extSessECU:
            LOG_INF("Step: Enter whole vehicle external session...");
            _pdata->ReqID = _comdata.pUdsCfg->funReqID;
            _pdata->pData[0] = 0x10;
            _pdata->pData[1] = 0x83;
            _pdata->DataLen = 2;
        break;
        case proSession:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x10;
            _pdata->pData[1] = 0x02;
            _pdata->DataLen = 2;
        break;
        default:
            LOG_ERR("Bad session state, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case defSession:
            if (_pdata->DataLen == 6 && _pdata->pData[0] == 0x50 && _pdata->pData[1] == 0x01) {
                LOG_INF("Step: Enter default session...");
                P21ServerMax = ((tS16)_pdata->pData[4] << 8) + (tS16)_pdata->pData[5];
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x10);
                retval = -1;
            }
        break;
        case extSession:
            if (_pdata->DataLen == 6 && _pdata->pData[0] == 0x50 && _pdata->pData[1] == 0x03) {
                LOG_INF("Step: Enter external session...");
                P21ServerMax = ((tS16)_pdata->pData[4] << 8) + (tS16)_pdata->pData[5];
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x10);
                retval = -1;
            }
        break;
        case proSession:
            if (_pdata->DataLen == 6 && _pdata->pData[0] == 0x50 && _pdata->pData[1] == 0x02) {
                LOG_INF("Step: Enter Program Session...");
                P21ServerMax = ((tS16)_pdata->pData[4] << 8) + (tS16)_pdata->pData[5];
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x10);
                retval = -1;
            }
        break;
        case extSessECU:
            LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x10);
            retval = -1;
        default:
            LOG_ERR("Bad session state, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

static tS16 ctl_access_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    static tU8 seed[4] = {0x00, 0x00, 0x00, 0x00};
    static tU8 key[4]  = {0x00, 0x00, 0x00, 0x00};
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case reqSeed_L1:
            LOG_INF("Step: Request Seed - level 1...");
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x27;
            _pdata->pData[1] = 0x01;
            _pdata->DataLen = 2;
        break;
        case submKey_L1:
            LOG_INF("Step: Submit Key - level 1...");
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x27;
            _pdata->pData[1] = 0x02;
            memcpy(&_pdata->pData[2], key, 4);
            _pdata->DataLen = 6;
        break;
        case reqSeed_FL:
            LOG_INF("Step: Request Seed - level FBL...");
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x27;
            _pdata->pData[1] = 0x09;
            _pdata->DataLen = 2;
        break;
        case submKey_FL:
            LOG_INF("Step: Submit Key - level FBL...");
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x27;
            _pdata->pData[1] = 0x0A;
            memcpy(&_pdata->pData[2], key, 4);
            _pdata->DataLen = 6;
        break;
        default:
            LOG_ERR("Wrong security access level, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case reqSeed_L1:
            if (_pdata->DataLen == 6 && _pdata->pData[0] == 0x67 && _pdata->pData[1] == 0x01) {
                LOG_INF("Step: Accept Seed - level 1...");
                memcpy(seed, &_pdata->pData[2], 4);
                GenerateKeyEx(seed, key, 1);
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x27);
                retval = -1;
            }
        break;
        case submKey_L1:
            if (_pdata->DataLen == 2 && _pdata->pData[0] == 0x67 && _pdata->pData[1] == 0x02) {
                LOG_INF("Step: Unlock Security Level 1 Successfully!");
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x27);
                retval = -1;
            }
        break;
        case reqSeed_FL:
            if (_pdata->DataLen == 6 && _pdata->pData[0] == 0x67 && _pdata->pData[1] == 0x09) {
                LOG_INF("Step: Accept Seed - level FBL...");
                memcpy(seed, &_pdata->pData[2], 4);
                GenerateKeyEx(seed, key, 9);
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x27);
                retval = -1;
            }
        break;
        case submKey_FL:
            if (_pdata->DataLen == 2 && _pdata->pData[0] == 0x67 && _pdata->pData[1] == 0x0A) {
                LOG_INF("Step: Unlock Security Level 1 Successfully!");
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x27);
                retval = -1;
            }
        break;
        default:
            LOG_ERR("Wrong security access level, undefined!");
            return -1;
        break;
        }
    }

    return retval;
}

static tS16 ctl_Rout_Ser_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tU32 startaddr, datalength;
    tS16 retval;

    if (dSend == _dir) {
        retval = 0;
        switch (_step)
        {
        case preProgDet:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x31;
            _pdata->pData[1] = 0x01;
            _pdata->pData[2] = 0x02;
            _pdata->pData[3] = 0x03;
            _pdata->DataLen = 4;
            LOG_INF("Step: Check Flash Environment...");
        break;
        case chkDrvEffe:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x31;
            _pdata->pData[1] = 0x01;
            _pdata->pData[2] = 0x02;
            _pdata->pData[3] = 0x02;
            _pdata->DataLen = 4;
            LOG_INF("Step: Check Driver Download Completeness...");
        break;
        case chkAppEffe:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x31;
            _pdata->pData[1] = 0x01;
            _pdata->pData[2] = 0xFF;
            _pdata->pData[3] = 0x01;
            _pdata->DataLen = 4;
            LOG_INF("Step: Check App Download Completeness...");
        break;
        case eraseAppFs:
            downSegData.dataAddress = _comdata.pFileInfo[1].segInfo[downSegData.segCount].startAddr;
            downSegData.dataLength = _comdata.pFileInfo[1].segInfo[downSegData.segCount].dataLength;
            downSegData.dataOffset = _comdata.pFileInfo[1].segInfo[downSegData.segCount].buffIndex;
            downSegData.dataCrc32 = _comdata.pFileInfo[1].segInfo[downSegData.segCount].crc32;
            downSegData.dataCursor = 0U;
            downSegData.blkCount = 0U;

            if (0U == downSegData.segCount || 0U == addressInSameSector(
                    _comdata.pFileInfo[1].segInfo[downSegData.segCount - 1U].startAddr +
                        _comdata.pFileInfo[1].segInfo[downSegData.segCount - 1U].dataLength,
                    downSegData.dataAddress)) {
                startaddr = downSegData.dataAddress;
                datalength = downSegData.dataLength;
            } else {
                startaddr = ((downSegData.dataAddress / FLASH_SECTOR_SIZE) + 1U) * FLASH_SECTOR_SIZE;
                datalength = downSegData.dataLength + downSegData.dataAddress - startaddr + 1;
            }

            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x31;
            _pdata->pData[1] = 0x01;
            _pdata->pData[2] = 0xFF;
            _pdata->pData[3] = 0x00;
            _pdata->pData[4] = (tU8)(startaddr >> 24U);
            _pdata->pData[5] = (tU8)(startaddr >> 16U);
            _pdata->pData[6] = (tU8)(startaddr >> 8U);
            _pdata->pData[7] = (tU8)(startaddr >> 0U);
            _pdata->pData[8] = (tU8)(datalength >> 24U);
            _pdata->pData[9] = (tU8)(datalength >> 16U);
            _pdata->pData[10] = (tU8)(datalength >> 8U);
            _pdata->pData[11] = (tU8)(datalength >> 0U);
            _pdata->DataLen = 12;
            LOG_INF("Step: Erase App Flash - Address: 0x%08X,Length: 0x%08X...", startaddr, datalength);
        break;
        default:
            LOG_ERR("Bad routine service, undefined!");
            retval = -1;
        break;
        }
    } else {
        retval = -1;
        switch (_step)
        {
        case preProgDet:
            if (_pdata->DataLen == 5 && _pdata->pData[0] == 0x71 && _pdata->pData[1] == 0x01 && _pdata->pData[2] == 0x02 && _pdata->pData[3] == 0x03) {
                if (_pdata->pData[4] != 0x00) {
                    LOG_ERR("Step: Check Flash Environment Failed!");
                } else {
                    retval = 0;
                }
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x31);
            }
        break;
        case chkDrvEffe:
            if (_pdata->DataLen == 5 && _pdata->pData[0] == 0x71 && _pdata->pData[1] == 0x01 && _pdata->pData[2] == 0x02 && _pdata->pData[3] == 0x02) {
                if (_pdata->pData[4] != 0x00) {
                    LOG_INF("Step: Check Driver Download Completeness Failed!");
                } else {
                    retval = 0;
                }
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x31);
            }
        break;
        case chkAppEffe:
            if (_pdata->DataLen == 5 && _pdata->pData[0] == 0x71 && _pdata->pData[1] == 0x01 && _pdata->pData[2] == 0xFF && _pdata->pData[3] == 0x01) {
                if (_pdata->pData[4] != 0x00) {
                    LOG_INF("Step: Check App Download Completeness Failed!");
                } else {
                    retval = 0;
                }
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x31);
            }
        break;
        case eraseAppFs:
            if (_pdata->DataLen == 4 && _pdata->pData[0] == 0x71 && _pdata->pData[1] == 0x01 && _pdata->pData[2] == 0xFF && _pdata->pData[3] == 0x00) {
                retval = 0;
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x31);
            }
        break;
        default:
            LOG_ERR("Bad routine service, undefined!");
        break;
        }
    }

    return retval;
}

static tS16 ctl_Dtc_Record_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case disDTCRecd:
            LOG_INF("Step: Prohibit Recording DTC...");
            _pdata->ReqID = _comdata.pUdsCfg->funReqID;
            _pdata->pData[0] = 0x85;
            _pdata->pData[1] = 0x82;
            _pdata->DataLen = 2;
        break;
        default:
            LOG_ERR("Bad DTC control service, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case disDTCRecd:
           LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x85);
           retval = -1;
        break;
        default:
            LOG_ERR("Bad DTC control service, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

static tS16 ctl_Net_Msg_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step) 
        {
        case closNetMsg:
            LOG_INF("Step: Close Network Management Communication...");
            _pdata->ReqID = _comdata.pUdsCfg->funReqID;
            _pdata->pData[0] = 0x28;
            _pdata->pData[1] = 0x83;
            _pdata->pData[2] = 0x03;
            _pdata->DataLen = 3;
        break;
        case openNetMsg:
            LOG_INF("Step: Open Network Management Communication...");
            _pdata->ReqID = _comdata.pUdsCfg->funReqID;
            _pdata->pData[0] = 0x28;
            _pdata->pData[1] = 0x80;
            _pdata->pData[2] = 0x03;
            _pdata->DataLen = 3;
        break;
        default:
            LOG_ERR("Bad network control service, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step) 
        {
        case closNetMsg:
        case openNetMsg:
            LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x28);
            retval = -1;
        default:
            LOG_ERR("Bad network control service, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

static tS16 wri_Cfg_Info_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    struct tm* timeinfo;
    time_t rawtime;
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case wFingerInf:
            LOG_INF("Step: Write Finger information...");

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x2E;
            _pdata->pData[1] = 0xF1;
            _pdata->pData[2] = 0x84;
            _pdata->pData[3] = 0x03;
            _pdata->pData[4] = (tU8)(timeinfo->tm_yday - 100);
            _pdata->pData[5] = (tU8)(timeinfo->tm_mon + 1);
            _pdata->pData[6] = (tU8)(timeinfo->tm_mday);
            _pdata->pData[7] = '9';
            _pdata->pData[8] = '5';
            _pdata->pData[9] = '2';
            _pdata->pData[10] = '7';
            _pdata->pData[11] = 'N';
            _pdata->pData[12] = 'B';
            _pdata->DataLen = 13;
        break;
        default:
            LOG_ERR("Wrong configuration information written, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case wFingerInf:
            if (_pdata->DataLen == 3 && _pdata->pData[0] == 0x6E && _pdata->pData[1] == 0xF1 && _pdata->pData[2] == 0x84) {
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x2E);
                retval = -1;
            }
        break;
        default:
            LOG_ERR("Wrong configuration information written, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

static tS16 req_File_Down_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case reqDrvDown:
            downSegData.dataAddress = _comdata.pFileInfo[0].segInfo[downSegData.segCount].startAddr;
            downSegData.dataLength = _comdata.pFileInfo[0].segInfo[downSegData.segCount].dataLength;
            downSegData.dataOffset = _comdata.pFileInfo[0].segInfo[downSegData.segCount].buffIndex;
            downSegData.dataCrc32 = _comdata.pFileInfo[0].segInfo[downSegData.segCount].crc32;
            downSegData.dataCursor = 0;
            downSegData.blkCount = 0;

            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x34;
            _pdata->pData[1] = 0x00;
            _pdata->pData[2] = 0x44;
            _pdata->pData[3] = (tU8)(downSegData.dataAddress >> 24U);
            _pdata->pData[4] = (tU8)(downSegData.dataAddress >> 16U);
            _pdata->pData[5] = (tU8)(downSegData.dataAddress >> 8U);
            _pdata->pData[6] = (tU8)(downSegData.dataAddress >> 0U);
            _pdata->pData[7] = (tU8)(downSegData.dataLength >> 24U);
            _pdata->pData[8] = (tU8)(downSegData.dataLength >> 16U);
            _pdata->pData[9] = (tU8)(downSegData.dataLength >> 8U);
            _pdata->pData[10] = (tU8)(downSegData.dataLength >> 0U);
            _pdata->DataLen = 11;
            LOG_INF("Step: Request Driver Download - Address: 0x%08X, Length: 0x%08X...", downSegData.dataAddress, downSegData.dataLength);
        break;
        case reqAppDown:
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x34;
            _pdata->pData[1] = 0x00;
            _pdata->pData[2] = 0x44;
            _pdata->pData[3] = (tU8)(downSegData.dataAddress >> 24U);
            _pdata->pData[4] = (tU8)(downSegData.dataAddress >> 16U);
            _pdata->pData[5] = (tU8)(downSegData.dataAddress >> 8U);
            _pdata->pData[6] = (tU8)(downSegData.dataAddress >> 0U);
            _pdata->pData[7] = (tU8)(downSegData.dataLength >> 24U);
            _pdata->pData[8] = (tU8)(downSegData.dataLength >> 16U);
            _pdata->pData[9] = (tU8)(downSegData.dataLength >> 8U);
            _pdata->pData[10] = (tU8)(downSegData.dataLength >> 0U);
            _pdata->DataLen = 11;
            LOG_INF("Step: Request App Download - Address: 0x%08X,Length: 0x%08X...", downSegData.dataAddress, downSegData.dataLength);
        break;
        default:
            LOG_ERR("Bad file download request, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case reqDrvDown:
        case reqAppDown:
            if (_pdata->DataLen == 4 && _pdata->pData[0] == 0x74 && _pdata->pData[1] == 0x20) {
                downSegData.maxDataLen = ((tU32)_pdata->pData[2] << 8U) + ((tU32)_pdata->pData[3] << 0U);
                downSegData.maxDataLen = (downSegData.maxDataLen == 0 ? UDS_DATA_SIZE_MAX : downSegData.maxDataLen);
                downSegData.maxDataLen = (downSegData.maxDataLen > UDS_DATA_SIZE_MAX ? UDS_DATA_SIZE_MAX : downSegData.maxDataLen);
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x34);
                retval = -1;
            }
        break;
        default:
            LOG_ERR("Bad file download request, undefined!");
            retval = -1;
        break;
        }
    }
    
    return retval;
}

static tS16 req_File_Tran_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tU32 datapostion = 0;
    tU32 datasize = 0;
    tS16 retval = 0;

    if (dSend == _dir) {
        if (downSegData.dataCursor + downSegData.maxDataLen - 2 >= downSegData.dataLength) {
            datasize = downSegData.dataLength - downSegData.dataCursor;
        } else {
            datasize = downSegData.maxDataLen - 2;
        }
        datapostion = downSegData.dataCursor + downSegData.dataOffset;
        downSegData.blkCount ++;

        _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
        _pdata->pData[0] = 0x36;
        _pdata->pData[1] = downSegData.blkCount;
        _pdata->DataLen = datasize + 2;
        switch (_step)
        {
        case reqDrvTran:
            memcpy(&_pdata->pData[2], _comdata.pFileInfo[0].buffer + datapostion, datasize);
            LOG_INF("Step: Request Driver Data Transfer - block:[%03d], length[%03d]...", downSegData.blkCount, datasize);
        break;
        case reqAppTran:
            memcpy(&_pdata->pData[2], _comdata.pFileInfo[1].buffer + datapostion, datasize);
            LOG_INF("Step: Request App Data Transfer - block:[%03d], length:[%03d]...", downSegData.blkCount, datasize);
        break;
        default:
            LOG_ERR("Bad file transfer status, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case reqDrvTran:
        case reqAppTran:
            if (_pdata->DataLen == 2 && _pdata->pData[0] == 0x76 && _pdata->pData[1] == downSegData.blkCount) {
                if (downSegData.dataCursor + downSegData.maxDataLen - 2 >= downSegData.dataLength) {
                    downSegData.dataCursor = downSegData.dataLength;
                } else {
                    downSegData.dataCursor += downSegData.maxDataLen - 2;
                    retval = 1;
                }
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x36);
                retval = -1;
            }
        break;
        default:
            LOG_ERR("Bad file transfer status, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

static tS16 req_File_Exit_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
        _pdata->pData[0] = 0x37;
        _pdata->pData[1] = (tU8)(downSegData.dataCrc32 >> 24U);
        _pdata->pData[2] = (tU8)(downSegData.dataCrc32 >> 16U);
        _pdata->pData[3] = (tU8)(downSegData.dataCrc32 >> 8U);
        _pdata->pData[4] = (tU8)(downSegData.dataCrc32 >> 0U);
        _pdata->DataLen = 5;
        switch (_step)
        {
        case reqDrvExit:
            LOG_INF("Step: Request Driver Download Exit...");
        break;
        case reqAppExit:
            LOG_INF("Step: Request App Download Exit...");
        break;
        default:
            LOG_ERR("Bad file transfer abort request, undefined!");
            retval = -1;
        break;
        }
    } else {
        if (_pdata->DataLen == 5 && 
            _pdata->pData[1] == (tU8)(downSegData.dataCrc32 >> 24U) &&
            _pdata->pData[2] == (tU8)(downSegData.dataCrc32 >> 16U) &&
            _pdata->pData[3] == (tU8)(downSegData.dataCrc32 >> 8U) &&
            _pdata->pData[4] == (tU8)(downSegData.dataCrc32 >> 0U)) {

            downSegData.dataAddress = 0;
            downSegData.dataCursor = 0;
            downSegData.dataLength = 0;
            downSegData.dataOffset = 0;
            downSegData.maxDataLen = 0;
            downSegData.dataCrc32 = 0;
            downSegData.blkCount = 0;

            switch (_step)
            {
            case reqDrvExit:
                if (++ downSegData.segCount < _comdata.pFileInfo[0].segCount) {
                    LOG_INF("Step: Prepare to request the next driver block...");
                    retval = 1;
                } else {
                    LOG_INF("Step: Drive Download Exit Successfully...");
                    downSegData.segCount = 0;
                }
            break;
            case reqAppExit:
                if (++ downSegData.segCount < _comdata.pFileInfo[1].segCount) {
                    LOG_INF("Step: Prepare to request the next app block...");
                    retval = 1;
                } else {
                    LOG_INF("Step: App Download Exit Successfully...");
                    downSegData.segCount = 0;
                }
            break;
            default:
                LOG_ERR("Bad file transfer abort request, undefined!");
                retval = -1;
            break;
            }
        } else {
            LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x37);
            retval = -1;
        }
    }

    return retval;
}

static tS16 req_App_Reset_fun(com_dir_t _dir, uds_step_name_t _step, uds_data_t* _pdata, com_data_t _comdata)
{
    tS16 retval = 0;

    if (dSend == _dir) {
        switch (_step)
        {
        case reqMcuRest:
            LOG_INF("Step: Request ECU restart...");
            _pdata->ReqID = _comdata.pUdsCfg->phyReqID;
            _pdata->pData[0] = 0x11;
            _pdata->pData[1] = 0x01;
            _pdata->DataLen = 2;
        break;
        default:
            LOG_ERR("Error restarting service, undefined!");
            retval = -1;
        break;
        }
    } else {
        switch (_step)
        {
        case reqMcuRest:
            if (_pdata->DataLen == 2 && _pdata->pData[0] == 0x51 && _pdata->pData[1] == 0x01) {
            } else {
                LOG_ERR("Service: <%02X> Unexpected Message Received!", 0x11);
                retval = -1;
            }
        break;
        default:
            LOG_ERR("Error restarting service, undefined!");
            retval = -1;
        break;
        }
    }

    return retval;
}

/*******************************************************************************
* Function implementations      (scope: module-exported)
*******************************************************************************/
tS16 boot_communction_process(file_info_t* _pinfo, config_file_t* _pcfg)
{
    thread_info_t threadinfo = {0};
    tU32 threadID = 0;
    HANDLE handle;

    LOG_INF("Creating communication subprocess!");

    threadinfo.pFileInfo = _pinfo;
    threadinfo.pUdsCfg = &_pcfg->udsCfg;
    switch (_pcfg->comInfo.comType)
    {
    case COM_CAN:
        threadinfo.pComUdsRequest = can_uds_request;
        threadinfo.pComUdsresponse = can_uds_response;
    break;
    case COM_LIN:
        threadinfo.pComUdsRequest = lin_uds_request;
        threadinfo.pComUdsresponse = lin_uds_response;
    break;
    default:
        LOG_ERR("The toolbox does not currently support this communication method!");
        return -1;
    break;
    }

    handle = (HANDLE)_beginthreadex(NULL, 0, boot_thread, (void*)&threadinfo, 0, &threadID);
    if (handle != NULL) {
        if (WaitForSingleObject(handle, INFINITE) != WAIT_FAILED) {
            CloseHandle(handle);
            printf("====================================================================\r\n");
        } else {
            LOG_ERR("The subprocess failed to wait and the communication exited early!");
        }
    } else {
        LOG_ERR("The subprocess creation failed and communication exited!");
    }

    return 0;
}

/* End of file */
