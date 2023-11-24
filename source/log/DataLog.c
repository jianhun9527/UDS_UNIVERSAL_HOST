/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\log\DataLog.c
 * @Author       : jianhun
 * @CreationTime : 2023-11-15 00:15:39
 * @Version       : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-11-15 12:20:20
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include <sys/time.h>
#include "DataLog.h"
#include <direct.h>
#include <time.h>

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static const tS8* num2str[] = {
    "00","01","02","03","04","05","06","07","08","09",
    "10","11","12","13","14","15","16","17","18","19",
    "20","21","22","23","24","25","26","27","28","29",
    "30","31","32","33","34","35","36","37","38","39",
    "40","41","42","43","44","45","46","47","48","49",
    "50","51","52","53","54","55","56","57","58","59",
};
static struct timeval creattime = {0, 0};
static FILE* pfile = NULL;

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
tS16 start_data_log(void)
{   
    tS8 filename[30] = {"log/ComLog_"};
    struct tm* ptimeinfo;
    time_t moment = 0;
    tS8* pstr = NULL;

    // Configuration file name, used to save message data
    moment = time(NULL);

    ptimeinfo = localtime(&moment);
    strcat(filename, num2str[ptimeinfo->tm_year - 100]);
    strcat(filename, num2str[ptimeinfo->tm_mon + 1]);
    strcat(filename, num2str[ptimeinfo->tm_mday]);
    strcat(filename, "_");
    strcat(filename, num2str[ptimeinfo->tm_hour]);
    strcat(filename, num2str[ptimeinfo->tm_min]);
    strcat(filename, num2str[ptimeinfo->tm_sec]);
    strcat(filename, ".asc");

    // Determine whether the folder exists
    if (_access("./log",0) != 0) _mkdir("./log");
    if ((pfile = fopen(filename, "w+")) == NULL) {
        LOG_ERR("Log file creation failed!");
        return -1;
    }

    gettimeofday(&creattime, NULL);
    pstr = ctime(&moment);
    fprintf(pfile,"data %sbase hex timestamps absolute\ninternal events logged\nBegin Triggerblock %s 0.000000 Start of measurement\n",pstr,pstr);

    return 0;
}

tS16 end_data_log(void)
{
    if (pfile != NULL) {
        fprintf(pfile,"End TriggerBlock");
        fclose(pfile);
        pfile = NULL;
    }

    return 0;
}

tS16 write_data_log(tU32 _id, tU8* _pdata, tU32 _len, tU8 _type)
{
    struct timeval time;
    tS32 sec, usec;

    if (pfile == NULL) return -1;

    gettimeofday(&time, NULL);
    sec = time.tv_sec - creattime.tv_sec;
    usec = time.tv_usec - creattime.tv_usec;
    if (time.tv_usec < creattime.tv_usec) {
        sec  -= 1;
        usec += 1000000;
    }

    switch (_type)
    {
    case DATA_ERROR_PROCESS:
        _len = 0U;
        fprintf(pfile, "%2d.%06d 1 %03hX Error code: %d", sec, usec, _id, (tS8)_pdata[0]);
    break;
    case DATA_SEND_PROCESS:
        fprintf(pfile, "%2d.%06d 1 %03hX Tx d %3d ", sec, usec, _id, _len);
    break;
    case DATA_RECV_PROCESS:
        fprintf(pfile, "%2d.%06d 1 %03hX Rx d %3d ", sec, usec, _id, _len);
    break;
    default:
        return -1;
    break;
    }

    for (tU32 idx = 0; idx < _len; idx ++)
    {
        fprintf(pfile, "%02hX ", _pdata[idx]);
    }
    fprintf(pfile,"\n");

    return 0;
}

/* End of file */
