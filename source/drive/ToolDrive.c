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
#include "usb_device.h"
#include "ToolDrive.h"

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
comm_tool_t CommToolCnt = {0};

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
tS16 comm_tool_scan(comm_tool_t* _pctrl, const tS8* _pDevicePort)
{
    (void)_pDevicePort;

    //扫描查找设备
    _pctrl->CommToolCnt = (tS16)USB_ScanDevice(_pctrl->DevicePort);
    LOG_INF("TCANLINPro device search completed!");
    if (_pctrl->CommToolCnt == 0) {
        _pctrl->mHand = 0;
        LOG_ERR("No TCANLINPro device connected!");
        return -1;
    } else {
        LOG_INF("TCANLINPro Device List:");
        for (tU16 idx = 0; idx < _pctrl->CommToolCnt; idx++)
            LOG_INF("No: %d - UTA0%d0%d-[%X]", idx, (_pctrl->DevicePort[idx] & 0xF0000000) >> 28, 
                (_pctrl->DevicePort[idx] & 0x0F000000) >> 24, _pctrl->DevicePort[idx]);
        printf("====================================================================\r\n");
    }

	return 0;
}

tS16 comm_tool_open(comm_tool_t* _pctrl, tU8 _size)
{
    tS8 funstr[256] = {0};
    DEVICE_INFO devinfo;

    _pctrl->mHand = _pctrl->DevicePort[_pctrl->CommToolNo];
    if (0 == USB_OpenDevice(_pctrl->mHand)) {
        LOG_ERR("Failed opened the TCANLINPro!");
        return -1;
    }
    LOG_INF("Successfully opened the TCANLINPro!");
    
    if (!DEV_GetDeviceInfo(_pctrl->mHand, &devinfo, funstr)) {
        LOG_ERR("Get device infomation error!");
        return -2;
    }
    
    LOG_INF("Firmware Info:");
    LOG_INF("Firmware Name: %s",devinfo.FirmwareName);
    LOG_INF("Firmware Build Date: %s",devinfo.BuildDate);
    LOG_INF("Firmware Version: v%d.%d.%d",(devinfo.FirmwareVersion>>24)&0xFF,(devinfo.FirmwareVersion>>16)&0xFF,devinfo.FirmwareVersion&0xFFFF);
    LOG_INF("Hardware Version: v%d.%d.%d",(devinfo.HardwareVersion>>24)&0xFF,(devinfo.HardwareVersion>>16)&0xFF,devinfo.HardwareVersion&0xFFFF);
    LOG_INF("Firmware Functions: %08X",devinfo.Functions); 
    LOG_INF("Firmware Functions: %s",funstr);

    return 0;
}

tS16 comm_tool_close(comm_tool_t* _pctrl)
{
    if (_pctrl->mHand != 0) USB_CloseDevice(_pctrl->mHand);
    LOG_INF("Successfully closed the TCANLINPro!");

    return 0;
}

tS16 comm_tool_send_data(comm_tool_t* _pctrl, tU8* _pdata, tU8 _size)
{
    (void)_pctrl;
    (void)_pdata;
    (void)_size;
    
    return 0;
}

tS16 comm_tool_rece_data(comm_tool_t* _pctrl, tU8* _pdata, tU8* _psize)
{
    (void)_pctrl;
    (void)_pdata;
    (void)_psize;

    return 0;
}

/* End of file */
