/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\source\main.c
 * @Author       : jianhun
 * @CreationTime : 2023-06-25 23:13:28
 * @Version      : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-11-12 23:05:20
 * @Description  : 
 ******************************************************************************/

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include "FileParsing.h"
#include "CanUds.h"
#include "LinUds.h"
#include "boot.h"

/*******************************************************************************
* Defines and macros            (scope: module-local)
*******************************************************************************/
#define CONFIG_NAME             "Config.ini"
#define APP_BUFFER_SIZE         (64*1024) // 64k
#define DERIVE_BUFFER_SIZE      (8*1024) // 8K

/*******************************************************************************
* Typedefs and structures       (scope: module-local)
*******************************************************************************/

/*******************************************************************************
* Global variable definitions   (scope: module-local)
*******************************************************************************/
static const tS8 SoftwareVer[] = {"CAN_LIN_Tool Booterload_V1.2"};

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
/**
 * @brief software enter point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char const *argv[])
{
    config_file_t configFile = {0};
    file_info_t fileInfoTab[2];

    LOG_WAR("Software version: [%s]",SoftwareVer);
    printf("====================================================================\r\n");

    memset(&configFile, 0, sizeof(configFile));
    memset(fileInfoTab, 0, sizeof(fileInfoTab));

    if (parsing_config_file(CONFIG_NAME, &configFile)) goto __CLOSE_CFG_FILE;
    if (parsing_compile_file(
        configFile.pDerivePath,
        configFile.fileType,
        DERIVE_BUFFER_SIZE,
        &fileInfoTab[0])) goto __CLOSE_COMPILE_FILE;
    if (parsing_compile_file(
        configFile.pAppPath,
        configFile.fileType,
        APP_BUFFER_SIZE,
        &fileInfoTab[1])) goto __CLOSE_COMPILE_FILE;
    
    switch (configFile.comInfo.comType)
    {
    case COM_CAN:
        if (!set_can_device_init(configFile.pSerialName, CAN_TERMIAL_RESISTOR_ENABLE,
            configFile.comInfo.comBaud.value)) {
            boot_communction_process(fileInfoTab, &configFile);
        }
        set_can_device_deinit();
    break;
    case COM_LIN:
        if (!set_lin_device_init(configFile.pSerialName, configFile.comInfo.comBaud.value)) {
            boot_communction_process(fileInfoTab, &configFile);
        }
        set_lin_device_deinit();
    break;
    default:
    break;
    }

__CLOSE_COMPILE_FILE:
    destroy_file_buffer(fileInfoTab);

__CLOSE_CFG_FILE:
    destroy_config_file();

    system("pause");

    return 0;
}

/* End of file */
