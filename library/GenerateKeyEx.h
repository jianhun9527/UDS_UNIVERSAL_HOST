/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \KeyGenDll\GenerateKeyEx.h
 * @Author       : JF.Cheng
 * @CreationTime : 2023-10-21 20:37:31
 * @Version      : V1.0
 * @LastEditors  : JF.Cheng
 * @LastEditTime : 2023-10-21 20:37:31
 * @Brief        : 
 ******************************************************************************/

#ifndef _GENERATEKEYEX_H_
#define _GENERATEKEYEX_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
| Header file declaration
------------------------------------------------------------------------------*/

/*******************************************************************************
| Defines and macros            (scope: global)
------------------------------------------------------------------------------*/

/*******************************************************************************
| Typedefs and structures       (scope: global)
------------------------------------------------------------------------------*/
typedef enum VKeyGenResultEx
{
  KGRE_Ok = 0,
  KGRE_BufferToSmall = 1,
  KGRE_SecurityLevelInvalid = 2,
  KGRE_VariantInvalid = 3,
  KGRE_UnspecifiedError = 4
} VKeyGenResultEx;

/*******************************************************************************
| Exported Variables
------------------------------------------------------------------------------*/

/*******************************************************************************
| Exported function prototypes
------------------------------------------------------------------------------*/
extern unsigned int GetFileCRC32(const unsigned char* pbuff, const unsigned int size);
extern VKeyGenResultEx GenerateKeyEx(const unsigned char* pSeed, unsigned char* pKey, unsigned char level);

/*******************************************************************************
| Inline functions
------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _GENERATEKEYEX_H_ */
