/*******************************************************************************
 *
 * Copyright 2020-2022, Ningbo Shenglong Automotive Powertrain Co.,Ltd
 * All rights reserved.
 *
 ******************************************************************************/
/*******************************************************************************
 * @FileName     : \UDS_Uart_Code\library\commondef.h
 * @Author       : jianhun
 * @CreationTime : 2023-10-15 14:09:53
 * @Version       : V1.0
 * @LastEditors  : jianhun
 * @LastEditTime : 2023-10-28 13:33:56
 * @Description  : 
 ******************************************************************************/

/*
 * @Author       : JF.Cheng
 * @Date         : 2022-05-01 23:45:40
 * @LastEditors  : jianhun
 * @LastEditTime : 2022-07-09 16:03:38
 * @FilePath     : \UDS_IHR_CANable\source\inc\commondef.h
 * @Description  : Define common data types and macro definitions
 */
#ifndef __COMMONDEF_H__
#define __COMMONDEF_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header file declaration
*******************************************************************************/
#include <windows.h>
#include <process.h>
#include <stdio.h>

/*******************************************************************************
* Typedefs and structures       (scope: global)
*******************************************************************************/
typedef unsigned char       tU8;
typedef unsigned short      tU16;
typedef unsigned int        tU32;
typedef char                tS8;
typedef short               tS16;
typedef int                 tS32;

/*******************************************************************************
* Defines and macros            (scope: global)
*******************************************************************************/
#define _WIN_SYS_           1
#define _MAC_SYS_           2
#define _LINUX_SYS_         3
#define _WIN_SYS2_          4

// get a byte or word at the specified address
#define MEM_B(x_add)        (*((tU8*)(x_add)))
#define MEM_W(x_add)        (*((tU16*)(x_add)))

// Get the address of a variable (word width)
#define B_PTR(var)          ((tU8*)(void*)&(var))
#define W_PTR(var)          ((tU16*)(void*)&(var))

// find max and min
#define MAX(x,y)            (((x)>(y))?(x):(y))
#define MIN(x,y)            (((x)<(y))?(x):(y))

// Get the offset of a field in the structure (struct)
#define FPOS(type,field)    ((void*)(&(((type*)0)->field)))

// Convert two bytes to a Word according to LSB format
#define FLIPW(ray)          ((((tU16)(ray)[0])<<8)|((tU16)(ray)[1]))

// Convert a Word to two bytes according to LSB format
#define FLOPW(ray,val)      do{(ray)[0]= ((val)>>8);(ray)[1]=((val)&0xFF);}while(0)

// Get the high and low bytes of a word
#define WORD_LO(var)        ((tU8)((tU16)(var)&0xff))
#define WORD_HI(var)        ((tU8)((tU16)(var)>>8))

// Returns the nearest multiple of 8 greater than var
#define RND8(var)           ((((var)+7)>>3)<<3)

// Swap uppercase and lowercase letters
#define LOWER2UPPERCASE(c)  (((c)>='a'&&(c)<='z')?((c)-0x20):(c))
#define UPPER2LOWERCASE(c)  (((c)>='A'&&(c)<='Z')?((c)+0x20):(c))

// Determine character type
#define DECCHK(c)           ((c)>='0'&&(c)<='9')
#define HEXCHK(c)           (((c)>='0'&&(c)<='9')||((c)>='A'&&(c)<='F')||((c)>='a'&&(c)<='f'))

// Get the number of elements in a one-dimensional array
#define ARRAY_SIZE(a)       (((tU16)sizeof(a))/((tU16)sizeof(a[0])))

// For the structure in which the IO space is mapped to the storage space, 
// input and output processing
#define INP(port)           (*((volatile tU8*)(port)))
#define INPW(port)          (*((volatile tU16*)(port)))
#define INPDW(port)         (*((volatile uint32*)(port)))
#define OUTP(port,val)      (*((volatile tU8*)(port))=((tU8)(val)))
#define OUTPW(port,val)     (*((volatile tU16*)(port))=((tU16)(val)))
#define OUTPDW(port,val)    (*((volatile uint32*)(port))=((uint32)(val)))

/* Software debugging display control */
// Trace debugging with some macros
// __LINE__, __FILE__ , __DATE__, __TIME__, __STDC__
// debug output area
#ifdef _ENABLE_LOG_FUNC_
#if _OS_SYSTEM_ == _LINUX_SYS_ || _OS_SYSTEM_ == _MAC_SYS_
// color macro definition
#define NONE                        "\033[m"
#define RED                         "\033[0;32;31m"
#define LIGHT_RED                   "\033[1;31m"
#define GREEN                       "\033[0;32;32m"
#define LIGHT_GREEN                 "\033[1;32m"
#define BLUE                        "\033[0;32;34m"
#define LIGHT_BLUE                  "\033[1;34m"
#define DARY_GRAY                   "\033[1;30m"
#define CYAN                        "\033[0;36m"
#define LIGHT_CYAN                  "\033[1;36m"
#define PURPLE                      "\033[0;35m"
#define LIGHT_PURPLE                "\033[1;35m"
#define BROWN                       "\033[0;33m"
#define YELLOW                      "\033[1;33m"
#define LIGHT_GRAY                  "\033[0;37m"
#define WHITE                       "\033[1;37m"

#define _LOG_DBG(...)               do{printf(    GREEN "[log D]%s#%d: " NONE,__FILE__,__LINE__);printf(__VA_ARGS__);}while(0)
#define _LOG_INF(...)               do{printf(     NONE "[log I]%s#%d: " NONE,__FILE__,__LINE__);printf(__VA_ARGS__);}while(0)
#define _LOG_WAR(...)               do{printf(   YELLOW "[log W]%s#%d: " NONE,__FILE__,__LINE__);printf(__VA_ARGS__);}while(0)
#define _LOG_ERR(...)               do{printf(LIGHT_RED "[log E]%s#%d: " NONE,__FILE__,__LINE__);printf(__VA_ARGS__);}while(0)
#elif _OS_SYSTEM_ == _WIN_SYS_
// color macro definition
#define FONT_COLOR_BLACK            (0x00)
#define FONT_COLOR_LAKE_BLUE        (0x01)
#define FONT_COLOR_LIGHT_GREEN      (0x02)
#define FONT_COLOR_LGHT_BLUE        (0x03)
#define FONT_COLOR_LIGHT_RED        (0x04)
#define FONT_COLOR_LIGHT_PURPLE     (0x05)
#define FONT_COLOR_LIGHT_YELLOW     (0x06)
#define FONT_COLOR_LIGHT_WHITE      (0x07)
#define FONT_COLOR_GRAY             (0x08)
#define FONT_COLOR_DARK_BLUE        (0x09)
#define FONT_COLOR_GREEN            (0x0A)
#define FONT_COLOR_BLUE             (0x0B)
#define FONT_COLOR_RED              (0x0C)
#define FONT_COLOR_PURPLE           (0x0D)
#define FONT_COLOR_YELLOW           (0x0E)
#define FONT_COLOR_WHITE            (0x0F)
#define FONT_COLOR_DEFAULT          FONT_COLOR_LIGHT_WHITE

#define GET_COLOR_CONTROL_WIN       GetStdHandle(STD_OUTPUT_HANDLE)
#define SET_CTR_WIN_COLOR(color)    SetConsoleTextAttribute(GET_COLOR_CONTROL_WIN,(color))

#define SET_WIN_RED                 SET_CTR_WIN_COLOR(FONT_COLOR_RED)
#define SET_WIN_GREEN               SET_CTR_WIN_COLOR(FONT_COLOR_GREEN)
#define SET_WIN_YELLOW              SET_CTR_WIN_COLOR(FONT_COLOR_YELLOW)
#define SET_WIN_WHITE               SET_CTR_WIN_COLOR(FONT_COLOR_DEFAULT)

// debug output area
#define _LOG_DBG(...)               do{SET_WIN_GREEN;printf("[log D]%s#%d: ",__FILE__,__LINE__);SET_WIN_WHITE;printf(__VA_ARGS__);}while(0)
#define _LOG_INF(...)               do{SET_WIN_WHITE;printf("[log I]%s#%d: ",__FILE__,__LINE__);SET_WIN_WHITE;printf(__VA_ARGS__);}while(0)
#define _LOG_WAR(...)               do{SET_WIN_YELLOW;printf("[log W]%s#%d: ",__FILE__,__LINE__);SET_WIN_WHITE;printf(__VA_ARGS__);}while(0)
#define _LOG_ERR(...)               do{SET_WIN_RED;printf("[log E]%s#%d: ",__FILE__,__LINE__);SET_WIN_WHITE;printf(__VA_ARGS__);}while(0)
#elif _OS_SYSTEM_ == _WIN_SYS2_
// color macro definition
#define FONT_COLOR_BLACK            (0x00)
#define FONT_COLOR_LAKE_BLUE        (0x01)
#define FONT_COLOR_LIGHT_GREEN      (0x02)
#define FONT_COLOR_LGHT_BLUE        (0x03)
#define FONT_COLOR_LIGHT_RED        (0x04)
#define FONT_COLOR_LIGHT_PURPLE     (0x05)
#define FONT_COLOR_LIGHT_YELLOW     (0x06)
#define FONT_COLOR_LIGHT_WHITE      (0x07)
#define FONT_COLOR_GRAY             (0x08)
#define FONT_COLOR_DARK_BLUE        (0x09)
#define FONT_COLOR_GREEN            (0x0A)
#define FONT_COLOR_BLUE             (0x0B)
#define FONT_COLOR_RED              (0x0C)
#define FONT_COLOR_PURPLE           (0x0D)
#define FONT_COLOR_YELLOW           (0x0E)
#define FONT_COLOR_WHITE            (0x0F)
#define FONT_COLOR_DEFAULT          FONT_COLOR_LIGHT_WHITE

#define GET_COLOR_CONTROL_WIN       GetStdHandle(STD_OUTPUT_HANDLE)
#define SET_CTR_WIN_COLOR(color)    SetConsoleTextAttribute(GET_COLOR_CONTROL_WIN,(color))

#define SET_WIN_RED                 SET_CTR_WIN_COLOR(FONT_COLOR_RED)
#define SET_WIN_GREEN               SET_CTR_WIN_COLOR(FONT_COLOR_GREEN)
#define SET_WIN_YELLOW              SET_CTR_WIN_COLOR(FONT_COLOR_YELLOW)
#define SET_WIN_WHITE               SET_CTR_WIN_COLOR(FONT_COLOR_DEFAULT)

// debug output area
#define _LOG_DBG(...)               do{ SET_WIN_GREEN;printf("[log D] ");printf(__VA_ARGS__);printf("\r\n");SET_WIN_WHITE;}while(0)
#define _LOG_INF(...)               do{ SET_WIN_WHITE;printf("[log I] ");printf(__VA_ARGS__);printf("\r\n");SET_WIN_WHITE;}while(0)
#define _LOG_WAR(...)               do{SET_WIN_YELLOW;printf("[log W] ");printf(__VA_ARGS__);printf("\r\n");SET_WIN_WHITE;}while(0)
#define _LOG_ERR(...)               do{   SET_WIN_RED;printf("[log E] ");printf(__VA_ARGS__);printf("\r\n");SET_WIN_WHITE;}while(0)
#else
#define _LOG_DBG(...)
#define _LOG_INF(...)
#define _LOG_WAR(...)
#define _LOG_ERR(...)
#endif
#define LOG_DBG(...)                _LOG_DBG(__VA_ARGS__)
#define LOG_INF(...)                _LOG_INF(__VA_ARGS__)
#define LOG_WAR(...)                _LOG_WAR(__VA_ARGS__)
#define LOG_ERR(...)                _LOG_ERR(__VA_ARGS__)
#else
#define LOG_DBG(...)
#define LOG_INF(...)
#define LOG_WAR(...)
#define LOG_ERR(...)
#endif

/*******************************************************************************
* Exported Variables
*******************************************************************************/

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/

/*******************************************************************************
* Inline functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __COMMONDEF_H__