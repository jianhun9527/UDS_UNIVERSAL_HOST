/**
  ******************************************************************************
  * @file    usb2canfd.h
  * $Author: wdluo $
  * $Revision: 447 $
  * $Date:: 2013-06-29 18:24:57 +0800 #$
  * @brief   usb2canfd相关函数和数据类型定义.
  ******************************************************************************
  * @attention
  *
  *<center><a href="http:\\www.toomoss.com">http://www.toomoss.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef __USB2CANFD_H_
#define __USB2CANFD_H_

#include <stdint.h>
#ifndef OS_UNIX
#include <Windows.h>
#else
#include <unistd.h>
#ifndef WINAPI
#define WINAPI
#endif
#endif
//函数返回值错误信息定义
#define CANFD_SUCCESS             (0)   //函数执行成功
#define CANFD_ERR_NOT_SUPPORT     (-1)  //适配器不支持该函数
#define CANFD_ERR_USB_WRITE_FAIL  (-2)  //USB写数据失败
#define CANFD_ERR_USB_READ_FAIL   (-3)  //USB读数据失败
#define CANFD_ERR_CMD_FAIL        (-4)  //命令执行失败
//CANFD_MSG.ID定义
#define CANFD_MSG_FLAG_RTR      (0x40000000)
#define CANFD_MSG_FLAG_IDE      (0x80000000)
#define CANFD_MSG_FLAG_ID_MASK  (0x1FFFFFFF)
//CANFD_MSG.Flags定义
#define CANFD_MSG_FLAG_BRS      (0x01)  //CANFD加速帧标志
#define CANFD_MSG_FLAG_ESI      (0x02)
#define CANFD_MSG_FLAG_FDF      (0x04)  //CANFD帧标志
#define CANFD_MSG_FLAG_TXD      (0x80)
//CANFD_DIAGNOSTIC.Flags定义
#define CANFD_DIAGNOSTIC_FLAG_NBIT0_ERR     (0x0001)//在发送报文（或应答位、主动错误标志或过载标志）期间，器件要发送显性电平（逻辑值为0的数据或标识符位），但监视的总线值为隐性。
#define CANFD_DIAGNOSTIC_FLAG_NBIT1_ERR     (0x0002)//在发送报文（仲裁字段除外）期间，器件要发送隐性电平（逻辑值为1的位），但监视到的总线值为显性。
#define CANFD_DIAGNOSTIC_FLAG_NACK_ERR      (0x0004)//发送报文未应答。
#define CANFD_DIAGNOSTIC_FLAG_NFORM_ERR     (0x0008)//接收报文的固定格式部分格式错误。
#define CANFD_DIAGNOSTIC_FLAG_NSTUFF_ERR    (0x0010)//在接收报文的一部分中，序列中包含了5个以上相等位，而报文中不允许出现这种序列。
#define CANFD_DIAGNOSTIC_FLAG_NCRC_ERR      (0x0020)//接收的报文的CRC校验和不正确。输入报文的CRC与通过接收到的数据计算得到的CRC不匹配。
#define CANFD_DIAGNOSTIC_FLAG_TXBO_ERR      (0x0080)//器件进入离线状态（且自动恢复）。
#define CANFD_DIAGNOSTIC_FLAG_DBIT0_ERR     (0x0100)//见NBIT0_ERR
#define CANFD_DIAGNOSTIC_FLAG_DBIT1_ERR     (0x0200)//见NBIT1_ERR
#define CANFD_DIAGNOSTIC_FLAG_DFORM_ERR     (0x0800)//见NFORM_ERR
#define CANFD_DIAGNOSTIC_FLAG_DSTUFF_ERR    (0x1000)//见NSTUFF_ERR
#define CANFD_DIAGNOSTIC_FLAG_DCRC_ERR      (0x2000)//见NCRC_ERR
#define CANFD_DIAGNOSTIC_FLAG_ESI_ERR       (0x4000)//接收的CAN FD报文的ESI标志置1
#define CANFD_DIAGNOSTIC_FLAG_DLC_MISMATCH  (0x8000)//DLC不匹配,在发送或接收期间，指定的DLC大于FIFO元素的PLSIZE
//CANFD_BUS_ERROR.Flags定义
#define CANFD_BUS_ERROR_FLAG_TX_RX_WARNING   (0x01)
#define CANFD_BUS_ERROR_FLAG_RX_WARNING      (0x02)
#define CANFD_BUS_ERROR_FLAG_TX_WARNING      (0x04)
#define CANFD_BUS_ERROR_FLAG_RX_BUS_PASSIVE  (0x08)
#define CANFD_BUS_ERROR_FLAG_TX_BUS_PASSIVE  (0x10)
#define CANFD_BUS_ERROR_FLAG_TX_BUS_OFF      (0x20)
//1.CANFD信息帧的数据类型定义
typedef  struct  _CANFD_MSG
{
    unsigned int    ID;           //报文ID,bit[30]-RTR,bit[31]-IDE,bit[28..0]-ID
    unsigned char   DLC;          //数据字节长度，可设置为-0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
    unsigned char   Flags;        //bit[0]-BRS,bit[1]-ESI,bit[2]-FDF,bit[6..5]-通道号,bit[7]-发送标志
    unsigned char   __Res0;       //保留
    unsigned char   TimeStampHigh;//时间戳高位
    unsigned int    TimeStamp;    //帧接收或者发送时的时间戳，单位为10us
    unsigned char   Data[64];     //报文的数据。
}CANFD_MSG;

//2.CANFD初始化配置数据类型定义
typedef  struct  _CANFD_INIT_CONFIG
{
    unsigned char Mode; //0-正常模式，1-自发自收模式
    unsigned char ISOCRCEnable;//0-禁止ISO CRC,1-使能ISO CRC
    unsigned char RetrySend;//0-禁止重发，1-无限制重发
    unsigned char ResEnable;//0-不接入内部120欧终端电阻，1-接入内部120欧终端电阻
    //波特率参数可以用TCANLINPro软件里面的波特率计算工具计算
    //仲裁段波特率参数,波特率=40M/NBT_BRP*(1+NBT_SEG1+NBT_SEG2)
    unsigned char NBT_BRP;
    unsigned char NBT_SEG1;
    unsigned char NBT_SEG2;
    unsigned char NBT_SJW;
    //数据段波特率参数,波特率=40M/DBT_BRP*(1+DBT_SEG1+DBT_SEG2)
    unsigned char DBT_BRP;
    unsigned char DBT_SEG1;
    unsigned char DBT_SEG2;
    unsigned char DBT_SJW;
    unsigned char __Res0[8];
}CANFD_INIT_CONFIG;
//3.CANFD诊断帧信息结构体定义
typedef struct _CANFD_DIAGNOSTIC {
    unsigned char NREC;//标称比特率接收错误计数
    unsigned char NTEC;//标称比特率发送错误计数
    unsigned char DREC;//数据比特率接收错误计数
    unsigned char DTEC;//数据比特率发送错误计数
    unsigned short ErrorFreeMsgCount;//无错误帧计数
    unsigned short Flags;//参考诊断标志定义
}CANFD_DIAGNOSTIC;
//4.CANFD总线错误信息结构体定义
typedef struct _CANFD_BUS_ERROR {
    unsigned char TEC;//发送错误计数
    unsigned char REC;//接收错误计数
    unsigned char Flags;//参考总线错误标志定义
    unsigned char __Res0;
}CANFD_BUS_ERROR;
//5.CAN 滤波器设置数据类型定义
typedef struct _CANFD_FILTER_CONFIG{
    unsigned char   Enable;   //使能该过滤器，1-使能，0-禁止
    unsigned char   Index;    //过滤器索引号，取值范围为0到31
    unsigned char __Res0;
    unsigned char __Res1;
    unsigned int    ID_Accept;//验收码ID,bit[28..0]为有效ID位，bit[31]为IDE
    unsigned int    ID_Mask;  //屏蔽码，对应bit位若为1，则需要对比对应验收码bit位，相同才接收
}CANFD_FILTER_CONFIG;

#ifdef __cplusplus
extern "C"
{
#endif
    int WINAPI CANFD_GetCANSpeedArg(int DevHandle, CANFD_INIT_CONFIG* pCanConfig, unsigned int SpeedBpsNBT, unsigned int SpeedBpsDBT);
    int WINAPI CANFD_Init(int DevHandle, unsigned char CANIndex, CANFD_INIT_CONFIG *pCanConfig);
    int WINAPI CANFD_StartGetMsg(int DevHandle, unsigned char CANIndex);
    int WINAPI CANFD_StopGetMsg(int DevHandle, unsigned char CANIndex);
    int WINAPI CANFD_SendMsg(int DevHandle, unsigned char CANIndex, CANFD_MSG *pCanSendMsg,unsigned int SendMsgNum);
    int WINAPI CANFD_GetMsg(int DevHandle, unsigned char CANIndex, CANFD_MSG *pCanGetMsg,int BufferSize);
    int WINAPI CANFD_ClearMsg(int DevHandle, unsigned char CANIndex);
    int WINAPI CANFD_SetFilter(int DevHandle, unsigned char CANIndex, CANFD_FILTER_CONFIG *pCanFilter,unsigned char Len);
    int WINAPI CANFD_GetDiagnostic(int DevHandle, unsigned char CANIndex, CANFD_DIAGNOSTIC *pCanDiagnostic);
    int WINAPI CANFD_GetBusError(int DevHandle, unsigned char CANIndex, CANFD_BUS_ERROR *pCanBusError);
    int WINAPI CANFD_Stop(int DevHandle, unsigned char CANIndex);
    /**
      * @brief  设置CAN调度表数据
      * @param  DevHandle 设备索引号
      * @param  CANIndex CAN通道号，取值0或者1
      * @param  pCanMsgTab CAN调度表列表首地址
      * @param  pMsgNum 调度表列表中每个调度表包含消息帧数
      * @param  pSendTimes 每个调度表里面帧发送次数，若为0xFFFF，则循环发送，通过调用CAN_StopSchedule函数停止调用
      * @param  MsgTabNum 调度表数
      * @retval 函数执行状态，小于0函数执行出错
      */
    int WINAPI CANFD_SetSchedule(int DevHandle, unsigned char CANIndex, CANFD_MSG *pCanMsgTab,unsigned char *pMsgNum,unsigned short *pSendTimes,unsigned char MsgTabNum);
    /**
      * @brief  启动调度表
      * @param  DevHandle 设备索引号
      * @param  CANIndex CAN通道号，取值0或者1
      * @param  MsgTabIndex CAN调度表索引号
      * @param  TimePrecMs 调度表时间精度，比如调度表里面最小帧周期为10ms，那么就建议设置为10
      * @param  OrderSend 设置为1则顺序发送调度表里面的帧，设置为0则并行发送调度表里面的帧 
      * @retval 函数执行状态，小于0函数执行出错
      */
    int WINAPI CANFD_StartSchedule(int DevHandle, unsigned char CANIndex, unsigned char MsgTabIndex,unsigned char TimePrecMs,unsigned char OrderSend);
    /**
      * @brief  停止调度方式发送数据
      * @param  DevHandle 设备索引号
      * @param  CANIndex CAN通道号，取值0或者1
      * @param  MsgTabIndex CAN调度表索引号
      * @retval 函数执行状态，小于0函数执行出错
      */
    int WINAPI CANFD_StopSchedule(int DevHandle, unsigned char CANIndex);
#ifdef __cplusplus
}
#endif
#endif

