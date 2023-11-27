# UDS_UNIVERSAL_HOST

通用Bootloader上位机，通过修改底层驱动支持不同硬件

项目语言：C

管理工具：CMake

通信方式：CAN，LIN

解析文件：S19，HEX，BIN

支持功能：flash drive

### 231106 首次提交代码
1. 上位机底层是串口透传，当前实现UART->CAN
   1. 检索WIN系统注册表，自动搜寻CAN_LIN_Tool工具，默认使用注册表搜寻到的第一个设备
   2. 串口通信波特率19.2k，1个停止位，无校验位
   3. 串口支持异步通信，底层使用环形队列，应用层发送数据与硬件发送数据解耦
   4. 串口收发实现多线程，分三个线程：接受线程，发送线程，UDS应用线程
2. CAN通信功能实现
   1. 对串口通信包装，实现CAN驱动的收发函数
   2. 对CAN驱动包装，实现CAN UDS的收发函数
   3. 应用层仅使用CAN UDS的Request和Response函数实现CAN UDS的TP层功能
3. 文件解析
   1. 支持INI文件的解析，可通过修改INI的配置文件，修改软件的支持功能
   2. 支持S19文件的解析，通过修改INI配置文件
   3. 支持HEX文件的解析，通过修改INI配置文件
   4. 支持BIN文件的解析，通过修改INI配置文件
   5. 支持对解析文件的CRC32校验，当前校验方式以DLL形式提供
   6. 支持通过INI配置UDS的ID
4. Boot UDS实现
   1. 在boot文件中通过表驱的方式实现状态机转换，可通过修改表格实现不同的转换方式
   2. UDS服务可根据实际项目自行增改
   3. UDS刷写流程需与下位机的刷写流程一致，通过协议握手才能正常刷写
   4. UDS的27解锁服务，通过DLL形式提供解锁服务


### 231106 修复部分BUG，完成与下位机正常通信
1. 修复设备未连接系统退出异常问题
2. 修复解锁等级异常问题
3. 修复UDS负响应处理策略
4. 修复CRC32校验错误问题


### 231109 Note
1. 死机问题好像是偶发问题，今天未复现，无需解决
2. 优化CAN UDS接收流程
3. 解决因缓冲区BUFF溢出导致的接收失败
4. 新增默认会话等待功能，接收到响应后向后续流程跳转


### 231114 完成LIN UDS功能
1. 对串口通信包装，实现CAN驱动的收发函数
2. 对LIN驱动包装，实现LIN UDS的收发函数
3. 应用层仅使用LIN UDS的Request和Response函数实现LIN UDS的TP层功能
4. LIN和CAN兼容，共用一套顶层UDS服务


### 231115 完成UDS LOG功能
1. 记录发送和接收的数据
2. 记录通信错误码
3. 记录软件每帧运行时刻和总运行时间


### 231124 修改仓库名为 -> UDS_UNIVERSAL_HOST
1. 修改仓库名为UDS_UNIVERSAL_HOST
2. 并首次提交代码


### 231127 CAN_LIN_TOOL_V5740记录
1. 新增CAN id接收过滤功能


### 待实现功能
N/A


### 疑问点
1. 软件Debug时会因为free内存卡死，抛出异常，原因未知
2. 软件刷写过程中偶发异常卡死，原因未知，怀疑和底层透传驱动相关
