cmake_minimum_required(VERSION 3.0.0)

project(UDS_Universal_Host)

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

set(USER_SUPPORT_DEBUG "-DDBG_RX_STREAM -DDBG_TX_STREAM")
set(USER_SUPPORT_CHINESE "-fexec-charset=gbk -finput-charset=utf-8")
set(USER_SUPPORT_LOG_COR "-D_ENABLE_LOG_FUNC_ -D_OS_SYSTEM_=_WIN_SYS2_")

set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -Wall ${USER_SUPPORT_CHINESE} ${USER_SUPPORT_LOG_COR}")
set(CMAKE_C_FLAGS_DEBUG "-g -Wall ${USER_SUPPORT_DEBUG} ${USER_SUPPORT_CHINESE} ${USER_SUPPORT_LOG_COR}")

set(ICON_FILE ./images/logo.rc)

include_directories(
    ./library
    ./source/boot
    ./source/fifo
    ./source/file_parsing
    ./source/log
    ./source/drive
    ./source/uds_can
    ./source/uds_lin)
link_directories(./library)

aux_source_directory(./source SRC1_LIST)
aux_source_directory(./source/boot SRC2_LIST)
aux_source_directory(./source/file_parsing SRC3_LIST)
aux_source_directory(./source/drive SRC4_LIST)
aux_source_directory(./source/uds_can SRC5_LIST)
aux_source_directory(./source/uds_lin SRC6_LIST)
aux_source_directory(./source/fifo SRC7_LIST)
aux_source_directory(./source/log SRC8_LIST)

SET(SRC_LIST ${SRC1_LIST} ${SRC2_LIST} ${SRC3_LIST} ${SRC4_LIST} ${SRC5_LIST} ${SRC6_LIST} ${SRC7_LIST} ${SRC8_LIST})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/debug)

add_executable(UDS_Universal_Host ${SRC_LIST} ${ICON_FILE})
target_link_libraries(UDS_Universal_Host USB2XXX.dll GenerateKey.dll iniparser.dll)