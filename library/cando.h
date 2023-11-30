#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* cando_list_handle;
typedef void* cando_handle;

typedef enum {
    CAN_ERR_BUSOFF       		= 0x00000001U, 	/* bus off */
    CAN_ERR_RX_TX_WARNING  		= 0x00000002U,	/* reached warning level for RX/TX errors */
    CAN_ERR_RX_TX_PASSIVE  		= 0x00000004U,	/* reached error passive status RX/TX */
    CAN_ERR_OVERLOAD    		= 0x00000008U, 	/* bus overload */
    CAN_ERR_STUFF       		= 0x00000010U, 	/* bit stuffing error */
    CAN_ERR_FORM        		= 0x00000020U, 	/* frame format error */
    CAN_ERR_ACK          		= 0x00000040U, 	/* received no ACK on transmission */
    CAN_ERR_BIT_RECESSIVE		= 0x00000080U,	/* bit recessive error */
    CAN_ERR_BIT_DOMINANT		= 0x00000100U,	/* bit dominant error */
    CAN_ERR_CRC					= 0x00000200U,	/* crc error */
} cando_can_errorcode;

typedef enum {
    CANDO_MODE_NORMAL        = 0,
    CANDO_MODE_LISTEN_ONLY   = (1<<0),
    CANDO_MODE_LOOP_BACK     = (1<<1),
    CANDO_MODE_ONE_SHOT      = (1<<3),
    CANDO_MODE_NO_ECHO_BACK  = (1<<8),
} cando_flags_t;

enum {
    CANDO_ID_EXTENDED = 0x80000000,
    CANDO_ID_RTR      = 0x40000000,
    CANDO_ID_ERR      = 0x20000000,
    CANDO_ID_MASK     = 0x1FFFFFFF,
};

#pragma pack(push,1)

typedef struct {
    uint32_t echo_id;   // 0: echo; 0xFFFFFFFF: not echo
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;      // CAN Rx FIFO Overflow indicate
    uint8_t reserved;
    uint8_t data[8];
    uint32_t timestamp_us;
} cando_frame_t;

typedef struct {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} cando_bittiming_t;

#pragma pack(pop)

#define DLL

bool __stdcall DLL cando_list_malloc(cando_list_handle *list);
bool __stdcall DLL cando_list_free(cando_list_handle list);
bool __stdcall DLL cando_list_scan(cando_list_handle list);
bool __stdcall DLL cando_list_num(cando_list_handle list, uint8_t *num);

bool __stdcall DLL cando_malloc(cando_list_handle list, uint8_t index, cando_handle *hdev);
bool __stdcall DLL cando_free(cando_handle hdev);
bool __stdcall DLL cando_open(cando_handle hdev);
bool __stdcall DLL cando_close(cando_handle hdev);
wchar_t __stdcall DLL *cando_get_serial_number_str(cando_handle hdev);
bool __stdcall DLL cando_get_dev_info(cando_handle hdev, uint32_t *fw_version, uint32_t *hw_version);
bool __stdcall DLL cando_set_timing(cando_handle hdev, cando_bittiming_t *timing);

bool __stdcall DLL cando_start(cando_handle hdev, uint32_t mode);
bool __stdcall DLL cando_stop(cando_handle hdev);

bool __stdcall DLL cando_frame_send(cando_handle hdev, cando_frame_t *frame);
bool __stdcall DLL cando_frame_read(cando_handle hdev, cando_frame_t *frame, uint32_t timeout_ms);

bool __stdcall DLL cando_parse_err_frame(cando_frame_t *frame, uint32_t *err_code, uint8_t *err_tx, uint8_t *err_rx);

#ifdef __cplusplus
}
#endif
