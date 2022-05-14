 /** 
 **************************************************************
 * @file lib_hci.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief HCI interface library header file
 ***************************************************************
 */

#ifndef LIB_HCI_H
#define LIB_HCI_H

#include "tsk_ble.h"

typedef enum {
    hci_hr = 0,
    hci_json = 1
} hci_print_t;

uint8_t hci_encode_packet(ble_ctrl_request_packet_t* msg, char* buf,
        uint8_t buf_len);

uint8_t hci_print_shell(char* data, hci_print_t type);

#define HCI_PREAMBLE_BYTE 0xAAU
#define HCI_HEADER_LEN 0x2U
#define HCI_REQUEST_NIBBLE 0x1
#define HCI_RESPONSE_NIBBLE 0x2
#define HCI_MAX_PAYLOAD_LEN 16

#endif //LIB_HCI_H