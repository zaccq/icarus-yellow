 /** 
 **************************************************************
 * @file lib_hci.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief HCI interface library source file
 ***************************************************************
 */

#include <string.h>
#include <stdio.h>
#include <logging/log.h>

#include "lib_hci.h"
#include "lib_shell.h"

#define JSON_DID


LOG_MODULE_REGISTER(lib_hci_log_module, LOG_LEVEL_WRN);

uint8_t hci_encode_packet(ble_ctrl_request_packet_t* msg, char* buf,
        uint8_t buf_len) {

    uint8_t cursor = 0;

    if(buf_len < msg->payload_length + HCI_HEADER_LEN + 1) {
        return -1;
    }

    buf[cursor++] = HCI_PREAMBLE_BYTE;
    buf[cursor++] = (HCI_REQUEST_NIBBLE << 4) |
            ((msg->payload_length + 1) & 0xf);
    buf[cursor++] = msg->dest;

    if(msg->payload_length > 0) {
        
        for(uint8_t i = 0; i < msg->payload_length; i++) {

            buf[cursor++] = msg->payload[i];
            
        }

    }

    return cursor;

}

static int8_t hci_peripheral_unit(char* unit, uint8_t val) {

    ble_ctrl_dest_t peripheral = val;

    switch(peripheral) {
        case ble_hts221_temp:
            strcpy(unit, "C");
            break;
        case ble_hts221_humidity:
            strcpy(unit, "%");
            break;
        case ble_lps22_pressure:
            strcpy(unit, "hPa");
            break;
        case ble_ccs811_volatiles:
            strcpy(unit, "ppb");
            break;
        case ble_ccs811_co2:
            strcpy(unit, "ppm");
            break;
        case ble_lis2dh_x:
        case ble_lis2dh_y:
        case ble_lis2dh_z:
            strcpy(unit, "g");
            break;
        case ble_rssi:
            strcpy(unit, "dBmW");
            break;
        case ble_ultra:
            strcpy(unit, "cm");
            break;
        case ble_pushbutton:
            unit[0]= '\0';
            break;
        default:
            unit[0]= '\0';
            LOG_DBG("Response not from sensor!");
            break;
    }

    return 0;

}

static int8_t hci_peripheral_hr_name(char* buf, uint8_t val) {

    ble_ctrl_dest_t peripheral = val;

    switch(peripheral) {
        case ble_hts221_temp:
            strcpy(buf, "Temperature");
            break;
        case ble_hts221_humidity:
            strcpy(buf, "Humidity");
            break;
        case ble_lps22_pressure:
            strcpy(buf, "Pressure");
            break;
        case ble_ccs811_volatiles:
            strcpy(buf, "VOC concentration");
            break;
        case ble_ccs811_co2:
            strcpy(buf, "CO2 concentration");
            break;
        case ble_lis2dh_x:
            strcpy(buf, "X-axis acceleration");
            break;
        case ble_lis2dh_y:
            strcpy(buf, "Y-axis acceleration");
            break;
        case ble_lis2dh_z:
            strcpy(buf, "Z-axis acceleration");
            break;
        case ble_rssi:
            strcpy(buf, "RSSI");
            break;
        case ble_ultra:
            strcpy(buf, "Ultrasonic distance");
            break;
        case ble_pushbutton:
            strcpy(buf, "Pushbutton");
            break;
        default:
            LOG_DBG("Response not from sensor!");
            break;
    }

    return 0;

}

static int8_t hci_peripheral_json_name(char* buf, uint8_t val) {

#ifdef JSON_DID

    sprintf(buf, "%d", val);

#else

    ble_ctrl_dest_t peripheral = val;

    switch(peripheral) {
        case ble_hts221_temp:
            strcpy(buf, "Temperature");
            break;
        case ble_hts221_humidity:
            strcpy(buf, "Humidity");
            break;
        case ble_lps22_pressure:
            strcpy(buf, "Pressure");
            break;
        case ble_ccs811_volatiles:
            strcpy(buf, "VOC");
            break;
        case ble_ccs811_co2:
            strcpy(buf, "CO2");
            break;
        case ble_lis2dh_x:
            strcpy(buf, "X_Acceleration");
            break;
        case ble_lis2dh_y:
            strcpy(buf, "Y_Acceleration");
            break;
        case ble_lis2dh_z:
            strcpy(buf, "Z_Acceleration");
            break;
        case ble_pushbutton:
            strcpy(buf, "Pushbutton");
            break;
        default:
            LOG_DBG("Response not from sensor!");
            break;
    }

#endif

    return 0;

}

uint8_t hci_validate_packet(char* buf) {

    uint8_t length = buf[1] & 0xf;
    buf[length + HCI_HEADER_LEN] = 0;
        
    //LOG_DBG("preamble: %#02x, type/len: %02x, payload: %#02x\"%s\"",
    //        buf[0], buf[1], buf[2],buf+3);

    if(buf == NULL) {
        LOG_ERR("Packet is NULL, ignoring");
        return 1;
    }

    if(buf[0] != 0xAA) {
        LOG_ERR("Incorrect Preamble, ignoring");
        return 2;
    }

    uint8_t type = (buf[1] & 0xF0) >> 4;
    //LOG_DBG("Packet type: %d", type);

    if( type != HCI_RESPONSE_NIBBLE) {
        LOG_ERR("Packet is not a response, ignoring");
        return 3;
    }

    if(length == 0) {
        LOG_ERR("Empty response payload, ignoring");
        return 4;
    }

    return 0;

}

uint8_t hci_print_shell(char* data, hci_print_t type) {

    //LOG_DBG("Printing human readable HCI response, %hhu", data[2]);

    uint8_t err = hci_validate_packet(data);
    uint8_t length = (data[1] & 0xF) + HCI_HEADER_LEN;
    int8_t packet[19];
    memcpy(packet, data, length*sizeof(uint8_t));
    packet[length] = '\0';
    
    if(err != 0) {
        LOG_ERR("Malformed HCI packet");
        return err;
    }

    char name_buf[20];
    char unit_buf[8];

    switch(type) {
        case hci_hr:

            hci_peripheral_hr_name(name_buf, packet[2]);
            hci_peripheral_unit(unit_buf, packet[2]);

            if (packet[2] == ble_rssi) {

                shell_print(shell_backend_uart_get_ptr(), "{\"node\":\"%012llX\""
                ",\"rssi\":\"%d\",\"timestamp\":\"%llu\"}\n",
                (*(uint64_t*)(packet+1)) >> 16, *((int8_t*)(packet+9)), *((uint64_t*)(packet+10)));
                
            } else if (packet[2] == ble_ultra) {

                LOG_WRN("packet: %#02x,%#02x,%#02x,%#02x,%#02x,%#02x,"
                "%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x",
                packet[0],packet[1],packet[2],packet[3],packet[4],
                packet[5],packet[6],packet[7],packet[8],
                packet[9],packet[10],packet[11],packet[12],
                packet[13],packet[14],packet[15],packet[16],packet[17]);

                shell_print(shell_backend_uart_get_ptr(), "{\"node\":\"%012llX\""
                        ",\"ultra\":\"%d\",\"timestamp\":\"%llu\"}\n",
                        (*(uint64_t*)(packet+1)) >> 16, (uint8_t)packet[9],
                        *((uint64_t*)(packet+10)));
            
            } else if (packet[2] == ble_mpu9250) {

                LOG_WRN("packet: %#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,"
                "%#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,%#02hhx,"
                "%#02hhx,%#02hhx,%#02hhx,%#02hhx",
                packet[0],packet[1],packet[2],packet[3],packet[4],
                packet[5],packet[6],packet[7],packet[8],
                packet[9],packet[10],packet[11],packet[12],
                packet[13],packet[14],packet[15],packet[16],packet[17]);

                shell_print(shell_backend_uart_get_ptr(),
                "%%%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                packet[3], packet[4], packet[5], packet[6], packet[7],
                packet[8], packet[9], packet[10], packet[11], packet[12],
                packet[13], packet[14]);
            } else {
                printk("%s: %s %s\n", name_buf, packet + 3, unit_buf);
            }
            break;

        case hci_json:

            hci_peripheral_json_name(name_buf, packet[2]);

            printk("{%s : %s}\n", name_buf, packet + 3);
            break;

        default:
            break;
    }

    return 0;

}