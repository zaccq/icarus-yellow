/*
***************************************************************************
* @file oslib/scu_drivers/scu_ble/scu_ble.c
* @author Matthew Kumar - 45303494
* @date 1-04-2021
* @brief Bluetooth functionalitiy
***************************************************************************
* EXTERNAL FUNCTIONS
***************************************************************************
* void scu_ble_init(void) - Function controls all bluetooth functionality
***************************************************************************
*/

#include <zephyr/types.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <string.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <scu_ble.h>
#include <logging/log.h>


//#define CONFIG_ULTRASONIC_NODE
//#define CONFIG_MOBILE_NODE

#ifdef CONFIG_ULTRASONIC_NODE

#include "ultra.h"

#endif //CONFIG_ULTRASONIC_NODE


#define STACK_SIZE 1024
#define PRIORITY 3

LOG_MODULE_REGISTER(scu_ble_log_module, LOG_LEVEL_WRN);

#define ERROR_VAL true
#define NO_ERROR_VAL false
#define HEADER_LEN 16
#define VAL_BEFORE_PAYLOAD 3
#define HCI_MAX_LEN 20


#ifdef CONFIG_ULTRASONIC_NODE

#define BLE_ADV_PARAMS BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
					    BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
					    0x100, \
					    0x110, NULL)
#define HCI_PRIMARY_SERVICE_VAL \
BT_UUID_128_ENCODE(0x89aa2ea4,0x6423,0x4874,0x8ffe,0x6d45353d4e61)

#else

#define BLE_ADV_PARAMS BT_LE_ADV_CONN_NAME
#define HCI_PRIMARY_SERVICE_VAL \
BT_UUID_128_ENCODE(0x322d269c,0x9d12,0x40a2,0x9b18,0x2d832e084a47)

#endif //CONFIG_ULTRASONIC_NODE
// Structs
struct packet ahuMessage;
struct packet recMessage;

// Message Queues
K_MSGQ_DEFINE(ahu_msg_to_scu, sizeof(struct packet), 10, 4);
K_MSGQ_DEFINE(scu_msg_to_ahu, sizeof(struct packet), 10, 4);
K_MSGQ_DEFINE(notify_msgq, sizeof(struct packet), 10, 4);

/* Custom Service Variables */

static struct bt_uuid_128 hci_uuid = BT_UUID_INIT_128(HCI_PRIMARY_SERVICE_VAL);

static struct bt_uuid_128 hci_client_read = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x08c58f2b,0x9f97,0x4310,0xb272,0x1322947406e7));

static struct bt_uuid_128 hci_client_write = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x45aff3ab,0x73bc,0x4fff,0x8f51,0xae909ff59908));

static uint8_t simulate_vnd;

static bool ble_connected = false;

// Functions Created

static ssize_t hci_read_callback(struct bt_conn *conn,
        const struct bt_gatt_attr *attr, void *buf, uint16_t len,
        uint16_t offset);

static ssize_t hci_write_callback(struct bt_conn *conn,
        const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
        uint16_t offset, uint8_t flags);

static void hci_ccc_cfg_changed(const struct bt_gatt_attr *attr,
        uint16_t value);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx);

static void connected(struct bt_conn *conn, uint8_t err);

static void disconnected(struct bt_conn *conn, uint8_t reason);

static void bt_ready(void);

static void hci_notify();

#ifdef CONFIG_MOBILE_NODE

static inline int start_scan(void);

static void ble_device_found_callback(const bt_addr_le_t *addr, int8_t rssi,
        uint8_t adv_type, struct net_buf_simple *adv);

#endif //CONFIG_MOBILE_NODE

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(hci_svc,
	BT_GATT_PRIMARY_SERVICE(&hci_uuid),
	BT_GATT_CHARACTERISTIC(&hci_client_read.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
		hci_read_callback, NULL, NULL),
	BT_GATT_CCC(hci_ccc_cfg_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&hci_client_write.uuid,
		BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		NULL, hci_write_callback, NULL),
);

#ifdef CONFIG_ULTRASONIC_NODE

static uint8_t ultra_dist = 0;

//bluetooth advertisement paremeters
struct bt_data ad[] = {
	BT_DATA(0x2f, &ultra_dist, sizeof(uint8_t))
};

#else

//bluetooth advertisement paremeters
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, HCI_PRIMARY_SERVICE_VAL),
};

#endif //CONFIG_ULTRASONIC_NODE


static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


/**
 * @brief Encodes the message for the the scu to use in 
 * the hci_notify to send to AHU
 * 
 * @param msg gives a reference to the packet
 * @param buffer an array to encode to
 * @param buffer_len a length of the array
 */
static uint8_t encode_scu_msg(struct packet *msg, 
		char* buffer, uint8_t buffer_len) {

	uint8_t cursor = 0;

	if (buffer_len < msg->len + VAL_BEFORE_PAYLOAD + 1) {
		return -1;
	}

	buffer[cursor++] = 0xAA;
    if(msg->dest >= 15) {
        buffer[cursor++] = ((msg->type & 0xf) << 4) | ((msg->len) & 0xf);
    } else {
        buffer[cursor++] = ((msg->type & 0xf) << 4) | ((msg->len + 1) & 0xf);
    }
	buffer[cursor++] = msg->dest;

	if (msg->payload != NULL) {
		for (uint8_t i = 0; i < msg->len; i++) {
			buffer[cursor++] = msg->payload[i];
		}
	}
    if(msg->type != 15) {
        LOG_WRN("buffer: %#02x,%#02x,%#02x,%#02x,%#02x,%#02x,"
            "%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x",
            buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],
            buffer[5],buffer[6],buffer[7],buffer[8],
            buffer[9],buffer[10],buffer[11],buffer[12],
            buffer[13],buffer[14],buffer[15],buffer[16],buffer[17]);
    }
    

	return cursor;
}

/**
 * @brief Function decodes an AHU message sent
 * 
 * @param msg gives a reference to the packet
 * @param buffer an array to encode to
 * @param buffer_len a length of the array
 */
// 
uint8_t decode_ahu_msg(char* msg) {

	if (msg == NULL) {
		LOG_ERR("Invalid Message");
		return -1;
	}

	if (msg[0] != 0xAA) {
		LOG_ERR("Invalid Preamble");
		return -2;
	}

	uint8_t type = (msg[1] & 0xF0) >> 4;
    LOG_DBG("Packet type: %d", type);

    if ( type != 0x1) {
        LOG_ERR("Packet is not a request, ignoring");
        return 3;
    }

    uint8_t length = msg[1] & 0xf;

    if (length == 0) {
        LOG_ERR("Empty response payload, ignoring");
        return 4;
    }

	if (msg[2] == 0) {
		LOG_ERR("Invalid Device/Sensor");
		return -4;
	}

	// Put all of these values into a struct and send this to the queue
	ahuMessage.preamble = 0xAA;
	ahuMessage.type = (msg[1] & 0xF0U) >> 4;
	ahuMessage.len = (msg[1] & 0xF) - 1; //ID should be here
	ahuMessage.dest = msg[2];

    LOG_DBG("AHU message: %#02x,%#02x,%#02x,%#02x,\"%s\"", ahuMessage.preamble,
            ahuMessage.type, ahuMessage.dest, ahuMessage.len,
            ahuMessage.payload);

	// adding the packet data to the struct so it comes cleanly
	memset(ahuMessage.payload, '\0', sizeof(ahuMessage.payload));
	strncpy(ahuMessage.payload, msg+VAL_BEFORE_PAYLOAD, ahuMessage.len);

	return 0; // If all good!

}

//callback for readable characteristic
static ssize_t hci_read_callback(struct bt_conn* conn,
        const struct bt_gatt_attr *attr, void* buf, uint16_t len,
        uint16_t offset) {
	const char* value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

//callback for writeable characteristic
// For SCU, this is where we get the values and process
static ssize_t hci_write_callback(struct bt_conn* conn,
        const struct bt_gatt_attr *attr,
        const void* buf, uint16_t len, uint16_t offset, uint8_t flags) {
    
	//uint8_t* value = attr->user_data;

	// Print out the package
    LOG_DBG("preamble: %#02x, type/len: %02x, payload: %#02x\"%s\"",
            ((uint8_t*)buf)[0], ((uint8_t*)buf)[1], ((uint8_t*)buf)[2],
            ((char*)buf+3));

	// Decode the package
	decode_ahu_msg((char*)buf);

	// Send struct to queue
	if (k_msgq_put(&ahu_msg_to_scu, &ahuMessage, K_NO_WAIT) != 0) {
		k_msgq_purge(&ahu_msg_to_scu);
	}	

	return len;
}

// callback for enabling notifications
static void hci_ccc_cfg_changed(const struct bt_gatt_attr *attr,
        uint16_t value) {
	simulate_vnd = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

//callback in case maximum packet length changes
void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
	} else {
        ble_connected = true;
		LOG_INF("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    ble_connected = false;
	LOG_INF("Disconnected (reason 0x%02x)", reason);
}

static void bt_ready(void) {

	int err;

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BLE_ADV_PARAMS, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");

}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("Passkey for %s: %06u", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

/**
 * @brief Sending the scu message back to the AHU
 * 
 * @param data is a reference to the packet struct
 */
static void hci_notify(struct packet* data) {

	char buff[19] = {0}; // 19 characters is the maximum that the packet can be
	int rc;

    memset(buff, 0x0, 19);

	encode_scu_msg(data, buff, 19);

	// debugging purposes
    LOG_DBG("Msg Buffer: \"%s\"", data->payload);
    LOG_DBG("Msg Buffer Length: \"%d\"", data->len);
    LOG_DBG("Notify Buffer: \"%s\"", buff);

	rc = bt_gatt_notify(NULL, &hci_svc.attrs[1], &buff, data->len +
            VAL_BEFORE_PAYLOAD);

	LOG_DBG("Notification complete");
}

void scu_ble_thread(void) {
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);

	bt_conn_auth_cb_register(&auth_cb_display);

    #ifdef CONFIG_MOBILE_NODE
    while(1) {
        LOG_DBG("Attempting to start Bluetooth scan");
        if (start_scan() == NO_ERROR_VAL) {
            break;
        }
        k_msleep(500);
    }
    #endif //CONFIG_MOBILE_NODE

    struct packet data;

	while (1) {

		if (k_msgq_get(&scu_msg_to_ahu, &data, K_FOREVER) == 0) {
			LOG_DBG("Got here"); // debugging purposes
			if (ble_connected) {
                k_msgq_put(&notify_msgq, &data, K_NO_WAIT);
			}
        }
	}
}

#ifdef CONFIG_MOBILE_NODE
static inline void send_rssi(const bt_addr_t* addr, int8_t rssi) {
    char dev_addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_to_str(addr, dev_addr_str, sizeof(dev_addr_str));

    struct packet data;
    uint8_t cursor = 0;

    data.preamble = 0xAA;
    data.type = 2;
    data.dest = 15;

    for (uint8_t i = 0; i < 6; i++) {
        data.payload[cursor++] = addr->val[i];
        
    }

    data.payload[cursor++] = rssi;

    uint64_t sample_timestamp = k_ticks_to_us_near64(k_uptime_ticks());

    for (uint8_t i = 0; i < 8; i++) {
        data.payload[cursor++] = ((char*)&sample_timestamp)[i];
        
    }

    data.len = 15;

    k_msgq_put(&notify_msgq, &data, K_NO_WAIT);

}

static inline void send_ultra(const bt_addr_t* addr, uint8_t* buf) {
    char dev_addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_to_str(addr, dev_addr_str, sizeof(dev_addr_str));

    struct packet data;
    uint8_t cursor = 0;

    data.preamble = 0xAA;
    data.type = 2;
    data.dest = 16;

    for (uint8_t i = 0; i < 6; i++) {
        data.payload[cursor++] = addr->val[i];
        
    }

    data.payload[cursor++] = buf[0];
    printk("ultra: %#02hhx\n", buf[0]);

    uint64_t sample_timestamp = k_ticks_to_us_near64(k_uptime_ticks());

    for (uint8_t i = 0; i < 8; i++) {

        data.payload[cursor++] = ((char*)&sample_timestamp)[i];
        
    }

    data.len = 15;

    k_msgq_put(&notify_msgq, &data, K_NO_WAIT);

}

static void ble_device_found_callback(const bt_addr_le_t *addr, int8_t rssi,
        uint8_t adv_type, struct net_buf_simple *adv) {

    if (ble_connected) {

        send_rssi(&addr->a, rssi);
        if(adv->len >= 3 && adv->data[1] == 0x2f) {

            send_ultra(&addr->a, &(adv->data[2]));

        }
        
	}

}

static inline int start_scan(void) {
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x20,
		.window     = 0x20
	};

	err = bt_le_scan_start(&scan_param, ble_device_found_callback);

	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return ERROR_VAL;
	}

	LOG_INF("Scanning successfully started");
    return NO_ERROR_VAL;
}

#endif //CONFIG_MOBILE_NODE

#ifdef CONFIG_ULTRASONIC_NODE
void ad_update_thread(void *a, void *b, void *c) {

    while(1) {

        uint8_t data;

        if(k_msgq_get(&ultra_q, &data, K_FOREVER) == 0) {

            ultra_dist = data;
            bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);

        }

    }

}

K_THREAD_DEFINE(ad_upd_id, STACK_SIZE, ad_update_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

#endif //CONFIG_ULTRASONIC_NODE

void notify_thread(void *a, void *b, void *c) {

    while(1) {

        struct packet data;

        if(k_msgq_get(&notify_msgq, &data, K_FOREVER) == 0) {

            hci_notify(&data);

        }

    }

}

K_THREAD_DEFINE(notify_ble_id, STACK_SIZE, notify_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);


K_THREAD_DEFINE(scu_ble_id, STACK_SIZE, scu_ble_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

