 /** 
 **************************************************************
 * @file tsk_ble.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth task source file
 ***************************************************************
 */


/*********************************Includes*************************************/
#include <stdlib.h>
#include <string.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>


#include "tsk_ble.h"
#include "tsk_led.h"
#include "lib_hci.h"


/**************************Preprocessor Definitions****************************/

#define ERROR_VAL true
#define NO_ERROR_VAL false

#define GATT_WRITE_BUF_LEN HCI_HEADER_LEN + HCI_MAX_PAYLOAD_LEN + 1

/**********************************Macros**************************************/

LOG_MODULE_REGISTER(tsk_ble_log_module, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(ble_ctrl_thread_stack_area, BLE_CTRL_THREAD_STACK_SIZE);
struct k_thread ble_ctrl_thread_data;

K_THREAD_STACK_DEFINE(ble_led_thread_stack_area, BLE_LED_THREAD_STACK_SIZE);
struct k_thread ble_led_thread_data;

K_MSGQ_DEFINE(ble_ctrl_msgq, sizeof(ble_ctrl_request_packet_t), 10, 4);

/****************************Function Prototypes*******************************/

// thread entry point prototypes

void tsk_ble_ctrl_entry_point(void *a, void *b, void *c);
void tsk_ble_led_entry_point(void *a, void *b, void *c);

// callback prototypes

static void ble_disconnect_event_callback(struct bt_conn *conn, uint8_t reason);

static void ble_connect_event_callback(struct bt_conn *conn, uint8_t err);

static void ble_device_found_callback(const bt_addr_le_t *addr, int8_t rssi,
        uint8_t type, struct net_buf_simple *ad);

static bool ble_eir_found_callback(struct bt_data *eir_data, void *user_data);

static uint8_t ble_notify_event_callback(struct bt_conn *conn,
		struct bt_gatt_subscribe_params *params,
		const void *data, uint16_t length);

static void ble_write_event_callback(struct bt_conn *conn, uint8_t err,
		struct bt_gatt_write_params *params);

//helper function prototypes
static int start_scan(void);

static inline bool ble_stop_scan();

static inline bool ble_connect(bt_addr_le_t* addr);

static void begin_discovery(struct bt_conn *conn);

static bool ble_validate_eir_uuid(struct bt_data *eir_data);

static inline bool mobile_uuid_matches(struct bt_uuid* uuid);

static inline bool read_char_uuid_matches(struct bt_uuid* uuid);

static inline bool write_char_matches(struct bt_uuid* uuid);

static inline bool ccc_uuid_matches(struct bt_uuid* uuid);

static inline void discover_read_characteristic(struct bt_conn *conn,
        const struct bt_gatt_attr *attr);

static inline void discover_write_characteristic(struct bt_conn *conn,
        const struct bt_gatt_attr *attr);

static inline void discover_ccc_descriptor(struct bt_conn *conn,
        const struct bt_gatt_attr *attr);

static inline void subscribe_to_read_notifications(struct bt_conn *conn,
        const struct bt_gatt_attr *attr);

static void log_uuid(struct bt_uuid* uuid);

/******************************Global Variables********************************/

static struct bt_uuid_128 mobile_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(
        0x322d269c,0x9d12,0x40a2,0x9b18,0x2d832e084a47));

static struct bt_uuid_128 ble_read_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x08c58f2b,0x9f97,0x4310,0xb272,0x1322947406e7));

static struct bt_uuid_128 ble_write_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x45aff3ab,0x73bc,0x4fff,0x8f51,0xae909ff59908));

static struct bt_uuid_16 gatt_ccc_uuid = BT_UUID_INIT_16(0);

static struct bt_gatt_discover_params disc_params;

static struct bt_gatt_subscribe_params subscribe_params;

static struct bt_conn *default_conn;

bool ble_connected = 0;
bool ble_all_sensors = 0;

static uint8_t gatt_write_buf[GATT_WRITE_BUF_LEN];

static struct bt_conn_cb conn_callbacks = {
    .connected = ble_connect_event_callback,
    .disconnected = ble_disconnect_event_callback,
};

static struct bt_gatt_write_params write_gatt_char_params = {
    .func = ble_write_event_callback,
    .handle = 0,
    .offset = 0,
    .data = gatt_write_buf,
    .length = GATT_WRITE_BUF_LEN
};

/**************************Function Implementations****************************/

//Public Member Functions

int8_t ble_ctrl_request_send(ble_ctrl_request_packet_t* msg) {

    if (k_msgq_put(&ble_ctrl_msgq, msg, K_NO_WAIT) != 0) {
        //Queue full, return failure
        return -1;
    }

    LOG_DBG("BLE control request message sent: <%d, %d, \"%s\">", msg->dest,
        msg->payload_length, msg->payload);
    
    return 0;
}

k_tid_t ble_ctrl_tsk_init() {

    memcpy(&gatt_ccc_uuid, BT_UUID_GATT_CCC, sizeof(gatt_ccc_uuid));

    return k_thread_create(&ble_ctrl_thread_data, ble_ctrl_thread_stack_area,
        K_THREAD_STACK_SIZEOF(ble_ctrl_thread_stack_area),
        tsk_ble_ctrl_entry_point, NULL, NULL, NULL, BLE_CTRL_THREAD_PRIORITY,
        0, K_MSEC(10));

}

k_tid_t ble_led_tsk_init(void) {

    return k_thread_create(&ble_led_thread_data, ble_led_thread_stack_area,
        K_THREAD_STACK_SIZEOF(ble_led_thread_stack_area),
        tsk_ble_ctrl_entry_point, NULL, NULL, NULL, BLE_CTRL_THREAD_PRIORITY,
        0, K_MSEC(10));

}


//Task Entry Points

void tsk_ble_ctrl_entry_point(void *a, void *b, void *c) {

    LOG_DBG("BT LE thread entry");

    int bt_enable_err = 0;
    bt_enable_err = bt_enable(NULL);
    if (bt_enable_err) {
        LOG_ERR("Bluetooth initialisation failed (err %d)", bt_enable_err);
        return;
    }

    default_conn = NULL;

    LOG_INF("Bluetooth initialised successufully");

    bt_conn_cb_register(&conn_callbacks);

    while(1) {
        LOG_DBG("Attempting to start Bluetooth scan");
        if(start_scan() == NO_ERROR_VAL) {
            break;
        }
        k_msleep(500);
    }

    write_gatt_char_params.func = ble_write_event_callback;
    write_gatt_char_params.data = gatt_write_buf;
    write_gatt_char_params.length = 13;

    ble_ctrl_request_packet_t msg;

    while(1) {

        if(k_msgq_get(&ble_ctrl_msgq, &msg, K_FOREVER) == 0) {

            LOG_DBG("BLE control request message received: <%d, %d, \"%s\">",
                    msg.dest, msg.payload_length, msg.payload);
            
            if(!ble_connected) {

                LOG_ERR("Bluetooth command received but"
                            " bluetooth disconnected!");
                continue;
            }
            
            LOG_DBG("Attempting gatt write");
            hci_encode_packet(&msg, gatt_write_buf, GATT_WRITE_BUF_LEN);

            LOG_DBG("HCI Packet: %#02x,%#02x,%#02x,%s", gatt_write_buf[0],
                gatt_write_buf[1], gatt_write_buf[2], &gatt_write_buf[3]);

            int err = bt_gatt_write(default_conn, &write_gatt_char_params);

            if (err) {
                LOG_DBG("Gatt write failed!");
                continue;
            }

            if(msg.dest == ble_all) {
                if(msg.payload[0] == '1') {

                    ble_all_sensors = 1;
                } else {
                    ble_all_sensors = 0;
                }
            }

        }

    }

}

void tsk_ble_led_entry_point(void *a, void *b, void *c) {

    led_ctrl_packet_t msg = {led0_g, toggle_led, 0};

    while(1) {

        k_msgq_put(&led_ctrl_msgq, &msg, K_NO_WAIT);

        if(ble_connected) {
            k_msleep(100);
        } else {
            k_msleep(1000);
        }

    }
    
}

// Callbacks

static void ble_device_found_callback(const bt_addr_le_t *addr, int8_t rssi,
        uint8_t adv_type, struct net_buf_simple *adv) {
    
    char dev_addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_to_str(&addr->a, dev_addr_str, sizeof(dev_addr_str));
    LOG_DBG("[DEVICE]: %s, ADV event type %u, ADV eir_data length %u, RSSI %i",
        dev_addr_str, adv_type, adv->len, rssi);
    //printk("%s,%i\n",dev_addr_str,rssi);

    /* We're only interested in connectable events */
    if (adv_type == BT_GAP_ADV_TYPE_ADV_IND ||
            adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_data_parse(adv, ble_eir_found_callback, (void *)addr);
    }

}


static uint8_t ble_discovery_event_callback(struct bt_conn *conn,
        const struct bt_gatt_attr *attr,
        struct bt_gatt_discover_params *params) {

	if (attr == 0) {
		LOG_INF("Discover complete");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}
    
    log_uuid(disc_params.uuid);

	if (mobile_uuid_matches(disc_params.uuid)) {

        discover_read_characteristic(conn, attr);

	} else if (read_char_uuid_matches(disc_params.uuid)) {

        discover_ccc_descriptor(conn, attr);

	} else if (ccc_uuid_matches(disc_params.uuid)) {

        subscribe_to_read_notifications(conn, attr);

        discover_write_characteristic(conn, attr);

	} else if (write_char_matches(disc_params.uuid)) {

        LOG_DBG("Write characteristic uuid match");
        //no idea why +1 is required here
        write_gatt_char_params.handle = attr->handle+1;
        LOG_DBG("Write characteristic handle: %#x",
                write_gatt_char_params.handle);
        return BT_GATT_ITER_STOP;

    } else {

        LOG_ERR("Desired attribute(s) not found, stopping discovery");

    }

	return BT_GATT_ITER_STOP;
}

/* extended inofrmation response callback*/
static bool ble_eir_found_callback(struct bt_data *eir_data, void *user_data) {

	bt_addr_le_t* addr = (bt_addr_le_t*)user_data;
        
    if(ble_validate_eir_uuid(eir_data) == ERROR_VAL) {
        return ERROR_VAL;
    }
    
    if(ble_stop_scan() == ERROR_VAL) {
        return ERROR_VAL;
    }
    
    if(ble_connect(addr) == ERROR_VAL) {
        return ERROR_VAL;
    }
    
    return NO_ERROR_VAL;
	
}

static void ble_connect_event_callback(struct bt_conn *conn, uint8_t conn_err) {

    char addr[BT_ADDR_LE_STR_LEN];

    // Create human-readable address string for debugging
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_WRN("Failed to connect to %s (%u)", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		return;
	}

    if (conn != default_conn) {
        LOG_WRN("Connection event is not from the default connection");
        return;
    }

    ble_connected = true;

	LOG_INF("Connected: %s", addr);

    begin_discovery(conn);

}


static void ble_disconnect_event_callback(struct bt_conn *conn,
        uint8_t reason) {

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	if (default_conn == conn) {
		
        bt_conn_unref(default_conn);
        default_conn = NULL;

        ble_connected = false;

	}

    start_scan();

}

static uint8_t ble_notify_event_callback(struct bt_conn *conn,
		struct bt_gatt_subscribe_params *params,
		const void *data, uint16_t length) {
	if (!data) {
		LOG_INF("[UNSUBSCRIBED]");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

    //LOG_DBG("[NOTIFICATION] data %p length %u", data, length);

    if(ble_all_sensors == true) {
        
        //LOG_DBG("All mode");

        hci_print_shell((char*)data, hci_json);

    } else {

        hci_print_shell((char*)data, hci_hr);
    }

	return BT_GATT_ITER_CONTINUE;
}

static void ble_write_event_callback(struct bt_conn *conn, uint8_t err,
		struct bt_gatt_write_params *params) {

    if(err) {
        LOG_ERR("GATT write failed (err %d)", err);
    } else {
        LOG_INF("GATT write successful");
    }
        
}

// Helper Function Implementations

static int start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x60,
		.window     = 0x50
	};

	err = bt_le_scan_start(&scan_param, ble_device_found_callback);

	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return ERROR_VAL;
	}

	LOG_INF("Scanning successfully started");
    return NO_ERROR_VAL;
}


static inline bool ble_stop_scan() {

    int err = bt_le_scan_stop();
    if (err) {
        LOG_WRN("Stop LE scan failed (err %d)", err);
        return ERROR_VAL;
    }

    return NO_ERROR_VAL;

}

static inline bool ble_connect(bt_addr_le_t* addr) {

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
            BT_LE_CONN_PARAM_DEFAULT, &default_conn);
    
    if (err) {
        LOG_WRN("Create conn failed (err %d)", err);
    }

    return NO_ERROR_VAL;

}

static bool ble_validate_eir_uuid(struct bt_data *eir_data) {

    LOG_DBG("[Advertisement] type: %u, data_len: %u", eir_data->type,
                eir_data->data_len);

    if (eir_data->type != BT_DATA_UUID128_ALL) {
        LOG_DBG("UUID Type Mismatch!");
        return ERROR_VAL;
    }

    LOG_DBG("UUID Type Match!");
    

    if (eir_data->data_len != 16U) {
        LOG_DBG("Incorrect UUID length, ignoring");
        return ERROR_VAL;
    }
    
    struct bt_uuid uuid;

    bt_uuid_create(&uuid, eir_data->data, 16U);
    
    if (bt_uuid_cmp(&uuid, (struct bt_uuid *)&mobile_uuid) != 0) {
        LOG_DBG("Incorrect UUID, ignoring");
        return ERROR_VAL;
    }

    return NO_ERROR_VAL;

}

static void begin_discovery(struct bt_conn *conn) {

    LOG_DBG("Beginning Discovery");

    if (conn == default_conn) {
		disc_params.uuid = &mobile_uuid.uuid;
		disc_params.func = ble_discovery_event_callback;
		disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		disc_params.type = BT_GATT_DISCOVER_PRIMARY;

		int err = bt_gatt_discover(default_conn, &disc_params);
		if (err) {
			LOG_ERR("Discover failed(err %d)", err);
		} else {
            LOG_DBG("Service discovery succeeded");
        }
	}
}

static inline bool mobile_uuid_matches(struct bt_uuid* uuid) {
    return bt_uuid_cmp(disc_params.uuid, (struct bt_uuid*)&mobile_uuid) == 0;
}

static inline bool read_char_uuid_matches(struct bt_uuid* uuid) {
    return bt_uuid_cmp(disc_params.uuid, (struct bt_uuid*)&ble_read_uuid) == 0;
}

static inline bool ccc_uuid_matches(struct bt_uuid* uuid) {
    
    return bt_uuid_cmp(disc_params.uuid, (struct bt_uuid*)&gatt_ccc_uuid) == 0;
}

static inline bool write_char_matches(struct bt_uuid* uuid) {
    return bt_uuid_cmp(disc_params.uuid, (struct bt_uuid*)&ble_write_uuid) == 0;
}

static inline void discover_read_characteristic(struct bt_conn *conn,
        const struct bt_gatt_attr *attr) {
    
    LOG_DBG("Primary service handle: %#x", attr->handle);

    disc_params.uuid = &ble_read_uuid.uuid;
    disc_params.start_handle = attr->handle + 1;
    disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &disc_params);
    if (err) {
        LOG_ERR("Discover failed (err %d)", err);
    } else {
        LOG_DBG("Read characteristic discovery succeeded");
    }
    
}

static inline void discover_ccc_descriptor(struct bt_conn *conn,
        const struct bt_gatt_attr *attr) {
    
    LOG_DBG("Read characteristic handle: %#x", attr->handle);
    
    disc_params.uuid = &gatt_ccc_uuid.uuid;
    disc_params.start_handle = attr->handle + 2;
    disc_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
    subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

    int err = bt_gatt_discover(conn, &disc_params);
    if (err) {
        LOG_ERR("Discover failed (err %d)", err);
    } else {
        LOG_DBG("CCC descriptor discovery succeeded");
    }
    
}

static inline void subscribe_to_read_notifications(struct bt_conn *conn,
        const struct bt_gatt_attr *attr) {
    
    LOG_DBG("CCC descriptor handle: %#x", attr->handle);

    subscribe_params.notify = ble_notify_event_callback;
    subscribe_params.value = BT_GATT_CCC_NOTIFY;
    subscribe_params.ccc_handle = attr->handle;

    int err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err != 0 && err != -EALREADY) {
        LOG_ERR("Subscribe failed (err %d)", err);
    } else {
        LOG_INF("[SUBSCRIBED]");
    }
    
}

static inline void discover_write_characteristic(struct bt_conn *conn,
        const struct bt_gatt_attr *attr) {
    
    disc_params.uuid = &ble_write_uuid.uuid;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &disc_params);
    if (err) {
        LOG_ERR("Discover failed (err %d)", err);
    } else {
        LOG_INF("Write characteristic discovery succeeded");
    }
    
}

static void log_uuid(struct bt_uuid* uuid) {

    switch(uuid->type) {
        case BT_UUID_TYPE_16:
            LOG_DBG("UUID is 16-bit");
            uint16_t dbg_uuid_16 = ((struct bt_uuid_16*)uuid)->val;
            LOG_DBG("[UUID] %#04x", dbg_uuid_16);
            break;
        case BT_UUID_TYPE_128:
            LOG_DBG("UUID is 128-bit");
            uint8_t* dbg_uuid = ((struct bt_uuid_128*)uuid)->val;
            LOG_DBG("[UUID] %#02x,%#02x,%#02x,%#02x,%#02x,%#02x,"
            "%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x,%#02x",
            dbg_uuid[15],dbg_uuid[14],dbg_uuid[13],dbg_uuid[12],
            dbg_uuid[11],dbg_uuid[10],dbg_uuid[9],dbg_uuid[8],
            dbg_uuid[7],dbg_uuid[6],dbg_uuid[5],dbg_uuid[4],
            dbg_uuid[3],dbg_uuid[2],dbg_uuid[1],dbg_uuid[0]);
            break;
        default:
            break;
    }

}