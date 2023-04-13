#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <structs.h>

LOG_MODULE_DECLARE(HouseEnvironmentMonitor);

int16_t temp_reading = 0;
uint32_t press_reading = 0;
// uint16_t humid_reading = 0;

static void ble_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	LOG_INF("Notifications %s.", (value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
}

static ssize_t ble_read_operation(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_reading, sizeof(temp_reading));
}

BT_GATT_SERVICE_DEFINE(
    ess_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

    /* Temperature Sensor */
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, ble_read_operation, NULL, &temp_reading),
    BT_GATT_CCC(ble_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("BME280 Temperature Sensor", BT_GATT_PERM_READ),

    /* Pressure Sensor */
    BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, ble_read_operation, NULL, NULL),
    BT_GATT_CCC(ble_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("BME280 Pressure Sensor", BT_GATT_PERM_READ),

	/*
    Humidity Sensor
    BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, ble_read_operation, NULL, &humid_reading),
    BT_GATT_CCC(ble_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("BME280 Humidity Sensor", BT_GATT_PERM_READ),
	*/
);


/* This array defines the BLE advertisement data. The data is used to advertise the
 * BLE services. The data is sent to the BLE scanner. The scanner can then use the
 * data to connect to the BLE device. The data needs to be defined in bt_le_adv_start().
 */
static const struct bt_data advertisment_data[] = {
	BT_DATA_BYTES(
		BT_DATA_FLAGS,
		(BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)
	),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(
		BT_DATA_UUID16_ALL,
		BT_UUID_16_ENCODE(BT_UUID_ESS_VAL),
	),
};

/* This function is called when the BLE connection is established. If the connection
 * is successful, the function will print "INFO: Connected". Otherwise, it will print
 * "ERROR: Connection failed (error 0x%02x)". This function is called by the Zephyr
 * Bluetooth stack. No need to call it manually.
 */
static void callback_ble_connect(struct bt_conn *connection, uint8_t error) {
	if (error) {
		LOG_ERR("Connection failed (error 0x%02x).", error);
	} else {
		LOG_INF("A new BLE connection handled!");
	}
}

/* This function is called when the BLE connection is terminated. If the connection
 * is terminated, the function will print "Disconnected (reason 0x%02x)". This function
 * is called by the Zephyr Bluetooth stack. No need to call it manually.
 */
static void callback_ble_disconnect(struct bt_conn *connection, uint8_t reason) {
	LOG_INF("BLE disconnected (reason 0x%02x).", reason);
}

/* This macro defines the connection callbacks. The callbacks are called by the Zephyr
 * Bluetooth stack. No need to call them manually. Notice the similarity with the Linux
 * kernel callback macros.
 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = callback_ble_connect,
	.disconnected = callback_ble_disconnect,
};

/* This function is the main handler for the BLE advertisement. It should be called
 * after the BLE services are started. The function will start the BLE advertisement. 
 */
void start_bluetooth_advertisement(void) {
	LOG_INF("Bluetooth initialized.");

	int error = bt_le_adv_start(
        BT_LE_ADV_CONN_NAME,
        advertisment_data,
        ARRAY_SIZE(advertisment_data),
        NULL,
        0
    );

	if (error) {
		LOG_ERR("Advertising failed to start (err %d)", error);
		return;
	}

	LOG_DBG("Advertising successfully started.");
}

/* This function is called when the BLE pairing is cancelled. If the pairing is
 * cancelled, the function will print "Pairing cancelled: %s.". This function is
 * called by the Zephyr Bluetooth stack. No need to call it manually.
 */
static void callback_ble_pairing_cancel(struct bt_conn *connection) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(connection), addr, sizeof(addr));

	LOG_ERR("BLE pairing cancelled: %s.", addr);
}

/* This struct defines the BLE pairing callbacks. The callbacks are called by the
 * Zephyr Bluetooth stack. No need to call them manually. Notice the similarity with
 * the Linux kernel callback macros.
 */
struct bt_conn_auth_cb callback_ble_display_info = {
	.cancel = callback_ble_pairing_cancel,
};


ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	const uint16_t *u16 = attr->user_data;
	uint16_t value = sys_cpu_to_le16(*u16);
 	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}


void notify_ble_connected_device(struct values_for_ble_t *value_to_notify) {
	// Save the int part and dec part of the reading into an array.
	temp_reading = value_to_notify->temp_reading_int * 100 + value_to_notify->temp_reading_dec * 0.0001;
	press_reading = value_to_notify->press_reading_int * 10 + value_to_notify->press_reading_dec * 0.00001;

	// Convert the int part and dec part of the reading into a single integer. It's not realistically implemented.
	// humid_reading = value_to_notify->hum_reading_int * 100 + value_to_notify->hum_reading_dec;

	LOG_DBG("Notifying BLE connected device with temperature: %d.%d, pressure: %d.%d", 
		value_to_notify->temp_reading_int, value_to_notify->temp_reading_dec, 
		value_to_notify->press_reading_int, value_to_notify->press_reading_dec);
	LOG_DBG("Temperature: %d", temp_reading);
	LOG_DBG("Pressure: %d", press_reading);
	// LOG_INF("Humidity: %d", humid_reading);

	// BLE requires the data to be in little endian format.
	temp_reading = sys_cpu_to_le16(temp_reading);
	press_reading = sys_cpu_to_le16(press_reading);
	
	// Notify the connected BLE device about the new value.
	bt_gatt_notify_uuid(NULL, BT_UUID_TEMPERATURE, &ess_service.attrs[0], &temp_reading, sizeof(temp_reading));
	bt_gatt_notify_uuid(NULL, BT_UUID_PRESSURE, &ess_service.attrs[0], &press_reading, sizeof(press_reading));
	// bt_gatt_notify_uuid(NULL, BT_UUID_HUMIDITY, &ess_service.attrs[0], &humid_reading, sizeof(humid_reading));
}
