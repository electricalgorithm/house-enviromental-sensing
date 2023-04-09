#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(HouseEnvironmentMonitor);


/* This array defines the BLE advertisement data. The data is used to advertise the
 * BLE services. The data is sent to the BLE scanner. The scanner can then use the
 * data to connect to the BLE device. The data needs to be defined in bt_le_adv_start().
 */
static const struct bt_data advertisment_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
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