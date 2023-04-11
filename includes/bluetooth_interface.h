#ifndef BLUETOOTH_INTERFACE_H
#define BLUETOOTH_INTERFACE_H

/* This function is the main handler for the BLE advertisement. It should be called
 * after the BLE services are started. The function will start the BLE advertisement. 
 */
void start_bluetooth_advertisement(void);

/* This struct defines the BLE pairing callbacks. The callbacks are called by the
 * Zephyr Bluetooth stack. No need to call them manually. Notice the similarity with
 * the Linux kernel callback macros.
 */
extern struct bt_conn_auth_cb callback_ble_display_info;

/* This function is a wrapper for sending data to the BLE client. It is called by
 * the sensor_read_thread. The function will notify the BLE client.
 */
void notify_ble_connected_device(struct sensor_value_data_t*);

#endif // BLUETOOTH_INTERFACE_H