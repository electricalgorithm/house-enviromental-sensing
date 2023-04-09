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

#endif // BLUETOOTH_INTERFACE_H