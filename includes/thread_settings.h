#ifndef THREAD_SETTINGS_H
#define THREAD_SETTINGS_H

#define STACKSIZE 1024
#define PRIORITY 7
#define THREAD_SENSOR_SLEEP 1	 // for 1 Hz, 1 seconds
#define THREAD_BLE_SLEEP 30		 // for 0.33 Hz, 30 seconds

void sensor_read_thread(void);	 // Thread function to read sensor data.
void ble_advertise_thread(void); // Thread function to advertise BLE services.

K_THREAD_DEFINE(sensor_read_thread_id, STACKSIZE, sensor_read_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_advertise_thread_id, STACKSIZE, ble_advertise_thread, NULL, NULL, NULL, PRIORITY, 0, 0);

#endif // THREAD_SETTINGS_H