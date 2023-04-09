#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <sensor_interface.h>
#include <bluetooth_interface.h>
#include <thread_settings.h>

LOG_MODULE_REGISTER(HouseEnvironmentMonitor);

/* Thread for producer task. This thread will read the sensor data and store it in
 * the sensor_value_lifo. The thread will sleep for THREAD_SENSOR_SLEEP seconds
 * before reading the sensor data again.
 */
void sensor_read_thread(void) {
	// Get the BME280 device.
	const struct device *bme280_device = get_bme280_device();

	// Start the sensor readings.
	while (1) {
		update_sensor_data(bme280_device);

		// Sleep for THREAD_SLEEP seconds.
		k_sleep(K_SECONDS(THREAD_SENSOR_SLEEP));
	}
}

/* Thread for consumer task. This thread will read the sensor data from the sensor_value_lifo
 * and advertise the BLE services. The thread will sleep for THREAD_BLE_SLEEP seconds before
 * starting the advertisment again.
 */
void ble_advertise_thread(void) {
	int error = bt_enable(NULL);
	if (error) {
		LOG_ERR("ERROR: Bluetooth initilisation is failed with error code %d.", error);
		return;
	}

	// Start BLE services.
	start_bluetooth_advertisement();
	bt_conn_auth_cb_register(&callback_ble_display_info);
	
	while (1) {
		// Get the sensor data from the filter.
		struct sensor_value_data_t *filtered_values = filter_sensor_value(5);
		
		// Check if the sensor data is available.
		if (filtered_values == NULL) {
			LOG_ERR("ERROR: No sensor data available.");
			continue;
		}

		LOG_INF("Sending sensor values with BLE: (%.2f C) (%.2f mmHg) (%.2f %%)",
			filtered_values->temp_reading, filtered_values->press_reading, filtered_values->humid_reading);

		/* Heartrate measurements simulation */
		bt_hrs_notify(filtered_values->temp_reading);

		/* Battery level simulation */
		bt_bas_set_battery_level(filtered_values->humid_reading);

		k_free(filtered_values);

		// Sleep for THREAD_SLEEP seconds.
		k_sleep(K_SECONDS(THREAD_BLE_SLEEP));
	}
}
