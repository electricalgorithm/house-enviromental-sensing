
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

/***** DEVICE TREE IMPORTS *****/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

/***** BLUETOOTH IMPORTS *****/
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

/***** THREADING SETTINGS *****/
#define STACKSIZE 1024
#define PRIORITY 7
#define THREAD_SENSOR_SLEEP 3	 // for 1 Hz, 1 seconds
#define THREAD_BLE_SLEEP 9		 // for 0.33 Hz, 30 seconds
void sensor_read_thread(void);	 // Thread function to read sensor data.
void ble_advertise_thread(void); // Thread function to advertise BLE services.
K_THREAD_DEFINE(sensor_read_thread_id, STACKSIZE, sensor_read_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_advertise_thread_id, STACKSIZE, ble_advertise_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
LOG_MODULE_REGISTER(HouseEnvironmentMonitor);

/***** SIMULATION SETTINGS *****/
#define DEFAULT_BATTERY_LEVEL 73
#define DEFAULT_TEMPERATURE 25
#define DEFAULT_PRESSURE 255
#define DEFAULT_HUMIDITY 50
static uint8_t sim_battery = DEFAULT_BATTERY_LEVEL;
static uint8_t sim_temperature = DEFAULT_TEMPERATURE;
static uint8_t sim_pressure = DEFAULT_PRESSURE;
static uint8_t sim_humidity = DEFAULT_HUMIDITY;

/***** SENSOR READING ELEMENTS *****/
#define DEFAULT_WINDOW_SIZE 5
struct sensor_value_data_t {
	void *lifo_reserved;
	double temp_reading;
	double press_reading;
	double humid_reading;
};
K_LIFO_DEFINE(sensor_value_lifo);

/***** FUNCTION INTERFACES *****/
static const struct device *get_bme280_device(void);
static struct bt_conn_auth_cb callback_ble_display_info;
static void update_sensor_data(const struct device*);
static void start_bluetooth_advertisement(void);
struct sensor_value_data_t* filter_sensor_value(uint8_t window_size);

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
		struct sensor_value_data_t *filtered_values = filter_sensor_value(DEFAULT_WINDOW_SIZE);
		
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
		LOG_ERR("Connection failed (error 0x%02x).\n", error);
	} else {
		LOG_INF("A new BLE connection handled!\n");
	}
}

/* This function is called when the BLE connection is terminated. If the connection
 * is terminated, the function will print "Disconnected (reason 0x%02x)". This function
 * is called by the Zephyr Bluetooth stack. No need to call it manually.
 */
static void callback_ble_disconnect(struct bt_conn *connection, uint8_t reason) {
	LOG_INF("BLE disconnected (reason 0x%02x).\n", reason);
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
static void start_bluetooth_advertisement(void) {
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

	LOG_ERR("BLE pairing cancelled: %s.\n", addr);
}

/* This macro defines the pairing callbacks. The callbacks are called by the Zephyr
 * Bluetooth stack. No need to call them manually. Notice the similarity with the Linux
 * kernel callback macros.
 */
static struct bt_conn_auth_cb callback_ble_display_info = {
	.cancel = callback_ble_pairing_cancel,
};


/* This function gets the device from the device tree. Note that the device
 * should have "bosch,bme280" as compatible string.
 */
static const struct device *get_bme280_device(void) {
	const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		LOG_WRN("No BME280 sensor found. Will be using simulator instead.");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("Error: Device \"%s\" is not ready. Check the driver initialization logs for errors.",
		       dev->name);
		return NULL;
	}

	LOG_INF("Found BME280 device \"%s\"", dev->name);
	return dev;
}

/* Read the I2C BME280 sensor, or simulate the sensor data if the sensor is not
 * available. 
 */
static void update_sensor_data(const struct device *dev) {
	// Variables to store sensor data.
	struct sensor_value temp_value, press_value, humidity_value;

	// Check if the device is ready.
	if (dev != NULL) {
		// LOG
		LOG_DBG("Fetching sensor data.\n");
		// Read the sensor data.
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press_value);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity_value);
	} else {
		// If the device is not ready, or not found, sample the sensor 
		// with dummy simulated values.
		// printk("INFO: Simulating sensor data.\n");

		// Sample the sensor with dummy values.
		temp_value.val1 = sim_temperature;
		temp_value.val2 = sim_temperature * 2 * 1000;
		press_value.val1 = sim_pressure;
		press_value.val2 = sim_pressure * 2 * 1000;
		humidity_value.val1 = sim_humidity;
		humidity_value.val2 = sim_humidity * 2 * 1000;

		// Decrease the dummy values.
		sim_temperature -= 1;
		sim_pressure -= 3;
		sim_humidity -= 2;

		// If the dummy values are below 0, reset them.
		if (sim_temperature < 0)
			sim_temperature = DEFAULT_TEMPERATURE;
		if (sim_pressure < 0)
			sim_pressure = DEFAULT_PRESSURE;
		if (sim_humidity < 0)
			sim_humidity = DEFAULT_HUMIDITY;
	}

	// Convert the sensor reading into a float.
	double temperature = sensor_value_to_double(&temp_value);
	double pressure = sensor_value_to_double(&press_value);
	double humidity = sensor_value_to_double(&humidity_value);

	// Debug LOG
	LOG_DBG("Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f\n", temperature, pressure, humidity);

	// Create a struct to store the sensor readings.
	struct sensor_value_data_t tx_data = {
		.temp_reading = temperature,
		.press_reading = pressure,
		.humid_reading = humidity,
	};

	size_t datatype_size = sizeof(struct sensor_value_data_t);
	struct sensor_value_data_t *mem_ptr = k_malloc(datatype_size);
	__ASSERT_NO_MSG(mem_ptr != 0);
	memcpy(mem_ptr, &tx_data, datatype_size);

	k_lifo_put(&sensor_value_lifo, mem_ptr);
}

/* Read the queued sensor values for a given window size, and return the
 * median values for each sensor as a sensor_value_data_t pointer. The returned
 * pointer must be freed by the caller. This function's output is used to
 * update the BLE advertisement data.
 */
struct sensor_value_data_t* filter_sensor_value(uint8_t window_size) {
	// Check if the window size is an odd number.
	if (window_size % 2 == 0) {
		LOG_ERR("Window size must be an odd number.");
		return NULL;
	}

	// Create a window to store the sensor values.
	uint8_t temp_window_index = 0;
	float temp_window[window_size];
	uint8_t press_window_size = 0;
	float press_window[window_size];
	uint8_t humid_window_size = 0;
	float humid_window[window_size];
	
	// Get the last sensor values for windowing operation.
	for (int i = 0; i < window_size; i++) {
		// Get the last sensor values.
		struct sensor_value_data_t *rx_data = k_lifo_get(&sensor_value_lifo, K_FOREVER);
		
		// Store the sensor values in the window.
		temp_window[i] = rx_data->temp_reading;
		press_window[i] = rx_data->press_reading;
		humid_window[i] = rx_data->humid_reading;

		k_free(rx_data);
	}

	// Sort the windows.
	for (int i = 0; i < window_size; i++) {
		for (int j = i + 1; j < window_size; j++) {
			if (temp_window[i] > temp_window[j]) {
				float temp = temp_window[i];
				temp_window[i] = temp_window[j];
				temp_window[j] = temp;
			}
			if (press_window[i] > press_window[j]) {
				float temp = press_window[i];
				press_window[i] = press_window[j];
				press_window[j] = temp;
			}
			if (humid_window[i] > humid_window[j]) {
				float temp = humid_window[i];
				humid_window[i] = humid_window[j];
				humid_window[j] = temp;
			}
		}
	}

	// Get the median value.
	float temp_median = temp_window[window_size / 2];
	float press_median = press_window[window_size / 2];
	float humid_median = humid_window[window_size / 2];

	// Debug LOG
	LOG_DBG("Temperature median: %.2f C, Pressure median: %.2f hPa, Humidity median: %.2f %%", temp_median, press_median, humid_median);

	// Return the median as sensor_value_data_t.
	struct sensor_value_data_t median_values = {
		.temp_reading = temp_median,
		.press_reading = press_median,
		.humid_reading = humid_median,
	};

	size_t datatype_size = sizeof(struct sensor_value_data_t);
	struct sensor_value_data_t *median_values_ptr = k_malloc(datatype_size);
	__ASSERT_NO_MSG(median_values_ptr != 0);
	memcpy(median_values_ptr, &median_values, datatype_size);

	return median_values_ptr;
}