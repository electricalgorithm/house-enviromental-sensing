#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include <sensor_interface.h>

LOG_MODULE_DECLARE(HouseEnvironmentMonitor);
K_LIFO_DEFINE(sensor_value_lifo);

/***** SIMULATION SETTINGS *****/
#define DEFAULT_BATTERY_LEVEL 73
#define DEFAULT_TEMPERATURE 25
#define DEFAULT_PRESSURE 255
#define DEFAULT_HUMIDITY 50
static uint8_t sim_battery = DEFAULT_BATTERY_LEVEL;
static uint8_t sim_temperature = DEFAULT_TEMPERATURE;
static uint8_t sim_pressure = DEFAULT_PRESSURE;
static uint8_t sim_humidity = DEFAULT_HUMIDITY;

/* This function gets the device from the device tree. Note that the device
 * should have "bosch,bme280" as compatible string.
 * 
 * @param None
 * @return Pointer to the device, or NULL if the device is not found.
 */
const struct device *get_bme280_device(void) {
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
 *
 * @param dev Pointer to the device (struct device).
 * @return None
 */
void update_sensor_data(const struct device *dev) {
	// Variables to store sensor data.
	struct sensor_value temp_value, press_value, humidity_value;

	// Check if the device is ready.
	if (dev != NULL) {
		// LOG
		LOG_DBG("Fetching sensor data.");
		// Read the sensor data.
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press_value);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity_value);
	} else {
		// If the device is not ready, or not found, sample the sensor 
		// with dummy simulated values.
		// printk("INFO: Simulating sensor data.");

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
	LOG_DBG("Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f", temperature, pressure, humidity);

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
 * 
 * @param window_size The window size to use for the median filter.
 * @return Pointer to the sensor_value_data_t struct, or NULL if the window size
 *        is not an odd number.
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