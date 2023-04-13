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
#define DEFAULT_TEMPERATURE 250
#define DEFAULT_PRESSURE 100
// #define DEFAULT_HUMIDITY 99
static uint8_t sim_temperature = DEFAULT_TEMPERATURE;
static uint8_t sim_pressure = DEFAULT_PRESSURE;
// static uint8_t sim_humidity = DEFAULT_HUMIDITY;

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
	struct sensor_value temp_value, press_value;
	// struct sensor_value humidity_value;

	// Check if the device is ready.
	if (dev != NULL) {
		// LOG
		LOG_DBG("Fetching sensor data.");
		// Read the sensor data.
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press_value);
		// sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity_value);

		// Print the sensor data.
		LOG_DBG("Temperature: %.2f C, Pressure: %.2f Pa", 
			sensor_value_to_double(&temp_value),
			sensor_value_to_double(&press_value));

	} else {
		// If the device is not ready, or not found, sample the sensor 
		// with dummy simulated values.
		// printk("INFO: Simulating sensor data.");

		// Sample the sensor with dummy values.
		temp_value.val1 = sim_temperature;
		temp_value.val2 = sim_temperature * 2 * 1000;
		press_value.val1 = sim_pressure;
		press_value.val2 = sim_pressure * 2 * 1000;
		// humidity_value.val1 = sim_humidity;
		// humidity_value.val2 = sim_humidity * 2 * 1000;

		// Decrease the dummy values.
		sim_temperature -= 1;
		sim_pressure -= 3;
		// sim_humidity -= 2;

		// If the dummy values are below 0, reset them.
		if (sim_temperature < 0)
			sim_temperature = DEFAULT_TEMPERATURE;
		if (sim_pressure < 0)
			sim_pressure = DEFAULT_PRESSURE;
		// if (sim_humidity < 0)
		//	sim_humidity = DEFAULT_HUMIDITY;
	}

	// Debug LOG
	LOG_DBG("Temperature: %.2f C, Pressure: %.2f Pa", 
		sensor_value_to_double(&temp_value),
		sensor_value_to_double(&press_value)
	//	sensor_value_to_double(&humidity_value)
	);

	// Create a struct to store the sensor readings.
	struct sensor_value_store_t tx_data = {
		.temperature = temp_value,
		.pressure = press_value
	//	.humidity = humidity_value,
	};

	size_t datatype_size = sizeof(struct sensor_value_store_t);
	struct sensor_value_store_t *mem_ptr = k_malloc(datatype_size);
	__ASSERT_NO_MSG(mem_ptr != 0);
	memcpy(mem_ptr, &tx_data, datatype_size);

	k_lifo_put(&sensor_value_lifo, mem_ptr);
}

/* Read the queued sensor values for a given window size, and return the
 * median values for each sensor as a values_for_ble_t pointer. The returned
 * pointer must be freed by the caller. This function's output is used to
 * update the BLE advertisement data.
 * 
 * @param window_size The window size to use for the median filter.
 * @return Pointer to the values_for_ble_t struct, or NULL if the window size
 *        is not an odd number.
 */
struct values_for_ble_t* filter_sensor_value(uint8_t window_size) {
	// Check if the window size is an odd number.
	if (window_size % 2 == 0) {
		LOG_ERR("Window size must be an odd number.");
		return NULL;
	}

	// Create a window to store the sensor values.
	uint8_t temp_window_index = 0;
	struct sensor_value temp_window[window_size];
	uint8_t press_window_size = 0;
	struct sensor_value press_window[window_size];
	// uint8_t humid_window_size = 0;
	// struct sensor_value humid_window[window_size];
	
	// Get the last sensor values for windowing operation.
	for (int i = 0; i < window_size; i++) {
		// Get the last sensor values.
		struct sensor_value_store_t *rx_data = k_lifo_get(&sensor_value_lifo, K_FOREVER);
		
		// Store the sensor values in the window.
		temp_window[i] = rx_data->temperature;
		press_window[i] = rx_data->pressure;
		// humid_window[i] = rx_data->humidity;

		k_free(rx_data);
	}

	// Sort the windows.
	for (int i = 0; i < window_size; i++) {
		for (int j = i + 1; j < window_size; j++) {

			// Compare and reorder the temperature values.
			if (temp_window[i].val1 > temp_window[j].val1) {
				// Change the order of the values.
				struct sensor_value temp = temp_window[i];
				temp_window[i] = temp_window[j];
				temp_window[j] = temp;
			} else if (temp_window[i].val1 == temp_window[j].val1) {
				// If the values are equal, compare the decimal values.
				if (temp_window[i].val2 > temp_window[j].val2) {
					// Change the order of the values.
					struct sensor_value temp = temp_window[i];
					temp_window[i] = temp_window[j];
					temp_window[j] = temp;
				}
			}

			// Compare and reorder the pressure values.
			if (press_window[i].val1 > press_window[j].val1) {
				// Change the order of the values.
				struct sensor_value temp = press_window[i];
				press_window[i] = press_window[j];
				press_window[j] = temp;
			} else if (press_window[i].val1 == press_window[j].val1) {
				// If the values are equal, compare the decimal values.
				if (press_window[i].val2 > press_window[j].val2) {
					// Change the order of the values.
					struct sensor_value temp = press_window[i];
					press_window[i] = press_window[j];
					press_window[j] = temp;
				}
			}

			/*
			// Compare and reorder the humidity values.
			if (humid_window[i].val1 > humid_window[j].val1) {
				// Change the order of the values.
				struct sensor_value temp = humid_window[i];
				humid_window[i] = humid_window[j];
				humid_window[j] = temp;
			} else if (humid_window[i].val1 == humid_window[j].val1) {
				// If the values are equal, compare the decimal values.
				if (humid_window[i].val2 > humid_window[j].val2) {
					// Change the order of the values.
					struct sensor_value temp = humid_window[i];
					humid_window[i] = humid_window[j];
					humid_window[j] = temp;
				}
			}
			*/
		}
	}

	// Get the median value.
	struct sensor_value temp_median = temp_window[window_size / 2];
	struct sensor_value press_median = press_window[window_size / 2];
	// struct sensor_value humid_median = humid_window[window_size / 2];

	// Debug LOG
	LOG_DBG("Temperature median: %d.%d C, Pressure median: %d.%d Pa", 
		temp_median.val1, temp_median.val2, press_median.val1, press_median.val2);

	// Return the median as values_for_ble_t.
	struct values_for_ble_t median_values = {
		.temp_reading_int = temp_median.val1,
		.temp_reading_dec = temp_median.val2,
		.press_reading_int = press_median.val1,
		.press_reading_dec = press_median.val2,
	//	.hum_reading_int = humid_median.val1,
	//	.hum_reading_dec = humid_median.val2
	};

	// Allocate memory for the median values and return them.
	size_t datatype_size = sizeof(struct values_for_ble_t);
	struct values_for_ble_t *median_values_ptr = k_malloc(datatype_size);
	__ASSERT_NO_MSG(median_values_ptr != 0);
	memcpy(median_values_ptr, &median_values, datatype_size);

	return median_values_ptr;
}