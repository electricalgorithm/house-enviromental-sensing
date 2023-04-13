#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <structs.h>

extern struct k_lifo sensor_value_lifo;


/* This function gets the device from the device tree. Note that the device
 * should have "bosch,bme280" as compatible string.
 * 
 * @param None
 * @return Pointer to the device, or NULL if the device is not found.
 */
const struct device *get_bme280_device(void);

/* Read the I2C BME280 sensor, or simulate the sensor data if the sensor is not
 * available. 
 *
 * @param dev Pointer to the device (struct device).
 * @return None
 */
void update_sensor_data(const struct device *dev);

/* Read the queued sensor values for a given window size, and return the
 * median values for each sensor as a values_for_ble_t pointer. The returned
 * pointer must be freed by the caller. This function's output is used to
 * update the BLE advertisement data.
 * 
 * @param window_size The window size to use for the median filter.
 * @return Pointer to the values_for_ble_t struct, or NULL if the window size
 *        is not an odd number.
 */
struct values_for_ble_t* filter_sensor_value(uint8_t window_size);

#endif // SENSOR_INTERFACE_H