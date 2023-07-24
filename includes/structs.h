#include <zephyr/drivers/sensor.h>

struct values_for_ble_t {
	void *lifo_reserved;
	int32_t temp_reading_int;
	int32_t temp_reading_dec;
	int32_t press_reading_int;
	int32_t press_reading_dec;
	// int32_t hum_reading_int;
	// int32_t hum_reading_dec;
};

struct sensor_value_store_t {
	void *lifo_reserved;
	struct sensor_value temperature;
	struct sensor_value pressure;
	// struct sensor_value humidity;
};