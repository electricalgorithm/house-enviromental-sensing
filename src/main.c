#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

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

/* Function interfaces to be used in main */
static const struct device *get_bme280_device(void);
static void fetch_sensor_data(const struct device*, struct sensor_value*, struct sensor_value*, struct sensor_value*);
static void start_bluetooth_advertisement(void);
static struct bt_conn_auth_cb callback_ble_display_info;

/***** Simulation Values (if needed) *****/
#define DEFAULT_BATTERY_LEVEL 73
#define DEFAULT_TEMPERATURE 25
#define DEFAULT_PRESSURE 255
#define DEFAULT_HUMIDITY 50
static uint8_t sim_battery = DEFAULT_BATTERY_LEVEL;
static uint8_t sim_temperature = DEFAULT_TEMPERATURE;
static uint8_t sim_pressure = DEFAULT_PRESSURE;
static uint8_t sim_humidity = DEFAULT_HUMIDITY;

void main(void) {
	int error = bt_enable(NULL);
	if (error) {
		printk("ERROR: Bluetooth initilisation is failed with error code %d.\n", error);
		return;
	}

	// Get the BME280 device.
	const struct device *bme280_device = get_bme280_device();

	// Start BLE services.
	start_bluetooth_advertisement();
	bt_conn_auth_cb_register(&callback_ble_display_info);

	// Variables to store sensor data.
	struct sensor_value temp, press, humidity;

	while (1) {
		k_sleep(K_SECONDS(1));

		/* Temperature, pressure and humidity measurements simulation */
		fetch_sensor_data(bme280_device, &temp, &press, &humidity);
		printk("T: %d C; P: %d kPa; H: %d %%\n", temp.val1, press.val1, humidity.val1);

		/* Heartrate measurements simulation */
		bt_hrs_notify(temp.val1);

		/* Battery level simulation */
		bt_bas_set_battery_level(press.val1);
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
		printk("ERROR: Connection failed (error 0x%02x).\n", error);
	} else {
		printk("INFO: Connected\n");
	}
}

/* This function is called when the BLE connection is terminated. If the connection
 * is terminated, the function will print "Disconnected (reason 0x%02x)". This function
 * is called by the Zephyr Bluetooth stack. No need to call it manually.
 */
static void callback_ble_disconnect(struct bt_conn *connection, uint8_t reason) {
	printk("INFO: Disconnected (reason 0x%02x).\n", reason);
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
	printk("INFO: Bluetooth initialized.\n");

	int error = bt_le_adv_start(
        BT_LE_ADV_CONN_NAME,
        advertisment_data,
        ARRAY_SIZE(advertisment_data),
        NULL,
        0
    );

	if (error) {
		printk("ERROR: Advertising failed to start (err %d)\n", error);
		return;
	}

	printk("INFO: Advertising successfully started.\n");
}

/* This function is called when the BLE pairing is cancelled. If the pairing is
 * cancelled, the function will print "Pairing cancelled: %s.". This function is
 * called by the Zephyr Bluetooth stack. No need to call it manually.
 */
static void callback_ble_pairing_cancel(struct bt_conn *connection) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(connection), addr, sizeof(addr));

	printk("ERROR: Pairing cancelled: %s.\n", addr);
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
		printk("WARNING: No BME280 sensor found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready. Check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("INFO: Found BME280 device \"%s\", \n", dev->name);
	return dev;
}

/* Read the I2C BME280 sensor, or simulate the sensor data if the sensor is not
 * available. 
 */
static void fetch_sensor_data(
	const struct device *dev, 
	struct sensor_value *temp,
	struct sensor_value *press,
	struct sensor_value *humidity
) {
	// Check if the device is ready.
	if (dev != NULL) {
		// LOG
		printk("INFO: Fetching sensor data.\n");
		// Read the sensor data.
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, humidity);
		return;
	}
	// If the device is not ready, or not found, sample the sensor 
	// with dummy simulated values.
	printk("INFO: Simulating sensor data.\n");

	// Sample the sensor with dummy values.
	temp->val1 = sim_temperature;
	temp->val2 = 0;
	press->val1 = sim_pressure;
	press->val2 = 0;
	humidity->val1 = sim_humidity;
	humidity->val2 = 0;

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