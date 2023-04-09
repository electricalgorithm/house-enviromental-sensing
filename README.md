# House Environmental Monitoring
A Zephyr RTOS application shares the environmental data from BME280 sensor with BLE advertisement. 

- It shares temperature, pressure and the humidity.
- Uses Bluetooth Low Energy.
- Written with Zephyr -- ready to use in any board.
- No need for BME280 hardware, simulation included.

## Building
1. Enter the virtualenv created for your Zephyr environment.
2. Run the script "zephyr-env.sh" which is in the `zephyr/` directory in your Zephyr Environment.
3. Build the firmware with `west build -p auto -b esp32`.
4. Upload it with `west flash --esp-baud-rate 115200`.
5. You can watch the serial monitor with `west espressif monitor`

## Installing Zephyr
Please refer to the [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) on Zephyr's own documentation.
