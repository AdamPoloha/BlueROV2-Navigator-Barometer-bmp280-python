This fork unfortunately does not seek to provide documentation on what the Compensation and Data classes do.

As is written in the library file, this is inspired by the [Adafruit library](https://github.com/adafruit/Adafruit_CircuitPython_BMP280/tree/main). Otherwise, refer to the [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).

This sensor is treated as extra and so there is little incentive to study fully what it does for the project it is used by.

The library was lightly modified for python 2 support, and test.py was modified to simply print the relevant sensor values.

**Library Interface Structure:**
+ \_\_init\_\_(bus=1, osrs_t=0b010, osrs_p=0b101, mode=0b11, filter=0b100) - open port at device bus, set configuration parameters, and run initialize()
+ initialize() - run reset(), check for correct device, run (get_compensation(), write_config(), and write_ctrl()), and set a short delay
+ write_ctrl() - write oversampling and mode parameters to REG_CTRL_MEAS - see 4.3.4
+ write_config() - write standby time and filter config to REG_CONFIG - see 4.3.5
+ reset() - write reset command to reset register
+ read_id() - return result of reading REG_ID register
+ get_compensation() - read all compensation data from the BMP
+ get_data() - read all sensor data from the BMP
+ read(register, length=1) - get a length of data from the starting and successive registers
+ write(register_address, data) - write some data to the select register
+ close() - close the BMP port

If you want to understand the initialization process, it is recommended to read section 4.2 (Memory map) from the datasheet.

If you do not need to configure anything, copy the contents of test.py into your own code as it should run worry-free.
