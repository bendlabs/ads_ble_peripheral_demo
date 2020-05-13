# ads_ble_peripheral_demo
 Stream Bend Labs 1 & 2 - axis sensor data to Bend Labs Android Demo app and central_ads_eval_kit demo
 
 *  This sketch configures an NRF52 based Arduino development board
 *  to read data from a Bend Labs One or Two Axis Sensor.
 *  Implements the BLE Peripheral functionality of the Bend Labs
 *  Eval Kit. Allows to send data to Bend Labs Android Demo App or
 *  another NRF52 based Arduino development board with central_ads_eval_kit.ino 
 *  
 *  Default configuration is for One Axis Sensor. Change the preprocessor definition
 *  #define DEMO_DEVICE_ONE_AXIS    (1) to #define DEMO_DEVICE_ONE_AXIS    (0) for 
 *  interfacing with a Two Axis Sensor. 
 *  
 *  Copy the folder 'ads_combined_driver' into your Arduino\libraries folder
 *  
 *  Tested Board: Adafruit Feather NRF52840
 *  Should work with: Adafruit Feather NRF52832 or similar 
 *  NRF52 Based Bluefruit Board
 *  
 *  Tested With Sparkfun Pro nRF52840 Mini it works but you may have to 
 *  change lines 99 & 100 in variant.h to:
 *  #define PIN_SERIAL1_RX       (15)
 *  #define PIN_SERIAL1_TX       (17)
 *  
 *  This demo interfaces with the ads sensor in polled mode.
 *  SDA   - I2C Data
 *  SCL   - I2C Clock
 *  VCC   - 3.3 V
 *  GND   - Ground
 *  nRST  - 5 RESET line is necessary to operate
 *  nDRDY - Not Used
