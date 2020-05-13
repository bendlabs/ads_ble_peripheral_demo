/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* 
 *  Read Data from One and Two Axis Sensors from Bend Labs
 *  then sends to Bend Labs Android BLE Demo App or central_ads_eval_kit sketch 
 *  By: Colton Ottley @ Bend Labs
 *  Date: May 11th, 2020
 *  
 *  This sketch configures an NRF52 based Arduino development board
 *  to read data from a Bend Labs One or Two Axis Sensor.
 *  Implements the BLE Peripheral functionality of the Bend Labs
 *  Eval Kit. Allows to send data to Bend Labs Android Demo App or
 *  another NRF52 based Arduino development board with central_ads_eval_kit.ino 
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
 */

#include "Arduino.h"
#include "ads_combined.h"

#include <bluefruit.h>
#include <string.h>

#define ADS_RESET_PIN       (5)           // Pin number attached to ads reset line. NECESSARY!
#define ADS_INTERRUPT_PIN   (0xFF)        // Pin number attached to the ads data ready line. Not used 


BLEService        angms = BLEService(0x1820);
BLECharacteristic angmc = BLECharacteristic(0x2A70);


BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

// function prototypes
void startAdv(void);
void setupANGM(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void ads_data_callback(float * sample, uint8_t data_type);
void deadzone_filter(float * sample);
void signal_filter(float * sample);
void parse_serial_port(void);
int send_calibration(uint8_t c);
void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, unsigned char * rx, short unsigned len);
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
 
/* Calback unused, sketch utilizes polled mode */
void ads_data_callback(float * sample, uint8_t data_type)
{

}

#define DEMO_DEVICE_ONE_AXIS    (0)               // Set to 0 for two axis


#if DEMO_DEVICE_ONE_AXIS
const ADS_DEV_IDS_T ads_device_type = ADS_ONE_AXIS;
const char model_str[] = "ADS_ONE_AXIS";
const uint8_t ads_transfer_size = 3;                 // Change to 5 for two axis sensor
#else
const ADS_DEV_IDS_T ads_device_type = ADS_TWO_AXIS;
const char model_str[] = "ADS_TWO_AXIS";
const uint8_t ads_transfer_size = 5;                 // Change to 5 for two axis sensor
#endif

//ADS Initialization structure
ads_init_t ads_init_s =  { ADS_100_HZ,                //sps
                           &ads_data_callback,        //ads_sample_callback, used for interrupt mode
                           ADS_RESET_PIN,             //reset_pin
                           ADS_INTERRUPT_PIN,         //datardy_pin, used for interrupt mode
                           ads_device_type,           //ads_dev_id, ADS_ONE_AXIS, ADS_TWO_AXIS
                           ads_transfer_size,         //xfer_size, ADS_ONE_AXIS = 3, ADS_TWO_AXIS = 5
                           0x0                        //addr, 0->default vaue, if ads i2c address is non-default update value
};
  

void setup() {
  Serial.begin(115200);
  
  Serial.println("ADS BLE Example");
  Serial.println("-----------------------\n");
  
  delay(500);

  Serial.println("Initializing the Angular Displacement Sensor");                        

  int ret_val = ads_init(&ads_init_s);

  if(ret_val != ADS_OK)
  {
    Serial.print("ADS initialization failed with reason: ");
    Serial.println(ret_val);
  }

  // Shutdown the ads, requires ads_wake() to resume, HARDWARE RESET necessary
  ads_shutdown();

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  
  // Set the advertised device name
  Serial.println("Setting Device Name to 'ads_demo'");
  Bluefruit.setName("ads_demo");


  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Bluefruit.Periph.setConnIntervalMS(7.5,30);

  // Configure and Start the Device Information Service
  //Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Bend Labs");
  bledis.setModel(model_str);
  bledis.setHardwareRev("REV: 0.0.0");
  bledis.setSoftwareRev("SD: 132.2.0.0");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  //Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Angle Measurement service using
  // BLEService and BLECharacteristic classes
  setupANGM();

  // Setup the advertising packet
  startAdv();
  
  Serial.println("\nAdvertising"); 
}

// Command from the characteristic write callback write_callback
static volatile uint8_t write_cb_command = 0xFF;
static volatile uint16_t dms = 10;

void loop() {
  // put your main code here, to run repeatedly:
    delay(dms);

    static float sample[2];
    uint8_t sample_type;

    if(Bluefruit.connected())
    {
      // Read sample from ADS sensor
      if(ads_read_polled(sample, &sample_type) == ADS_OK)
      {
        if(ads_init_s.ads_dev_id == ADS_ONE_AXIS)
        {
          if(sample_type == ADS_SAMPLE)
          {
            signal_filter(sample);
            deadzone_filter(sample);
            
            uint8_t ang_encoded[8];
            memcpy(ang_encoded, &sample[0], 2*sizeof(float));
            angmc.notify(ang_encoded, sizeof(ang_encoded));
          }
        }
        else
        {
          deadzone_filter(sample);
        
          uint8_t ang_encoded[8];
          memcpy(ang_encoded, &sample[0], 2*sizeof(float));
          angmc.notify(ang_encoded, sizeof(ang_encoded));
        }
      }

      // Parse received calibration commands from the characteristic write callback
      if(write_cb_command != 0xFF)
      {
        if(send_calibration(write_cb_command) == ADS_OK)
          write_cb_command = 0xFF;    // Clear value
      }
    }
}

/*
 * Sends calibration commands to sensor received through the AMS Char Write Callback
 *  
 * One Axis Sensor Calibration Commands
 * Step | Angle | Stretch | Description
 *  1      0°       0mm     First angle calibration step     (sample[0])
 *  2     90°       0mm     Second angle calibration step    (sample[0])
 *  3      0°       0mm     First stretch calibration step   (sample[1])
 *  4      0°      30mm     Second stretch calibration step  (sample[1])
 *  -      -         -      Restores factory calibration
 * Notes: One axis sensor angle calibration coefficients won't 
 *        update before the 0° and 90° are sent.  When the 2nd stretch
 *        step is sent a decorrelation coefficient is calculated to 
 *        decorellate bend and stretch measurements. Positions of angle
 *        and stretch shown above should be observed. Order is also 
 *        important for correct operation, Step 1 before Step 2 for just angle. 
 *        Step 3 before Step 4 for just stretch. Step 1-4 for full calibration 
 *        
 *        clear calibration will replace most recent calibration with the calibration 
 *        stored at factory.
 *   
 * Two Axis Sensor Calibration Commands
 * Step | Ang1 | Ang2 | Description
 *  1      0°     0°    First angle calibration step (sample[0], sample[1])
 *  2     90°     0°    Second angle calibration step for first axis  (sample[0])
 *  3      0°    90°    Second angle calibration step for second axis (sample[1])
 *  -      -      -     Restores factory calibration
 * Notes: Positions of both angles shown above should be observed for
 *        each step. Order is also important for correct operation. 
 *        
 *        clear calibration will replace most recent calibration with the calibration 
 *        stored at factory. 
 */
int send_calibration(uint8_t c)
{
  int ret_val = ADS_OK;
  
  if(ads_init_s.ads_dev_id == ADS_ONE_AXIS)
  {
    switch(c) {
      case 0:
        ret_val = ads_calibrate(ADS_CALIBRATE_FIRST, 0);
        break;
      case 1:
        ret_val = ads_calibrate(ADS_CALIBRATE_SECOND, 90);
        break;
      case 4:
        ret_val = ads_calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
        break;
      case 5:
        ret_val = ads_calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
        break;
      case 3:
        ret_val = ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
        break;
      default:
        break;
    }
  }
  else
  {
    switch(c) {
      case 0:
        ret_val = ads_calibrate(ADS_TWO_CALIBRATE_FIRST, 0);
        break;
      case 1:
        ret_val = ads_calibrate(ADS_TWO_CALIBRATE_FLAT, 90);
        break;
      case 2:
        ret_val = ads_calibrate(ADS_TWO_CALIBRATE_PERP, 90);
        break;
      case 3:
        ret_val = ads_calibrate(ADS_TWO_CALIBRATE_CLEAR, 0);
        break;
      default:
        break;
    }       
  }  

  return ret_val;
}

void signal_filter(float * sample)
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  //Bluefruit.Advertising.addTxPower();

  // Include Angle Measurement Service UUID
  Bluefruit.Advertising.addService(angms);

  // Include Name
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.addAppearance(113);
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(500,800);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setupANGM(void)
{
  // Configure the Angle Measurement service
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Angle Measurement            0x2A70  Mandatory   Notify/Write    
  angms.begin();

  // Configure the Angle Measurement characteristic
  // Properties = Notify
  // Min Len    = 4
  // Max Len    = 4
  // Little Endian Float
  angmc.setProperties(CHR_PROPS_NOTIFY|CHR_PROPS_WRITE);
  angmc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  angmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  angmc.setWriteCallback(write_callback);
  angmc.begin();
  uint8_t ang_initial[] = {0, 0, 0, 0};
  angmc.notify(ang_initial, 4);                   // Use .notify instead of .write!
}

void connect_callback(uint16_t conn_handle)
{
    Serial.println("Connected");
    ads_wake();       // bring ADS out of shutdown
    ads_polled(true); // ADS in polled mode
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{

  Serial.println(reason);
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");

  // Shutdown the ads 
  ads_shutdown();
}

void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, unsigned char * rx, short unsigned len)
{
  if(len == 1)
  {
    write_cb_command = rx[0];
  }
  else if (len == 2)
  {
      uint16_t sps = ((((uint16_t)((uint8_t *)rx)[0])) |
                     (((uint16_t)((uint8_t *)rx)[1]) << 8 ));

      if(sps&0x8000)
      {
        if(sps&0x00ff)
        {
          ads_stretch_en(true);
        }
        else
        {
          ads_stretch_en(false);
        }
      }
      else
      {
        dms = sps/16; // Convert 32khz clock ticks to ms
      }
  }
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    //Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    //Serial.print(cccd_value);
    //Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == angmc.uuid) {
        if (chr->notifyEnabled()) {
            Serial.println("Angle Measurement 'Notify' enabled");
        } else {
            Serial.println("Angle Measurement 'Notify' disabled");
        }
    }
}
