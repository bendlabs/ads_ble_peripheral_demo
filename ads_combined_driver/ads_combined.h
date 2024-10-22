/**
 * Created by cottley on 4/13/2018.
 */

#ifndef ADS_H_
#define ADS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ads_combined_hal.h"
#include "ads_err.h"
#include "ads_util.h"

typedef void (*ads_callback)(float*,uint8_t);	// Callback function prototype for interrupt mode

/* 
 * If compilation fails due to insufficent memory in Arduino IDE set to (0) 
 */
#define ADS_DFU_CHECK				(1)		// Set this to 1 to check if the newest firmware is on the ADS

#if ADS_DFU_CHECK
#include "ads_fw.h"
#include "ads_dfu.h"
#endif


typedef enum {
	ADS_CALIBRATE_FIRST = 0,			// First calibration point, typically 0 degrees
	ADS_CALIBRATE_SECOND,				// Second calibration point, 45-255 degrees
	ADS_CALIBRATE_CLEAR,				// Clears user calibration, restores factory calibration
	ADS_CALIBRATE_STRETCH_ZERO,			// 0mm strain calibration point
	ADS_CALIBRATE_STRETCH_SECOND,		// Second calibration point for stretch, typically 30mm
	ADS_TWO_CALIBRATE_FIRST = 0,		// Two Axis and Two Region calibration commands
	ADS_TWO_CALIBRATE_FLAT,				
	ADS_TWO_CALIBRATE_PERP,
	ADS_TWO_CALIBRATE_CLEAR,
	ADS_TWO_CALIBRATE_NONE
} ADS_CALIBRATION_STEP_T;


typedef enum {
	ADS_1_HZ   = 16384,
	ADS_10_HZ  = 1638,
	ADS_20_HZ  = 819,
	ADS_50_HZ  = 327,
	ADS_100_HZ = 163,
	ADS_200_HZ = 81,
	ADS_333_HZ = 49,
	ADS_500_HZ = 32,
} ADS_SPS_T;

typedef struct {
	ADS_SPS_T sps;
	ads_callback ads_sample_callback;
	uint32_t reset_pin;
	uint32_t datardy_pin;
	ADS_DEV_IDS_T ads_dev_id;
	uint8_t xfer_size;
	uint8_t addr;
} ads_init_t;


/**
 * @brief Reads ADS sample data when ADS is in polled mode
 *
 * @param	sample	floating point array returns new sample
 * @param   data_type data back type, bend or stretch 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_read_polled(float * sample, uint8_t * data_type);


/**
 * @brief Places ADS in free run or sleep mode
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_run(bool run);

/**
 * @brief Places ADS in poll mode. Each time sensor data is read a new sample is taken
 *
 * @param	poll	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_polled(bool poll);

/**
 * @brief Sets the sample rate of the ADS in free run mode
 *
 * @param	sps ADS_SPS_T sample rate
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_set_sample_rate(ADS_SPS_T sps);

/**
 * @brief Enables the ADS data ready interrupt line
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_enable_interrupt(bool enable);

/**
 * @brief Updates the I2C address of the selected ADS. The default address 
 *		  is 0x12. Use this function to program an ADS to allow multiple
 *		  devices on the same I2C bus.
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_ERR_BAD_PARAM if failed
 */
int ads_update_device_address(uint8_t device, uint8_t address);

/**
 * @brief Initializes the hardware abstraction layer and sample rate of the ADS
 *
 * @param	ads_init_t	initialization structure of the ADS
 * @return	ADS_OK if successful ADS_ERR if failed
 */
int ads_init(ads_init_t * ads_init);

/**
 * @brief Calibrates one axis ADS. ADS_CALIBRATE_FIRST should be at 0 degrees on
 *				ADS_CALIBRATE_SECOND can be at 45 - 255 degrees, recommended 90 degrees.
 *
 * @param	ads_calibration_step 	ADS_CALIBRATE_STEP_T to perform
 * @param degrees uint8_t angle at which sensor is bent when performing 
 *				ADS_CALIBRATE_FIRST, and ADS_CALIBRATE_SECOND
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 */
int ads_calibrate(ADS_CALIBRATION_STEP_T ads_calibration_step, uint8_t degrees);

/**
 * @brief Enables/disables individual axes of the sensor. Both axes are
 *				enabled at reset. 
 *				ADS_AXIS_0_EN | ADS_AXIS_1_EN enables both.
 *				ADS_AXIS_0_EN enables axis zero and disables axis one
 *				ADS_AXIS_1_EN enables axis one and disables axis zero
 *
 * @param	axes_enabled	bit mask of which axes to enable
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 *			ADS_ERR_DEV_ID if function is used with one axis sensor
 */
int ads_enable_axis(uint8_t axes_enable);

/**
 * @brief Enables and Disables the reading of linear displacment data
 *			for one axis sensor only 
 *
 * @param	enable	true if enabling ADS to read stretch, false is disabling
 * @return	ADS_OK if successful ADS_ERR_IO if failed, ADS_ERR_DEV_ID if unsupported device 
 */
int ads_stretch_en(bool enable);

/**
 * @brief Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_shutdown(void);

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_wake(void);

/**
 * @brief Checks that the device id is ADS_ONE_AXIS. ADS should not be in free run
 * 			when this function is called.
 *
 * @return	ADS_OK if dev_id is ADS_ONE_AXIS, ADS_ERR_DEV_ID if not
 */
 int ads_get_dev_id(void);
 
 int ads_get_fw_ver(uint16_t * fw_ver);
 
 void ads_set_xfer_size(uint8_t xfer_size);


#endif /* ADS_H_ */
