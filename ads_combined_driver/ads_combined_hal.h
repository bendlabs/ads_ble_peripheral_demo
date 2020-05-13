/**
 * Created by cottley on 4/13/2018.
 */

#ifndef ADS_COMBINED_HAL_
#define ADS_COMBINED_HAL_

#include <stdint.h>
#include "ads_err.h"

#define ADS_TRANSFER_SIZE		(5)					// In the combined used the preprocessor definition is always two regions

#define ADS_COUNT				(10)				// Number of ADS devices attached to bus

#define ADS_DEFAULT_ONE_AXIS_ADDR		(0x12)
#define ADS_DEFAULT_TWO_AXIS_ADDR		(0x13)
#define ADS_DEFAULT_TWO_REGION_ADDR		(0x14)


void ads_hal_delay(uint16_t delay_ms);

void ads_hal_pin_int_enable(bool enable);

/**
 * @brief Write buffer of data to the Angular Displacement Sensor
 *
 * @param buffer[in]	Write buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_write_buffer(uint8_t * buffer, uint8_t len);

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_read_buffer(uint8_t * buffer, uint8_t len);

/**
 * @brief Reset the Angular Displacement Sensor
 */
void ads_hal_reset(void);

/**
 * @brief Initializes the hardware abstraction layer 
 *
 * @param callback to ads.cpp, 
 * @param reset_pin pin number for reset line of ads
 * @param datardy_pin pin number for data ready interrupt 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_init(void (*callback)(uint8_t*), uint32_t reset_pin, uint32_t datardy_pin, uint8_t xfer_size);

/**
 * @brief Selects the current device address the driver is communicating with
 *
 * @param device select device 0 - ADS_COUNT
 * @return	ADS_OK if successful ADS_ERR_BAD_PARAM if invalid device number
 */
int ads_hal_select_device(uint8_t device);

/**
 * @brief Updates the I2C address in the ads_addrs[] array. Updates the current
 *		  selected address.
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_BAD_PARAM if failed
 */
int ads_hal_update_device_addr(uint8_t device, uint8_t address);

/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address
 */
uint8_t ads_hal_get_address(void);

/**
 * @brief Sets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu) 
 */
void ads_hal_set_address(uint8_t address);

/**
 * @brief Sets the i2c transfer size in hardware abstraction layer.
 */
void ads_hal_set_xfer_size(uint8_t xfer_size);

#endif /* ADS_COMBINED_HAL_ */
