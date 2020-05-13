/**
 * Created by cottley on 4/17/2018.
 */

#ifndef ADS_DFU_
#define ADS_DFU_

#include <stdint.h>
#include <stdbool.h>
#include "ads_err.h"
#include "ads_combined_hal.h"
#include "ads_util.h"


/**
 * @brief Checks if the firmware image in the driver is newer than 
 *			the firmware on the device.
 *
 * @param ads_get_fw_ver	Get fw version command
 * @param drive_ver			version of fw in the driver 
 * @param xfer_size		    I2C transaction size
 * @return	TRUE if update needed. FALSE if no updated needed
 */
bool ads_dfu_check(uint8_t ads_get_fw_ver, uint16_t drive_ver, uint8_t xfer_size);


/**
 * @brief Resets the ADS into bootloader mode
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
 int ads_dfu_reset(void);


/**
 * @brief Resets the ADS into bootloader mode
 *
 * @param xfer size 		I2C transaction size 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
 int ads_dfu_reset(uint8_t xfer_size);

/**
 * @brief Writes firmware image to ADS contained in ads_fw.h
 * 
 * @param ads_fw pointer to the fw image 
 * @param len length of fw image
 * @return	ADS_OK if successful ADS_ERR_TIMEOUT if failed
 */
int ads_dfu_update(const uint8_t * ads_fw_img, uint32_t len);

#endif /* ADS_DFU_ */
