/**
 * Created by cottley on 4/16/2018.
 */

#include "ads_combined.h"
#include <string.h>

//static ads_callback ads_data_callback;


// Stores the type of device 
static ads_init_t m_ads;

// Stores the stretch enabled state for one axis sensors 
static bool stretch_en = false;

/**
 * @brief Parses sample buffer from one axis ADS. Scales to degrees and
 *				executes callback registered in ads_init. 
 *				This function is called from ads_hal. Application should never call this function.
 */	
void ads_parse_read_buffer(uint8_t * buffer)
{
	float sample[2];
	int16_t temp;
		
	if(buffer[0] == ADS_SAMPLE)
	{
		if(m_ads.ads_dev_id == ADS_ONE_AXIS)
		{	
			temp = ads_int16_decode(&buffer[1]);
			sample[0] = (float)temp/64.0f;
			
			if(!stretch_en)
				sample[1] = 0.0f;
		}
		else
		{
			temp = ads_int16_decode(&buffer[1]);
			sample[0] = (float)temp/32.0f;
			
			temp = ads_int16_decode(&buffer[3]);
			sample[1] = (float)temp/32.0f;			
		}
		
		// Return new data through callback
		m_ads.ads_sample_callback(sample, buffer[0]);
	}
	else if(buffer[0] == ADS_STRETCH_SAMPLE)
	{
		if(m_ads.ads_dev_id == ADS_ONE_AXIS)
		{
			temp = ads_int16_decode(&buffer[1]);
			sample[1] = (float)temp/64.0f;
			
			// Return new data through callback
			m_ads.ads_sample_callback(sample, buffer[0]);
		}
	}
}

/**
 * @brief Reads ADS sample data when ADS is in polled mode
 *
 * @param	sample	floating point array returns new sample
 * @param   data_type data back type, bend or stretch 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_read_polled(float * sample, uint8_t * data_type)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	int16_t temp;
	
	// Read data from sensor
	int ret_val = ads_hal_read_buffer(buffer, m_ads.xfer_size);

	// Parse data if successful read 
	if(ret_val == ADS_OK)
	{
		data_type[0] = buffer[0];		// Update packet type
		
		// Device type dictates scaling factor and number of angles
		if(m_ads.ads_dev_id == ADS_ONE_AXIS)
		{
			if(!stretch_en) {
				sample[1] = 0.0f;
			}
			
			// Check that read packet is either bend or stretch data
			if(buffer[0] == ADS_SAMPLE)
			{
				temp = ads_int16_decode(&buffer[1]);
				sample[0] = (float)temp/64.0f;	
			}
			else if(buffer[0] == ADS_STRETCH_SAMPLE)
			{
				temp = ads_int16_decode(&buffer[1]);
				sample[1] = (float)temp/64.0f;					
			}
			else 
			{
				ret_val = ADS_ERR; // Set to general error, data packet not found
			}
		}
		else
		{
			// Check that the read packet is a data packet
			if(buffer[0] == ADS_SAMPLE)
			{
				temp = ads_int16_decode(&buffer[1]);
				sample[0] = (float)temp/32.0f;
				
				temp = ads_int16_decode(&buffer[3]);
				sample[1] = (float)temp/32.0f;					
			}
			else
			{
				ret_val = ADS_ERR; // Set to general error, data packet not found
			}
		}
	}
	
	return ret_val;
}

/**
 * @brief Places ADS in free run or sleep mode
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_run(bool run)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
		
	buffer[0] = ADS_RUN;
	buffer[1] = run;
		
	return ads_hal_write_buffer(buffer, m_ads.xfer_size);
}

/**
 * @brief Places ADS in poll mode. Each time sensor data is read a new sample is taken
 *
 * @param	poll	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_polled(bool poll)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
		
	buffer[0] = ADS_POLLED_MODE;
	buffer[1] = poll;
		
	return ads_hal_write_buffer(buffer, m_ads.xfer_size);
}

/**
 * @brief Sets the sample rate of the ADS in free run mode
 *
 * @param	sps ADS_SPS_T sample rate
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_set_sample_rate(ADS_SPS_T sps)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_SPS;
	ads_uint16_encode(sps, &buffer[1]);
	
	return ads_hal_write_buffer(buffer, m_ads.xfer_size);
}

/**
 * @brief Updates the I2C address of the selected ADS. The default address 
 *		  is 0x12. Use this function to program an ADS to allow multiple
 *		  devices on the same I2C bus.
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_ERR_BAD_PARAM if failed
 */
int ads_update_device_address(uint8_t device, uint8_t address)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_SET_ADDRESS;
	buffer[1] = address;
	
	if(ads_hal_write_buffer(buffer, m_ads.xfer_size) != ADS_OK)
		return ADS_ERR_IO;
	
	return ads_hal_update_device_addr(device, address);	
}

/**
 * @brief Updates selected device number the HAL is communicating with
 *
 * @param	device number 0-9 
 * @return	ADS_OK if successful ADS_ERR_BAD_PARAM if failed
 */
int ads_select_device(uint8_t device)
{
	return ads_hal_select_device(device);
}
#include <Arduino.h>
/**
 * @brief Initializes the hardware abstraction layer and sample rate of the ADS
 *
 * @param	ads_init_t	initialization structure of the ADS
 * @return	ADS_OK if successful ADS_ERR if failed
 */
int ads_init(ads_init_t * ads_init)
{
	// Copy initialization structure into local variable
	memcpy(&m_ads, ads_init, sizeof(ads_init_t));
	
	if(ads_init->addr)
	{
		ads_hal_set_address(ads_init->addr);
	}
	else
	{
		if(m_ads.ads_dev_id == ADS_ONE_AXIS)
		{
			ads_hal_set_address(ADS_DEFAULT_ONE_AXIS_ADDR);
		}
		else if(m_ads.ads_dev_id == ADS_TWO_AXIS)
		{
			ads_hal_set_address(ADS_DEFAULT_TWO_AXIS_ADDR);
		}
		else if(m_ads.ads_dev_id == ADS_TWO_REGION)
		{
			ads_hal_set_address(ADS_DEFAULT_TWO_REGION_ADDR);
		}
		else
		{
			return ADS_ERR_BAD_PARAM;
		}
	}
	
	// Initialize hardware abstraction layer 
	ads_hal_init(&ads_parse_read_buffer, ads_init->reset_pin, ads_init->datardy_pin, \
					ads_init->xfer_size);	
	
	// Check that the device id matches device type id
	if(ads_get_dev_id() != ADS_OK)
		return ADS_ERR_DEV_ID;
	
	ads_hal_delay(2);
	
	/* Checks if the firmware in the driver (ads_fw.h) is newer than the firmware on the ads.
	 * Updates firwmare on the ads if out of date. */
#if ADS_DFU_CHECK
	uint16_t drive_ver = (ads_init->ads_dev_id == ADS_ONE_AXIS) ? ads_fw_rev : ads_two_axis_fw_rev;

 	if(ads_dfu_check((uint8_t)ADS_GET_FW_VER, drive_ver, ads_init->xfer_size))
	{
		if(ads_dfu_reset(ads_init->xfer_size) == ADS_OK)
		{
			ads_hal_delay(50);		// Give ADS time to reset
			const uint8_t * ads_fw;
			ads_fw = (ads_init->ads_dev_id == ADS_ONE_AXIS) ? ads_fw : ads_two_axis_fw;
			uint32_t len = (ads_init->ads_dev_id == ADS_ONE_AXIS) ? ads_fw_len : ads_two_axis_fw_len;
			ads_dfu_update(ads_fw, len);
			ads_hal_delay(2000);	// Let it reinitialize		
		}

	}
#endif
 	
 	if(ads_set_sample_rate(ads_init->sps))
		return ADS_ERR;
	
	ads_hal_delay(2);
	
	return ADS_OK;
}

/**
 * @brief Calibrates one axis ADS. ADS_CALIBRATE_FIRST should be at 0 degrees on
 *				ADS_CALIBRATE_SECOND can be at 45 - 255 degrees, recommended 90 degrees.
 *
 * @param	ads_calibration_step 	ADS_CALIBRATE_STEP_T to perform
 * @param degrees uint8_t angle at which sensor is bent when performing 
 *				ADS_CALIBRATE_FIRST, and ADS_CALIBRATE_SECOND
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 */
int ads_calibrate(ADS_CALIBRATION_STEP_T ads_calibration_step, uint8_t degrees)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_CALIBRATE;
	buffer[1] = ads_calibration_step;
	buffer[2] = degrees;
	
	return ads_hal_write_buffer(buffer, m_ads.xfer_size);
}

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
int ads_enable_axis(uint8_t axes_enable)
{
	int ret_val = ADS_ERR_DEV_ID;
	
	if(m_ads.ads_dev_id != ADS_ONE_AXIS)
	{
		if(!(axes_enable & (ADS_AXIS_0_EN | ADS_AXIS_1_EN)))
				return ADS_ERR_BAD_PARAM;
		
		uint8_t buffer[ADS_TRANSFER_SIZE];
		
		buffer[0] = ADS_AXES_ENALBED;
		buffer[1] = axes_enable;
		
		ret_val = ads_hal_write_buffer(buffer, m_ads.xfer_size);
	}
	
	return ret_val;
}

/**
 * @brief Enables and Disables the reading of linear displacment data
 *			for one axis sensor only 
 *
 * @param	enable	true if enabling ADS to read stretch, false is disabling
 * @return	ADS_OK if successful ADS_ERR_IO if failed, ADS_ERR_DEV_ID if unsupported device 
 */
int ads_stretch_en(bool enable)
{
	int ret_val = ADS_ERR_DEV_ID;
	
	if(m_ads.ads_dev_id == ADS_ONE_AXIS)
	{
		uint8_t buffer[ADS_TRANSFER_SIZE];
			
		buffer[0] = ADS_READ_STRETCH;
		buffer[1] = enable;
			
		ret_val = ads_hal_write_buffer(buffer, m_ads.xfer_size);
	}
	
	if(ret_val == ADS_OK)
	{
		stretch_en = enable;
	}
	
	return ret_val;
}

/**
 * @brief Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_shutdown(void)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_SHUTDOWN;
	
	return ads_hal_write_buffer(buffer, m_ads.xfer_size);
}

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_wake(void)
{
	// Reset ADS to wake from shutdown
	ads_hal_reset();
	
	// Allow time for ADS to reinitialize 
	ads_hal_delay(100);	
	
	return ADS_OK;
}

/**
 * @brief Checks that the device id is ADS_ONE_AXIS. ADS should not be in free run
 * 			when this function is called.
 *
 * @return	ADS_OK if dev_id is ADS_ONE_AXIS, ADS_ERR_DEV_ID if not
 */
int ads_get_dev_id(void)
{
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_GET_DEV_ID;
	
	// Disable interrupt to prevent callback from reading out device id
	ads_hal_pin_int_enable(false);
	
	ads_hal_write_buffer(buffer, m_ads.xfer_size);
	ads_hal_delay(2);
	ads_hal_read_buffer(buffer, m_ads.xfer_size);
	
	ads_hal_pin_int_enable(true);
	
	/* Check that packet read is a device id packet and that
	 * and that the device id is a two axis sensor */
	if(buffer[0] == ADS_DEV_ID && buffer[1] == m_ads.ads_dev_id)
	{
		return ADS_OK;
	}
	else
	{
		return ADS_ERR_DEV_ID;
	}
}

/**
 * @brief Checks if the firmware image in the driver is newer than 
 *			the firmware on the device.
 *
 * @param ads_get_fw_ver	Get fw version command
 * @return	TRUE if update needed. FALSE if no updated needed
 */
int ads_get_fw_ver(uint16_t * fw_ver)
{
	int ret_val = ADS_OK;
	uint8_t buffer[ADS_TRANSFER_SIZE];
	
	buffer[0] = ADS_GET_FW_VER;

	ads_hal_write_buffer(buffer, m_ads.xfer_size);
	ads_hal_delay(2);
	ads_hal_read_buffer(buffer, m_ads.xfer_size);
	
	if(buffer[0] == ADS_FW_VER)
	{
		fw_ver[0] = ads_uint16_decode(&buffer[1]);
	}
	else
	{
		fw_ver[0] = 0;
		ret_val = ADS_ERR;
	}
		
	return ret_val;
}

void ads_set_xfer_size(uint8_t xfer_size)
{
	m_ads.xfer_size = xfer_size;
}
