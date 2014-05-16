/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MPU9250 MPU9250 Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_mpu9250.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief      MPU9250 9-axis gyro accel and mag chip
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Project Includes */
#include "pios.h"
#include "pios_semaphore.h"
#include "physical_constants.h"

#if defined(PIOS_INCLUDE_MPU9250)

#include "pios_mpu9250.h"


/* Private constants */
#define MPU9250_TASK_PRIORITY	(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define MPU9250_TASK_STACK		(768 / 4)

#define MPU9250_WHOAMI_ID        0x71

#ifdef PIOS_MPU9250_SPI_HIGH_SPEED
#define MPU9250_SPI_HIGH_SPEED			PIOS_MPU9250_SPI_HIGH_SPEED
#else
#define MPU9250_SPI_HIGH_SPEED			20000000
#endif
#define MPU9250_SPI_LOW_SPEED			300000

#define PIOS_MPU9250_ACCEL_DLPF_CFG_REG 0x1D


/* Global Variables */

enum pios_mpu9250_dev_magic {
	PIOS_MPU9250_DEV_MAGIC = 0xb8a9624f,
};

#define PIOS_MPU9250_MAX_DOWNSAMPLE 2
struct mpu9250_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	enum pios_mpu60x0_accel_range accel_range;
	enum pios_mpu60x0_range gyro_range;
	xQueueHandle gyro_queue;
	xQueueHandle accel_queue;
	xQueueHandle mag_queue;
	xTaskHandle TaskHandle;
	struct pios_semaphore *data_ready_sema;
	const struct pios_mpu9250_cfg * cfg;
	enum pios_mpu9250_gyro_filter gyro_filter;
	enum pios_mpu9250_accel_filter accel_filter;
	enum pios_mpu9250_dev_magic magic;
};

//! Global structure for this device device
static struct mpu9250_dev * dev;

//! Private functions
static struct mpu9250_dev * PIOS_MPU9250_alloc(const struct pios_mpu9250_cfg * cfg);
static int32_t PIOS_MPU9250_Validate(struct mpu9250_dev * dev);
static void PIOS_MPU9250_Task(void *parameters);
static uint8_t PIOS_MPU9250_ReadReg(uint8_t reg);
static int32_t PIOS_MPU9250_WriteReg(uint8_t reg, uint8_t data);
static int32_t PIOS_MPU9250_ClaimBus(bool lowspeed);
static int32_t PIOS_MPU9250_ReleaseBus(bool lowspeed);

/**
 * @brief Allocate a new device
 */
static struct mpu9250_dev * PIOS_MPU9250_alloc(const struct pios_mpu9250_cfg * cfg)
{
	struct mpu9250_dev * mpu9250_dev;
	
	mpu9250_dev = (struct mpu9250_dev *)PIOS_malloc(sizeof(*mpu9250_dev));
	if (!mpu9250_dev) return (NULL);
	
	mpu9250_dev->magic = PIOS_MPU9250_DEV_MAGIC;
	
	mpu9250_dev->accel_queue = xQueueCreate(PIOS_MPU9250_MAX_DOWNSAMPLE, sizeof(struct pios_sensor_accel_data));
	if (mpu9250_dev->accel_queue == NULL) {
		vPortFree(mpu9250_dev);
		return NULL;
	}

	mpu9250_dev->gyro_queue = xQueueCreate(PIOS_MPU9250_MAX_DOWNSAMPLE, sizeof(struct pios_sensor_gyro_data));
	if (mpu9250_dev->gyro_queue == NULL) {
		vPortFree(mpu9250_dev);
		return NULL;
	}

	if (cfg->use_magnetometer) {
		mpu9250_dev->mag_queue = xQueueCreate(PIOS_MPU9250_MAX_DOWNSAMPLE, sizeof(struct pios_sensor_mag_data));
		if (mpu9250_dev->mag_queue == NULL) {
			vPortFree(mpu9250_dev);
			return NULL;
		}
	}

	mpu9250_dev->data_ready_sema = PIOS_Semaphore_Create();
	if (mpu9250_dev->data_ready_sema == NULL) {
		vPortFree(mpu9250_dev);
		return NULL;
	}

	return(mpu9250_dev);
}

/**
 * @brief Validate the handle to the device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_MPU9250_Validate(struct mpu9250_dev * dev)
{
	if (dev == NULL) 
		return -1;
	if (dev->magic != PIOS_MPU9250_DEV_MAGIC)
		return -2;
	if (dev->spi_id == 0)
		return -3;
	return 0;
}

/**
 * @brief Claim the SPI bus for the communications and select this chip
 * \param[in] flag controls if low speed access for control registers should be used
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_MPU9250_ClaimBus(bool lowspeed)
{
	if (PIOS_MPU9250_Validate(dev) != 0)
		return -1;

	if (PIOS_SPI_ClaimBus(dev->spi_id) != 0)
		return -2;

	if (lowspeed)
		PIOS_SPI_SetClockSpeed(dev->spi_id, MPU9250_SPI_LOW_SPEED);

	PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 0);

	return 0;
}


/**
 * @brief Release the SPI bus for the communications and end the transaction
 * \param[in] must be true when bus was claimed in lowspeed mode
 * @return 0 if successful
 */
static int32_t PIOS_MPU9250_ReleaseBus(bool lowspeed)
{
	if (PIOS_MPU9250_Validate(dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 1);

	if (lowspeed)
		PIOS_SPI_SetClockSpeed(dev->spi_id, MPU9250_SPI_HIGH_SPEED);

	PIOS_SPI_ReleaseBus(dev->spi_id);

	return 0;
}



/**
 * @brief Read a register from MPU9250
 * @returns The register value
 * @param reg[in] Register address to be read
 */
static uint8_t PIOS_MPU9250_ReadReg(uint8_t reg)
{
	uint8_t data;

	PIOS_MPU9250_ClaimBus(true);

	PIOS_SPI_TransferByte(dev->spi_id, 0x80 | reg); // request byte
	data = PIOS_SPI_TransferByte(dev->spi_id, 0);   // receive response

	PIOS_MPU9250_ReleaseBus(true);

	return data;
}


/**
 * @brief Writes one byte to the MPU9250 register
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * @returns 0 when success
 */
static int32_t PIOS_MPU9250_WriteReg(uint8_t reg, uint8_t data)
{
	if (PIOS_MPU9250_ClaimBus(true) != 0)
		return -1;

	PIOS_SPI_TransferByte(dev->spi_id, 0x7f & reg);
	PIOS_SPI_TransferByte(dev->spi_id, data);

	PIOS_MPU9250_ReleaseBus(true);

	return 0;
}



/**
 * @brief Initialize the MPU9250 gyro & accel registers
 * \return none
 * \param[in] pios_mpu9250_cfg struct to be used to configure sensor.
 *
 */
static int32_t PIOS_MPU9250_Config(struct pios_mpu9250_cfg const * cfg)
{
	// reset chip
	if (PIOS_MPU9250_WriteReg(PIOS_MPU60X0_PWR_MGMT_REG, PIOS_MPU60X0_PWRMGMT_IMU_RST) != 0)
		return -1;

	// give chip some time to initialize
	PIOS_DELAY_WaitmS(50);

	// power management config
	PIOS_MPU9250_WriteReg(PIOS_MPU60X0_PWR_MGMT_REG, PIOS_MPU60X0_PWRMGMT_PLL_X_CLK);

	// user control
	PIOS_MPU9250_WriteReg(PIOS_MPU60X0_USER_CTRL_REG, PIOS_MPU60X0_USERCTL_DIS_I2C | PIOS_MPU60X0_USERCTL_I2C_MST_EN);

	// Digital low-pass filter and scale
	// set this before sample rate else sample rate calculation will fail
	PIOS_MPU9250_SetAccelLPF(cfg->default_accel_filter);
	PIOS_MPU9250_SetGyroLPF(cfg->default_gyro_filter);

	// Sample rate
	PIOS_MPU9250_SetSampleRate(cfg->default_samplerate);

	// Set the gyro scale
	PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);

	// Set the accel scale
	PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);

	// TODO mag config

	// Interrupt configuration
	PIOS_MPU9250_WriteReg(PIOS_MPU60X0_INT_CFG_REG, cfg->interrupt_cfg);

	// Interrupt enable
	PIOS_MPU9250_WriteReg(PIOS_MPU60X0_INT_EN_REG, PIOS_MPU60X0_INTEN_DATA_RDY);

	return 0;
}

/**
 * @brief Initialize the MPU9250 3-axis gyro sensor.
 * @return 0 for success, -1 for failure to allocate, -2 for failure to get irq
 */
int32_t PIOS_MPU9250_SPI_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_mpu9250_cfg * cfg)
{
	dev = PIOS_MPU9250_alloc(cfg);
	if (dev == NULL)
		return -1;

	dev->spi_id = spi_id;
	dev->slave_num = slave_num;
	dev->cfg = cfg;

	// DEBUG_PRINTF(0, "who am i = 0x%02x", who);

	/* Configure the MPU9250 Sensor */
	if (PIOS_MPU9250_Config(cfg) != 0)
		return -2;

	/* Set up EXTI line */
	PIOS_WDG_Clear();
	PIOS_EXTI_Init(cfg->exti_cfg);
	// while(1) PIOS_WDG_Clear();

	// Wait 5 ms for data ready interrupt and make sure it happens
	// twice
	if ((PIOS_Semaphore_Take(dev->data_ready_sema, 20) != true) ||
		(PIOS_Semaphore_Take(dev->data_ready_sema, 20) != true)) {
		return -10;
	}

	int result = xTaskCreate(PIOS_MPU9250_Task, (const signed char *)"PIOS_MPU9250_Task",
						 MPU9250_TASK_STACK, NULL, MPU9250_TASK_PRIORITY,
						 &dev->TaskHandle);
	PIOS_Assert(result == pdPASS);

	PIOS_SENSORS_Register(PIOS_SENSOR_ACCEL, dev->accel_queue);
	PIOS_SENSORS_Register(PIOS_SENSOR_GYRO, dev->gyro_queue);

	if (dev->cfg->use_magnetometer)
		PIOS_SENSORS_Register(PIOS_SENSOR_MAG, dev->mag_queue);

	return 0;
}

/**
 * @brief Test MPU9250 presence on the bus
 * @returns 0 if success
 */
int32_t PIOS_MPU9250_Test()
{
	uint8_t id = PIOS_MPU9250_ReadReg(PIOS_MPU60X0_WHOAMI);
	if (id != MPU9250_WHOAMI_ID)
		return 1;

	return 0;
}


/**
 * @brief Set gyroscope range
 * @returns 0 if successful
 * @param range[in] gyroscope range
 */

int32_t PIOS_MPU9250_SetGyroRange(enum pios_mpu60x0_range range)
{
	if (PIOS_MPU9250_WriteReg(PIOS_MPU60X0_GYRO_CFG_REG, range) != 0)
		return -1;

	switch(range) {
	case PIOS_MPU60X0_SCALE_250_DEG:
		PIOS_SENSORS_SetMaxGyro(250);
		break;
	case PIOS_MPU60X0_SCALE_500_DEG:
		PIOS_SENSORS_SetMaxGyro(500);
		break;
	case PIOS_MPU60X0_SCALE_1000_DEG:
		PIOS_SENSORS_SetMaxGyro(1000);
		break;
	case PIOS_MPU60X0_SCALE_2000_DEG:
		PIOS_SENSORS_SetMaxGyro(2000);
		break;
	}

	dev->gyro_range = range;
	return 0;
}


/**
 * @brief Set accelerometer range
 * @returns 0 if success
 * @param range[in] accelerometer range
 */

int32_t PIOS_MPU9250_SetAccelRange(enum pios_mpu60x0_accel_range range)
{
	if (PIOS_MPU9250_WriteReg(PIOS_MPU60X0_ACCEL_CFG_REG, range) != 0)
		return -1;
	dev->accel_range = range;

	return 0;
}

/**
 * @brief Set sampling frequency of accels and gyros axes
 * @returns 0 if successful
 * @param samplerate_hz[in] Sampling frequency in Hz
 */

int32_t PIOS_MPU9250_SetSampleRate(uint16_t samplerate_hz)
{
	// mpu9250 ODR divider is unable to run from 8kHz clock :(
	if ((dev->gyro_filter == PIOS_MPU9250_GYRO_LOWPASS_250_HZ) && (samplerate_hz != 8000)) {
		return -1;
	}

	uint16_t filter_frequency = 1000;

	// limit samplerate to filter frequency
	if (samplerate_hz > filter_frequency)
		samplerate_hz = filter_frequency;

	// calculate divisor, round to nearest integeter
	int32_t divisor = (int32_t)(((float)filter_frequency / samplerate_hz) + 0.5f) - 1;

	// limit resulting divisor to register value range
	if (divisor < 0)
		divisor = 0;

	if (divisor > 0xff)
		divisor = 0xff;

	return PIOS_MPU9250_WriteReg(PIOS_MPU60X0_SMPLRT_DIV_REG, (uint8_t)divisor);
}


/**
 * @brief Set gyroscope lowpass filter cut-off frequency
 * @param filter[in] Filter frequency
 */

void PIOS_MPU9250_SetGyroLPF(enum pios_mpu9250_gyro_filter filter)
{
	PIOS_MPU9250_WriteReg(PIOS_MPU60X0_DLPF_CFG_REG, filter);

	dev->gyro_filter = filter;
}

/**
 * @brief Set accelerometer lowepass filter cut-off frequency
 * @param filter[in] Filter frequency
 */
void PIOS_MPU9250_SetAccelLPF(enum pios_mpu9250_accel_filter filter)
{
	PIOS_MPU9250_WriteReg(PIOS_MPU9250_ACCEL_DLPF_CFG_REG, filter);

	dev->accel_filter = filter;
}

/**
 * @brief Get current gyro scale for deg/s
 * @returns scale
 */
static float PIOS_MPU9250_GetGyroScale()
{
	switch (dev->gyro_range) {
		case PIOS_MPU60X0_SCALE_250_DEG:
			return 1.0f / 131.0f;
		case PIOS_MPU60X0_SCALE_500_DEG:
			return 1.0f / 65.5f;
		case PIOS_MPU60X0_SCALE_1000_DEG:
			return 1.0f / 32.8f;
		case PIOS_MPU60X0_SCALE_2000_DEG:
			return 1.0f / 16.4f;
	}
	return 0;
}

/**
 * @brief Get current gyro scale for ms^-2
 * @returns scale
 */
static float PIOS_MPU9250_GetAccelScale()
{
	switch (dev->accel_range) {
		case PIOS_MPU60X0_ACCEL_2G:
			return GRAVITY / 16384.0f;
		case PIOS_MPU60X0_ACCEL_4G:
			return GRAVITY / 8192.0f;
		case PIOS_MPU60X0_ACCEL_8G:
			return GRAVITY / 4096.0f;
		case PIOS_MPU60X0_ACCEL_16G:
			return GRAVITY / 2048.0f;
	}
	return 0;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool PIOS_MPU9250_IRQHandler(void)
{
	if (PIOS_MPU9250_Validate(dev) != 0)
		return false;

    bool need_yield = false;

    PIOS_Semaphore_Give_FromISR(dev->data_ready_sema, &need_yield);

    return need_yield;
}

static void PIOS_MPU9250_Task(void *parameters)
{
	// static int test = 0;
	while (1) {
		//Wait for data ready interrupt
		// PIOS_WDG_Clear();
		if (PIOS_Semaphore_Take(dev->data_ready_sema, PIOS_SEMAPHORE_TIMEOUT_MAX) != true)
			continue;

		enum {
		    IDX_REG = 0,
			IDX_ACCEL_XOUT_H,
		    IDX_ACCEL_XOUT_L,
		    IDX_ACCEL_YOUT_H,
		    IDX_ACCEL_YOUT_L,
		    IDX_ACCEL_ZOUT_H,
		    IDX_ACCEL_ZOUT_L,
		    IDX_TEMP_OUT_H,
		    IDX_TEMP_OUT_L,
		    IDX_GYRO_XOUT_H,
		    IDX_GYRO_XOUT_L,
		    IDX_GYRO_YOUT_H,
		    IDX_GYRO_YOUT_L,
		    IDX_GYRO_ZOUT_H,
		    IDX_GYRO_ZOUT_L,
		    BUFFER_SIZE,
		};


		uint8_t mpu9250_rec_buf[BUFFER_SIZE];
		uint8_t mpu9250_tx_buf[BUFFER_SIZE] = {PIOS_MPU60X0_ACCEL_X_OUT_MSB | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

		// claim bus in high speed mode
		if (PIOS_MPU9250_ClaimBus(false) != 0)
			continue;

		if (PIOS_SPI_TransferBlock(dev->spi_id, mpu9250_tx_buf, mpu9250_rec_buf, sizeof(mpu9250_rec_buf), 0) < 0) {
			PIOS_MPU9250_ReleaseBus(false);
			continue;
		}

		PIOS_MPU9250_ReleaseBus(false);
		// Rotate the sensor to OP convention.  The datasheet defines X as towards the right
		// and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
		// to our convention

		// Currently we only support rotations on top so switch X/Y accordingly
		struct pios_sensor_accel_data accel_data;
		struct pios_sensor_gyro_data gyro_data;
		// struct pios_sensor_mag_data mag_data;

		float accel_x = (int16_t)(mpu9250_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9250_rec_buf[IDX_ACCEL_XOUT_L]);
		float accel_y = (int16_t)(mpu9250_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9250_rec_buf[IDX_ACCEL_YOUT_L]);
		float accel_z = (int16_t)(mpu9250_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9250_rec_buf[IDX_ACCEL_ZOUT_L]);
		float gyro_x = (int16_t)(mpu9250_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9250_rec_buf[IDX_GYRO_XOUT_L]);
		float gyro_y = (int16_t)(mpu9250_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9250_rec_buf[IDX_GYRO_YOUT_L]);
		float gyro_z = (int16_t)(mpu9250_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9250_rec_buf[IDX_GYRO_ZOUT_L]);

		switch (dev->cfg->orientation) {
		case PIOS_MPU9250_TOP_0DEG:
			accel_data.y = accel_x;
			accel_data.x = accel_y;;
			accel_data.z = - accel_z;
			gyro_data.y  = gyro_x;
			gyro_data.x  = gyro_y;
			gyro_data.z  = - gyro_z;

			break;
		case PIOS_MPU9250_TOP_90DEG:
			accel_data.y = - accel_y;
			accel_data.x = accel_x;
			accel_data.z = - accel_z;
			gyro_data.y  = - gyro_y;
			gyro_data.x  = gyro_x;
			gyro_data.z  = - gyro_z;


			break;
		case PIOS_MPU9250_TOP_180DEG:
			accel_data.y = - accel_x;
			accel_data.x = - accel_y;
			accel_data.z = - accel_z;
			gyro_data.y  = - gyro_x;
			gyro_data.x  = - gyro_y;
			gyro_data.z  = - gyro_z;
			break;
		case PIOS_MPU9250_TOP_270DEG:
			accel_data.y = accel_y;
			accel_data.x = - accel_x;
			accel_data.z = - accel_z;
			gyro_data.y  = gyro_y;
			gyro_data.x  = - gyro_x;
			gyro_data.z  = - gyro_z;
			break;
		case PIOS_MPU9250_BOTTOM_0DEG:
			accel_data.y = - accel_x;
			accel_data.x = accel_y;
			accel_data.z = accel_z;
			gyro_data.y  = - gyro_x;
			gyro_data.x  = gyro_y;
			gyro_data.z  = gyro_z;
			break;

		case PIOS_MPU9250_BOTTOM_90DEG:
			accel_data.y = - accel_y;
			accel_data.x = - accel_x;
			accel_data.z = accel_z;
			gyro_data.y  = - gyro_y;
			gyro_data.x  = - gyro_x;
			gyro_data.z  = gyro_z;
			break;

		case PIOS_MPU9250_BOTTOM_180DEG:
			accel_data.y = accel_x;
			accel_data.x = - accel_y;
			accel_data.z = accel_z;
			gyro_data.y  = gyro_x;
			gyro_data.x  = - gyro_y;
			gyro_data.z  = gyro_z;
			break;

		case PIOS_MPU9250_BOTTOM_270DEG:
			accel_data.y = accel_y;
			accel_data.x = accel_x;
			gyro_data.y  = gyro_y;
			gyro_data.x  = gyro_x;
			gyro_data.z  = gyro_z;
			accel_data.z = accel_z;
			break;

		}

		int16_t raw_temp = (int16_t)(mpu9250_rec_buf[IDX_TEMP_OUT_H] << 8 | mpu9250_rec_buf[IDX_TEMP_OUT_L]);
		float temperature = 21.0f + ((float)raw_temp) / 333.87f;

		// Apply sensor scaling
		float accel_scale = PIOS_MPU9250_GetAccelScale();
		accel_data.x *= accel_scale;
		accel_data.y *= accel_scale;
		accel_data.z *= accel_scale;
		accel_data.temperature = temperature;

		float gyro_scale = PIOS_MPU9250_GetGyroScale();
		gyro_data.x *= gyro_scale;
		gyro_data.y *= gyro_scale;
		gyro_data.z *= gyro_scale;
		gyro_data.temperature = temperature;

		xQueueSendToBack(dev->accel_queue, (void *)&accel_data, 0);
		xQueueSendToBack(dev->gyro_queue, (void *)&gyro_data, 0);

//		test++;
//		if (test % 1000 == 0) {
//			int i = 0;
//			DEBUG_PRINTF(0, "\r\nrecbuf = ")
//			for (i = 0; i < BUFFER_SIZE; i++)
//				DEBUG_PRINTF(0, "0x%02x,", mpu9250_rec_buf[i]);
//		}

		// Check for mag data ready.  Reading it clears this flag.
//		if (PIOS_MPU9250_Mag_GetReg(MPU9250_MAG_STATUS) > 0) {
//			struct pios_sensor_mag_data mag_data;
//			uint8_t mpu9250_mag_buffer[6];
//			if (PIOS_MPU9250_Mag_Read(MPU9250_MAG_XH, mpu9250_mag_buffer, sizeof(mpu9250_mag_buffer)) == 0) {
//				switch(dev->cfg->orientation) {
//				case PIOS_MPU9250_TOP_0DEG:
//					mag_data.x = (int16_t) (mpu9250_mag_buffer[1] << 0x08 | mpu9250_mag_buffer[0]);
//					mag_data.y = (int16_t) (mpu9250_mag_buffer[3] << 0x08 | mpu9250_mag_buffer[2]);
//					break;
//				case PIOS_MPU9250_TOP_90DEG:
//					mag_data.y = (int16_t)  (mpu9250_mag_buffer[1] << 0x08 | mpu9250_mag_buffer[0]);
//					mag_data.x = (int16_t) -(mpu9250_mag_buffer[3] << 0x08 | mpu9250_mag_buffer[2]);
//					break;
//				case PIOS_MPU9250_TOP_180DEG:
//					mag_data.x = (int16_t) -(mpu9250_mag_buffer[1] << 0x08 | mpu9250_mag_buffer[0]);
//					mag_data.y = (int16_t) -(mpu9250_mag_buffer[3] << 0x08 | mpu9250_mag_buffer[2]);
//					break;
//				case PIOS_MPU9250_TOP_270DEG:
//					mag_data.y = (int16_t) -(mpu9250_mag_buffer[1] << 0x08 | mpu9250_mag_buffer[0]);
//					mag_data.x = (int16_t)  (mpu9250_mag_buffer[3] << 0x08 | mpu9250_mag_buffer[2]);
//					break;
//				}
//				mag_data.z = (int16_t) (mpu9250_mag_buffer[5] << 0x08 | mpu9250_mag_buffer[4]);
//
//				float mag_scale = PIOS_MPU9250_GetMagScale();
//				mag_data.x *= mag_scale;
//				mag_data.y *= mag_scale;
//				mag_data.z *= mag_scale;
//
//				// Trigger another measurement
//				PIOS_MPU9250_Mag_SetReg(MPU9250_MAG_CNTR, 0x01);
//
//				xQueueSendToBack(dev->mag_queue, (void *) &mag_data, 0);
//			}
//		}
	}
}

#endif

/**
 * @}
 * @}
 */
