/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup Draco Draco support files
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @brief      The board specific initialization routines
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
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

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.  
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */

#include "board_hw_defs.c"

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include "hwdraco.h"
#include "manualcontrolsettings.h"
#include "modulesettings.h"

/**
 * Sensor configurations
 */
#if defined(PIOS_INCLUDE_HMC5883)
#include "pios_hmc5883_priv.h"

static const struct pios_hmc5883_cfg pios_hmc5883_external_cfg = {
	.M_ODR = PIOS_HMC5883_ODR_75,
	.Meas_Conf = PIOS_HMC5883_MEASCONF_NORMAL,
	.Gain = PIOS_HMC5883_GAIN_1_9,
	.Mode = PIOS_HMC5883_MODE_SINGLE,
	.Default_Orientation = PIOS_HMC5883_TOP_0DEG,
};
#endif /* PIOS_INCLUDE_HMC5883 */

/**
 * Configuration for the MS5611 chip
 */
#if defined(PIOS_INCLUDE_MS5611_SPI)
#include "pios_ms5611_priv.h"
static const struct pios_ms5611_cfg pios_ms5611_cfg = {
	.oversampling = MS5611_OSR_1024,
	.temperature_interleaving = 1,
};
#endif /* PIOS_INCLUDE_MS5611 */


/**
 * Configuration for the MPU9250 chip
 */
#if defined(PIOS_INCLUDE_MPU9250_SPI)
#include "pios_mpu9250.h"
	static const struct pios_exti_cfg pios_exti_mpu9250_cfg __exti_config = {
		.vector = PIOS_MPU9250_IRQHandler,
		.line = EXTI_Line0,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_0,
				.GPIO_Speed = GPIO_Speed_100MHz,
				.GPIO_Mode = GPIO_Mode_IN,
				.GPIO_OType = GPIO_OType_OD,
				.GPIO_PuPd = GPIO_PuPd_NOPULL,
			},
		},
		.irq = {
			.init = {
				.NVIC_IRQChannel = EXTI0_IRQn,
				.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
				.NVIC_IRQChannelSubPriority = 0,
				.NVIC_IRQChannelCmd = ENABLE,
			},
		},
		.exti = {
			.init = {
				.EXTI_Line = EXTI_Line0, // matches above GPIO pin
				.EXTI_Mode = EXTI_Mode_Interrupt,
				.EXTI_Trigger = EXTI_Trigger_Rising,
				.EXTI_LineCmd = ENABLE,
			},
		},
	};

	static const struct pios_mpu9250_cfg pios_mpu9250_intmag_cfg = {
		.exti_cfg = &pios_exti_mpu9250_cfg,
		.default_samplerate = 500,
		.interrupt_cfg = PIOS_MPU60X0_INT_CLR_ANYRD,
		.use_magnetometer = true,
		.default_gyro_filter = PIOS_MPU9250_GYRO_LOWPASS_184_HZ,
		.default_accel_filter = PIOS_MPU9250_ACCEL_LOWPASS_184_HZ,
		.orientation = PIOS_MPU9250_BOTTOM_270DEG,
	};

	static const struct pios_mpu9250_cfg pios_mpu9250_extmag_cfg = {
		.exti_cfg = &pios_exti_mpu9250_cfg,
		.default_samplerate = 500,
		.interrupt_cfg = PIOS_MPU60X0_INT_CLR_ANYRD,
		.use_magnetometer = false,
		.default_gyro_filter = PIOS_MPU9250_GYRO_LOWPASS_184_HZ,
		.default_accel_filter = PIOS_MPU9250_ACCEL_LOWPASS_184_HZ,
		.orientation = PIOS_MPU9250_BOTTOM_270DEG,
	};
#endif

/**
 * Configuration for the OSD SPI comm layer
 */
#ifdef DRACO_INCLUDE_OSD_SUPPORT
#include "osd_spicomm.h"
#include "osd_task.h"
	static const struct pios_exti_cfg osd_exti_cfg __exti_config= {
		.vector = draco_osd_comm_irq_handler,
		.line = EXTI_Line2,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_2,
				.GPIO_Speed = GPIO_Speed_100MHz,
				.GPIO_Mode = GPIO_Mode_IN,
				.GPIO_OType = GPIO_OType_OD,
				.GPIO_PuPd = GPIO_PuPd_UP,
			},
		},
		.irq = {
			.init = {
				.NVIC_IRQChannel = EXTI2_IRQn,
				.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW,
				.NVIC_IRQChannelSubPriority = 0,
				.NVIC_IRQChannelCmd = ENABLE,
			},
		},
		.exti = {
			.init = {
				.EXTI_Line = EXTI_Line2, // matches above GPIO pin
				.EXTI_Mode = EXTI_Mode_Interrupt,
				.EXTI_Trigger = EXTI_Trigger_Falling,
				.EXTI_LineCmd = ENABLE,
			},
		},
	};

	static const struct osd_comm_cfg osd_comm_config = {
		.exti_cfg = &osd_exti_cfg,
		.transfer_granularity = 20,
	};
#endif

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, SPEKTRUM1, SPEKTRUM2, SBUS
 * NOTE: No slot in this map for NONE.
 */
uintptr_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define PIOS_COM_TELEM_RF_RX_BUF_LEN 512
#define PIOS_COM_TELEM_RF_TX_BUF_LEN 512

#define PIOS_COM_GPS_RX_BUF_LEN 32
#define PIOS_COM_GPS_TX_BUF_LEN 16

#define PIOS_COM_TELEM_USB_RX_BUF_LEN 65
#define PIOS_COM_TELEM_USB_TX_BUF_LEN 65

#define PIOS_COM_BRIDGE_RX_BUF_LEN 65
#define PIOS_COM_BRIDGE_TX_BUF_LEN 12

#define PIOS_COM_MAVLINK_TX_BUF_LEN 128

#define PIOS_COM_HOTT_RX_BUF_LEN 16
#define PIOS_COM_HOTT_TX_BUF_LEN 16

#define PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN 128

#define PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN 19

#define PIOS_COM_PICOC_RX_BUF_LEN 128
#define PIOS_COM_PICOC_TX_BUF_LEN 128

#define PIOS_COM_FRSKYSPORT_TX_BUF_LEN 16
#define PIOS_COM_FRSKYSPORT_RX_BUF_LEN 16

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN 40
uintptr_t pios_com_debug_id;
#endif /* PIOS_INCLUDE_DEBUG_CONSOLE */

uintptr_t pios_com_gps_id;
uintptr_t pios_com_telem_usb_id;
uintptr_t pios_com_telem_rf_id;
uintptr_t pios_com_vcp_id;
uintptr_t pios_com_bridge_id;
uintptr_t pios_com_overo_id;
uintptr_t pios_com_mavlink_id;
uintptr_t pios_com_hott_id;
uintptr_t pios_com_frsky_sensor_hub_id;
uintptr_t pios_com_lighttelemetry_id;
uintptr_t pios_com_picoc_id;
uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_waypoints_settings_fs_id;
uintptr_t pios_internal_adc_id;
uintptr_t pios_com_frsky_sport_id;

/*
 * Setup a com port based on the passed cfg, driver and buffer sizes. rx or tx size of 0 disables rx or tx
 */
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
static void PIOS_Board_configure_com (const struct pios_usart_cfg *usart_port_cfg, size_t rx_buf_len, size_t tx_buf_len,
		const struct pios_com_driver *com_driver, uintptr_t *pios_com_id)
{
	uintptr_t pios_usart_id;
	if (PIOS_USART_Init(&pios_usart_id, usart_port_cfg)) {
		PIOS_Assert(0);
	}

	uint8_t * rx_buffer;
	if (rx_buf_len > 0) {
		rx_buffer = (uint8_t *) PIOS_malloc(rx_buf_len);
		PIOS_Assert(rx_buffer);
	} else {
		rx_buffer = NULL;
	}

	uint8_t * tx_buffer;
	if (tx_buf_len > 0) {
		tx_buffer = (uint8_t *) PIOS_malloc(tx_buf_len);
		PIOS_Assert(tx_buffer);
	} else {
		tx_buffer = NULL;
	}

	if (PIOS_COM_Init(pios_com_id, com_driver, pios_usart_id,
				rx_buffer, rx_buf_len,
				tx_buffer, tx_buf_len)) {
		PIOS_Assert(0);
	}
}
#endif	/* PIOS_INCLUDE_USART && PIOS_INCLUDE_COM */

#ifdef PIOS_INCLUDE_DSM
static void PIOS_Board_configure_dsm(const struct pios_usart_cfg *pios_usart_dsm_cfg, const struct pios_dsm_cfg *pios_dsm_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_dsm_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup,uint8_t *bind)
{
	uintptr_t pios_usart_dsm_id;
	if (PIOS_USART_Init(&pios_usart_dsm_id, pios_usart_dsm_cfg)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_id;
	if (PIOS_DSM_Init(&pios_dsm_id, pios_dsm_cfg, pios_usart_com_driver,
			pios_usart_dsm_id, *proto, *bind)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_rcvr_id;
	if (PIOS_RCVR_Init(&pios_dsm_rcvr_id, &pios_dsm_rcvr_driver, pios_dsm_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_dsm_rcvr_id;
}
#endif

#ifdef PIOS_INCLUDE_HSUM
static void PIOS_Board_configure_hsum(const struct pios_usart_cfg *pios_usart_hsum_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_hsum_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup)
{
	uintptr_t pios_usart_hsum_id;
	if (PIOS_USART_Init(&pios_usart_hsum_id, pios_usart_hsum_cfg)) {
		PIOS_Assert(0);
	}
	
	uintptr_t pios_hsum_id;
	if (PIOS_HSUM_Init(&pios_hsum_id, pios_usart_com_driver,
			  pios_usart_hsum_id, *proto)) {
		PIOS_Assert(0);
	}
	
	uintptr_t pios_hsum_rcvr_id;
	if (PIOS_RCVR_Init(&pios_hsum_rcvr_id, &pios_hsum_rcvr_driver, pios_hsum_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_hsum_rcvr_id;
}
#endif

/**
 * Indicate a target-specific error code when a component fails to initialize
 * 1 pulse - flash chip
 * 2 pulses - MPU9250
 * 3 pulses - external HMC5883
 * 4 pulses - MS5611
 * 5 pulses - external I2C bus locked
 */
void panic(int32_t code) {
	while(1){
		for (int32_t i = 0; i < code; i++) {
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
		}
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(100);
	}
}

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

void PIOS_Board_Init(void) {

	/* Delay system */
	PIOS_DELAY_Init();
	
	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
#endif	/* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_I2C)
	if (PIOS_I2C_Init(&pios_i2c_external_adapter_id, &pios_i2c_external_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_I2C_CheckClear(pios_i2c_external_adapter_id) != 0)
		panic(5);
#endif

#if defined(PIOS_INCLUDE_SPI)
	if (PIOS_SPI_Init(&pios_spi_internal_id, &pios_spi_internal_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
#endif

#if defined(PIOS_INCLUDE_FLASH)
	/* Inititialize all flash drivers */
	if (PIOS_Flash_Jedec_Init(&pios_external_flash_id, pios_spi_internal_id, 0, &flash_n25q_cfg) != 0)
		panic(1);
	if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
		panic(1);

	/* Register the partition table */
	const struct pios_flash_partition * flash_partition_table;
	uint32_t num_partitions;
	flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
	PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

	/* Mount all filesystems */
	if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0)
		panic(1);
	if (PIOS_FLASHFS_Logfs_Init(&pios_waypoints_settings_fs_id, &flashfs_waypoints_cfg, FLASH_PARTITION_LABEL_WAYPOINTS) != 0)
		panic(1);
#endif	/* PIOS_INCLUDE_FLASH */

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	/* Initialize the alarms library */
	AlarmsInitialize();

	HwDracoInitialize();
	ModuleSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

#ifndef ERASE_FLASH
	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}
#endif

	/* Set up pulse timers */

	PIOS_TIM_InitClock(&tim_5_cfg);
	PIOS_TIM_InitClock(&tim_1_cfg);
	PIOS_TIM_InitClock(&tim_9_cfg);
	PIOS_TIM_InitClock(&tim_3_cfg);
	PIOS_TIM_InitClock(&tim_4_cfg);

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwDracoSetDefaults(HwDracoHandle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;
	bool usb_cdc_present = false;

#if defined(PIOS_INCLUDE_USB_CDC)
	if (PIOS_USB_DESC_HID_CDC_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
	usb_cdc_present = true;
#else
	if (PIOS_USB_DESC_HID_ONLY_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
#endif

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, PIOS_BOARD_HW_DEFS_GetUsbCfg(bdinfo->board_rev));

#if defined(PIOS_INCLUDE_USB_CDC)

	uint8_t hw_usb_vcpport;
	/* Configure the USB VCP port */
	HwDracoUSB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWDRACO_USB_VCPPORT_DISABLED;
	}

	uintptr_t pios_usb_cdc_id;
	if (PIOS_USB_CDC_Init(&pios_usb_cdc_id, &pios_usb_cdc_cfg, pios_usb_id)) {
		PIOS_Assert(0);
	}

	switch (hw_usb_vcpport) {
	case HWDRACO_USB_VCPPORT_DISABLED:
		break;
	case HWDRACO_USB_VCPPORT_USBTELEMETRY:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDRACO_USB_VCPPORT_COMBRIDGE:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_BRIDGE_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_BRIDGE_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_vcp_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_BRIDGE_RX_BUF_LEN,
						tx_buffer, PIOS_COM_BRIDGE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDRACO_USB_VCPPORT_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_COM)
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
		{
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_debug_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						NULL, 0,
						tx_buffer, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDRACO_USB_VCPPORT_PICOC:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_PICOC_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_PICOC_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_picoc_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_PICOC_RX_BUF_LEN,
						tx_buffer, PIOS_COM_PICOC_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	}
#endif	/* PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID)
	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;
	HwDracoUSB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWDRACO_USB_HIDPORT_DISABLED;
	}

	uintptr_t pios_usb_hid_id;
	if (PIOS_USB_HID_Init(&pios_usb_hid_id, &pios_usb_hid_cfg, pios_usb_id)) {
		PIOS_Assert(0);
	}

	switch (hw_usb_hidport) {
	case HWDRACO_USB_HIDPORT_DISABLED:
		break;
	case HWDRACO_USB_HIDPORT_USBTELEMETRY:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_hid_com_driver, pios_usb_hid_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	}

#endif	/* PIOS_INCLUDE_USB_HID */

	if (usb_hid_present || usb_cdc_present) {
		PIOS_USBHOOK_Activate();
	}
#endif	/* PIOS_INCLUDE_USB */

	/* Configure the IO ports */
	uint8_t hw_DSMxBind;
	HwDracoDSMxBindGet(&hw_DSMxBind);

	/* init sensor queue registration */
	PIOS_SENSORS_Init();

#if defined(PIOS_INCLUDE_MS5611_SPI)
	if (PIOS_MS5611_SPI_Init(pios_spi_internal_id, 2, &pios_ms5611_cfg) != 0)
		panic(4);
	if (PIOS_MS5611_SPI_Test() != 0)
		panic(4);
#endif

	/* UART Xbee Port */
	uint8_t hw_uart_xbee;
	HwDracoUartXbeeGet(&hw_uart_xbee);
	switch (hw_uart_xbee) {
	case HWDRACO_UARTXBEE_DISABLED:
		break;
	case HWDRACO_UARTXBEE_TELEMETRY:
#if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;
	case HWDRACO_UARTXBEE_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDRACO_UARTXBEE_DSM2:
	case HWDRACO_UARTXBEE_DSMX10BIT:
	case HWDRACO_UARTXBEE_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart_xbee) {
			case HWDRACO_UARTXBEE_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDRACO_UARTXBEE_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDRACO_UARTXBEE_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_uart_xbee_dsm_hsum_cfg, &pios_uart_xbee_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
		break;
	case HWDRACO_UARTXBEE_HOTTSUMD:
	case HWDRACO_UARTXBEE_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart_xbee) {
			case HWDRACO_UARTXBEE_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDRACO_UARTXBEE_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_uart_xbee_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDRACO_UARTXBEE_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_debug_id);
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;
	case HWDRACO_UARTXBEE_COMBRIDGE:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
#endif
		break;
	case HWDRACO_UARTXBEE_MAVLINKTX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTXBEE_MAVLINKTX_GPS_RX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK) && defined(PIOS_INCLUDE_GPS)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTXBEE_HOTTTELEMETRY:
#if defined(PIOS_INCLUDE_HOTT) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_HOTT_RX_BUF_LEN, PIOS_COM_HOTT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_hott_id);
#endif /* PIOS_INCLUDE_HOTT */
		break;
	case HWDRACO_UARTXBEE_FRSKYSENSORHUB:
#if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
#endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;
	case HWDRACO_UARTXBEE_LIGHTTELEMETRYTX:
#if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
#endif  
		break;
	case HWDRACO_UARTXBEE_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_xbee_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	}

	/* UART GPS Port */
	uint8_t hw_uart_gps;
	HwDracoUartGPSGet(&hw_uart_gps);
	switch (hw_uart_gps) {
	case HWDRACO_UARTGPS_DISABLED:
		break;
	case HWDRACO_UARTGPS_TELEMETRY:
#if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;
	case HWDRACO_UARTGPS_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDRACO_UARTGPS_DSM2:
	case HWDRACO_UARTGPS_DSMX10BIT:
	case HWDRACO_UARTGPS_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart_gps) {
			case HWDRACO_UARTGPS_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDRACO_UARTGPS_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDRACO_UARTGPS_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_uart_gps_dsm_hsum_cfg, &pios_uart_gps_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
		break;
	case HWDRACO_UARTGPS_HOTTSUMD:
	case HWDRACO_UARTGPS_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart_gps) {
			case HWDRACO_UARTGPS_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDRACO_UARTGPS_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_uart_gps_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDRACO_UARTGPS_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_debug_id);
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;
	case HWDRACO_UARTGPS_COMBRIDGE:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
#endif
		break;
	case HWDRACO_UARTGPS_MAVLINKTX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTGPS_MAVLINKTX_GPS_RX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK) && defined(PIOS_INCLUDE_GPS)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTGPS_HOTTTELEMETRY:
#if defined(PIOS_INCLUDE_HOTT) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_HOTT_RX_BUF_LEN, PIOS_COM_HOTT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_hott_id);
#endif /* PIOS_INCLUDE_HOTT */
		break;
	case HWDRACO_UARTGPS_FRSKYSENSORHUB:
#if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
#endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;
	case HWDRACO_UARTGPS_LIGHTTELEMETRYTX:
#if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
#endif  
		break;
	case HWDRACO_UARTGPS_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_gps_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	}

	/* UART onewire half-duplex line */
	uint8_t hw_uart_onewire;
	HwDracoUartOnewireGet(&hw_uart_onewire);
	switch(hw_uart_onewire) {
	case HWDRACO_UARTONEWIRE_FRSKYSPORTTELEMETRY:
#if defined(PIOS_INCLUDE_FRSKY_SPORT_TELEMETRY) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_onewire_sport_cfg, PIOS_COM_FRSKYSPORT_RX_BUF_LEN, PIOS_COM_FRSKYSPORT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sport_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	default:
		break;
	}
	/* UART Extension Port */
	uint8_t hw_uart_extension;
	HwDracoUartExtensionGet(&hw_uart_extension);
	switch (hw_uart_extension) {
	case HWDRACO_UARTEXTENSION_DISABLED:
		break;
	case HWDRACO_UARTEXTENSION_TELEMETRY:
#if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;
	case HWDRACO_UARTEXTENSION_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDRACO_UARTEXTENSION_DSM2:
	case HWDRACO_UARTEXTENSION_DSMX10BIT:
	case HWDRACO_UARTEXTENSION_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart_extension) {
			case HWDRACO_UARTEXTENSION_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDRACO_UARTEXTENSION_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDRACO_UARTEXTENSION_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_uart_extension_dsm_hsum_cfg, &pios_uart_extension_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
		break;
	case HWDRACO_UARTEXTENSION_HOTTSUMD:
	case HWDRACO_UARTEXTENSION_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart_extension) {
			case HWDRACO_UARTEXTENSION_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDRACO_UARTEXTENSION_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_uart_extension_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDRACO_UARTEXTENSION_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_debug_id);
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;
	case HWDRACO_UARTEXTENSION_COMBRIDGE:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
#endif
		break;
	case HWDRACO_UARTEXTENSION_MAVLINKTX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTEXTENSION_MAVLINKTX_GPS_RX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK) && defined(PIOS_INCLUDE_GPS)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDRACO_UARTEXTENSION_HOTTTELEMETRY:
#if defined(PIOS_INCLUDE_HOTT) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_HOTT_RX_BUF_LEN, PIOS_COM_HOTT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_hott_id);
#endif /* PIOS_INCLUDE_HOTT */
		break;
	case HWDRACO_UARTEXTENSION_FRSKYSENSORHUB:
#if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
#endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;
	case HWDRACO_UARTEXTENSION_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	case HWDRACO_UARTEXTENSION_LIGHTTELEMETRYTX:
#if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
		PIOS_Board_configure_com(&pios_uart_extension_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
#endif  
		break;
	}

	/* Configure PWM inputs & outputs */
	uint8_t hw_outputport;
	HwDracoOutputPortGet(&hw_outputport);
	switch(hw_outputport) {
	case HWDRACO_OUTPUTPORT_12OUTPUTS:
#ifdef PIOS_INCLUDE_SERVO
		{
				PIOS_Servo_Init(&pios_servo_12_out_cfg);
		}
#endif

		break;
	case HWDRACO_OUTPUTPORT_6OUTPUTS6INPUTS:
#ifdef PIOS_INCLUDE_SERVO
		{
				PIOS_Servo_Init(&pios_servo_6_out_6_in_cfg);
		}
#endif
#if defined(PIOS_INCLUDE_PWM)
		{
			uintptr_t pios_pwm_id;
			PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_cfg);

			uintptr_t pios_pwm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PWM */

		break;
	}


	/* Configure Receiver PIN */
	uint8_t hw_rcvr_pin;
	HwDracoRcvrPinGet(&hw_rcvr_pin);

	switch(hw_rcvr_pin) {
	case HWDRACO_RCVRPIN_DISABLED:
		break;

	case HWDRACO_RCVRPIN_SBUS:
#if defined(PIOS_INCLUDE_SBUS) && defined(PIOS_INCLUDE_USART)
		{
			uintptr_t pios_usart_sbus_id;
			if (PIOS_USART_Init(&pios_usart_sbus_id, &pios_uart_rcvr_sbus_cfg)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_id;
			if (PIOS_SBus_Init(&pios_sbus_id, &pios_uart_rcvr_sbus_aux_cfg, &pios_usart_com_driver, pios_usart_sbus_id)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_rcvr_id;
			if (PIOS_RCVR_Init(&pios_sbus_rcvr_id, &pios_sbus_rcvr_driver, pios_sbus_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_SBUS] = pios_sbus_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_SBUS */
	break;

	case HWDRACO_RCVRPIN_CPPM:
#if defined(PIOS_INCLUDE_PPM)
		{
			uintptr_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uintptr_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PPM */
	break;
	}

#if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uintptr_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uintptr_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif	/* PIOS_INCLUDE_GCSRCVR */

#ifdef PIOS_DEBUG_ENABLE_DEBUG_PINS
	PIOS_DEBUG_Init(&pios_tim_servoport_all_channels, NELEMENTS(pios_tim_servoport_all_channels));
#endif

	PIOS_WDG_Clear();

	uint8_t hw_magnetometer;
	HwDracoMagnetometerGet(&hw_magnetometer);
#if defined(PIOS_INCLUDE_I2C)
#if defined(PIOS_INCLUDE_HMC5883)
	{
		if (hw_magnetometer == HWDRACO_MAGNETOMETER_EXTERNALI2C) {
			if (PIOS_HMC5883_Init(pios_i2c_external_adapter_id, &pios_hmc5883_external_cfg) != 0)
				panic(3);
			if (PIOS_HMC5883_Test() != 0)
				panic(3);

			uint8_t ExtMagOrientation;
			HwDracoExtMagOrientationGet(&ExtMagOrientation);
			enum pios_hmc5883_orientation hmc5883_orientation = \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_TOP0DEGCW) ? PIOS_HMC5883_TOP_0DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_TOP90DEGCW) ? PIOS_HMC5883_TOP_90DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_TOP180DEGCW) ? PIOS_HMC5883_TOP_180DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_TOP270DEGCW) ? PIOS_HMC5883_TOP_270DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_BOTTOM0DEGCW) ? PIOS_HMC5883_BOTTOM_0DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_BOTTOM90DEGCW) ? PIOS_HMC5883_BOTTOM_90DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_BOTTOM180DEGCW) ? PIOS_HMC5883_BOTTOM_180DEG : \
				(ExtMagOrientation == HWDRACO_EXTMAGORIENTATION_BOTTOM270DEGCW) ? PIOS_HMC5883_BOTTOM_270DEG : \
				pios_hmc5883_external_cfg.Default_Orientation;
			PIOS_HMC5883_SetOrientation(hmc5883_orientation);
		}
	}
#endif

	PIOS_WDG_Clear();
#if defined (PIOS_INCLUDE_MPU9250_SPI) && defined(PIOS_INCLUDE_SPI)
	{
		const struct pios_mpu9250_cfg *mpu9250_cfg;
		if (hw_magnetometer == HWDRACO_MAGNETOMETER_INTERNAL) {
			mpu9250_cfg = &pios_mpu9250_intmag_cfg;
		} else {
			mpu9250_cfg = &pios_mpu9250_extmag_cfg;
		}


		if (PIOS_MPU9250_SPI_Init(pios_spi_internal_id, 1, mpu9250_cfg) != 0)
			panic(2);

		if (PIOS_MPU9250_Test() != 0)
			panic(2);

		uint8_t hw_gyro_range;
		HwDracoGyroRangeGet(&hw_gyro_range);
		switch(hw_gyro_range) {
		case HWDRACO_GYRORANGE_250:
			PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_250_DEG);
			break;

		case HWDRACO_GYRORANGE_500:
			PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);
			break;

		case HWDRACO_GYRORANGE_1000:
			PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_1000_DEG);
			break;

		case HWDRACO_GYRORANGE_2000:
			PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_2000_DEG);
			break;
		}

		uint8_t hw_accel_range;
		HwDracoAccelRangeGet(&hw_accel_range);
		switch(hw_accel_range) {
		case HWDRACO_ACCELRANGE_2G:
			PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_2G);
			break;
		case HWDRACO_ACCELRANGE_4G:
			PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_4G);
			break;
		case HWDRACO_ACCELRANGE_8G:
			PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);
			break;
		case HWDRACO_ACCELRANGE_16G:
			PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_16G);
			break;
		}
		// the filter has to be set before rate else divisor calculation will fail

		uint8_t hw_mpu9250_gyro_dlpf;
		HwDracoMPU9250DLPFGyroGet(&hw_mpu9250_gyro_dlpf);
		enum pios_mpu9250_gyro_filter mpu9250_gyro_dlpf = \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_184) ? PIOS_MPU9250_GYRO_LOWPASS_184_HZ : \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_92) ? PIOS_MPU9250_GYRO_LOWPASS_92_HZ : \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_41) ? PIOS_MPU9250_GYRO_LOWPASS_41_HZ : \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_20) ? PIOS_MPU9250_GYRO_LOWPASS_20_HZ : \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_10) ? PIOS_MPU9250_GYRO_LOWPASS_10_HZ : \
			(hw_mpu9250_gyro_dlpf == HWDRACO_MPU9250DLPFGYRO_5) ? PIOS_MPU9250_GYRO_LOWPASS_5_HZ : \
			mpu9250_cfg->default_gyro_filter;
		PIOS_MPU9250_SetGyroLPF(mpu9250_gyro_dlpf);

		uint8_t hw_mpu9250_accel_dlpf;
		HwDracoMPU9250DLPFAccelGet(&hw_mpu9250_accel_dlpf);
		enum pios_mpu9250_accel_filter mpu9250_accel_dlpf = \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_460) ? PIOS_MPU9250_ACCEL_LOWPASS_460_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_184) ? PIOS_MPU9250_ACCEL_LOWPASS_184_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_92) ? PIOS_MPU9250_ACCEL_LOWPASS_92_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_41) ? PIOS_MPU9250_ACCEL_LOWPASS_41_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_20) ? PIOS_MPU9250_ACCEL_LOWPASS_20_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_10) ? PIOS_MPU9250_ACCEL_LOWPASS_10_HZ : \
			(hw_mpu9250_accel_dlpf == HWDRACO_MPU9250DLPFACCEL_5) ? PIOS_MPU9250_ACCEL_LOWPASS_5_HZ : \
			mpu9250_cfg->default_accel_filter;
		PIOS_MPU9250_SetAccelLPF(mpu9250_accel_dlpf);

		uint8_t hw_mpu9250_samplerate;
		HwDracoMPU9250RateGet(&hw_mpu9250_samplerate);
		uint16_t samplerate = mpu9250_cfg->default_samplerate;
		samplerate = \
			(samplerate == HWDRACO_MPU9250RATE_200) ? 200 : \
			(samplerate == HWDRACO_MPU9250RATE_333) ? 333 : \
			(samplerate == HWDRACO_MPU9250RATE_500) ? 500 : \
			(samplerate == HWDRACO_MPU9250RATE_1000) ? 1000 : \
			mpu9250_cfg->default_samplerate;
		PIOS_MPU9250_SetSampleRate(samplerate);
	}
#endif

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(150);
	PIOS_WDG_Clear();

#endif	/* PIOS_INCLUDE_I2C */

#if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
#endif

#if defined(PIOS_INCLUDE_ADC)
	uint32_t internal_adc_id;
	PIOS_INTERNAL_ADC_Init(&internal_adc_id, &pios_adc_cfg);
	PIOS_ADC_Init(&pios_internal_adc_id, &pios_internal_adc_driver, internal_adc_id);
#endif

#ifdef DRACO_INCLUDE_OSD_SUPPORT
	draco_osd_comm_init(pios_spi_internal_id, 3, &osd_comm_config);
#endif

	/* Make sure we have at least one telemetry link configured or else fail initialization */
	PIOS_Assert(pios_com_telem_rf_id || pios_com_telem_usb_id);
}
/**
 * @}
 */
