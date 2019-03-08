/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup TauOSD Tau Labs OSD support files
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014-2016
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
#include <pios_hal.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include <pios_dacbeep_priv.h>
#include <pios_fskdac_priv.h>
#include "flightbatterysettings.h"
#include "hwtauosd.h"
#include "modulesettings.h"
#include "onscreendisplaysettings.h"

#define PIOS_COM_FSKDAC_BUF_LEN 19
 
uintptr_t pios_uavo_settings_fs_id;

uintptr_t pios_can_id;
uintptr_t pios_internal_adc_id = 0;

uintptr_t dacbeep_handle;


/**
* Initialise PWM Output for black/white level setting
*/

#if defined(PIOS_INCLUDE_VIDEO)
void OSD_configure_bw_levels(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* --------------------------- System Clocks Configuration -----------------*/
	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* Connect TIM1 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = (PIOS_SYSCLK / 25500000) - 1; // Get clock to 25 MHz on STM32F2/F4
	TIM_TimeBaseStructure.TIM_Period = 255;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Enable TIM1 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* TIM PWM1 Mode configuration */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 90;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* Output Compare PWM1 Mode configuration: Channel1 PA.08/09 */
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	TIM1->CCR1 = 30;
	TIM1->CCR2 = 110;
}
#endif /* PIOS_INCLUDE_VIDEO */


void video_input_select(uint8_t ch) {
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	if (ch) {
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
	} else {
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	}
}

/**
 * Indicate a target-specific error code when a component fails to initialize
 * 1 pulse - MPU9150 - no irq
 * 2 pulses - MPU9150 - failed configuration or task starting
 * 3 pulses - internal I2C bus locked
 * 4 pulses - external I2C bus locked
 * 5 pulses - flash
 * 6 pulses - CAN
 */
void panic(int32_t code) {
	PIOS_HAL_Panic(PIOS_LED_ALARM, code);
}

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

/**
 * Check the brown out reset threshold is 2.7 volts and if not
 * resets it.  This solves an issue that can prevent boards
 * powering up with some BEC
 */
void check_bor()
{
    uint8_t bor = FLASH_OB_GetBOR();

    if (bor != OB_BOR_LEVEL3) {
        FLASH_OB_Unlock();
        FLASH_OB_BORConfig(OB_BOR_LEVEL3);
        FLASH_OB_Launch();
        while (FLASH_WaitForLastOperation() == FLASH_BUSY) {
            ;
        }
        FLASH_OB_Lock();
        while (FLASH_WaitForLastOperation() == FLASH_BUSY) {
            ;
        }
    }
}

// Force the OSD to initialize for pass through
static void blank_osd()
{
	GPIO_InitTypeDef init = {
		.GPIO_Pin   = GPIO_Pin_6,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode  = GPIO_Mode_OUT,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL
	};
	GPIO_Init(GPIOA, &init);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);

	init.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &init);
	GPIO_ResetBits(GPIOA, GPIO_Pin_14);
}

void set_vtx_channel(HwTauOsdVTX_ChOptions channel)
{
	uint8_t chan = 0;
	uint8_t band = 0xFF; // Set to "A" band

	switch (channel) {
	case HWTAUOSD_VTX_CH_BOSCAMACH15725:
		chan = 0;
		band = 0;
	case HWTAUOSD_VTX_CH_BOSCAMACH25745:
		chan = 1;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH35765:
		chan = 2;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH45785:
		chan = 3;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH55805:
		chan = 4;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH65825:
		chan = 5;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH75845:
		chan = 6;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMACH85865:
		chan = 7;
		band = 0;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH15733:
		chan = 0;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH25752:
		chan = 1;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH35771:
		chan = 2;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH45790:
		chan = 3;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH55809:
		chan = 4;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH65828:
		chan = 5;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH75847:
		chan = 6;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMBCH85866:
		chan = 7;
		band = 1;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH15705:
		chan = 0;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH25685:
		chan = 1;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH35665:
		chan = 2;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH45645:
		chan = 3;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH55885:
		chan = 4;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH65905:
		chan = 5;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH75925:
		chan = 6;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_BOSCAMECH85945:
		chan = 7;
		band = 2;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH15740:
		chan = 0;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH25760:
		chan = 1;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH35780:
		chan = 2;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH45800:
		chan = 3;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH55820:
		chan = 4;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH65840:
		chan = 5;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH75860:
		chan = 6;
		band = 3;
		break;
	case HWTAUOSD_VTX_CH_AIRWAVECH85860:
		chan = 7;
		band = 3;
		break;
	}

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (chan & 0x01) {
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	} else {
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	}

	if (chan & 0x02) {
		GPIO_SetBits(GPIOD, GPIO_Pin_2);
	} else {
		GPIO_ResetBits(GPIOD, GPIO_Pin_2);
	}

	if (chan & 0x04) {
		GPIO_SetBits(GPIOC, GPIO_Pin_12);
	} else {
		GPIO_ResetBits(GPIOC, GPIO_Pin_12);
	}

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	if (band & 0x01) {
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
	} else {
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	}

	if (band & 0x02) {
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
	} else {
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	}
}

void PIOS_Board_Init(void)
{

	blank_osd();

	check_bor();

	/* Delay system */
	PIOS_DELAY_Init();

	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
#endif	/* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_CAN)
	if (PIOS_CAN_Init(&pios_can_id, &pios_can_cfg) != 0)
		panic(1);
#endif

#if defined(PIOS_INCLUDE_FLASH)
	/* This is required to write to flash, but should not be. Indicates something odd on power up */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	/* Inititialize all flash drivers */
	if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
		panic(2);

	/* Register the partition table */
	const struct pios_flash_partition * flash_partition_table;
	uint32_t num_partitions;
	flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
	PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

	/* Mount all filesystems */
	if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0) {
		panic(2);
	}

#if defined(ERASE_FLASH)
	PIOS_FLASHFS_Format(pios_uavo_settings_fs_id);
#endif

#endif	/* PIOS_INCLUDE_FLASH */

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	/* Initialize the alarms library */
	AlarmsInitialize();

	HwTauOsdInitialize();
	ModuleSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwTauOsdSetDefaults(HwTauOsdHandle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;

#if defined(PIOS_INCLUDE_USB_CDC)
	bool usb_cdc_present = false;
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
	HwTauOsdUSB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWTAUOSD_USB_VCPPORT_DISABLED;
	}

	PIOS_HAL_ConfigureCDC(hw_usb_vcpport, pios_usb_id, &pios_usb_cdc_cfg);
#endif /* PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID)
	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;
	HwTauOsdUSB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWTAUOSD_USB_HIDPORT_DISABLED;
	}
	PIOS_HAL_ConfigureHID(hw_usb_hidport, pios_usb_id, &pios_usb_hid_cfg);
#endif /* PIOS_INCLUDE_USB_HID */

	if (usb_hid_present || usb_cdc_present) {
		PIOS_USBHOOK_Activate();
	}

#endif /* PIOS_INCLUDE_USB */

	/* Configure main USART port */
	uint8_t hw_mainport;
	HwTauOsdMainPortGet(&hw_mainport);

	PIOS_HAL_ConfigurePort(hw_mainport,          // port type protocol
			&pios_main_usart_cfg,                // usart_port_cfg
			&pios_main_usart_cfg,                // frsky usart_port_cfg
			&pios_usart_com_driver,              // com_driver 
			NULL,                                // i2c_id 
			NULL,                                // i2c_cfg 
			NULL,                                // i2c_cfg 
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			NULL,                                // usart_dsm_hsum_cfg 
			NULL,                                // dsm_cfg
			(HwSharedDSMxModeOptions) 0,         // dsm_mode 
			NULL,                                // sbus_rcvr_cfg 
			NULL,                                // sbus_cfg 
			false);                              // sbus_toggle

#if defined(PIOS_INCLUDE_ADC)
	uint32_t internal_adc_id;
	PIOS_INTERNAL_ADC_Init(&internal_adc_id, &pios_adc_cfg);
	PIOS_ADC_Init(&pios_internal_adc_id, &pios_internal_adc_driver, internal_adc_id);

	// If ADC initialized on TauOSD, go ahead and configure specifically for hardawre
	FlightBatterySettingsInitialize();
	FlightBatterySettingsData flightBattery;
	FlightBatterySettingsGet(&flightBattery); // Load so that capacity and warnings can be saved
	flightBattery.CurrentPin = FLIGHTBATTERYSETTINGS_CURRENTPIN_NONE;
	flightBattery.VoltagePin = FLIGHTBATTERYSETTINGS_CURRENTPIN_ADC0;
	flightBattery.SensorCalibrationFactor[0] = 130;
	FlightBatterySettingsSet(&flightBattery);
#endif


#if defined(PIOS_INCLUDE_VIDEO)
	// make sure the mask pin is low
	GPIO_Init(pios_video_cfg.mask.miso.gpio, (GPIO_InitTypeDef*)&pios_video_cfg.mask.miso.init);
	GPIO_ResetBits(pios_video_cfg.mask.miso.gpio, pios_video_cfg.mask.miso.init.GPIO_Pin);

	// Initialize settings
	OnScreenDisplaySettingsInitialize();

	uint8_t osd_state;
	OnScreenDisplaySettingsOSDEnabledGet(&osd_state);
	if (osd_state == ONSCREENDISPLAYSETTINGS_OSDENABLED_ENABLED) {
		OSD_configure_bw_levels();
	}
#endif

	uint8_t dac_mode;
	HwTauOsdDacGet(&dac_mode);
	
	// Select what the DAC is used for
	switch(dac_mode) {
	case HWTAUOSD_DAC_NONE:
		break;
	case HWTAUOSD_DAC_BEEP:
#if defined(PIOS_INCLUDE_DAC_BEEPS)
	{
		uintptr_t dacbeep_id;
		PIOS_DACBEEP_Init(&dacbeep_id);
		dacbeep_handle = dacbeep_id;
	}
#endif /* PIOS_INCLUDE_DAC_BEEPS */
		break;
	case HWTAUOSD_DAC_FSKTELEM:
#if defined(PIOS_INCLUDE_FSK)
	{
		uintptr_t fskdac_id;
		PIOS_FSKDAC_Init(&fskdac_id);

		uintptr_t fskdac_com_id;
		uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_FSKDAC_BUF_LEN);
		PIOS_Assert(tx_buffer);
		if (PIOS_COM_Init(&fskdac_com_id, &pios_fskdac_com_driver, fskdac_id,
		                  NULL, 0,
		                  tx_buffer, PIOS_COM_FSKDAC_BUF_LEN))
			panic(6);

		uint8_t baud = MODULESETTINGS_LIGHTTELEMETRYSPEED_1200;
		ModuleSettingsLightTelemetrySpeedSet(&baud);
		pios_com_lighttelemetry_id = fskdac_com_id; // send from light telemetry when enabled
	}
#endif /* PIOS_INCLUDE_FSK */
		break;
	}

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();

#if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
#endif

	video_input_select(0);

	uint8_t vtx_selection;
	HwTauOsdVTX_ChGet(&vtx_selection);
	set_vtx_channel(vtx_selection);
}

/**
 * @}
 */
