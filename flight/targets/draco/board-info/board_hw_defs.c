/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup Draco Draco support files
 * @{
 *
 * @file       board_hw_defs.c 
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @brief      Defines board specific static initializers for hardware for the
 *             Draco board.
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
 
#include <pios_config.h>
#include <pios_board_info.h>

#if defined(PIOS_INCLUDE_LED)

#include <pios_led_priv.h>
static const struct pios_led pios_leds[] = {
	[PIOS_LED_RED] = {
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin   = GPIO_Pin_7,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_OUT,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd = GPIO_PuPd_DOWN
			},
		},
		.remap = 0,
		.active_high = true,
	},
	[PIOS_LED_BLUE] = {
		.pin = {
			.gpio = GPIOC,
			.init = {
				.GPIO_Pin   = GPIO_Pin_5,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_OUT,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd = GPIO_PuPd_DOWN
			},
		},
		.remap = 0,
		.active_high = true,
	},
};

static const struct pios_led_cfg pios_led_cfg = {
	.leds     = pios_leds,
	.num_leds = NELEMENTS(pios_leds),
};

const struct pios_led_cfg * PIOS_BOARD_HW_DEFS_GetLedCfg (uint32_t board_revision)
{
	return &pios_led_cfg;
}

#endif	/* PIOS_INCLUDE_LED */


#if defined(PIOS_INCLUDE_SPI)
#include <pios_spi_priv.h>

/* SPI3 Interface
 *      - Used for MPU9250, OSD MCU, flash, MS5611
 */

void PIOS_SPI_internal_irq_handler(void);
void DMA1_Stream0_IRQHandler(void) __attribute__((alias("PIOS_SPI_internal_irq_handler")));

static const struct pios_spi_cfg pios_spi_internal_cfg = {
		.regs = SPI3,
		.remap = GPIO_AF_SPI3,
		.init = {
				.SPI_Mode              = SPI_Mode_Master,
				.SPI_Direction         = SPI_Direction_2Lines_FullDuplex,
				.SPI_DataSize          = SPI_DataSize_8b,
				.SPI_NSS               = SPI_NSS_Soft,
				.SPI_FirstBit          = SPI_FirstBit_MSB,
				.SPI_CRCPolynomial     = 7,
				.SPI_CPOL              = SPI_CPOL_High,
				.SPI_CPHA              = SPI_CPHA_2Edge,
				.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4,	//@ APB1 PCLK1 42MHz / 4 == 10.5MHz
		},
		.use_crc = false,
		.dma = {
				.irq = {
						// Note this is the stream ID that triggers interrupts (in this case RX)
						.flags = (DMA_IT_TCIF0 | DMA_IT_TEIF0 | DMA_IT_HTIF0),
						.init = {
								.NVIC_IRQChannel = DMA1_Stream0_IRQn,
								.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
								.NVIC_IRQChannelSubPriority = 0,
								.NVIC_IRQChannelCmd = ENABLE,
						},
				},

				.rx = {
						.channel = DMA1_Stream0,
						.init = {
								.DMA_Channel            = DMA_Channel_0,
								.DMA_PeripheralBaseAddr = (uint32_t) & (SPI3->DR),
								.DMA_DIR                = DMA_DIR_PeripheralToMemory,
								.DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
								.DMA_MemoryInc          = DMA_MemoryInc_Enable,
								.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
								.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
								.DMA_Mode               = DMA_Mode_Normal,
								.DMA_Priority           = DMA_Priority_Medium,
								//TODO: Enable FIFO
								.DMA_FIFOMode           = DMA_FIFOMode_Disable,
								.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full,
								.DMA_MemoryBurst        = DMA_MemoryBurst_Single,
								.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
						},
				},
				.tx = {
						.channel = DMA1_Stream7,
						.init = {
								.DMA_Channel            = DMA_Channel_0,
								.DMA_PeripheralBaseAddr = (uint32_t) & (SPI3->DR),
								.DMA_DIR                = DMA_DIR_MemoryToPeripheral,
								.DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
								.DMA_MemoryInc          = DMA_MemoryInc_Enable,
								.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
								.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
								.DMA_Mode               = DMA_Mode_Normal,
								.DMA_Priority           = DMA_Priority_Medium,
								.DMA_FIFOMode           = DMA_FIFOMode_Disable,
								.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full,
								.DMA_MemoryBurst        = DMA_MemoryBurst_Single,
								.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
						},
				},
		},
		.sclk = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin = GPIO_Pin_3,
						.GPIO_Speed = GPIO_Speed_50MHz,
						.GPIO_Mode = GPIO_Mode_AF,
						.GPIO_OType = GPIO_OType_PP,
						.GPIO_PuPd = GPIO_PuPd_NOPULL
				},
				.pin_source = GPIO_PinSource3,
		},
		.miso = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin = GPIO_Pin_4,
						.GPIO_Speed = GPIO_Speed_50MHz,
						.GPIO_Mode = GPIO_Mode_AF,
						.GPIO_OType = GPIO_OType_PP,
						.GPIO_PuPd = GPIO_PuPd_NOPULL
				},
				.pin_source = GPIO_PinSource4,
		},
		.mosi = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin = GPIO_Pin_5,
						.GPIO_Speed = GPIO_Speed_50MHz,
						.GPIO_Mode = GPIO_Mode_AF,
						.GPIO_OType = GPIO_OType_PP,
						.GPIO_PuPd = GPIO_PuPd_NOPULL
				},
				.pin_source = GPIO_PinSource5,
		},
		.slave_count = 4,

		.ssel = {
				/* flash chip */
				{
						.gpio = GPIOD,
						.init = {
								.GPIO_Pin = GPIO_Pin_4,
								.GPIO_Speed = GPIO_Speed_50MHz,
								.GPIO_Mode  = GPIO_Mode_OUT,
								.GPIO_OType = GPIO_OType_PP,
								.GPIO_PuPd = GPIO_PuPd_UP
						},
				},
				/* MPU-9250 */
				{
						.gpio = GPIOD,
						.init = {
								.GPIO_Pin = GPIO_Pin_7,
								.GPIO_Speed = GPIO_Speed_50MHz,
								.GPIO_Mode  = GPIO_Mode_OUT,
								.GPIO_OType = GPIO_OType_PP,
								.GPIO_PuPd = GPIO_PuPd_UP
						},
				},
				/* MS5611 */
				{
						.gpio = GPIOD,
						.init = {
								.GPIO_Pin = GPIO_Pin_3,
								.GPIO_Speed = GPIO_Speed_50MHz,
								.GPIO_Mode  = GPIO_Mode_OUT,
								.GPIO_OType = GPIO_OType_PP,
								.GPIO_PuPd = GPIO_PuPd_UP
						},
				},
				/* OSD */
				{
						.gpio = GPIOE,
						.init = {
								.GPIO_Pin = GPIO_Pin_1,
								.GPIO_Speed = GPIO_Speed_50MHz,
								.GPIO_Mode  = GPIO_Mode_OUT,
								.GPIO_OType = GPIO_OType_PP,
								.GPIO_PuPd = GPIO_PuPd_UP
						},
				},
		},
};

uint32_t pios_spi_internal_id;
void PIOS_SPI_internal_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_SPI_IRQ_Handler(pios_spi_internal_id);
}



#endif	/* PIOS_INCLUDE_SPI */


#if defined(PIOS_INCLUDE_I2C)

#include <pios_i2c_priv.h>

/*
 * I2C Adapters
 */


void PIOS_I2C_external_ev_irq_handler(void);
void PIOS_I2C_external_er_irq_handler(void);
void I2C1_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_ev_irq_handler")));
void I2C1_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_er_irq_handler")));

static const struct pios_i2c_adapter_cfg pios_i2c_external_adapter_cfg = {
		.regs = I2C1,
		.remap = GPIO_AF_I2C1,
		.init = {
				.I2C_Mode                = I2C_Mode_I2C,
				.I2C_OwnAddress1         = 0,
				.I2C_Ack                 = I2C_Ack_Enable,
				.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
				.I2C_DutyCycle           = I2C_DutyCycle_2,
				.I2C_ClockSpeed          = 400000,	/* bits/s */
		},
		.transfer_timeout_ms = 50,
		.scl = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin = GPIO_Pin_8,
						.GPIO_Mode  = GPIO_Mode_AF,
						.GPIO_Speed = GPIO_Speed_50MHz,
						.GPIO_OType = GPIO_OType_OD,
						.GPIO_PuPd  = GPIO_PuPd_NOPULL,
				},
				.pin_source = GPIO_PinSource6,
		},
		.sda = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin = GPIO_Pin_9,
						.GPIO_Mode  = GPIO_Mode_AF,
						.GPIO_Speed = GPIO_Speed_50MHz,
						.GPIO_OType = GPIO_OType_OD,
						.GPIO_PuPd  = GPIO_PuPd_NOPULL,
				},
				.pin_source = GPIO_PinSource7,
		},
		.event = {
				.flags   = 0,		/* FIXME: check this */
				.init = {
						.NVIC_IRQChannel = I2C1_EV_IRQn,
						.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
						.NVIC_IRQChannelSubPriority = 0,
						.NVIC_IRQChannelCmd = ENABLE,
				},
		},
		.error = {
				.flags   = 0,		/* FIXME: check this */
				.init = {
						.NVIC_IRQChannel = I2C1_ER_IRQn,
						.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
						.NVIC_IRQChannelSubPriority = 0,
						.NVIC_IRQChannelCmd = ENABLE,
				},
		},
};

uint32_t pios_i2c_external_adapter_id;
void PIOS_I2C_external_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_external_adapter_id);
}

void PIOS_I2C_external_er_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_ER_IRQ_Handler(pios_i2c_external_adapter_id);
}

#endif /* PIOS_INCLUDE_I2C */


#if defined(PIOS_INCLUDE_FLASH)
#include "pios_flashfs_logfs_priv.h"

static const struct flashfs_logfs_cfg flashfs_settings_cfg = {
	.fs_magic      = 0x3bb141cf,
	.arena_size    = 0x00010000, /* 256 * slot size */
	.slot_size     = 0x00000100, /* 256 bytes */
};

static const struct flashfs_logfs_cfg flashfs_waypoints_cfg = {
	.fs_magic      = 0x9a365a64,
	.arena_size    = 0x00010000, /* 1024 * slot size */
	.slot_size     = 0x00000040, /* 64 bytes */
};

#if defined(PIOS_INCLUDE_FLASH_JEDEC)
#include "pios_flash_jedec_priv.h"

static const struct pios_flash_jedec_cfg flash_n25q_cfg = {
	.expect_manufacturer = 0x20, /* Micron flash */
	.expect_memorytype   = 0xba,
	.expect_capacity     = 0x16,
	.sector_erase        = 0x20,
};

static const struct pios_flash_jedec_cfg flash_s25fl032_cfg = {
	.expect_manufacturer = 0x01, /* Spansion flash */
	.expect_memorytype   = 0x02,
	.expect_capacity     = 0x15,
	.sector_erase        = 0x20,
};

#endif	/* PIOS_INCLUDE_FLASH_JEDEC */

#if defined(PIOS_INCLUDE_FLASH_INTERNAL)
#include "pios_flash_internal_priv.h"

static const struct pios_flash_internal_cfg flash_internal_cfg = {
};
#endif	/* PIOS_INCLUDE_FLASH_INTERNAL */

#include "pios_flash_priv.h"

#if defined(PIOS_INCLUDE_FLASH_INTERNAL)
static const struct pios_flash_sector_range stm32f4_sectors[] = {
	{
		.base_sector = 0,
		.last_sector = 3,
		.sector_size = FLASH_SECTOR_16KB,
	},
	{
		.base_sector = 4,
		.last_sector = 4,
		.sector_size = FLASH_SECTOR_64KB,
	},
	{
		.base_sector = 5,
		.last_sector = 11,
		.sector_size = FLASH_SECTOR_128KB,
	},

};

uintptr_t pios_internal_flash_id;
static const struct pios_flash_chip pios_flash_chip_internal = {
	.driver        = &pios_internal_flash_driver,
	.chip_id       = &pios_internal_flash_id,
	.page_size     = 16, /* 128-bit rows */
	.sector_blocks = stm32f4_sectors,
	.num_blocks    = NELEMENTS(stm32f4_sectors),
};
#endif	/* PIOS_INCLUDE_FLASH_INTERNAL */

#if defined(PIOS_INCLUDE_FLASH_JEDEC)
static const struct pios_flash_sector_range n25q_sectors[] = {
	{
		.base_sector = 0,
		.last_sector = 1023,
		.sector_size = FLASH_SECTOR_4KB,
	},
};

uintptr_t pios_external_flash_id;
static const struct pios_flash_chip pios_flash_chip_external = {
	.driver        = &pios_jedec_flash_driver,
	.chip_id       = &pios_external_flash_id,
	.page_size     = 256,
	.sector_blocks = n25q_sectors,
	.num_blocks    = NELEMENTS(n25q_sectors),
};
#endif /* PIOS_INCLUDE_FLASH_JEDEC */

static const struct pios_flash_partition pios_flash_partition_table[] = {
#if defined(PIOS_INCLUDE_FLASH_INTERNAL)
	{
		.label        = FLASH_PARTITION_LABEL_BL,
		.chip_desc    = &pios_flash_chip_internal,
		.first_sector = 0,
		.last_sector  = 1,
		.chip_offset  = 0,
		.size         = (1 - 0 + 1) * FLASH_SECTOR_16KB,
	},

	/* NOTE: sectors 2-4 of the internal flash are currently unallocated */

	{
		.label        = FLASH_PARTITION_LABEL_FW,
		.chip_desc    = &pios_flash_chip_internal,
		.first_sector = 5,
		.last_sector  = 6,
		.chip_offset  = (4 * FLASH_SECTOR_16KB) + (1 * FLASH_SECTOR_64KB),
		.size         = (6 - 5 + 1) * FLASH_SECTOR_128KB,
	},

	/* NOTE: sectors 7-11 of the internal flash are currently unallocated */

#endif /* PIOS_INCLUDE_FLASH_INTERNAL */

#if defined(PIOS_INCLUDE_FLASH_JEDEC)
	{
		.label        = FLASH_PARTITION_LABEL_SETTINGS,
		.chip_desc    = &pios_flash_chip_external,
		.first_sector = 0,
		.last_sector  = 511,
		.chip_offset  = 0,
		.size         = (511 - 0 + 1) * FLASH_SECTOR_4KB,
	},

	{
		.label        = FLASH_PARTITION_LABEL_WAYPOINTS,
		.chip_desc    = &pios_flash_chip_external,
		.first_sector = 512,
		.last_sector  = 1023,
		.chip_offset  = (512 * FLASH_SECTOR_4KB),
		.size         = (1023 - 512 + 1) * FLASH_SECTOR_4KB,
	},
#endif	/* PIOS_INCLUDE_FLASH_JEDEC */
};

const struct pios_flash_partition * PIOS_BOARD_HW_DEFS_GetPartitionTable (uint32_t board_revision, uint32_t * num_partitions)
{
	PIOS_Assert(num_partitions);

	*num_partitions = NELEMENTS(pios_flash_partition_table);
	return pios_flash_partition_table;
}

#endif	/* PIOS_INCLUDE_FLASH */

#if defined(PIOS_INCLUDE_USART)

#include "pios_usart_priv.h"

#if defined(PIOS_INCLUDE_DSM)
/*
 * Spektrum/JR DSM USART
 */
#include <pios_dsm_priv.h>

static const struct pios_dsm_cfg pios_uart_xbee_dsm_aux_cfg = {
	.bind = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_OUT,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL
		},
	},
};

static const struct pios_dsm_cfg pios_uart_gps_dsm_aux_cfg = {
	.bind = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_OUT,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL
		},
	},
};

static const struct pios_dsm_cfg pios_uart_extension_dsm_aux_cfg = {
	.bind = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_7,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_OUT,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL
		},
	},
};



#endif	/* PIOS_INCLUDE_DSM */

#if defined(PIOS_INCLUDE_HSUM)
/*
 * Graupner HoTT SUMD/SUMH USART
 */
#include <pios_hsum_priv.h>

#endif	/* PIOS_INCLUDE_HSUM */

#if (defined(PIOS_INCLUDE_DSM) || defined(PIOS_INCLUDE_HSUM))
/*
 * Spektrum/JR DSM or Graupner HoTT SUMD/SUMH USART
 */

static const struct pios_usart_cfg pios_uart_xbee_dsm_hsum_cfg = {
	.regs = USART3,
	.remap = GPIO_AF_USART3,
	.init = {
		.USART_BaudRate = 115200,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART3_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource9,
	},
};

static const struct pios_usart_cfg pios_uart_gps_dsm_hsum_cfg = {
	.regs = USART2,
	.remap = GPIO_AF_USART2,
	.init = {
		.USART_BaudRate = 115200,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART2_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource6,
	},
};

static const struct pios_usart_cfg pios_uart_extension_dsm_hsum_cfg = {
	.regs = USART1,
	.remap = GPIO_AF_USART1,
	.init = {
		.USART_BaudRate = 115200,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART1_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_7,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource7,
	},
};


#endif	/* PIOS_INCLUDE_DSM || PIOS_INCLUDE_HSUM */

#if defined(PIOS_INCLUDE_SBUS)
/*
 * S.Bus USART
 */
#include <pios_sbus_priv.h>

static const struct pios_usart_cfg pios_uart_rcvr_sbus_cfg = {
	.regs = UART4,
	.remap = GPIO_AF_UART4,
	.init = {
		.USART_BaudRate            = 100000,
		.USART_WordLength          = USART_WordLength_8b,
		.USART_Parity              = USART_Parity_Even,
		.USART_StopBits            = USART_StopBits_2,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode                = USART_Mode_Rx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = UART4_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		  },
	},
	.rx = {
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_1,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource1,
	},
};

static const struct pios_sbus_cfg pios_uart_rcvr_sbus_aux_cfg = {
	/* Inverter configuration */
	/* configurable inverter not present on draco */
	.gpio_inv_enable = Bit_SET,
};

#endif	/* PIOS_INCLUDE_SBUS */

static const struct pios_usart_cfg pios_uart_xbee_cfg = {
	.regs = USART3,
	.remap = GPIO_AF_USART3,
	.init = {
		.USART_BaudRate = 57600,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART3_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource9,
	},
	.tx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_8,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource8,
	},
};

static const struct pios_usart_cfg pios_uart_gps_cfg = {
	.regs = USART2,
	.remap = GPIO_AF_USART2,
	.init = {
		.USART_BaudRate = 57600,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART2_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource6,
	},
	.tx = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin   = GPIO_Pin_5,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource5,
	},
};

static const struct pios_usart_cfg pios_uart_extension_cfg = {
	.regs = USART1,
	.remap = GPIO_AF_USART1,
	.init = {
		.USART_BaudRate = 57600,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART1_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_7,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource7,
	},
	.tx = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource6,
	},
};

static const struct pios_usart_cfg pios_uart_onewire_sport_cfg = {
	.regs = USART6,
	.remap = GPIO_AF_USART6,
	.init = {
		.USART_BaudRate = 57600,
		.USART_WordLength = USART_WordLength_8b,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = USART6_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.rx = {
		.gpio = GPIOC,
		.init = {
			.GPIO_Pin   = GPIO_Pin_7,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource7,
	},
	.tx = {
		.gpio = GPIOC,
		.init = {
			.GPIO_Pin   = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd  = GPIO_PuPd_UP
		},
		.pin_source = GPIO_PinSource6,
	},
};


#endif  /* PIOS_INCLUDE_USART */

#if defined(PIOS_INCLUDE_COM)

#include "pios_com_priv.h"

#endif	/* PIOS_INCLUDE_COM */

#if defined(PIOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <pios_rtc_priv.h>

void PIOS_RTC_IRQ_Handler (void);
void RTC_WKUP_IRQHandler() __attribute__ ((alias ("PIOS_RTC_IRQ_Handler")));
static const struct pios_rtc_cfg pios_rtc_main_cfg = {

	.clksrc = RCC_RTCCLKSource_HSE_Div8, // Divide 8 Mhz crystal down to 1
	.prescaler = 100, // Every 100 cycles gives 625 Hz
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = RTC_WKUP_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

void PIOS_RTC_IRQ_Handler (void)
{
	PIOS_RTC_irq_handler ();
}

#endif


#include "pios_tim_priv.h"

/* Timers used for inputs and outputs (5, 9, 3, 1, 4)
 * We use TIM_Period 0xfffff for each timer,
 * because it's desired when timer is used for inputs
 * and will be changed by Actuator module to desired
 * refresh rate when used for PWM outputs
 *
 * */

static const TIM_TimeBaseInitTypeDef tim_3_4_5_time_base = {
	.TIM_Prescaler = (PIOS_PERIPHERAL_APB1_CLOCK / 1000000) - 1,
	.TIM_ClockDivision = TIM_CKD_DIV1,
	.TIM_CounterMode = TIM_CounterMode_Up,
	.TIM_Period = 0xFFFF,
	.TIM_RepetitionCounter = 0x0000,
};

static const TIM_TimeBaseInitTypeDef tim_1_9_time_base = {
	.TIM_Prescaler = (PIOS_PERIPHERAL_APB2_CLOCK / 1000000) - 1,
	.TIM_ClockDivision = TIM_CKD_DIV1,
	.TIM_CounterMode = TIM_CounterMode_Up,
	.TIM_Period = 0xFFFF,
	.TIM_RepetitionCounter = 0x0000,
};


static const struct pios_tim_clock_cfg tim_5_cfg = {
	.timer = TIM5,
	.time_base_init = &tim_3_4_5_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM5_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_1_cfg = {
	.timer = TIM1,
	.time_base_init = &tim_1_9_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM1_CC_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.irq2 = {
		.init = {
			.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_9_cfg = {
	.timer = TIM9,
	.time_base_init = &tim_1_9_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM1_BRK_TIM9_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_3_cfg = {
	.timer = TIM3,
	.time_base_init = &tim_3_4_5_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM3_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_4_cfg = {
	.timer = TIM4,
	.time_base_init = &tim_3_4_5_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM4_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};


/**
 * Pios servo configuration structures
 */

/*
 * 	OUTPUTS
	1:  TIM9_CH1 (PA2)
	2:  TIM9_CH2 (PA3)
	3:  TIM3_CH1 (PA6)
	4:  TIM3_CH2 (PA7)
	5:  TIM3_CH3 (PB0)
	6:  TIM3_CH4 (PB1)
	7:  TIM1_CH1 (PE9)   -- may be used as input in alternative OutputPort config
	8:  TIM1_CH2 (PE11)  -- may be used as input in alternative OutputPort config
	9:  TIM1_CH3 (PE13)  -- may be used as input in alternative OutputPort config
	10: TIM_1_CH4 (PE14) -- may be used as input in alternative OutputPort config
	11: TIM4_CH2 (PD13)  -- may be used as input in alternative OutputPort config
	12: TIM4_CH1 (PD12)  -- may be used as input in alternative OutputPort config
 */
static const struct pios_tim_channel pios_tim_servoport_all_pins[] = {
	{
		.timer = TIM9,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM9,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin = GPIO_Pin_2,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource2,
		},
	},
	{
		.timer = TIM9,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM9,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin = GPIO_Pin_3,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource3,
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM3,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin = GPIO_Pin_6,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource6,
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM3,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin = GPIO_Pin_7,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource7,
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_3,
		.remap = GPIO_AF_TIM3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin = GPIO_Pin_0,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource0,
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_4,
		.remap = GPIO_AF_TIM3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin = GPIO_Pin_1,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource1,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_9,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource9,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_11,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource11,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_3,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_13,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource13,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_4,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_14,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource14,
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM4,
		.pin = {
			.gpio = GPIOD,
			.init = {
				.GPIO_Pin = GPIO_Pin_13,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource13,
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM4,
		.pin = {
			.gpio = GPIOD,
			.init = {
				.GPIO_Pin = GPIO_Pin_12,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource12,
		},
	},
};




#if defined(PIOS_INCLUDE_SERVO) && defined(PIOS_INCLUDE_TIM)
/*
 * Servo outputs
 */
#include <pios_servo_priv.h>

const struct pios_servo_cfg pios_servo_12_out_cfg = {
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.channels = pios_tim_servoport_all_pins,
	.num_channels = NELEMENTS(pios_tim_servoport_all_pins),
};

const struct pios_servo_cfg pios_servo_6_out_6_in_cfg = {
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.channels = pios_tim_servoport_all_pins,
	.num_channels = NELEMENTS(pios_tim_servoport_all_pins) - 6, /* last 6 pins used as pwm in */
};
#endif	/* PIOS_INCLUDE_SERVO && PIOS_INCLUDE_TIM */

/*
 * 	INPUTS
	1: TIM1_CH1 (PE9)
	2: TIM1_CH2 (PE11)
	3: TIM1_CH3 (PE13)
	4: TIM_1_CH4 (PE14)
	5: TIM4_CH2 (PD13)
	6: TIM4_CH1 (PD12)
 */
static const struct pios_tim_channel pios_tim_rcvrport_all_channels[] = {
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_9,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource9,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_11,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource11,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_3,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_13,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource13,
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_4,
		.remap = GPIO_AF_TIM1,
		.pin = {
			.gpio = GPIOE,
			.init = {
				.GPIO_Pin = GPIO_Pin_14,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource14,
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM4,
		.pin = {
			.gpio = GPIOD,
			.init = {
				.GPIO_Pin = GPIO_Pin_13,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource13,
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_1,
		.remap = GPIO_AF_TIM4,
		.pin = {
			.gpio = GPIOD,
			.init = {
				.GPIO_Pin = GPIO_Pin_12,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource12,
		},
	},
};

/*
 * PWM Inputs
 */
#if defined(PIOS_INCLUDE_PWM) || defined(PIOS_INCLUDE_PPM)
#include <pios_pwm_priv.h>

const struct pios_pwm_cfg pios_pwm_cfg = {
	.tim_ic_init = {
		.TIM_ICPolarity = TIM_ICPolarity_Rising,
		.TIM_ICSelection = TIM_ICSelection_DirectTI,
		.TIM_ICPrescaler = TIM_ICPSC_DIV1,
		.TIM_ICFilter = 0x0,
	},
	.channels = pios_tim_rcvrport_all_channels,
	.num_channels = NELEMENTS(pios_tim_rcvrport_all_channels),
};
#endif

/*
 * PPM Input
 */
#if defined(PIOS_INCLUDE_PPM)
#include <pios_ppm_priv.h>

static const struct pios_tim_channel pios_tim_ppm_channels[] = {
	{
		.timer = TIM5,
		.timer_chan = TIM_Channel_2,
		.remap = GPIO_AF_TIM5,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin = GPIO_Pin_1,
				.GPIO_Speed = GPIO_Speed_2MHz,
				.GPIO_Mode  = GPIO_Mode_AF,
				.GPIO_OType = GPIO_OType_PP,
				.GPIO_PuPd  = GPIO_PuPd_UP
			},
			.pin_source = GPIO_PinSource1,
		},
	},
};

static const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_ic_init = {
		.TIM_ICPolarity = TIM_ICPolarity_Falling,   /* input is inverted */
		.TIM_ICSelection = TIM_ICSelection_DirectTI,
		.TIM_ICPrescaler = TIM_ICPSC_DIV1,
		.TIM_ICFilter = 0x0,
		.TIM_Channel = TIM_Channel_3,
	},
	.channels = pios_tim_ppm_channels,
	.num_channels = NELEMENTS(pios_tim_ppm_channels),
};

#endif //PPM


#if defined(PIOS_INCLUDE_GCSRCVR)
#include "pios_gcsrcvr_priv.h"
#endif	/* PIOS_INCLUDE_GCSRCVR */


#if defined(PIOS_INCLUDE_RCVR)
#include "pios_rcvr_priv.h"
#endif /* PIOS_INCLUDE_RCVR */


#if defined(PIOS_INCLUDE_USB)
#include "pios_usb_priv.h"

static const struct pios_usb_cfg pios_usb_main_cfg = {
	.irq = {
		.init    = {
			.NVIC_IRQChannel                   = OTG_FS_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.vsense = {
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Speed = GPIO_Speed_25MHz,
			.GPIO_Mode  = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL,
		},
	}
};

const struct pios_usb_cfg * PIOS_BOARD_HW_DEFS_GetUsbCfg (uint32_t board_revision)
{
	return &pios_usb_main_cfg;
}

#include "pios_usb_board_data_priv.h"
#include "pios_usb_desc_hid_cdc_priv.h"
#include "pios_usb_desc_hid_only_priv.h"
#include "pios_usbhook.h"

#endif	/* PIOS_INCLUDE_USB */

#if defined(PIOS_INCLUDE_COM_MSG)

#include <pios_com_msg_priv.h>

#endif /* PIOS_INCLUDE_COM_MSG */

#if defined(PIOS_INCLUDE_USB_HID) && !defined(PIOS_INCLUDE_USB_CDC)
#include <pios_usb_hid_priv.h>

const struct pios_usb_hid_cfg pios_usb_hid_cfg = {
	.data_if = 0,
	.data_rx_ep = 1,
	.data_tx_ep = 1,
};
#endif /* PIOS_INCLUDE_USB_HID && !PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID) && defined(PIOS_INCLUDE_USB_CDC)
#include <pios_usb_cdc_priv.h>

const struct pios_usb_cdc_cfg pios_usb_cdc_cfg = {
	.ctrl_if = 0,
	.ctrl_tx_ep = 2,

	.data_if = 1,
	.data_rx_ep = 3,
	.data_tx_ep = 3,
};

#include <pios_usb_hid_priv.h>

const struct pios_usb_hid_cfg pios_usb_hid_cfg = {
	.data_if = 2,
	.data_rx_ep = 1,
	.data_tx_ep = 1,
};
#endif	/* PIOS_INCLUDE_USB_HID && PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_ADC)
#include "pios_adc_priv.h"
#include "pios_internal_adc_priv.h"

void PIOS_ADC_DMA_irq_handler(void);
void DMA2_Stream4_IRQHandler(void) __attribute__((alias("PIOS_ADC_DMA_irq_handler")));
struct pios_internal_adc_cfg pios_adc_cfg = {
	.adc_dev_master = ADC1,
	.dma = {
		.irq = {
			.flags = (DMA_FLAG_TCIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4),
			.init = {
				.NVIC_IRQChannel = DMA2_Stream4_IRQn,
				.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW,
				.NVIC_IRQChannelSubPriority = 0,
				.NVIC_IRQChannelCmd = ENABLE,
			},
		},
		.rx = {
			.channel = DMA2_Stream4,
			.init = {
				.DMA_Channel = DMA_Channel_0,
				.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR
			},
		}
	},
	.half_flag = DMA_IT_HTIF4,
	.full_flag = DMA_IT_TCIF4,
};

void PIOS_ADC_DMA_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_INTERNAL_ADC_DMA_Handler();
}

#endif /* PIOS_INCLUDE_ADC */

/**
 * @}
 * @}
 */
