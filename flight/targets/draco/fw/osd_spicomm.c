/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup Draco Draco OSD communication layer
 * @{
 *
 * @file       osd_spicomm.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @brief      Board specific options that modify PiOS capabilities
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
#include "pios.h"
#ifdef DRACO_INCLUDE_OSD_SUPPORT

#include "osd_spicomm.h"
#include "pios_semaphore.h"
#include "pios_crc.h"
#include <string.h>

#define FRAME_SOFD                              0x00

#define FRAME_TYPE_DATA                         0x08
#define FRAME_TYPE_REQUEST_0                    0x02
#define FRAME_TYPE_REQUEST_1                    0x03
#define FRAME_TYPE_ANSWER_0                     0x04
#define FRAME_TYPE_ANSWER_1                     0x05
#define FRAME_TYPE_SEQNO                        0x01
#define FRAME_TYPE_SYNC                         0xFF

#define FLAGS_RX_ANSWER_READY                   0x01

enum rxstate {
	RX_STATE_WAIT_START =                     0,
	RX_STATE_WAIT_FRAME_TYPE =                1,
	RX_STATE_WAIT_LENGTH =                    2,
	RX_STATE_WAIT_DATA =                      3,
	RX_STATE_WAIT_CRC =                       4,
};

enum draco_osd_comm_dev_magic {
	DRACO_OSD_COMM_DEV_MAGIC = 0x5acc46f8,
};

struct draco_osd_comm_dev {
	const struct osd_comm_cfg *cfg;
	uint32_t spi;
	uint32_t slaveNum;
	uint8_t *rxblock;
	uint8_t *txblock;
	struct pios_semaphore *irqsem;
	draco_osd_comm_data_callback datacb;
	uint8_t txBuffer[264];
	enum rxstate rxState;
	uint8_t rxFlags;
	uint8_t rxCrc;
	uint8_t rxCobsBlockIdx;
	uint8_t rxCobsBlockSize;
	uint8_t rxBuffIdx;
	uint8_t rxLen;
	uint8_t *rxBuff;
	uint8_t rxDataPayload[256];
	uint8_t rxAnswerPayload[256];
	uint8_t rxAnswerNo;
	uint8_t rxAnswerLen;
	uint8_t lastRequestNo;
	bool irqFallback;
	uint32_t magic;
};

static struct draco_osd_comm_dev *dev;

/**
 * Test for driver validity
 * @return whether driver is valid or not
 */
static bool validate(void)
{
	if ((dev) && (dev->magic == DRACO_OSD_COMM_DEV_MAGIC))
		return true;

	return false;
}

/**
 * Read IRQ GPIO state of OSD MCU
 * @return whether IRQ GPIO is in active state
 */
static bool readIrqState(void)
{
	return GPIO_ReadInputDataBit(dev->cfg->exti_cfg->pin.gpio, dev->cfg->exti_cfg->pin.init.GPIO_Pin) == 0;
}

/**
 * Process input bytes from OSD MCU and compose
 * incoming frames
 * @param[in] b incoming byte
 */
static void rxbyte(uint8_t b)
{
	// on the fly COBS unstuffing
	if (b) {
		if (dev->rxCobsBlockIdx >= (dev->rxCobsBlockSize - 1)) {
			if (dev->rxCobsBlockSize < 0xff) {
				dev->rxCobsBlockSize = b;
				dev->rxCobsBlockIdx = 0;
				b = 0;
			} else {
				dev->rxCobsBlockSize = b;
				dev->rxCobsBlockIdx = 0;
				return;
			}
		} else {
			dev->rxCobsBlockIdx++;
		}
	} else {
		dev->rxState = RX_STATE_WAIT_START;
	}

	if (dev->rxState != RX_STATE_WAIT_CRC)
		dev->rxCrc = PIOS_CRC_updateByte(dev->rxCrc, b);

	switch (dev->rxState) {
	case RX_STATE_WAIT_DATA:
		if (dev->rxBuff)
			dev->rxBuff[dev->rxBuffIdx++] = b;

		if (dev->rxBuffIdx >= dev->rxLen)
			dev->rxState = RX_STATE_WAIT_CRC;
		break;

	case RX_STATE_WAIT_START:
		if (b == FRAME_SOFD) {
			dev->rxState = RX_STATE_WAIT_FRAME_TYPE;
			dev->rxCrc = 0;
			dev->rxCobsBlockIdx = 255;
			dev->rxCobsBlockSize = 255;
		}
		break;

	case RX_STATE_WAIT_FRAME_TYPE:
		dev->rxBuff = 0;
		dev->rxBuffIdx = 0;
		if (b == FRAME_TYPE_DATA) {
			dev->rxBuff = dev->rxDataPayload;
		} else if ((b == FRAME_TYPE_ANSWER_0) || (b == FRAME_TYPE_ANSWER_1)) {
			dev->rxBuff = dev->rxAnswerPayload;
			dev->rxAnswerNo = b & FRAME_TYPE_SEQNO;
		}

		dev->rxState = RX_STATE_WAIT_LENGTH;
		break;

	case RX_STATE_WAIT_LENGTH:
		dev->rxLen = b;
		dev->rxState = b ? RX_STATE_WAIT_DATA : RX_STATE_WAIT_CRC;
		break;

	case RX_STATE_WAIT_CRC:
		if (b == dev->rxCrc) {
			if (dev->rxBuff == dev->rxDataPayload) {
				if (dev->datacb)
					dev->datacb(dev->rxBuff, dev->rxLen);
			} else if (dev->rxBuff == dev->rxAnswerPayload) {
				dev->rxFlags |= FLAGS_RX_ANSWER_READY;
				dev->rxAnswerLen = dev->rxLen;
			}
		}

		dev->rxState = RX_STATE_WAIT_START;
		break;
	}
}

/**
 * Perform transfer(s) on SPI bus
 * I/Os are divided into transfers of max. size specified by granularity
 * configuration field. This helps to reduce jitter on SPI bus.
 * Function returns when all bytes are trasnmitted to OSD MCU
 * AND OSD MCU has nothing to transmitt (IRQ idle)
 * @param[in] tx pointer to data to be transmitted
 * @param[in] txlen number of bytes to transmitt
 */
static void transceive(const uint8_t *tx, uint16_t txlen)
{
	if (!validate())
		return;

	uint16_t transMax = dev->cfg->transfer_granularity;
	uint16_t txpos = 0;
	bool irqState;
	uint16_t irqFallbackCnt = 0;

	while (((irqState = (readIrqState() && !dev->irqFallback)) == true)
			|| (txpos < txlen)) {
		uint16_t transSize = ((txlen - txpos) > transMax) ? transMax : (txlen - txpos);
		if (irqState) {
			transSize = transMax;
			if (++irqFallbackCnt > 50)
				dev->irqFallback = true;
		}

		memset(dev->txblock, 0x00, transSize);
		if (txlen && txpos < txlen) {
			memcpy(dev->txblock, &tx[txpos], transSize);
			txpos += transSize;
		}

		PIOS_SPI_ClaimBus(dev->spi);
		PIOS_SPI_RC_PinSet(dev->spi, dev->slaveNum, 0);
		PIOS_SPI_TransferBlock(dev->spi, dev->txblock, dev->rxblock, transSize, 0);
		PIOS_SPI_RC_PinSet(dev->spi, dev->slaveNum, 1);
		PIOS_SPI_ReleaseBus(dev->spi);

		int i;
		for (i = 0; i < transSize; i++)
			rxbyte(dev->rxblock[i]);
	}
}

/**
 * Trasnmitt generic frame of given type
 * @param[in] payload pointer to payload portion of frame
 * @param[in] len length of payload
 * @param[in] frameType frame type byte
 */
static void txGenericFrame(const uint8_t *payload, uint8_t len, uint8_t frameType)
{
	// calculate CRC
	uint8_t crc = 0;
	crc = PIOS_CRC_updateByte(crc, frameType);
	crc = PIOS_CRC_updateByte(crc, len);
	int i;
	for (i = 0; i < len; i++)
		crc = PIOS_CRC_updateByte(crc, payload[i]);

	dev->txBuffer[0] = 0;

	// COBS stuffing
	int cobsStartIdx = 0;
	int blockIdx = 1;
	for (i = 0; i < (len + 2 + 1); i++) {
		uint8_t b = (i == 0) ? frameType : (i == 1) ? len : (i == (len + 2)) ? crc : payload[i - 2];
		if (b == 0) {
			dev->txBuffer[cobsStartIdx + 1] = blockIdx;
			cobsStartIdx += blockIdx;
			blockIdx = 1;
		} else {
			dev->txBuffer[cobsStartIdx + blockIdx + 1] = b;
			blockIdx++;
		}

		if (blockIdx == 255) {
			dev->txBuffer[cobsStartIdx + 1] = blockIdx;
			cobsStartIdx += blockIdx;
			blockIdx = 1;
		}
	}
	dev->txBuffer[cobsStartIdx + 1] = blockIdx;

	// SPI transmitt
	transceive(dev->txBuffer, cobsStartIdx + blockIdx + 1);
}

/**
 * Send synchronization frame
 */
void sendSync(void)
{
	txGenericFrame(0, 0, FRAME_TYPE_SYNC);
}

/**
 * OSD MCU IRQ handler
 * it will give semaphore to wake-up draco_osd_comm_wait()
 * @return whether RTOS should yield at interrupt return
 */
bool draco_osd_comm_irq_handler(void)
{
	if (!validate())
		return false;

	dev->irqFallback = false;
	bool yield = false;
	PIOS_Semaphore_Give_FromISR(dev->irqsem, &yield);
	return yield;
}

/**
 * Initialize OSD communication driver
 * @param[in] spi_id SPI bus
 * @param[in] slave_num slave select
 * @param[in] cfg pointer to configuration structure
 * @return 0 when initialization was successful
 */
int32_t draco_osd_comm_init(uint32_t spi_id, uint32_t slave_num, const struct osd_comm_cfg *cfg)
{
	dev = PIOS_malloc(sizeof(struct draco_osd_comm_dev));
	if (!dev)
		return -1;

	dev->cfg = cfg;
	dev->rxblock = PIOS_malloc(cfg->transfer_granularity);
	if (!dev->rxblock) {
		vPortFree(dev);
		return -1;
	}

	dev->txblock = PIOS_malloc(cfg->transfer_granularity);
	if (!dev->txblock) {
		vPortFree(dev->rxblock);
		vPortFree(dev);
		return -1;
	}

	dev->spi = spi_id;
	dev->slaveNum = slave_num;
	dev->datacb = 0;
	dev->rxFlags = 0;
	dev->rxState = RX_STATE_WAIT_START;
	dev->lastRequestNo = 1;
	dev->irqFallback = false;
	dev->magic = DRACO_OSD_COMM_DEV_MAGIC;

	dev->irqsem = PIOS_Semaphore_Create();
	if (!dev->irqsem) {
		vPortFree(dev->rxblock);
		vPortFree(dev->txblock);
		vPortFree(dev);
		return -2;
	}

	PIOS_Semaphore_Take(dev->irqsem, 0);

	if (PIOS_EXTI_Init(cfg->exti_cfg) != 0) {
		vPortFree(dev->rxblock);
		vPortFree(dev->txblock);
		vPortFree(dev);
		return -2;
	}

	sendSync();

	return 0;
}

/**
 * Wait for data from OSD MCU for specified timeout
 * All registered callbacks are called from context
 * of this function
 * @param[in] timeout in miliseconds
 * @return false when timeout reached
 */
bool draco_osd_comm_wait(uint32_t timeout)
{
	if (!validate())
		return false;

	bool taken = PIOS_Semaphore_Take(dev->irqsem, timeout);
	if (taken)
		transceive(0, 0);

	return taken;
}

/**
 * Send DATA frame to OSD MCU
 * @param[in] data pointer to data to be transmitted
 * @param[in] datalen number of data bytes
 */
void draco_osd_comm_send_data(const uint8_t *data, uint8_t datalen)
{
	if (!validate())
		return;

	txGenericFrame(data, datalen, FRAME_TYPE_DATA);
}

/**
 * Send REQUEST frame to OSD MCU and wait for ANSWER
 * Request is automatically repeated upto four-times when no answer arrived
 * @param[in] req pointer to request data
 * @param[in] reqlen length of request data
 * @param[out] ans pointer to buffer where answer will be stored
 * @param[out] anslen length of received answer
 * @return 0 when valid answer received
 */
int32_t draco_osd_comm_send_request(const uint8_t *req, uint8_t reqlen, uint8_t *ans, uint8_t *anslen)
{
	if (!validate())
		return -1;

	dev->rxFlags &= ~FLAGS_RX_ANSWER_READY;
	dev->lastRequestNo = (dev->lastRequestNo) ? 0 : 1;
	txGenericFrame(req, reqlen, FRAME_TYPE_REQUEST_0 | dev->lastRequestNo);
	uint32_t timeout = PIOS_DELAY_GetuS();
	uint8_t retry = 0;
	while (((dev->rxFlags & FLAGS_RX_ANSWER_READY) == 0) || (dev->rxAnswerNo != dev->lastRequestNo)) {
		if (PIOS_DELAY_GetuSSince(timeout) > 100000) {
			retry++;
			if (retry > 3)
				sendSync();
			if (retry > 4)
				break;
			txGenericFrame(req, reqlen, FRAME_TYPE_REQUEST_0 | dev->lastRequestNo);
			timeout = PIOS_DELAY_GetuS();
		}

		draco_osd_comm_wait(100);
		transceive(0, 0);
	}

	if ((dev->rxFlags & FLAGS_RX_ANSWER_READY) && (dev->rxAnswerNo == dev->lastRequestNo)) {
		if (ans) {
			memcpy(ans, dev->rxAnswerPayload, dev->rxAnswerLen);
			*anslen = dev->rxAnswerLen;
		}
		return 0;
	}

	return -1;
}

/**
 * Register callbacks for events
 * @param[in] datacb callback called when DATA frame is received
 */
void draco_osd_comm_register_callback(draco_osd_comm_data_callback datacb)
{
	if (!validate())
		return;

	dev->datacb = datacb;
}
#endif
