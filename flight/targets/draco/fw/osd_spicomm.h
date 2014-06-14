/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup Draco Draco support files
 * @{
 *
 * @file       osd_spicomm.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
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

#ifndef OSD_SPICOMM_H_
#define OSD_SPICOMM_H_
#include "pios.h"

struct osd_comm_cfg {
	const struct pios_exti_cfg *exti_cfg;
	uint16_t transfer_granularity;
};

typedef void (*draco_osd_comm_data_callback)(const uint8_t *data, uint8_t datalen);

extern int32_t draco_osd_comm_init(uint32_t spi_id, uint32_t slave_num, const struct osd_comm_cfg *cfg);
extern int32_t draco_osd_comm_send_request(const uint8_t *req, uint8_t reqlen, uint8_t *ans, uint8_t *anslen);
extern void draco_osd_comm_send_data(const uint8_t *data, uint8_t datalen);
extern void draco_osd_comm_register_callback(draco_osd_comm_data_callback datacb);
extern bool draco_osd_comm_wait(uint32_t timeout);

extern bool draco_osd_comm_irq_handler(void);
#endif /* OSD_SPICOMM_H_ */
