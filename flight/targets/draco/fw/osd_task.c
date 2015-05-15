/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup Draco Draco OSD communication task
 * @{
 *
 * @file       osd_task.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @brief      Target specific task acting like UAVO<-->Draco OSD MCU bridge
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
#include <openpilot.h>
#include "pios_semaphore.h"
#include "physical_constants.h"
#include "misc_math.h"
#include "osd_task.h"
#include "osd_spicomm.h"
#include "attitudeactual.h"
#include "positionactual.h"
#include "gpsposition.h"
#include "velocityactual.h"
#include "flightbatterysettings.h"
#include "flightbatterystate.h"
#include "flightstatus.h"
#include "waypointactive.h"
#include "waypoint.h"
#include "homelocation.h"
#include "airspeedactual.h"
#include "flighttelemetrystats.h"
#include "actuatordesired.h"
#include "hwdraco.h"
#include "hwdracoosdfwupdate.h"
#include "pios_thread.h"

#if defined(PIOS_DRACO_OSD_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_DRACO_OSD_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1024
#endif
#define TASK_PRIORITY               PIOS_THREAD_PRIO_LOW
#define TASK_RATE_HZ 25

#define REQ_ID_VERSION              0
#define REQ_ID_HUD_ENABLE           1
#define REQ_ID_POWER_LIMITS         10
#define REQ_ID_FLIGHT_MODE          11
#define REQ_ID_SET_UNITS            12
#define REQ_ID_FORCE_TV_SYSTEM      20

#define REQ_ID_WRITE_START          128
#define REQ_ID_WRITE_CHUNK          129
#define REQ_ID_READ_START           130
#define REQ_ID_READ_CHUNK           131
#define REQ_ID_EXIT_BOOT            132
#define REQ_ID_ENTER_BOOT           133

#define DATA_ID_LED_CONTROL         0
#define DATA_ID_PFD                 1
#define DATA_ID_WAYPOINT_HOME       2
#define DATA_ID_WAYPOINT_NAVI       3
#define DATA_ID_GNSS                4
#define DATA_ID_POWER               5
#define DATA_ID_STOPWATCH           6

#define COMM_VERSION_MODE_FIRMWARE      1
#define COMM_VERSION_MODE_BOOTLOADER    0

#define COMM_RESULT_OK                  0
#define COMM_RESULT_ERROR               1

#define HUD_UNITS_METRIC            0
#define HUD_UNITS_IMPERIAL          1
#define HUD_LED_RED                 0
#define HUD_LED_GREEN               1
#define HUD_LED_BLUE                2
#define HUD_LED_ORANGE              3
#define HUD_LED_MODE_OFF            0
#define HUD_LED_MODE_ON             1
#define HUD_LED_MODE_BLINK_SLOW     2
#define HUD_LED_MODE_BLINK_FAST     3
#define HUD_LED_ARMED               HUD_LED_GREEN
#define HUD_LED_GPS                 HUD_LED_BLUE
#define HUD_LED_TELEMETRY           HUD_LED_ORANGE
#define HUD_LED_ALARM               HUD_LED_RED

#define HUD_TVSYSTEM_PAL            0
#define HUD_TVSYSTEM_NTSC           1

enum HudWaypointType {
	HUD_WAYPOINT_HOME,
	HUD_WAYPOINT_NAVI
};

struct DataPfd {
	int16_t speed;      // 10^-2 m/s
	int16_t vspeed;     // 10^-2 m/s
	int32_t altitude;   // 10^-2 m
	int16_t roll;       // 10^-1 degrees
	int16_t pitch;      // 10^-1 degrees
	int16_t heading;    // 10^-1 degrees
} __attribute__((packed));

struct DataWp {
	uint8_t show;
	uint32_t distance;      // 10^-2 m
	int16_t heading;        // 10^-1 degrees
} __attribute__((packed));

struct DataGnss {
	uint8_t fix;
	uint8_t satCount;
	uint16_t pdop;          // 10^-2
	int32_t lat;            // 10^-7 degrees
	int32_t lon;            // 10^-7 degrees
} __attribute__((packed));


struct DataPower {
	uint16_t voltage;       // 10^-2 V
	uint16_t current;       // 10^-2 A
	uint16_t mahs;          // 10^-3 A
} __attribute__ ((packed));

struct DataPowerLimits {
	uint16_t minVoltage;    // 10^-2 V
	uint16_t maxmahs;       // 10^-3 A
} __attribute__ ((packed));

struct DataStopwatch {
	uint8_t running;
	uint16_t maxSeconds;
} __attribute__((packed));

static struct pios_thread *dracoOsdTaskHandle;

static uint8_t *txPayload;
static uint8_t *answerPayload;
static char versionString[33];
static bool firstPass = true;
static bool bootloaderMode = false;

static AttitudeActualData attitudeActual;
static PositionActualData positionActual;
static GPSPositionData gpsPosition;
static AirspeedActualData airspeedActual;
static FlightBatteryStateData flightBatteryState;
static VelocityActualData velocityActual;
static FlightStatusData flightStatus;
static WaypointData waypointActual;
static FlightTelemetryStatsData telemetryStats;
static ActuatorDesiredData actuatorDesired;
static HWDracoOsdFwUpdateData fwUpdate;
static struct pios_semaphore *osdFwUpdateSem;

static void dracoOsdTask(void *parameters);

/**
 * Read version information from OSD MCU
 * This is used to test OSD MCU is alive
 * @param[out] versionString git describe of OSD firmware
 * @param[out] mode whether MCU is in firmware or bootloader mode
 * @param[in] versionLen maximum length of versionString
 * @return true when successful
 */
static bool getVersion(uint8_t *mode, char *versionString, uint8_t versionLen)
{
	if (versionString)
		versionString[0] = 0;
	txPayload[0] = REQ_ID_VERSION;
	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 1, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < (versionLen + 1)) && (ansLen > 2)) {
		if (versionString != 0) {
			versionString[ansLen - 1] = 0;
			memcpy(versionString, &answerPayload[1], ansLen - 1);
		}
		if (mode != 0)
			*mode = answerPayload[0];
		return true;
	}
	return false;
}

/**
 * Test GPS status
 * GPS must be fixed in 3D mode and home location set
 * @return 0 when gps not detected, 1 when GPS detected, 2 when GPS 3D fixed and home location set
 */
static uint8_t getGpsStatus(void)
{
	uint8_t status = 0;
	if (gpsPosition.Status == GPSPOSITION_STATUS_NOGPS)
		return status;

	status = 1;

	uint8_t locationSet = HOMELOCATION_SET_FALSE;
	HomeLocationSetGet(&locationSet);
	if ((locationSet == HOMELOCATION_SET_TRUE) &&
			((gpsPosition.Status == GPSPOSITION_STATUS_FIX3D) ||
					(gpsPosition.Status == GPSPOSITION_STATUS_DIFF3D)))
		status = 2;

	return status;
}

/**
 * Determine if important alarms are in critical or warning states
 * @return 0 if no alarm, 1 if important alarm(s) is in warning state, 2 if important alarm is in error state
 */
static uint8_t getAlarmStatus(void)
{
	// read alarms
	SystemAlarmsData alarms;
	SystemAlarmsGet(&alarms);
	uint8_t status = 0;

	// Check each alarm
	for (int i = 0; i < SYSTEMALARMS_ALARM_NUMELEM; i++)
	{
		if (alarms.Alarm[i] == SYSTEMALARMS_ALARM_WARNING &&
			i != SYSTEMALARMS_ALARM_GPS &&
			i != SYSTEMALARMS_ALARM_TELEMETRY)
		{
			status = 1;
		}

		if (alarms.Alarm[i] >= SYSTEMALARMS_ALARM_ERROR &&
			i != SYSTEMALARMS_ALARM_GPS &&
			i != SYSTEMALARMS_ALARM_TELEMETRY)
		{
			return 2;
		}
	}

	return status;
}

/**
 * Enable OSD HUD
 * controls whether something will be drawn on the screen or not
 * @param[in] en enable when true, disable when false
 * @return 0 when successful
 */
static int32_t hudEnable(bool en)
{
	txPayload[0] = REQ_ID_HUD_ENABLE;
	txPayload[1] = en ? 1 : 0;
	if (draco_osd_comm_send_request(txPayload, 2, 0, 0) != 0)
		return -1;

	return 0;
}

/**
 * Set HUD units (metric / imperial)
 * @param[in] units units to be displayed
 * @return 0 when successful
 */
static int32_t hudSetUnits(uint8_t units)
{
	txPayload[0] = REQ_ID_SET_UNITS;
	txPayload[1] = units;

	return draco_osd_comm_send_request(txPayload, 2, 0, 0);
}

/**
 * Force using of PAL or NTSC TV system
 * @param[in] PAL or NTSC
 * @return 0 when successful
 */
static int32_t hudForceTvSystem(uint8_t tvsys)
{
	txPayload[0] = REQ_ID_FORCE_TV_SYSTEM;
	txPayload[1] = tvsys;

	return draco_osd_comm_send_request(txPayload, 2, 0, 0);
}

/**
 * Set HUD battery limits where warning should indicated
 * @param[in] minVoltage minimum voltage
 * @param[in] maxMahs maximum consumed capacity
 * @return 0 when successful
 */
static int32_t hudSetBatteryLimits(uint16_t minVoltage, uint16_t maxMahs)
{
	struct DataPowerLimits limits;
	limits.maxmahs = maxMahs;
	limits.minVoltage = minVoltage;

	txPayload[0] = REQ_ID_POWER_LIMITS;
	memcpy(&txPayload[1], &limits, sizeof(struct DataPowerLimits));
	return draco_osd_comm_send_request(txPayload, sizeof(struct DataPowerLimits) + 1, 0, 0);
}

/**
 * Send primary flight display data
 * attitude, heading, altitude and speeds
 */
static void hudSendPfd(void)
{
	struct DataPfd dataPfd;
	dataPfd.altitude = (int16_t)(-positionActual.Down * 100.0f);
	dataPfd.roll = (int16_t)(attitudeActual.Roll * 10.0f);
	dataPfd.pitch = (int16_t)(attitudeActual.Pitch * 10.0f);
	dataPfd.heading = (int16_t)(attitudeActual.Yaw * 10.0f);
	float hspeed = sqrtf(powf(velocityActual.East, 2) + powf(velocityActual.North, 2));
	if (AirspeedActualHandle() != 0)
		dataPfd.speed = (int16_t)(airspeedActual.TrueAirspeed * 100.0f);
	else
		dataPfd.speed = (int16_t)(hspeed * 100.0f);
	dataPfd.vspeed = (int16_t)(-velocityActual.Down * 100.0f);

	txPayload[0] = DATA_ID_PFD;
	memcpy(&txPayload[1], &dataPfd, sizeof(struct DataPfd));
	draco_osd_comm_send_data(txPayload, sizeof(struct DataPfd) + 1);
}

/**
 * Send battery state
 * it includes voltage, current and consumed capacity
 */
static void hudSendBattery(void)
{
	struct DataPower power;
	power.current = (uint16_t)(flightBatteryState.Current * 100.0f);
	power.voltage = (uint16_t)(flightBatteryState.Voltage * 100.0f);
	power.mahs = (uint16_t)flightBatteryState.ConsumedEnergy;

	txPayload[0] = DATA_ID_POWER;
	memcpy(&txPayload[1], &power, sizeof(struct DataPower));
	draco_osd_comm_send_data(txPayload, sizeof(struct DataPower) + 1);
}

/**
 * Send GPS state and position
 */
static void hudSendGnss(void)
{
	struct DataGnss gnss;
	memset(&gnss, 0, sizeof(struct DataGnss));
	if (getGpsStatus() == 2)
		gnss.fix = 1;
	else
		gnss.fix = 0;

	gnss.lat = gpsPosition.Latitude;
	gnss.lon = gpsPosition.Longitude;
	gnss.pdop = (uint16_t)(gpsPosition.PDOP * 100.0f);
	gnss.satCount = (uint8_t)gpsPosition.Satellites;

	txPayload[0] = DATA_ID_GNSS;
	memcpy(&txPayload[1], &gnss, sizeof(struct DataGnss));
	draco_osd_comm_send_data(txPayload, sizeof(struct DataGnss) + 1);
}

/**
 * Get text for current flight mode
 */
static const char *getModeText(void)
{
	switch (flightStatus.FlightMode) {
	case FLIGHTSTATUS_FLIGHTMODE_ALTITUDEHOLD:
		return "ALTHOLD";
	case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
		return "AUTOTUNE";
	case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
		return "MANUAL";
	case FLIGHTSTATUS_FLIGHTMODE_ACRO:
		return "ACRO";
	case FLIGHTSTATUS_FLIGHTMODE_LEVELING:
		return "LEVEL";
	case FLIGHTSTATUS_FLIGHTMODE_VIRTUALBAR:
		return "VBAR";
	case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
		return "PLANNER";
	case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
		return "POSHOLD";
	case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
		return "RTH";
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
		return "STAB1";
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
		return "STAB2";
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
		return "STAB3";
	case FLIGHTSTATUS_FLIGHTMODE_TABLETCONTROL:
		return "TABLET";
	case FLIGHTSTATUS_FLIGHTMODE_MWRATE:
		return "MWRATE";
	case FLIGHTSTATUS_FLIGHTMODE_HORIZON:
		return "HORIZON";
	case FLIGHTSTATUS_FLIGHTMODE_AXISLOCK:
		return "AXISLK";
	default:
		return "UNKNOWN";
	}
}

/**
 * Send current flight mode text
 * text is prefixed by "-" when system is disarmed
 * @return 0 when successful
 */
static int32_t hudSendMode(void)
{
	char text[32];
	const char *modeText = getModeText();
	if (flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED) {
		text[0] = '-';
		strncpy(&text[1], modeText, 31);
	} else {
		strncpy(&text[0], modeText, 31);
	}

	txPayload[0] = REQ_ID_FLIGHT_MODE;

	int slen = strnlen(text, 31);
	strncpy((char *)&txPayload[1], text, 31);

	return draco_osd_comm_send_request(txPayload, slen + 2, 0, 0);
}

/**
 * Start or stop stopwatch depending on Armed status
 */
static void hudSetupStopwatch(void)
{
	struct DataStopwatch stopwatch;
	if ((flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED) || (actuatorDesired.Throttle <= 0))
		stopwatch.running = 0;
	else
		stopwatch.running = 1;

	// todo make this limit configurable
	stopwatch.maxSeconds = 0xffff;

	txPayload[0] = DATA_ID_STOPWATCH;

	memcpy(&txPayload[1], &stopwatch, sizeof(struct DataStopwatch));
	draco_osd_comm_send_data(txPayload, sizeof(struct DataStopwatch) + 1);
}

/**
 * Control LEDs connected to OSD MCU
 * @param[in] led select led
 * @param[in] mode select mode
 */
static void hudControlLed(uint8_t led, uint8_t mode)
{
	txPayload[0] = DATA_ID_LED_CONTROL;
	txPayload[1] = led;
	txPayload[2] = mode;

	draco_osd_comm_send_data(txPayload, 3);
}

/**
 * Send waypoint informations
 * @param[in] type waypoint type
 * @param[in] show show waypoint when true, disable waypoint when false
 * @param[in] distance distance to the waypoint
 * @param[in] heading to the waypoint
 */
static void hudSendWaypoint(enum HudWaypointType type, bool show, float distance, float heading)
{
	struct DataWp wp;
	wp.distance = (uint32_t)(distance * 100.0f);
	wp.heading = (int16_t)(heading * 10.0f);
	wp.show = show;

	txPayload[0] = (type == HUD_WAYPOINT_HOME) ? DATA_ID_WAYPOINT_HOME : DATA_ID_WAYPOINT_NAVI;

	memcpy(&txPayload[1], &wp, sizeof(struct DataWp));
	draco_osd_comm_send_data(txPayload, sizeof(struct DataWp) + 1);
}

/**
 * Send waypoints
 * @param[in] showNaviWp whether to show actual waypoint on the path or not
 */
static void hudSendWaypoints(bool showNaviWp)
{
	static bool lastShowNaviWp = false;
	float distance = sqrtf(powf(positionActual.East, 2) + powf(positionActual.North, 2));
	float heading = atan2f(positionActual.East, positionActual.North) * RAD2DEG;
	heading -= attitudeActual.Yaw - 180.0f;
	heading = circular_modulus_deg(heading);
	hudSendWaypoint(HUD_WAYPOINT_HOME, true, distance, heading);

	if (showNaviWp) {
		float northDiff = waypointActual.Position[WAYPOINT_POSITION_NORTH] - positionActual.North;
		float eastDiff = waypointActual.Position[WAYPOINT_POSITION_EAST] - positionActual.East;
		distance = sqrtf(powf(eastDiff, 2) + powf(northDiff, 2));
		heading = atan2f(eastDiff, northDiff) * RAD2DEG;
		heading -= attitudeActual.Yaw;
		heading = circular_modulus_deg(heading);
		hudSendWaypoint(HUD_WAYPOINT_NAVI, true, distance, heading);
	} else if (lastShowNaviWp) {
		hudSendWaypoint(HUD_WAYPOINT_NAVI, false, 0, 0);
	}
	lastShowNaviWp = showNaviWp;
}

/**
 * Enter bootloader mode
 * @return true if successful
 */
static bool osdEnterBootloader(void)
{
	txPayload[0] = REQ_ID_ENTER_BOOT;
	uint8_t ansLen;
	if (draco_osd_comm_send_request(txPayload, 1, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < 1) || (answerPayload[0] != COMM_RESULT_OK))
		return false;
	else
		return true;
}

/**
 * Exit bootloader mode
 * @return 0 if successful, -1 when reques timeout and -2 if FW image is invalid
 */
static int32_t osdExitBootloader(void)
{
	txPayload[0] = REQ_ID_EXIT_BOOT;
	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 1, answerPayload, &ansLen) != 0)
		return -1;

	if ((ansLen < 1) || (answerPayload[0] != COMM_RESULT_OK))
		return -2;
	else
		return 0;
}

/**
 * Start flash memory write operation
 * @param[in] offset in flash memory
 * @param[in] size firmware image size
 * @return true if successful
 */
static bool osdStartWriteFw(uint32_t offset, uint32_t size)
{
	txPayload[0] = REQ_ID_WRITE_START;
	txPayload[1] = size & 0xff;
	txPayload[2] = (size >> 8) & 0xff;
	txPayload[3] = (size >> 16) & 0xff;
	txPayload[4] = (size >> 24) & 0xff;
	txPayload[5] = offset & 0xff;
	txPayload[6] = (offset >> 8) & 0xff;
	txPayload[7] = (offset >> 16) & 0xff;
	txPayload[8] = (offset >> 24) & 0xff;

	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 9, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < 1) || (answerPayload[0] != COMM_RESULT_OK))
		return false;

	return true;
}

/**
 * Write chunk of data to flash
 * @param[in] data pointer to data to write
 * @param[in] len length of data
 * @return true if successful
 */
static bool osdWriteChunk(const uint8_t *data, uint8_t len)
{
	if (!len)
		return true;
	if (!data || len > 254)
		return false;

	txPayload[0] = REQ_ID_WRITE_CHUNK;
	memcpy(&txPayload[1], data, len);

	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, len + 1, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < 1) || (answerPayload[0] != COMM_RESULT_OK))
		return false;

	return true;
}

/**
 * Start read flash memory operation
 * @param[in] offset in flash memory
 * @return true if successful
 */
static bool osdStartReadFw(uint32_t offset)
{
	txPayload[0] = REQ_ID_READ_START;
	txPayload[1] = offset & 0xff;
	txPayload[2] = (offset >> 8) & 0xff;
	txPayload[3] = (offset >> 16) & 0xff;
	txPayload[4] = (offset >> 24) & 0xff;

	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 5, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < 1) || (answerPayload[0] != COMM_RESULT_OK))
		return false;

	return true;
}

/**
 * Write chunk of data to flash
 * @param[out] data pointer to data
 * @param[in] len read size (mustn't exceed data buffer)
 * @return true if successful
 */
static bool osdReadChunk(uint8_t *data, uint8_t len)
{
	if (!len)
		return true;
	if (!data || len > 254)
		return false;

	txPayload[0] = REQ_ID_READ_CHUNK;
	txPayload[1] = len;

	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 2, answerPayload, &ansLen) != 0)
		return false;

	if ((ansLen < (1 + len)) || (answerPayload[0] != COMM_RESULT_OK))
		return false;

	memcpy(data, &answerPayload[1], len);
	return true;
}

/**
 * DracoOSDFwUpdate UAVO was just updated
 */
void hwDracoOsdFwUpdateEvent(UAVObjEvent* ev)
{
	(void) ev;
	PIOS_Semaphore_Give(osdFwUpdateSem);
}

/**
 * Process DracoOSDFwUpdate UAVO from osdTask thread context
 */
void hwDracoOsdFwUpdateProcessObjectChange(void)
{
	HWDracoOsdFwUpdateGet(&fwUpdate);
	if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_GETVERSION) {
		uint8_t mode;
		if (getVersion(&mode, versionString, sizeof(versionString))) {
			strncpy((char*)fwUpdate.Data, versionString, sizeof(fwUpdate.Data));
			fwUpdate.DataSize = strnlen((char*)fwUpdate.Data, sizeof(fwUpdate.Data));
			if (mode == COMM_VERSION_MODE_FIRMWARE) {
				bootloaderMode = false;
				fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			} else if (mode == COMM_VERSION_MODE_BOOTLOADER) {
				bootloaderMode = true;
				fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			}
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
			HWDracoOsdFwUpdateSet(&fwUpdate);
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDGETVERSION;
			HWDracoOsdFwUpdateSet(&fwUpdate);
		}
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_ENTERBOOTLOADER) {
		if (osdEnterBootloader()) {
			// wait until OSD MCU end up in bootloader mode
			PIOS_Thread_Sleep(100);
			bootloaderMode = true;
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDENTERBOOTLOADER;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_EXITBOOTLOADER) {
		int res = osdExitBootloader();
		if (res == 0) {
			// wait until OSD MCU end up in firmware mode
			PIOS_Thread_Sleep(100);
			bootloaderMode = false;
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else if (res == -1) {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDEXITBOOTLOADER;
		} else {
			bootloaderMode = true;
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_INVALIDFWIMAGE;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_STARTWRITEFW) {
		if (osdStartWriteFw(0, fwUpdate.DataSize)) {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDWRITE;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_WRITECHUNK) {
		if (osdWriteChunk(fwUpdate.Data, (uint8_t)fwUpdate.DataSize)) {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDWRITE;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_STARTREADFW) {
		if (osdStartReadFw(0)) {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDREAD;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	} else if (fwUpdate.Control == HWDRACOOSDFWUPDATE_CONTROL_READCHUNK) {
		if (fwUpdate.DataSize <= sizeof(fwUpdate.Data) && osdReadChunk(fwUpdate.Data, (uint8_t)fwUpdate.DataSize)) {
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_NOERROR;
		} else {
			fwUpdate.DataSize = 0;
			fwUpdate.Control = (bootloaderMode) ? HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER :
					HWDRACOOSDFWUPDATE_CONTROL_STATEIDLE;
			fwUpdate.Error = HWDRACOOSDFWUPDATE_ERROR_FAILEDREAD;
		}
		HWDracoOsdFwUpdateSet(&fwUpdate);
	}
}
/**
 * Initialize and start task
 */
void draco_osd_task_start(void)
{
	txPayload = PIOS_malloc(256);
	if (!txPayload)
		return;

	answerPayload = PIOS_malloc(256);
	if (!answerPayload) {
		PIOS_free(txPayload);
		return;
	}

	osdFwUpdateSem = PIOS_Semaphore_Create();
	if (!osdFwUpdateSem) {
		PIOS_free(txPayload);
		PIOS_free(answerPayload);
		return;
	}

	HWDracoOsdFwUpdateInitialize();
	dracoOsdTaskHandle = PIOS_Thread_Create(dracoOsdTask, "dracoOsdTask",
			STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
}

/**
 * Task routine
 */
static void dracoOsdTask(void *parameters)
{
	uint32_t lastTime = PIOS_Thread_Systime();

	PIOS_Thread_Sleep(100);
	uint8_t fwMode = 0;
	if (getVersion(&fwMode, versionString, sizeof(versionString))) {
		if (fwMode == COMM_VERSION_MODE_BOOTLOADER) {
			fwUpdate.Control = HWDRACOOSDFWUPDATE_CONTROL_STATEIDLEBOOTLOADER;
			fwUpdate.DataSize = 0;
			HWDracoOsdFwUpdateSet(&fwUpdate);
			bootloaderMode = true;
		}
	}

	HWDracoOsdFwUpdateConnectCallback(hwDracoOsdFwUpdateEvent);

	uint8_t osdEnabled = 0;
	HwDracoOSDEnableGet(&osdEnabled);

	hudEnable(true);

	uint8_t osdUnits = 0;
	HwDracoOSDUnitsGet(&osdUnits);
	if (osdUnits == HWDRACO_OSDUNITS_IMPERIAL)
		hudSetUnits(HUD_UNITS_IMPERIAL);
	else
		hudSetUnits(HUD_UNITS_METRIC);

	uint8_t tvSys;
	HwDracoOSDTvSystemGet(&tvSys);
	switch (tvSys) {
		case HWDRACO_OSDTVSYSTEM_PAL:
			hudForceTvSystem(HUD_TVSYSTEM_PAL);
			break;
		case HWDRACO_OSDTVSYSTEM_NTSC:
			hudForceTvSystem(HUD_TVSYSTEM_NTSC);
			break;
		default:
			break;
	}

	bool useBattery = false;
	if (FlightBatterySettingsHandle() != 0) {
		FlightBatterySettingsData batterySettings;
		FlightBatterySettingsGet(&batterySettings);
		float voltageLimit = batterySettings.VoltageThresholds[FLIGHTBATTERYSETTINGS_VOLTAGETHRESHOLDS_WARNING];
		uint16_t capacityLimit = (uint16_t)((float)(batterySettings.Capacity) * 0.80f);
		hudSetBatteryLimits((uint16_t)(voltageLimit * 100.0f), capacityLimit);
		useBattery = true;
	}

	uint8_t gpsStatusLast = 0;
	float throttleLast = 0;
	bool throttleActiveChange = false;
	uint8_t alarmStatusLast = getAlarmStatus();
	uint8_t telemetryStatusLast = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	while (1) {
		// check and process for HwDracoOsdFwUpdate command
		// and continue with checking and processomg if OSD bootloader mode
		// is active
		do {
			if (PIOS_Semaphore_Take(osdFwUpdateSem, bootloaderMode ? PIOS_SEMAPHORE_TIMEOUT_MAX : 0))
				hwDracoOsdFwUpdateProcessObjectChange();
		} while(bootloaderMode);

		// process loop at OSD refresh rate
		draco_osd_comm_wait(1000 / TASK_RATE_HZ);
		uint32_t currentTime = PIOS_Thread_Systime();
		if ((currentTime > lastTime) && ((currentTime - lastTime) < (1000 / TASK_RATE_HZ)))
			continue;
		lastTime = currentTime;

		if (AttitudeActualHandle() != 0)
			AttitudeActualGet(&attitudeActual);

		if (VelocityActualHandle() != 0)
			VelocityActualGet(&velocityActual);

		if (PositionActualHandle() != 0)
			PositionActualGet(&positionActual);

		if (AirspeedActualHandle() != 0)
			AirspeedActualGet(&airspeedActual);

		hudSendPfd();

		if ((FlightBatteryStateHandle() != 0) && (useBattery)) {
			FlightBatteryStateGet(&flightBatteryState);
			hudSendBattery();
		}

		if (GPSPositionHandle() != 0) {
			GPSPositionGet(&gpsPosition);
			hudSendGnss();
		}

		if (ActuatorDesiredHandle() != 0) {
			// check whether throttle command has changed from stopped
			// to running and vice versa
			ActuatorDesiredGet(&actuatorDesired);
			if (((actuatorDesired.Throttle > 0) && (throttleLast <= 0)) ||
					((throttleLast > 0) && (actuatorDesired.Throttle <= 0)))
				throttleActiveChange = true;
			else
				throttleActiveChange = false;

			throttleLast = actuatorDesired.Throttle;
		}

		if (FlightStatusHandle() != 0) {
			FlightStatusData flightStatusNew;
			FlightStatusGet(&flightStatusNew);

			bool armStatusChanged = false;
			if ((flightStatus.Armed != flightStatusNew.Armed) ||
					(flightStatus.FlightMode != flightStatusNew.FlightMode) || (firstPass)) {
				armStatusChanged = flightStatus.Armed != flightStatusNew.Armed;

				memcpy(&flightStatus, &flightStatusNew, sizeof(FlightStatusData));
				hudSendMode();
				if ((armStatusChanged) || (firstPass)) {
					// handle ARMED LED
					switch (flightStatus.Armed) {
					case FLIGHTSTATUS_ARMED_ARMED:
						hudControlLed(HUD_LED_ARMED, HUD_LED_MODE_ON);
						break;
					case FLIGHTSTATUS_ARMED_ARMING:
						hudControlLed(HUD_LED_ARMED, HUD_LED_MODE_BLINK_FAST);
						break;
					case FLIGHTSTATUS_ARMED_DISARMED:
						hudControlLed(HUD_LED_ARMED, HUD_LED_MODE_BLINK_SLOW);
						break;
					}
				}
			}

			if ((armStatusChanged) || (firstPass) || (throttleActiveChange)) {
				hudSetupStopwatch();
			}
		}
		bool showNaviWp = false;
		if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER) {
			if (WaypointActiveHandle() != 0) {
				WaypointActiveData waypointActive;
				WaypointActiveGet(&waypointActive);
				if (waypointActive.Index < WaypointGetNumInstances()) {
					WaypointInstGet(waypointActive.Index, &waypointActual);
					showNaviWp = true;
				}
			}
		}

		uint8_t gpsStatus = getGpsStatus();
		if (gpsStatus == 2)
			hudSendWaypoints(showNaviWp);

		// handle GPS LED
		if ((gpsStatus != gpsStatusLast) || (firstPass)) {
			switch (gpsStatus) {
			case 0:
				hudControlLed(HUD_LED_GPS, HUD_LED_MODE_OFF);
				break;
			case 1:
				hudControlLed(HUD_LED_GPS, HUD_LED_MODE_BLINK_FAST);
				break;
			case 2:
				hudControlLed(HUD_LED_GPS, HUD_LED_MODE_ON);
				break;
			}
		}

		// handle telemetry LED
		if (FlightTelemetryStatsHandle() != 0) {
			FlightTelemetryStatsGet(&telemetryStats);
			uint8_t telemetryStatus = telemetryStats.Status;
			if ((telemetryStatus != telemetryStatusLast) || (firstPass)) {
				switch (telemetryStatus) {
				case FLIGHTTELEMETRYSTATS_STATUS_CONNECTED:
					hudControlLed(HUD_LED_TELEMETRY, HUD_LED_MODE_ON);
					break;
				case FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED:
					hudControlLed(HUD_LED_TELEMETRY, HUD_LED_MODE_OFF);
					break;
				default:
					hudControlLed(HUD_LED_TELEMETRY, HUD_LED_MODE_BLINK_FAST);
					break;
				}
				telemetryStatusLast = telemetryStatus;
			}
		}

		// handle alarm LED
		uint8_t alarmStatus = getAlarmStatus();
		if ((alarmStatus != alarmStatusLast) || (firstPass)) {
			switch (alarmStatus) {
			case 0:
				hudControlLed(HUD_LED_ALARM, HUD_LED_MODE_OFF);
				break;
			case 1:
				hudControlLed(HUD_LED_ALARM, HUD_LED_MODE_BLINK_SLOW);
				break;
			case 2:
				hudControlLed(HUD_LED_ALARM, HUD_LED_MODE_ON);
				break;
			}
			alarmStatusLast = alarmStatus;
		}
		gpsStatusLast = gpsStatus;
		firstPass = false;
	}
}

#endif /* DRACO_INCLUDE_OSD_SUPPORT */
