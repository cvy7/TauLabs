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
#include "flighttelemetrystats.h"
#include "hwdraco.h"

#if defined(PIOS_DRACO_OSD_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_DRACO_OSD_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1024
#endif
#define TASK_PRIORITY               (tskIDLE_PRIORITY + 1)
#define TASK_RATE_HZ 25

#define REQ_ID_VERSION              0
#define REQ_ID_HUD_ENABLE           1
#define REQ_ID_POWER_LIMITS         10
#define REQ_ID_FLIGHT_MODE          11
#define REQ_ID_SET_UNITS            12

#define DATA_ID_LED_CONTROL         0
#define DATA_ID_PFD                 1
#define DATA_ID_WAYPOINT_HOME       2
#define DATA_ID_WAYPOINT_NAVI       3
#define DATA_ID_GNSS                4
#define DATA_ID_POWER               5
#define DATA_ID_STOPWATCH           6

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

static xTaskHandle dracoOsdTaskHandle;

static uint8_t *txPayload;
static uint8_t *answerPayload;
static char versionString[24];

static AttitudeActualData attitudeActual;
static PositionActualData positionActual;
static GPSPositionData gpsPosition;
static FlightBatteryStateData flightBatteryState;
static VelocityActualData velocityActual;
static FlightStatusData flightStatus;
static WaypointData waypointActual;
static FlightTelemetryStatsData telemetryStats;

static void dracoOsdTask(void *parameters);

/**
 * Read version information from OSD MCU
 * This is used to test OSD MCU is alive
 * @return 0 when successful
 */
static int32_t getVersion(void)
{
	versionString[0] = 0;
	txPayload[0] = REQ_ID_VERSION;
	uint8_t ansLen = 0;
	if (draco_osd_comm_send_request(txPayload, 1, answerPayload, &ansLen) != 0)
		return -1;
	if ((ansLen < 26) && (ansLen > 2)) {
		versionString[ansLen - 2] = 0;
		memcpy(versionString, &answerPayload[2], ansLen - 2);
		return 0;
	}
	return -2;
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
		break;
	case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
		return "AUTOTUNE";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
		return "MANUAL";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
		return "PLANNER";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
		return "POSHOLD";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
		return "RTH";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
		return "STAB1";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
		return "STAB2";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
		return "STAB3";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_TABLETCONTROL:
		return "TABLET";
		break;
	case FLIGHTSTATUS_FLIGHTMODE_VELOCITYCONTROL:
		return "VELCTRL";
		break;
	default:
		return "UNKNOWN";
		break;
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
	if (flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED)
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
	heading -= attitudeActual.Yaw;
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
 * Initialize and start task
 */
void draco_osd_task_start(void)
{
	txPayload = PIOS_malloc(128);
	if (!txPayload)
		return;

	answerPayload = PIOS_malloc(128);
	if (!answerPayload) {
		vPortFree(txPayload);
		return;
	}

	xTaskCreate(dracoOsdTask, (signed char*)"dracoOsdTask",
			STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY,
			&dracoOsdTaskHandle);
}

/**
 * Task routine
 */
static void dracoOsdTask(void *parameters)
{
	portTickType lastTime = xTaskGetTickCount();

	vTaskDelay(MS2TICKS(100));
	getVersion();
	uint8_t osdEnabled = 0;
	HwDracoOSDEnableGet(&osdEnabled);

	if (osdEnabled == HWDRACO_OSDENABLE_ENABLED)
		hudEnable(true);
	else
		hudEnable(false);

	uint8_t osdUnits = 0;
	HwDracoOSDUnitsGet(&osdUnits);
	if (osdUnits == HWDRACO_OSDUNITS_IMPERIAL)
		hudSetUnits(HUD_UNITS_IMPERIAL);
	else
		hudSetUnits(HUD_UNITS_METRIC);

	bool useBattery = false;
	if (FlightBatterySettingsHandle() != 0) {
		FlightBatterySettingsData batterySettings;
		FlightBatterySettingsGet(&batterySettings);
		float voltageLimit = batterySettings.VoltageThresholds[FLIGHTBATTERYSETTINGS_VOLTAGETHRESHOLDS_WARNING];
		uint16_t capacityLimit = (uint16_t)((float)(batterySettings.Capacity) * 0.80f);
		hudSetBatteryLimits((uint16_t)(voltageLimit * 100.0f), capacityLimit);
		useBattery = true;
	}

	bool firstPass = true;
	uint8_t gpsStatusLast = 0;
	uint8_t alarmStatusLast = getAlarmStatus();
	uint8_t telemetryStatusLast = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	while (1) {
		draco_osd_comm_wait(1000 / TASK_RATE_HZ);
		portTickType currentTime = xTaskGetTickCount();
		if ((currentTime > lastTime) && (TICKS2MS(currentTime - lastTime) < (1000 / TASK_RATE_HZ)))
			continue;
		lastTime = currentTime;

		if (AttitudeActualHandle() != 0)
			AttitudeActualGet(&attitudeActual);

		if (VelocityActualHandle() != 0)
			VelocityActualGet(&velocityActual);

		if (PositionActualHandle() != 0)
			PositionActualGet(&positionActual);

		hudSendPfd();

		if ((FlightBatteryStateHandle() != 0) && (useBattery)) {
			FlightBatteryStateGet(&flightBatteryState);
			hudSendBattery();
		}

		if (GPSPositionHandle() != 0) {
			GPSPositionGet(&gpsPosition);
			hudSendGnss();
		}

		if (FlightStatusHandle() != 0) {
			FlightStatusData flightStatusNew;
			FlightStatusGet(&flightStatusNew);

			if ((flightStatus.Armed != flightStatusNew.Armed) ||
					(flightStatus.FlightMode != flightStatusNew.FlightMode) || (firstPass)) {
				bool armStatusChanged = flightStatus.Armed != flightStatusNew.Armed;

				memcpy(&flightStatus, &flightStatusNew, sizeof(FlightStatusData));
				hudSendMode();
				if ((armStatusChanged) || (firstPass)) {
					hudSetupStopwatch();

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
