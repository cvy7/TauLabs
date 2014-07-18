/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 *
 * @file       failsafe_control.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @brief      Failsafe controller when transmitter control is lost
 *
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

#include "openpilot.h"
#include "control.h"
#include "failsafe_control.h"
#include "transmitter_control.h"

#include "flightstatus.h"
#include "stabilizationdesired.h"
#include "manualcontrolcommand.h"
#include "manualcontrolsettings.h"
#include "systemsettings.h"
#include "modulesettings.h"
#include "stateestimation.h"

static bool throttle_low_before_engage;
static bool armed_when_enabled;
static bool rth_active;

//! Check if RTH is possible and safe
static bool check_rth_preconditions_met()
{
	// If transmitter throttle was low before failsafe kicked in
	// RTH is not safe, because pilot probably just forgot to disarm
	// before powering transmitter off
	if (throttle_low_before_engage)
		return false;

	// No RTH when disarmed
	if (!armed_when_enabled)
		return false;

	// Only VTOLs are currently RTH capable
	uint8_t airframe_type;
	SystemSettingsAirframeTypeGet(&airframe_type);
	if (airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_QUADX &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_QUADP &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_HEXA &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_OCTO &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_HEXAX &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_OCTOV &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_OCTOCOAXP &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_HEXACOAX &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_TRI &&
			airframe_type != SYSTEMSETTINGS_AIRFRAMETYPE_VTOL)
		return false;


	uint8_t module_admin_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_admin_state);

	// VTOLPathFollower module must be enabled
	if (module_admin_state[MODULESETTINGS_ADMINSTATE_VTOLPATHFOLLOWER] != MODULESETTINGS_ADMINSTATE_ENABLED)
		return false;

	// GPS module must be enabled
	if (module_admin_state[MODULESETTINGS_ADMINSTATE_GPS] != MODULESETTINGS_ADMINSTATE_ENABLED)
		return false;

	// check path follower alarm
	if (AlarmsGet(SYSTEMALARMS_ALARM_PATHFOLLOWER) >= SYSTEMALARMS_ALARM_WARNING)
		return false;

	// attitude filter must be set to complementary or INSOutdoor and navigation filter must be INS
	StateEstimationData stateEstimation;
	StateEstimationGet(&stateEstimation);
	if (stateEstimation.AttitudeFilter == STATEESTIMATION_ATTITUDEFILTER_INSINDOOR ||
	               stateEstimation.NavigationFilter != STATEESTIMATION_NAVIGATIONFILTER_INS)
		return false;

	return true;
}

//! Initialize the failsafe controller
int32_t failsafe_control_initialize()
{
	return 0;
}

//! Perform any updates to the failsafe controller
int32_t failsafe_control_update()
{
	// periodically monitor for throttle command while transmitter
	// is in control
	ManualControlCommandData manualControl;
	ManualControlCommandGet(&manualControl);
	if (manualControl.Connected == MANUALCONTROLCOMMAND_CONNECTED_TRUE) {
		if (manualControl.Throttle <= 0.01f)
			throttle_low_before_engage = true;
		else
			throttle_low_before_engage = false;
	}

	return 0;
}

/**
 * Select and use failsafe control
 * @param [in] reset_controller True if previously another controller was used
 */
int32_t failsafe_control_select(bool reset_controller)
{
	if (reset_controller) {
		FlightStatusArmedOptions armed; 
		FlightStatusArmedGet(&armed);
		armed_when_enabled = (armed == FLIGHTSTATUS_ARMED_ARMED);
		rth_active = false;

		/* Get configured failsafe behavior */
		uint8_t failsafe_behavior;
		ManualControlSettingsFailsafeBehaviorGet(&failsafe_behavior);

		/* check whether RTH failsafe is configured and can be used */
		if (failsafe_behavior == MANUALCONTROLSETTINGS_FAILSAFEBEHAVIOR_RTH
				&& check_rth_preconditions_met())
			rth_active = true;
	}

	uint8_t flight_mode_desired = rth_active ? FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME : FLIGHTSTATUS_FLIGHTMODE_STABILIZED1;
	uint8_t flight_mode_actual;
	FlightStatusFlightModeGet(&flight_mode_actual);
	if (flight_mode_actual != flight_mode_desired || reset_controller) {
		FlightStatusFlightModeSet(&flight_mode_desired);
	}

#ifdef GIMBAL
	// Gimbals do not need failsafe
	StabilizationDesiredData stabilization_desired;
	StabilizationDesiredGet(&stabilization_desired);
	stabilization_desired.Throttle = -1;
	stabilization_desired.Roll = 0;
	stabilization_desired.Pitch = 0;
	stabilization_desired.Yaw = 0;
	stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_POI;
	stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_AXISLOCK;
	StabilizationDesiredSet(&stabilization_desired);
#else
	if (!rth_active) {
		// Pick default values that will roughly cause a plane to circle down
		// and a quad to fall straight down
		StabilizationDesiredData stabilization_desired;
		StabilizationDesiredGet(&stabilization_desired);

		if (!armed_when_enabled) {
			/* disable stabilization so outputs do not move when system was not armed */
			stabilization_desired.Throttle = -1;
			stabilization_desired.Roll  = 0;
			stabilization_desired.Pitch = 0;
			stabilization_desired.Yaw   = 0;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_NONE;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_NONE;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_NONE;
		} else {
			/* Pick default values that will roughly cause a plane to circle down and */
			/* a quad to fall straight down */
			stabilization_desired.Throttle = -1;
			stabilization_desired.Roll = -10;
			stabilization_desired.Pitch = 0;
			stabilization_desired.Yaw = -5;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
			stabilization_desired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
		}

		StabilizationDesiredSet(&stabilization_desired);
	}
#endif

	return 0;
}

//! Get any control events
enum control_events failsafe_control_get_events()
{
	// For now ARM / DISARM events still come from the transmitter.  This
	// means the normal disarm timeout still applies.  To be replaced later
	// by a full state machine determining how long to stay in failsafe before
	// disarming. THIS IS TRUE ONLY IF RTH IS NOT ACTIVE! Otherwise we don't
	// want trasnmitter to disarm by timeout when RTH mission is active.
	if (!rth_active)
		return transmitter_control_get_events();
	else
		return CONTROL_EVENTS_NONE;
}

/**
 * @}
 * @}
 */

