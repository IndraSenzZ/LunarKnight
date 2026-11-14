/*
 *  step_wise.c - A step-by-step Thermal throttling governor
 *
 *  Copyright (C) 2012 Intel Corp
 *  Copyright (C) 2012 Durgadoss R <durgadoss.r@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/thermal.h>
#include <trace/events/thermal.h>

#include "thermal_core.h"

/*
 * Above the trip: step the cooling device up, or jump to the trip's
 * upper/lower limit for the FULL trends. Below the hysteresis the
 * instance is dropped in one shot. TSENS delivers a single falling-edge
 * interrupt and does not poll, so walking cur_state - 1 leaves a device
 * that climbed two steps (gold thermal-cpufreq-4) stuck above idle
 * until reboot. Little only climbs one step, so the same edge hits 0.
 */
static unsigned long get_target_state(struct thermal_instance *instance,
				enum thermal_trend trend, bool throttle)
{
	struct thermal_cooling_device *cdev = instance->cdev;
	unsigned long cur_state;
	unsigned long next_target;

	/* Below hysteresis: release this trip completely. */
	if (!throttle)
		return THERMAL_NO_TARGET;

	cdev->ops->get_cur_state(cdev, &cur_state);
	next_target = instance->target;
	dev_dbg(&cdev->device, "cur_state=%ld\n", cur_state);

	if (!instance->initialized) {
		next_target = (cur_state + 1) >= instance->upper ?
				instance->upper :
				((cur_state + 1) < instance->lower ?
				instance->lower : (cur_state + 1));
		return next_target;
	}

	switch (trend) {
	case THERMAL_TREND_RAISING:
		next_target = cur_state < instance->upper ?
			    (cur_state + 1) : instance->upper;
		if (next_target < instance->lower)
			next_target = instance->lower;
		break;
	case THERMAL_TREND_RAISE_FULL:
		next_target = instance->upper;
		break;
	case THERMAL_TREND_DROPPING:
	case THERMAL_TREND_STABLE:
		/* Still above hysteresis: hold this step. */
		break;
	case THERMAL_TREND_DROP_FULL:
		if (cur_state != instance->lower)
			next_target = instance->lower;
		break;
	default:
		break;
	}

	return next_target;
}

static void update_passive_instance(struct thermal_zone_device *tz,
				enum thermal_trip_type type, int value)
{
	/*
	 * If value is +1, activate a passive instance.
	 * If value is -1, deactivate a passive instance.
	 */
	if (type == THERMAL_TRIP_PASSIVE)
		tz->passive += value;
}

static void thermal_zone_trip_update(struct thermal_zone_device *tz, int trip)
{
	int trip_temp, hyst_temp;
	enum thermal_trip_type trip_type;
	enum thermal_trend trend;
	struct thermal_instance *instance;
	bool throttle = false;
	int old_target;

	tz->ops->get_trip_temp(tz, trip, &trip_temp);
	if (tz->ops->get_trip_hyst) {
		tz->ops->get_trip_hyst(tz, trip, &hyst_temp);
		hyst_temp = trip_temp - hyst_temp;
	} else {
		hyst_temp = trip_temp;
	}
	tz->ops->get_trip_type(tz, trip, &trip_type);

	trend = get_tz_trend(tz, trip);

	dev_dbg(&tz->device, "Trip%d[type=%d,temp=%d]:trend=%d,throttle=%d\n",
				trip, trip_type, trip_temp, trend, throttle);

	mutex_lock(&tz->lock);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;

		old_target = instance->target;
		/*
		 * Step wise has to lower the mitigation only if the
		 * temperature goes below the hysteresis temperature.
		 * Atleast, it has to hold on to mitigation device lower
		 * limit if the temperature is above the hysteresis
		 * temperature.
		 */
		if (tz->temperature >= trip_temp ||
			(tz->temperature > hyst_temp &&
			 old_target != THERMAL_NO_TARGET))
			throttle = true;
		else
			throttle = false;

		instance->target = get_target_state(instance, trend, throttle);
		dev_dbg(&instance->cdev->device, "old_target=%d, target=%d\n",
					old_target, (int)instance->target);

		if (instance->initialized && old_target == instance->target)
			continue;

		if (!instance->initialized) {
			if (instance->target != THERMAL_NO_TARGET) {
				trace_thermal_zone_trip(tz, trip, trip_type,
							true);
				update_passive_instance(tz, trip_type, 1);
			}
		} else {
			/* Activate a passive thermal instance */
			if (old_target == THERMAL_NO_TARGET &&
				instance->target != THERMAL_NO_TARGET) {
				trace_thermal_zone_trip(tz, trip, trip_type,
							true);
				update_passive_instance(tz, trip_type, 1);
			/* Deactivate a passive thermal instance */
			} else if (old_target != THERMAL_NO_TARGET &&
				instance->target == THERMAL_NO_TARGET) {
				trace_thermal_zone_trip(tz, trip, trip_type,
							false);
				update_passive_instance(tz, trip_type, -1);
			}
		}

		instance->initialized = true;
		mutex_lock(&instance->cdev->lock);
		instance->cdev->updated = false; /* cdev needs update */
		mutex_unlock(&instance->cdev->lock);
	}

	mutex_unlock(&tz->lock);
}

/**
 * step_wise_throttle - throttles devices associated with the given zone
 * @tz - thermal_zone_device
 * @trip - trip point index
 *
 * Throttling Logic: This uses the trend of the thermal zone to throttle.
 * If the thermal zone is 'heating up' this throttles all the cooling
 * devices associated with the zone and its particular trip point, by one
 * step. If the zone is 'cooling down' it brings back the performance of
 * the devices by one step.
 */
static int step_wise_throttle(struct thermal_zone_device *tz, int trip)
{
	struct thermal_instance *instance;

	thermal_zone_trip_update(tz, trip);

	mutex_lock(&tz->lock);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node)
		thermal_cdev_update(instance->cdev);

	mutex_unlock(&tz->lock);

	return 0;
}

static struct thermal_governor thermal_gov_step_wise = {
	.name		= "step_wise",
	.throttle	= step_wise_throttle,
};

int thermal_gov_step_wise_register(void)
{
	return thermal_register_governor(&thermal_gov_step_wise);
}

void thermal_gov_step_wise_unregister(void)
{
	thermal_unregister_governor(&thermal_gov_step_wise);
}
