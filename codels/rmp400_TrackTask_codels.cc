/*
 * Copyright (c) 2017-2018 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/**
 ** Author: Matthieu Herrb
 ** Date: January 2017
 **/

#include <sys/param.h>
#include <math.h>
#include <stdio.h>
#include "acrmp400.h"

#include "rmp400_c_types.h"


/* --- Task TrackTask --------------------------------------------------- */

static genom_event
pumpSpeedReference(const or_genpos_cart_state *robot,
    const rmp400_cmd_vel *cmd_vel, or_genpos_cart_speed *ref,
    genom_context self)
{
	or_genpos_cart_speed *orders;

	if (cmd_vel->read(self) != genom_ok)
		return rmp400_port_not_found(self);
	orders = cmd_vel->data(self);
	if (orders == NULL)  {
		printf("%s: orders == NULL\n", __func__);
		return rmp400_pause_track_main;
	}
	ref->v = orders->v;
	ref->vt = 0;
	ref->w = orders->w;
	ref->vmax = orders->vmax;
	ref->wmax = orders->wmax;
	ref->linAccelMax = orders->linAccelMax;
	ref->angAccelMax = orders->angAccelMax;
	return rmp400_pause_track_main;
}

/* --- Activity Track --------------------------------------------------- */

/** Codel trackStart of activity Track.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_track_main, rmp400_end.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 *        rmp400_cmd_stop_track, rmp400_motors_off,
 *        rmp400_emergency_stop, rmp400_power_cord_connected.
 */
genom_event
trackStart(rmp400_mode *rs_mode, const rmp400_cmd_vel *cmd_vel,
           const genom_context self)
{
	printf("-- %s\n", __func__);
	if (cmd_vel->read(self) != genom_ok)
		return rmp400_port_not_found(self);
	if (cmd_vel->data(self) == NULL)
		return rmp400_port_not_found(self);
	*rs_mode = rmp400_mode_track;
	return rmp400_track_main;
}

/** Codel pumpReference of activity Track.
 *
 * Triggered by rmp400_track_main.
 * Yields to rmp400_pause_track_main, rmp400_end.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 *        rmp400_cmd_stop_track, rmp400_motors_off,
 *        rmp400_emergency_stop, rmp400_power_cord_connected.
 */
genom_event
pumpReference(const or_genpos_cart_state *robot, rmp400_mode rs_mode,
              const rmp400_cmd_vel *cmd_vel, or_genpos_cart_speed *ref,
              const genom_context self)
{
	/* Check if mode changed */
	if (rs_mode != rmp400_mode_track) {
		ref->v = 0;
		ref->w = 0;
		switch (rs_mode) {
		case rmp400_mode_emergency:
			return rmp400_emergency_stop(self);
		case rmp400_mode_motors_off:
			return rmp400_motors_off(self);
		default:
		return rmp400_bad_ref(self);
		}
	} else
		return pumpSpeedReference(robot, cmd_vel, ref, self);
}

/** Codel smoothStopTrack of activity Track.
 *
 * Triggered by rmp400_end.
 * Yields to rmp400_ether.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 *        rmp400_cmd_stop_track, rmp400_motors_off,
 *        rmp400_emergency_stop, rmp400_power_cord_connected.
 */
genom_event
smoothStopTrack(const or_genpos_cart_state *robot,
                const rmp400_dynamic_str *dynamics,
                rmp400_mode *rs_mode, or_genpos_cart_speed *ref,
                const genom_context self)
{
	printf("rmp400 smoothStopTrack\n");

	/* Set a null speed and stop the tracking */
	/* XXX should compute a decceleration ramp */
	ref->v = 0.0;
	ref->w = 0.0;
	*rs_mode = rmp400_mode_idle;
	return rmp400_ether;
}
