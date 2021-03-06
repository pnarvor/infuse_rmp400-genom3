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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "acrmp400.h"

#include "rmp400_c_types.h"

#include "codels.h"
#include "rmp400_Log.h"

/* --- Activity Track --------------------------------------------------- */

/** Validation codel trackControl of activity Track.
 *
 * Returns genom_ok.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 * rmp400_cmd_stop_track, rmp400_motors_off, rmp400_emergency_stop,
 * rmp400_power_cord_connected.
 */
genom_event
trackControl(RMP_DEV_TAB **rmp, const rmp400_data_str rs_data[2],
             const genom_context self)
{
	if (rmp == NULL)
		return rmp400_not_connected(self);
	return genom_ok;
}


/* --- Activity DirectTrack --------------------------------------------- */

/** Validation codel trackControl of activity DirectTrack.
 *
 * Returns genom_ok.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 * rmp400_cmd_stop_track, rmp400_motors_off, rmp400_emergency_stop,
 * rmp400_power_cord_connected, rmp400_invalid_parameter.
 */
/* already defined in service Track validation */



/* --- Function toggleInfuseTrackMode ----------------------------------- */

/** Codel toggleInfuseTrackMode of function toggleInfuseTrackMode.
 *
 * Returns genom_ok.
 */
genom_event
toggleInfuseTrackMode(rmp400_mode rs_mode, uint8_t *infuseTrackMode,
                      const genom_context self)
{
    if(*infuseTrackMode)
    {
        if(rs_mode == rmp400_mode_track)
        {
            printf("Error, rmp400 is in track activity. Deactivate track activity before toggling track mode.\nIgnoring command.");;
            return genom_ok;
        }
        *infuseTrackMode = 0;
        printf("Using idl track mode : cmd_vel\n");
    }
    else
    {
        *infuseTrackMode = 1;
        printf("Using infuse track mode : cmd_vel_Infuse\n");
    }
    return genom_ok;
}


/* --- Function Stop ---------------------------------------------------- */

/** Codel setZeroVelocity of function Stop.
 *
 * Returns genom_ok.
 */
genom_event
setZeroVelocity(rmp400_mode *rs_mode, or_genpos_cart_speed *ref,
                const genom_context self)
{
    ref->v = 0;
    ref->w = 0;

    *rs_mode = rmp400_mode_idle;

    return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel log_start of function log.
 *
 * Returns genom_ok.
 * Throws rmp400_sys_error.
 */
genom_event
log_start(const char path[64], rmp400_log_str **log,
          const genom_context self)
{
	FILE *f;

	log_stop(log, self);

	f = fopen(path, "w");
	if (f == NULL)
		return rmp400_sys_error(self);
	fprintf(f, rmp400_log_header "\n");

	*log = (rmp400_log_str*)malloc(sizeof(**log));
	if (*log == NULL) {
		fclose(f);
		unlink(path);
		errno = ENOMEM;
		return rmp400_sys_error(self);
	}

	(*log)->out = f;
	return genom_ok;

}


/* --- Function log_stop ------------------------------------------------ */

/** Codel log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
log_stop(rmp400_log_str **log, const genom_context self)
{
	if (*log == NULL)
		return genom_ok;

	fclose((*log)->out);
	free(*log);
	*log = NULL;
	return genom_ok;
}
