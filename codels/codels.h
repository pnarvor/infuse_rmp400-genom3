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
#ifndef _CODELS_H
#define _CODELS_H

#define DEBUG 1

/*----------------------------------------------------------------------*/

/*
 * Prototypes
 */

/* joystick.c */
extern void getJoystickSpeeds(struct or_joystick_state *joy, 
    double *v, double *w, double *avmax, double *awmax);

/* motion_helpers.c */
extern void rmp400DataUpdate(RMP_DEV_STR **, const rmp400_kinematics_str *,
    rmp400_status_str *status, rmp_status_str *statusgen);
extern void rmp400VelocityGet(rmp400_data_str *rs_data[2],
    rmp400_kinematics_str *kinematics,
    or_genpos_cart_state *robot);
extern genom_event rmp400VelocitySet(RMP_DEV_STR **rmp,
    double v, double w, or_genpos_cart_speed *ref, const genom_context self);
extern void bound_accels(rmp400_max_accel *acc, double t,
    double *vel_reference, double *ang_reference);
extern void control_yaw(rmp400_gyro_asserv *gyro,
    double t, double vel_reference, double yawr_reference,
    double yawr_measure, double yaw_measure, double *yawr_command);
extern bool joystickQuit(struct or_joystick_state *joy);

/* rmp400ReadTask.c */
extern void *rmp400ReadTask(void *);

#endif /* _CODELS_H */
