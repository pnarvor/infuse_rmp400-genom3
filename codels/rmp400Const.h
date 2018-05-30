/*
 * Copyright (c) 2009,2010 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
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
#ifndef _RMP400CONST_H
#define _RMP400CONST_H

/* Wheel radius for odometry */
#define RMP400_RIGHT_WHEEL_RADIUS 0.26
#define RMP400_LEFT_WHEEL_RADIUS 0.26
/* Distance between wheels for odometry */
#define RMP400_AXIS_WIDTH 0.615
#define RMP400_DIST_TO_CONTROLLED_POINT 0.0

/* Default uncertainties */
#define RMP400_VAR_XX	0.001
#define RMP400_VAR_YY	0.001
#define RMP400_VAR_TT	0.0001
#define RMP400_VAR_QCOEFF	0.05

/* Maximum speed */
#define RMP400_VMAX	2.0 /* for now. the real max is 5.0 */
#define RMP400_WMAX	1.1 /* real max is 1.1 */

/* Maximum accelerations */
#define RMP400_ACCEL_LIN_MAX	2.0
#define RMP400_DECEL_LIN_MAX	4.0
#define RMP400_ACCEL_ANG_MAX	3.0

/* Odometry z correction */
#define RMP400_ZODOCOR_ACCTHRES 0.75 /* acceleration threshold (factor of RMP400_ACCEL_LIN_MAX) to disable z update (m/s2) */
#define RMP400_ZODOCOR_LATENCY  4.0  /* how long it is disabled after last accel above threshold (s) */
#define RMP400_ZODOCOR_AVGPITCHSIZE 5 /* number of pitch samples to average for value to use when update is disabled */
#define RMP400_ZODOCOR_AVGVELSIZE 5 /* number of pitch samples to average for value to use when update is disabled */

/* Control Params */
#define RMP400_KP_LONGIT	2.
#define RMP400_KI_LONGIT	1.

#define RMP400_KP_TRANSV	10.
#define RMP400_KI_TRANSV	6.

#define RMP400_KP_ANG		2.
#define RMP400_KI_ANG		2.

/* gyro asserv params */
#define RMP400_DELAY_GYRO_OMEGA	0.3 /* delay between command and measure, in seconds */
#define RMP400_KP_GYRO_OMEGA	2.0
#define RMP400_KP_GYRO_THETA	10.0
#define RMP400_KI_GYRO_THETA	2.0

/* max errors */
#define RMP400_LONGIT_MAX_GAP	0.5
#define RMP400_TRANSV_MAX_GAP	0.5
#define RMP400_ANG_MAX_GAP	0.5

#define RMP400_LONGIT_ERROR_MAX	0.5
#define RMP400_TRANSV_ERROR_MAX	0.5
#define RMP400_ANG_ERROR_MAX	0.5

#define RMP400_LONGIT_MAX_ERR_SUM	1.0
#define RMP400_TRANSV_MAX_ERR_SUM	1.0
#define RMP400_ANG_MAX_ERR_SUM		1.0

/* Data Conversions */
/* Source: Interface Guide for Segway RMP 2.0.pdf,page 13 */
/* From RMP values to SI */
/* Minus signs are due to Mana going backward */
#define RMP400_CONVERT_PITCH(x) (-DEG_TO_RAD((x)/7.8))
#define RMP400_CONVERT_ROLL(x) (-DEG_TO_RAD((x)/7.8))
#define RMP400_CONVERT_YAW(x) (-DEG_TO_RAD((x)/7.8)) /* don't know why inverted */
#define RMP400_CONVERT_WHEEL_SPEED(x) (-((x)/332.0))
#define RMP400_CONVERT_INTEGRATED_POS(x) (-((x)/33215.0))
#define RMP400_CONVERT_INTEGRATED_TURN(x) (((x)*2*M_PI/112644.0))
#define RMP400_CONVERT_POWERBASE_VOLTAGE(x) ((x)/4.0)
#define RMP400_CONVERT_UI_VOLTAGE(x) (1.4+(x)*0.0125)
#define RMP400_CONVERT_TORQUE(x) (-(x)/1094.0)

/* From SI to RMP values */
#define RMP400_CONVERT_TO_RMP_SPEED(x)   (-(x)*332.0*1.29)
#define RMP400_CONVERT_FROM_RMP_SPEED(x)   (-(x)/332.0/1.29)
#define RMP400_CONVERT_TO_RMP_ANGLE(x)   (RAD_TO_DEG((x)*17.16))
#define RMP400_CONVERT_FROM_RMP_ANGLE(x)   (DEG_TO_RAD((x)/17.16))

/* joystick */
#define RMP400_JOYSTICK_DEVICE_PATH "/dev/input/js0"
#define RMP400_JOYSTICK_VGAIN	1./(32768.0*32768.0) /* 1 m/s max */
#define RMP400_JOYSTICK_WGAIN	1./(32768.0*32768.0) /* 1 rad/s max */

#endif
