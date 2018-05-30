/*
 * Copyright (c) 2009-2018 CNRS/LAAS
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
 ** rmp400MotionTaskCodels.c
 **
 ** Codels called by execution task rmp400MotionTask
 **
 ** Author: Matthieu Herrb
 ** Date: April 2009,
 **       updated for genom3, May 2018
 **
 **/

#include <sys/time.h>
#include <err.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rmp/rmpLib.h>
#include <fe/ftdi-emergency.h>
#include <gyroLib/gyro.h>

#include "acrmp400.h"

#include "rmp400_c_types.h"

#include "orMathLib.h"
#include "odoProba.h"
#include "codels.h"
#include "rmp400_Log.h"
#include "rmp400Const.h"

/* --- Task MotionTask -------------------------------------------------- */

static void
gyroUpdate(GYRO_DATA **gyroId, rmp400_gyro *gyro,
    rmp400_gyro_asserv *gyro_asserv, or_genpos_cart_state *robot)
{
	uint32_t date;
#if DEBUG>1
	static int count = 0;
#endif
	/* Gyro */
	if (gyroId != NULL /* && gyro->currentMode != RMP400_GYRO_OFF */) {
		if (gyroRead(*gyroId, &(gyro->gyroTheta),
			    &(gyro->gyroOmega), &date) != 0) {
			gyro->currentMode = rmp400_gyro_off;
			gyroEnd(*gyroId);
			*gyroId = NULL;
		}
		else {
		  gyro->gyroTheta = - gyro->gyroTheta;
		  gyro->gyroOmega = - gyro->gyroOmega;
		}
	}
	if (gyro->currentMode == rmp400_gyro_off ||
	    (gyro->currentMode == rmp400_gyro_on_if_motion
		&& fabs(robot->w) < 0.017
		&& fabs(robot->v) < 0.01)) {
		/* -- odo is reference */

		/* reset gyro offset to match odo */
		gyro->gyroToRobotOffset = angleLimit(robot->theta
		    - gyro->gyroTheta);
		/* switch back to odo */
		if (gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> odo (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = false;
		}
	} else {
		/* -- gyro is reference */
		robot->theta = angleLimit(gyro->gyroTheta +
		    gyro->gyroToRobotOffset);
		/* switch back to gyro */
		if (!gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> gyro (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = true;
			gyro_asserv->first = 1;
		}
	}

	/*debug */
#if DEBUG>1
	if (count==10) {
		printf ("MODE %s. gyro %5.3f + offset %5.3f = %5.3f =? %5.3f\n",
		    gyro->gyroOn ? "GYRO" : "ODO",
		    RAD_TO_DEG(gyro->gyroTheta),
		    RAD_TO_DEG(gyro->gyroToRobotOffset),
		    RAD_TO_DEG(angleLimit(gyro->gyroTheta +
			    gyro->gyroToRobotOffset)),
		    RAD_TO_DEG(robot->theta));
		count=0;
	}
	count++;
#endif
}

static void
yawToQuaternion(double yaw, or_t3d_pos *pos)
{
	pos->qw = cos(yaw * 0.5);
	pos->qx = 0.0;
	pos->qy = 0.0;
	pos->qz = sin(yaw * 0.5);
}

/*----------------------------------------------------------------------*/

/** Codel initOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_ether, rmp400_odo.
 * Throws rmp400_emergency_stop.
 */
genom_event
initOdoAndAsserv(rmp400_ids *ids,
		 const rmp400_StatusGeneric *StatusGeneric,
		 const genom_context self)
{
	rmp_status_str *statusgen = StatusGeneric->data(self);
	rmp400_kinematics_str *kinematics = &ids->kinematics;
	or_genpos_cart_state *robot = &ids->robot;
	or_genpos_cart_speed *ref = &ids->ref;
	rmp400_gyro *gyro = &ids->gyro;
	rmp400_gyro_asserv *gyro_asserv = &ids->gyro_asserv;
	rmp400_max_accel *max_accel = &ids->max_accel;

	memset(statusgen, 0, sizeof(rmp_status_str));
	statusgen->robot_model = rmp_model_400;

	ids->rs_mode = rmp400_mode_idle;

	/* Kinematics */
	kinematics->leftWheelRadius = RMP400_LEFT_WHEEL_RADIUS;
	kinematics->rightWheelRadius = RMP400_RIGHT_WHEEL_RADIUS;
	kinematics->axisWidth = RMP400_AXIS_WIDTH;

	/* configuration */
	robot->xRob = 0.;
	robot->yRob = 0.;
	robot->theta = 0.;
	robot->xRef = 0;
	robot->yRef = 0;
	robot->v = 0;
	robot->w = 0;

	/* Ref Data */
	ref->v = 0.;
	ref->w = 0.;
	/* XXX These 4 values are not used. */
	ref->vmax = RMP400_VMAX;
	ref->wmax = RMP400_WMAX;
	ref->linAccelMax = RMP400_ACCEL_LIN_MAX;
	ref->angAccelMax = RMP400_ACCEL_LIN_MAX;

	/* gyro */
	gyro->currentMode = rmp400_gyro_off;
	gyro->gyroOn = false;
	gyro->gyroToRobotOffset = 0.0;
	gyro->gyroTheta = 0.0;

	/* gyro asserv */
	gyro_asserv->enabled = 0;
	gyro_asserv->first = 1;
	gyro_asserv->straight = 0;

	/* max accel */
	max_accel->prev_vel_command = 0.;
	max_accel->prev_vel_command_t = -1.;

	return rmp400_odo;
}


/*----------------------------------------------------------------------*/

/** Codel odoAndAsserv of task MotionTask.
 *
 * Triggered by rmp400_odo.
 * Yields to rmp400_ether, rmp400_pause_odo, rmp400_end.
 * Throws rmp400_emergency_stop.
 */
genom_event
odoAndAsserv(RMP_DEV_STR *rmp[2],
	     const rmp400_kinematics_str *kinematics,
	     const rmp400_var_params *var_params,
	     const rmp400_log_str *log,
	     const rmp400_Joystick *Joystick, GYRO_DATA **gyroId,
	     FE_STR **fe, or_genpos_cart_state *robot,
	     or_genpos_cart_config_var *var, or_genpos_cart_speed *ref,
	     rmp400_max_accel *max_accel, rmp400_data_str rs_data[2],
	     rmp400_mode *rs_mode, rmp400_gyro *gyro,
	     rmp400_gyro_asserv *gyro_asserv, const rmp400_Pose *Pose,
	     const rmp400_Status *Status,
	     const rmp400_StatusGeneric *StatusGeneric,
	     const genom_context self)
{
	struct timespec ts;
	rmp400_status_str *status = Status->data(self);
	rmp_status_str *statusgen = StatusGeneric->data(self);
	or_pose_estimator_state *pose = Pose->data(self);
	genom_event report = genom_ok;
	struct cmd_str cmd;

	rmp400DataUpdate(rmp, kinematics, status, statusgen);
	rmp400VelocityGet(rs_data, kinematics, robot);

	robot->xRef = robot->xRob;
	robot->yRef = robot->yRob;

	odoProba(robot, var,
	    kinematics->axisWidth, var_params->coeffLinAng,
	    rmp400_sec_period);	/* XXX could use the actual measured period */

	gyroUpdate(gyroId, gyro, gyro_asserv, robot);

	/* fill pose */
	clock_gettime(CLOCK_MONOTONIC, &ts);
	pose->ts.sec = ts.tv_sec;
	pose->ts.nsec = ts.tv_nsec;
	pose->intrinsic = true;
	pose->pos._present = true;
	pose->pos._value.x = robot->xRob;
	pose->pos._value.y = robot->yRob;
	pose->pos._value.z = 0.0;	/* XXX */
	yawToQuaternion(robot->theta, &pose->pos._value); /* XXX */
	pose->vel._present = true;
	pose->vel._value.vx = robot->v;
	pose->vel._value.vy = 0;
	pose->vel._value.vz = 0;
	pose->vel._value.wx = 0;	/* XXX */
	pose->vel._value.wy = 0;	/* XXX */
	pose->vel._value.wz = robot->w;

	/*
	 * Asserv
	 */
	switch (*rs_mode) {
	case rmp400_mode_motors_off:
		/* No motion possible */
		return rmp400_pause_odo;

	case rmp400_mode_emergency:
	case rmp400_mode_idle:
		ref->v = 0.0;
		ref->w = 0.0;
		break;

	case rmp400_mode_manual:
		Joystick->read(self);
		getJoystickSpeeds(Joystick->data(self),
		    &ref->v, &ref->w, &ref->linAccelMax, &ref->angAccelMax);
		break;

	case rmp400_mode_track:
		/* ref has been updated by the trackTask */
		break;

	default:
		printf("-- invalid_rs_mode %d\n", *rs_mode);
		return rmp400_end;
	}
	if (report != genom_ok) {
		/* In case an error occured,
		   stop the robot and the tracking */
		*rs_mode = statusgen->rs_mode = rmp400_mode_idle;
		ref->v = 0;
		ref->w = 0;
		return report;
	}

	/* keep theoretical values */
	cmd.vReference = ref->v;
	cmd.wReference = ref->w;

	/* Adjustements depending on the robot */
	double t = ts.tv_sec + ts.tv_nsec*1e-9;
	if (*rs_mode == rmp400_mode_track)
		bound_accels(max_accel, t, &ref->v, &ref->w);

	if (gyro->gyroOn)
		control_yaw(gyro_asserv, t, ref->v, ref->w,
		    gyro->gyroOmega, gyro->gyroTheta, &ref->w);

	/* Send  commands to the wheels */
	report = rmp400VelocitySet(rmp, &cmd, self);

	/* log */
	if (log != NULL)
		rmp400LogData(log, pose, gyro, gyro_asserv, &cmd, rs_data);


	/* Publish */
	Pose->write(self);
	Status->write(self);
	StatusGeneric->write(self);

#if 0
	if (report != genom_ok)
		return report;
#endif
	return rmp400_pause_odo;
}


/** Codel endOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp400_end.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop.
 */
genom_event
endOdoAndAsserv(RMP_DEV_STR *rmp[2], rmp400_data_str rs_data[2],
		const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
}


/* --- Activity Init ---------------------------------------------------- */

/** Codel rmp400InitStart of activity Init.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_init_main.
 * Throws rmp400_emergency_stop, rmp400_already_initialized,
 *        rmp400_malloc_error, rmp400_rmplib_error.
 */
genom_event
rmp400InitStart(RMP_DEV_STR *rmp[2], FE_STR **fe,
		rmp400_data_str rs_data[2], const genom_context self)
{
	pthread_t tid;
	int i, n;

	/* connect emergency stop */
	*fe = fe_init(NULL);
	if (fe == NULL)
		return rmp400_malloc_error(self);

	if ((n = rmpOpenAll(&rmp)) < 0) {
		fe_end(*fe);
		*fe = NULL;
		return rmp400_rmplib_error(self);
	}
	printf("rmpOpenAll ok\n");
	/* start read tasks */
	for (i = 0; i < 2; i++) {
		if (pthread_create(&tid, NULL, rmp400ReadTask, rmp[i]) < 0) {
			warn("pthread_create rmp400ReadTask");
			fe_end(*fe);
			*fe = NULL;
			for (; i >= 0; i--)
				rmpClose(rmp[i]);
			free(rmp);
			return rmp400_rmplib_error(self);
		}
		pthread_detach(tid);
	}
	for (i = 0; i < 2; i++)
		if (rmpPowerOn(rmp[i]) < 0)
			return rmp400_rmplib_error(self);
	return rmp400_init_main;
}

/** Codel rmp400InitMain of activity Init.
 *
 * Triggered by rmp400_init_main.
 * Yields to rmp400_pause_init_main, rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_already_initialized,
 *        rmp400_malloc_error, rmp400_rmplib_error.
 */
genom_event
rmp400InitMain(RMP_DEV_STR *rmp[2], FE_STR **fe,
	       rmp400_data_str rs_data[2], rmp400_mode *rs_mode,
	       rmp400_dynamic_str *dynamics,
	       rmp400_kinematics_str *kinematics,
	       const genom_context self)
{
	/* Check motors status */
	*rs_mode = rmp400_mode_idle;
	return rmp400_ether;
}


/* --- Activity JoystickOn ---------------------------------------------- */

/** Codel rmp400JoystickOnStart of activity JoystickOn.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_js_main.
 * Throws rmp400_emergency_stop, rmp400_bad_ref, rmp400_rmplib_error,
 *        rmp400_joystick_error, rmp400_motors_off,
 *        rmp400_power_cord_connected.
 */
genom_event
rmp400JoystickOnStart(const rmp400_Joystick *Joystick,
		      rmp400_mode *rs_mode, const genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp400_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp400_joystick_error(self);
	}
	*rs_mode = rmp400_mode_manual;

	return rmp400_js_main;

}

/** Codel rmp400JoystickOnMain of activity JoystickOn.
 *
 * Triggered by rmp400_js_main.
 * Yields to rmp400_pause_js_main, rmp400_inter.
 * Throws rmp400_emergency_stop, rmp400_bad_ref, rmp400_rmplib_error,
 *        rmp400_joystick_error, rmp400_motors_off,
 *        rmp400_power_cord_connected.
 */
genom_event
rmp400JoystickOnMain(const rmp400_Joystick *Joystick,
		     rmp400_mode rs_mode, const genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp400_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp400_joystick_error(self);
	}

	/* Check if mode changed */
	switch (rs_mode) {
	case rmp400_mode_power_coord:
		return rmp400_power_cord_connected(self);
	case rmp400_mode_emergency:
		return  rmp400_emergency_stop(self);
	case rmp400_mode_motors_off:
		return rmp400_motors_off(self);
	default:
		break;
	}
	/* Check for user abort */
	if (joystickQuit(joy)) {
		printf("Stop joystick\n");
		return rmp400_inter;
	}
	return rmp400_pause_js_main;
}

/** Codel rmp400JoystickOnInter of activity JoystickOn.
 *
 * Triggered by rmp400_inter.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_bad_ref, rmp400_rmplib_error,
 *        rmp400_joystick_error, rmp400_motors_off,
 *        rmp400_power_cord_connected.
 */
genom_event
rmp400JoystickOnInter(rmp400_mode *rs_mode, or_genpos_cart_speed *ref,
		      const genom_context self)
{

	printf("%s\n", __func__);
	*rs_mode = rmp400_mode_idle; /* XXXXXX */
	return rmp400_ether;
}


/* --- Activity Gyro ---------------------------------------------------- */

/** Codel rmp400GyroExec of activity Gyro.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_gyro_error.
 */
genom_event
rmp400GyroExec(const rmp400_gyro_params *params,
	       const or_genpos_cart_state *robot, rmp400_gyro *gyro,
	       GYRO_DATA **gyroId, const genom_context self)
{

	if (*gyroId == NULL) {
		*gyroId = gyroInit(params->type, params->port,
			    params->latitude, params->woffset);
		if (*gyroId == NULL) {
			gyro->currentMode = rmp400_gyro_off;
			return rmp400_gyro_error(self);
		}
	}

	/* read gyro once */
	if (gyroReadAngle(*gyroId, &gyro->gyroTheta) != 0) {
		gyro->currentMode = rmp400_gyro_off;
		gyroEnd(*gyroId);
		*gyroId = NULL;
		return rmp400_gyro_error(self);
	} else
		gyro->gyroTheta = - gyro->gyroTheta;
	/* reset gyro offset to match odo */
	gyro->gyroToRobotOffset = robot->theta - gyro->gyroTheta;

	/* Finally set gyro mode */
	gyro->currentMode = params->mode;
	return rmp400_ether;
}
