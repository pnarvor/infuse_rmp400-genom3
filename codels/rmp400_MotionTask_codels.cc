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

extern "C" {
#include <rmp/rmpLib.h>
#include <fe/ftdi-emergency.h>
#include <gyroLib/gyro.h>
}

#include <MTI-clients/MTI.h>

#include "acrmp400.h"

#include "rmp400_c_types.h"

#include "orMathLib.h"
#include "odoProba.h"
#include "codels.h"
#include "rmp400_Log.h"
#include "rmp400Const.h"
#include "odo3d.h"

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

static void
eulerToQuaternion(double roll, double pitch, double yaw, or_t3d_pos *pos)
{
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	pos->qw = cy * cr * cp + sy * sr * sp;
	pos->qx = cy * sr * cp - sy * cr * sp;
	pos->qy = cy * cr * sp + sy * sr * cp;
	pos->qz = sy * cr * cp - cy * sr * sp;
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
	or_genpos_cart_3dstate *robot3d = &ids->robot3d;
	or_genpos_cart_speed *ref = &ids->ref;
	rmp400_gyro *gyro = &ids->gyro;
	rmp400_gyro_asserv *gyro_asserv = &ids->gyro_asserv;
	rmp400_max_accel *max_accel = &ids->max_accel;

	ids->rmp = NULL;
	memset(statusgen, 0, sizeof(rmp_status_str));
	statusgen->robot_model = rmp_model_400;

	ids->rs_mode = rmp400_mode_idle;
	memset(&ids->rs_data[0], 0, sizeof(struct rmp400_status_str));
	memset(&ids->rs_data[1], 0, sizeof(struct rmp400_status_str));

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

    /* MTI */
    ids->mtiHandle = NULL;

    ids->mti.mtiOn = false; 
    ids->mti.currentMode = rmp400_mti_off;
    ids->mti.data.acc[0] = 0.0;
    ids->mti.data.acc[1] = 0.0;
    ids->mti.data.acc[2] = 0.0;

    ids->mti.data.gyr[0] = 0.0;
    ids->mti.data.gyr[1] = 0.0;
    ids->mti.data.gyr[2] = 0.0;

    ids->mti.data.mag[0] = 0.0;
    ids->mti.data.mag[1] = 0.0;
    ids->mti.data.mag[2] = 0.0;

    ids->mti.data.euler[0] = 0.0;
    ids->mti.data.euler[1] = 0.0;
    ids->mti.data.euler[2] = 0.0;

    ids->mti.data.count = 0;
    
    ids->mti.data.timeStampRaw       = 0.0;
    ids->mti.data.timeStampUndelayed = 0.0;
    ids->mti.data.timeStampFiltered  = 0.0;

    ids->odoMode = rmp400_odometry_2d;

	robot3d->xRef  = 0.;
	robot3d->yRef  = 0.;
	robot3d->zRef  = 0.;
	robot3d->xRob  = 0.;
	robot3d->yRob  = 0.;
	robot3d->zRob  = 0.;
	robot3d->roll  = 0.;
	robot3d->pitch = 0.;
	robot3d->theta = 0.;
	robot3d->v     = 0.;
	robot3d->vt    = 0.;
	robot3d->w     = 0.;

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
odoAndAsserv(RMP_DEV_TAB **rmp, FE_STR **fe,
             const rmp400_kinematics_str *kinematics,
             const rmp400_var_params *var_params,
             const rmp400_log_str *log,
             const rmp400_Joystick *Joystick, GYRO_DATA **gyroId,
             or_genpos_cart_state *robot,
             or_genpos_cart_config_var *var, or_genpos_cart_speed *ref,
             rmp400_max_accel *max_accel, rmp400_data_str rs_data[2],
             rmp400_mode *rs_mode, rmp400_gyro *gyro,
             rmp400_gyro_asserv *gyro_asserv, MTI_DATA **mtiHandle,
             rmp400_mti *mti, rmp400_odometry_mode *odoMode,
             or_genpos_cart_3dstate *robot3d, const rmp400_Pose *Pose,
             const rmp400_Status *Status,
             const rmp400_StatusGeneric *StatusGeneric,
             const genom_context self)
{
	struct timespec ts;
	struct RMP_DEV_STR **r;
	rmp400_status_str *status = Status->data(self);
	rmp_status_str *statusgen = StatusGeneric->data(self);
	or_pose_estimator_state *pose = Pose->data(self);
	genom_event report = genom_ok;
	struct cmd_str cmd;
	double t;
	int i;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	t = ts.tv_sec + ts.tv_nsec*1e-9;
	memset(pose, 0, sizeof(or_pose_estimator_state));
	pose->ts.sec = ts.tv_sec;
	pose->ts.nsec = ts.tv_nsec;

	if (*rmp != NULL)
		r = (*rmp)->dev;
	else
		goto publish;
	
	if (r[0] == NULL || r[1] == NULL)
		goto publish; /* not initialized yet */

	rmp400DataUpdate(r, *fe, kinematics, rs_data, rs_mode);
	rmp400VelocityGet(rs_data, kinematics, robot);
	robot->xRef = robot->xRob;
	robot->yRef = robot->yRob;

	odoProba(robot, var,
	    kinematics->axisWidth, var_params->coeffLinAng,
	    rmp400_sec_period);	/* XXX could use the actual measured period */
	gyroUpdate(gyroId, gyro, gyro_asserv, robot);

    ////////////////////////////////////////////////////////////////////////
 
    if(*odoMode == rmp400_odometry_3d)
    {
        double measuredPeriod = -1.0;
        struct timeval tvLast, tvNew;
        if(measuredPeriod < 0)
        {
            gettimeofday(&tvLast, NULL);
            tvNew = tvLast;
            measuredPeriod = 0.0;
        }
        else
        {
            gettimeofday(&tvNew, NULL);
            measuredPeriod = (double)tvNew.tv_sec - (double)tvLast.tv_sec
                + 1.0e-6*((double)tvNew.tv_sec - (double)tvLast.tv_sec);
            tvLast = tvNew;
        }

        if(rmp400odo3d(mtiHandle, mti, robot, robot3d, odoMode, rmp400_sec_period))
        {
            printf("acc  : %2.2f %2.2f %2.2f\ngyr  : %2.2f %2.2f %2.2f\nmag  : %2.2f %2.2f %2.2f\neuler: %2.2f %2.2f %2.2f\nperiod: %2.2lf\n\n",
                mti->data.acc[0], 
                mti->data.acc[1], 
                mti->data.acc[2], 
                mti->data.gyr[0], 
                mti->data.gyr[1], 
                mti->data.gyr[2], 
                mti->data.mag[0], 
                mti->data.mag[1], 
                mti->data.mag[2],
                mti->data.euler[0], 
                mti->data.euler[1], 
                mti->data.euler[2],
                rmp400_sec_period);
            //fflush(stdout);
            printf("euler rpy: %2.2f %2.2f %2.2f\nperiods : %2.2lf %2.2lf\n\n",
                robot3d->roll, 
                robot3d->pitch, 
                robot3d->theta,
                rmp400_sec_period,
                measuredPeriod);
            fflush(stdout);
        }
    }

    ////////////////////////////////////////////////////////////////////////

	/* fill pose */
	pose->intrinsic = true;
	pose->pos._present = true;

    if(*odoMode != rmp400_odometry_3d)
    {
	    pose->pos._value.x = robot->xRob;
	    pose->pos._value.y = robot->yRob;
	    pose->pos._value.z = 0.0; 	/* XXX */
	    yawToQuaternion(robot->theta, &pose->pos._value); /* XXX */
    }
    else
    {
	    pose->pos._value.x = robot3d->xRob;
	    pose->pos._value.y = robot3d->yRob;
	    pose->pos._value.z = robot3d->zRob;
	    eulerToQuaternion(robot3d->roll,
                            robot3d->pitch,
                            robot3d->theta,
                            &pose->pos._value);
    }
	
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
		*rs_mode = rmp400_mode_idle;
		ref->v = 0;
		ref->w = 0;
		return report;
	}

	/* keep theoretical values */
	cmd.vReference = ref->v;
	cmd.wReference = ref->w;

	/* Adjustements depending on the robot */
	if (*rs_mode == rmp400_mode_track)
		bound_accels(max_accel, t, &ref->v, &ref->w);

	if (gyro->gyroOn)
		control_yaw(gyro_asserv, t, ref->v, ref->w,
		    gyro->gyroOmega, gyro->gyroTheta, &ref->w);

	/* Send  commands to the wheels */
	report = rmp400VelocitySet(r, &cmd, self);

	/* log */
	if (log != NULL)
		rmp400LogData(log, pose, gyro, gyro_asserv, &cmd, rs_data);

publish:
	/* Publish */
	Pose->write(self);

	for (i = 0; i < 2; i++) {
		memcpy(&status->rs_data[i], &rs_data[i],
		    sizeof(rmp400_data_str));
	}
	status->rs_mode = *rs_mode;
	Status->write(self);
	statusgen->receive_date = t;
	rmp400StatusgenUpdate(status, kinematics, statusgen);
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
endOdoAndAsserv(RMP_DEV_TAB **rmp, rmp400_data_str rs_data[2],
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
 *        rmp400_malloc_error, rmp400_felib_error,
 *        rmp400_rmplib_error.
 */
genom_event
rmp400InitStart(RMP_DEV_TAB **rmp, FE_STR **fe,
                rmp400_data_str rs_data[2], const genom_context self)
{
	struct RMP_DEV_STR **r;
	pthread_t tid;
	int i, n;

	*rmp = (RMP_DEV_TAB*)malloc(sizeof(struct RMP_DEV_TAB));
	if (*rmp == NULL)
		return rmp400_malloc_error(self);

	/* connect emergency stop */
	*fe = fe_init(NULL);
	if (*fe == NULL)
		return rmp400_felib_error(self);

	if ((n = rmpOpenAll(&r)) != 2) {
		printf("%s: rmpOpenAll failed: %d\n", __func__, n);
		fe_end(*fe);
		*fe = NULL;
		return rmp400_rmplib_error(self);
	}
	printf("rmpOpenAll ok\n");
	/* start read tasks */
	for (i = 0; i < 2; i++) {
		(*rmp)->dev[i] = r[i];
		if (pthread_create(&tid, NULL, rmp400ReadTask, r[i]) < 0) {
			warn("pthread_create rmp400ReadTask");
			fe_end(*fe);
			*fe = NULL;
			for (; i >= 0; i--)
				rmpClose(r[i]);
			free(*rmp);
			*rmp = NULL;
			return rmp400_rmplib_error(self);
		}
		pthread_detach(tid);
	}
	free(r);
	for (i = 0; i < 2; i++)
		if (rmpPowerOn((*rmp)->dev[i]) != 0) {
			printf("%s: failed to power on motor %d\n",
			    __func__, i);
			return rmp400_rmplib_error(self);
		}
	return rmp400_init_main;
}

/** Codel rmp400InitMain of activity Init.
 *
 * Triggered by rmp400_init_main.
 * Yields to rmp400_pause_init_main, rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_already_initialized,
 *        rmp400_malloc_error, rmp400_felib_error,
 *        rmp400_rmplib_error.
 */
genom_event
rmp400InitMain(RMP_DEV_TAB **rmp, FE_STR **fe,
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


/* --- Activity GyroBiasUpdate ------------------------------------------ */

/** Codel rmp400GyroBiasUpdate of activity GyroBiasUpdate.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_gyro_error.
 */
genom_event
rmp400GyroBiasUpdate(int32_t nbMeasures,
                     const or_genpos_cart_state *robot,
                     rmp400_gyro *gyro, GYRO_DATA **gyroId,
                     const genom_context self)
{
    if(gyro->currentMode == rmp400_gyro_off || *gyroId == NULL)
    {
        printf("Error gyroBiasEstimate : gyro must be initialized\n");
        return rmp400_gyro_error(self);
    }

	if (gyroUpdateWOffset(*gyroId, nbMeasures) != 0)
        return rmp400_gyro_error(self);

    return rmp400_ether;
}


/* --- Activity InitMTI ------------------------------------------------- */

/** Codel rmp400MTIopen of activity InitMTI.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_mti_error.
 */
genom_event
rmp400MTIopen(const rmp400_mti_params *params, MTI_DATA **mtiHandle,
              rmp400_mti *mti, rmp400_mti_config *mtiConfig,
              const genom_context self)
{
    MTI* mtiHandleP = (MTI*)*mtiHandle;
    if(mtiHandle == NULL)
        return rmp400_mti_error(self);

    // Checking input parameters
    switch(params->outputMode)
    {
        default:
            printf("Error MTI open : invalid outputMode parameter. Candidates are:\n\
                    MTI_OPMODE_CALIBRATED  = 2\n\
                    MTI_OPMODE_ORIENTATION = 4\n\
                    MTI_OPMODE_BOTH        = 6\n");
            return rmp400_mti_error(self);
            break;
        case (int)MTI_OPMODE_CALIBRATED:
            break;
        case (int)MTI_OPMODE_ORIENTATION:
            break;
        case (int)MTI_OPMODE_BOTH:
            break;
    }
    switch(params->outputFormat)
    {
        default:
            printf("Error MTI open : invalid outputFormat parameter. Candidates are:\n\
                    MTI_OPFORMAT_QUAT  = 0\n\
                    MTI_OPFORMAT_EULER = 4\n\
                    MTI_OPFORMAT_MAT   = 8\n");
            return rmp400_mti_error(self);
            break;
        case (int)MTI_OPFORMAT_QUAT:
            break;
        case (int)MTI_OPFORMAT_EULER:
            break;
        case (int)MTI_OPFORMAT_MAT:
            break;
    }

    // Config copied from MTIinitExec codel in robotpkg/localization/MTI
    mtiConfig->outputMode           = (OutputMode)params->outputMode;
    mtiConfig->outputFormat         = (OutputFormat)params->outputFormat;
    mtiConfig->syncOutMode          = MTI_SYNCOUTMODE_DISABLED;
    mtiConfig->syncOutPulsePolarity = MTI_SYNCOUTPULSE_POS;
    mtiConfig->syncOutSkipFactor    = 0;
    mtiConfig->syncOutOffset        = 0;
    mtiConfig->syncOutPulseWidth    = 29498; //1ms pulse

    //// Same config as mtiTest
    //mtiHandleP = new MTI(params->port,
    //    (OutputMode)mtiConfig->outputMode,
    //    (OutputFormat)mtiConfig->outputFormat,
    //    MTI_SYNCOUTMODE_DISABLED);
        
    
    mtiHandleP = new MTI(params->port,
        (OutputMode)mtiConfig->outputMode,
        (OutputFormat)mtiConfig->outputFormat);
    if(!mtiHandleP)
        return rmp400_mti_error(self);

    if(!mtiHandleP->set_syncOut((SyncOutMode)mtiConfig->syncOutMode,
        (SyncOutPulsePolarity)mtiConfig->syncOutPulsePolarity,
        mtiConfig->syncOutSkipFactor,
        mtiConfig->syncOutOffset,
        mtiConfig->syncOutPulseWidth))
    {
        printf("Error MTI open : set_SyncOut failed\n");
        delete mtiHandleP;
        return rmp400_mti_error(self);
    }
    
    *mtiHandle = (MTI_DATA*)mtiHandleP;
    mti->currentMode = params->mode;

    return rmp400_ether;
}


/* --- Activity ToggleOdometryMode -------------------------------------- */

/** Codel rmp400ToggleOdoMode of activity ToggleOdometryMode.
 *
 * Triggered by rmp400_start.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_odo3d_error.
 */
genom_event
rmp400ToggleOdoMode(MTI_DATA **mtiHandle, rmp400_mti *mti,
                    rmp400_odometry_mode *odoMode,
                    const genom_context self)
{
    MTI* mtiHandleP;
    if(*odoMode == rmp400_odometry_2d)
    {
        if(!mtiHandle)
            return rmp400_odo3d_error(self);
        mtiHandleP = (MTI*)*mtiHandle;
        if(!mtiHandleP)
        {
            printf("Error toggleOdoMode, did you initialize the mti ?\n");
            return rmp400_odo3d_error(self);
        }
        // read once to check if ok
        if(!mtiHandleP->read((INERTIAL_DATA*)(&mti->data),false))
            return rmp400_odo3d_error(self);

        *odoMode = rmp400_odometry_3d;
    }
    else
    {
        //carefull with reinit...
        *odoMode = rmp400_odometry_2d;
    }
    return rmp400_ether;
}
