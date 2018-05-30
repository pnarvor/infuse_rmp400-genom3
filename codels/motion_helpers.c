/*
 * Copyright (c) 2009-2017 CNRS/LAAS
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
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rmp/rmpLib.h>
#include <fe/ftdi-emergency.h>

#include "acrmp400.h"

#include "rmp400_c_types.h"

#include "orMathLib.h"
#include "codels.h"
#include "rmp400Const.h"

double convertBattery(double voltage)
{
	int i;
	double calib[14][2] = {
		{   0., 0.00 },
		{   0., 2.90 },
		{   5., 3.20 },
		{  10., 3.50 },
		{  20., 3.60 },
		{  30., 3.65 },
		{  40., 3.69 },
		{  50., 3.73 },
		{  60., 3.77 },
		{  70., 3.82 },
		{  80., 3.87 },
		{  90., 4.00 },
		{ 100., 4.10 },
		{ 100., 9.99 }};
	double element = voltage / 20;

	for(i = 1; i < 14-1; ++i)
	{
		if (element < calib[i][1])
			return calib[i-1][0] + (calib[i][0]-calib[i-1][0]) * (element-calib[i-1][1])/(calib[i][1]-calib[i-1][1]);
	}
	return 100.;
}


/*----------------------------------------------------------------------*/

/*
 * rmp400DataUpdate - copy data from 2 motors block to the IDS
 */
extern void
rmp400DataUpdate(RMP_DEV_STR **rmp, const rmp400_kinematics_str *kinematics,
    rmp400_status_str *status, rmp_status_str *statusgen)
{
	static int noData[2] = {0, 0};
	struct timeval tv;
	gettimeofday(&tv, NULL); // FIXME should be measured more thoroughly in rmp-libs and with blocking reads
	double receive_date = tv.tv_sec + tv.tv_usec*1e-6;

	struct RMP_DATA_STR *data;
	rmp400_data_str *dest;
	int i;

	for (i = 0; i < 2; i++) {
		data = rmpGetData(rmp[i]);
		if (data == NULL) {
			fprintf(stderr, "NULL state str %d\n", i);
			continue;
		}
#if 0
		if (data->servo_frames == prevFrame[i]) {
			fprintf(stderr,
				"Data not refreshed by rmp %d: %d\n",
				i, data->servo_frames);
			return -1;
		}
		prevFrame[i] = data->servo_frames;
#endif
		dest = &status->rs_data[i];

		pthread_mutex_lock(rmpGetMutex(rmp[i]));

		if (data->operational_mode != 1) {
			pthread_mutex_unlock(rmpGetMutex(rmp[i]));
			dest->operational_mode = data->operational_mode;
			noData[i]++;
			break;
		}
		noData[i] = 0;
		dest->pitch_angle = RMP400_CONVERT_PITCH(data->pitch_angle);
		dest->pitch_rate = RMP400_CONVERT_PITCH(data->pitch_rate);
		dest->roll_angle = RMP400_CONVERT_ROLL(data->roll_angle);
		dest->roll_rate = RMP400_CONVERT_ROLL(data->roll_rate);
		dest->lw_velocity = RMP400_CONVERT_WHEEL_SPEED(data->lw_velocity);
		dest->rw_velocity = RMP400_CONVERT_WHEEL_SPEED(data->rw_velocity);
		dest->yaw_rate = RMP400_CONVERT_YAW(data->yaw_rate);
		dest->servo_frames = data->servo_frames;
		dest->integrated_left_wheel =
		   RMP400_CONVERT_INTEGRATED_POS(data->integrated_left_wheel);
		dest->integrated_right_wheel =
		   RMP400_CONVERT_INTEGRATED_POS(data->integrated_right_wheel);
		dest->integrated_fore_aft =
		   RMP400_CONVERT_INTEGRATED_POS(data->integrated_fore_aft);
		dest->integrated_yaw =
		   RMP400_CONVERT_INTEGRATED_TURN(data->integrated_yaw);
		dest->left_torque = RMP400_CONVERT_TORQUE(data->left_torque);
		dest->right_torque = RMP400_CONVERT_TORQUE(data->right_torque);
		dest->operational_mode = data->operational_mode;
		dest->controller_gain_schedule =
		    data->controller_gain_schedule;
		dest->ui_voltage = RMP400_CONVERT_UI_VOLTAGE(data->ui_voltage);
		dest->powerbase_voltage =
		    RMP400_CONVERT_POWERBASE_VOLTAGE(data->powerbase_voltage);
		dest->battery_charge = convertBattery(dest->powerbase_voltage);
		dest->velocity_command = RMP400_CONVERT_FROM_RMP_SPEED(data->velocity_command); /* ?? */
		dest->turn_command = RMP400_CONVERT_FROM_RMP_ANGLE(data->turn_command); /* ?? */

		/* Corrections if wheel radius changed */
		dest->lw_velocity *= (kinematics->leftWheelRadius
		    / RMP400_LEFT_WHEEL_RADIUS);
		dest->lw_velocity *= (kinematics->rightWheelRadius
		    / RMP400_RIGHT_WHEEL_RADIUS);
		dest->integrated_yaw *= (((kinematics->leftWheelRadius
			/ RMP400_LEFT_WHEEL_RADIUS) +
		    (kinematics->rightWheelRadius
			/ RMP400_RIGHT_WHEEL_RADIUS))/2.0);

		pthread_mutex_unlock(rmpGetMutex(rmp[i]));
	}
	/* If not data for several periods - set motors off */
	if (status->rs_mode != rmp400_mode_motors_off &&
	    (noData[0] > 10 || noData[1] > 10)) {
		printf("No data - Motors OFF %d %d?\n",
		       status->rs_data[0].operational_mode,
		       status->rs_data[1].operational_mode);

		status->rs_mode = statusgen->rs_mode = rmp400_mode_motors_off;
	}

	statusgen->receive_date = receive_date;
	statusgen->propulsion_battery_level = fmin(status->rs_data[0].battery_charge, status->rs_data[1].battery_charge);
	statusgen->aux_battery_level = -1.;
	statusgen->pitch = (status->rs_data[0].pitch_angle + status->rs_data[1].pitch_angle)/2.0;
	statusgen->roll = (status->rs_data[0].roll_angle + status->rs_data[1].roll_angle)/2.0;
	statusgen->yaw_rate = (status->rs_data[0].yaw_rate + status->rs_data[1].yaw_rate)/2.0;
	statusgen->v = (status->rs_data[0].lw_velocity + status->rs_data[1].lw_velocity + status->rs_data[0].rw_velocity + status->rs_data[1].rw_velocity)/4.0;
	statusgen->w = ((status->rs_data[0].lw_velocity + status->rs_data[1].lw_velocity) - (status->rs_data[0].rw_velocity + status->rs_data[1].rw_velocity))/(2.*kinematics->axisWidth);
	statusgen->v_target = (status->rs_data[0].velocity_command + status->rs_data[1].velocity_command) / 2.0;
	statusgen->w_target = (status->rs_data[0].turn_command + status->rs_data[1].turn_command) / 2.0;
	statusgen->right_front_vel = status->rs_data[0].rw_velocity;
	statusgen->left_front_vel = status->rs_data[0].lw_velocity;
	statusgen->right_rear_vel = status->rs_data[1].rw_velocity;
	statusgen->left_rear_vel = status->rs_data[1].lw_velocity;
	statusgen->right_front_pos = status->rs_data[0].integrated_right_wheel;
	statusgen->left_front_pos = status->rs_data[0].integrated_left_wheel;
	statusgen->right_rear_pos = status->rs_data[1].integrated_right_wheel;
	statusgen->left_rear_pos = status->rs_data[1].integrated_left_wheel;
	statusgen->right_front_torque = status->rs_data[0].right_torque;
	statusgen->left_front_torque = status->rs_data[0].left_torque;
	statusgen->right_rear_torque = status->rs_data[1].right_torque;
	statusgen->left_rear_torque = status->rs_data[1].left_torque;

#ifdef WITH_FELIB
	/* Check emergency stop */
	fe_get_status(fe);
	/* ignore pause if motors are off */
	if ((fe_pins(fe) & FE_PAUSE) != 0 &&
	    status->rs_mode != RMP400_EMERGENCY &&
	    status->rs_mode != RMP400_MOTORS_OFF) {
		printf("Emergency pause!\n");
		if (status->rs_mode != RMP400_MOTORS_OFF)
			status->rs_mode = statusgen->rs_mode = RMP400_EMERGENCY;
	}
#endif
	return;
}


/*----------------------------------------------------------------------*/
/*
 * Compute speed
 *
 * sets robot.v and robot.w in the SDI from motion in the motor structs.
 */


void
rmp400VelocityGet(rmp400_data_str rs_data[2],
    const rmp400_kinematics_str *kinematics,
    or_genpos_cart_state *robot)
{
	double leftv, rightv;

	/* Mean angular speed of front and back wheels */
	leftv = (rs_data[0].lw_velocity + rs_data[1].lw_velocity) / 2.0;
	rightv = (rs_data[0].rw_velocity + rs_data[1].rw_velocity) / 2.0;
	/* printf("%.2lf %.2lf\n", leftv, rightv); */

	/* Angular speed of the robot */
#ifdef USE_INTERNAL_GYRO
	robot->w = (rs_data[0].yaw_rate + rs_data[1].yaw_rate) / 2.0;
#else
	robot->w = (leftv - rightv) / kinematics->axisWidth;
#endif
	/* linear speeds */
	robot->v = (leftv + rightv) / 2.0;

	return;
}

/*----------------------------------------------------------------------*/
/*
 * Send given linear and angular speeds to the hardware
 *
 */
genom_event
rmp400VelocitySet(RMP_DEV_STR **rmp, struct cmd_str *cmd,
    const genom_context self)
{
	int i;
	double v = cmd->vReference;
	double w = cmd->wReference;

	/* Last safety check, in case uninitialized values are used */
	if (fabs(v) > RMP400_VMAX) {
		fprintf(stderr, "WARNING: v > VMAX: %lf\n", v);
		v = v > 0.0 ? RMP400_VMAX : -RMP400_VMAX;
	}
	if (fabs(w) > RMP400_WMAX) {
		fprintf(stderr, "WARNING: w > WMAX: %lf\n", w);
		w = w > 0.0 ? RMP400_WMAX : -RMP400_WMAX;
	}

	/* Send the same speed to front and back axis */
	for (i = 0; i < 2; i++) {
		if (rmpSendSpeed(rmp[i],
			    RMP400_CONVERT_TO_RMP_SPEED(v),
			    RMP400_CONVERT_TO_RMP_ANGLE(w)) < 0) {
			return rmp400_rmplib_error(self);
		}
	}

	/* Return the set velocities */
	cmd->vCommand = v;
	cmd->wCommand = w;

	return genom_ok;
}


/*----------------------------------------------------------------------*/

/*
 * Filter velocities to keep accelerations within max bounds
 *
 * don't use real time but rather module's period, so that if some periods
 * are lost it doesn't accelerate too much afterwards
 *
 * When moving along an arc (keep_radius = true) adjust angular speed
 * to respect the circle's radius
 */
void
bound_accels(rmp400_max_accel *acc, double t,
    double *vel_reference, double *ang_reference)
{
	const double epsilon = 1e-3;
	int keep_radius = (fabs(*ang_reference) > epsilon &&
	    fabs(*vel_reference) > epsilon);
	double radius;

	if (keep_radius)
		radius = *vel_reference / *ang_reference;

	// linear velocity
	double sign_accel = *vel_reference > acc->prev_vel_command ?
	    +1.0 : -1.0;
	if (acc->prev_vel_command * *vel_reference < 0)
		*vel_reference = 0.0;

	double max_accel = fabs(acc->prev_vel_command) < fabs(*vel_reference) ?
	    RMP400_ACCEL_LIN_MAX : RMP400_DECEL_LIN_MAX;
	double max_vel = acc->prev_vel_command +
	    sign_accel * max_accel * rmp400_sec_period;
	*vel_reference = sign_accel *
	    fmin(sign_accel * *vel_reference, sign_accel * max_vel);
	acc->prev_vel_command = *vel_reference;

	// angular velocity to preserve radius
	if (keep_radius) *ang_reference = *vel_reference / radius;
}

/*----------------------------------------------------------------------*/

/*
 * Control robot rotations
 *
 * The internal control laws are so inefficient that the robot
 * is not able to follow an arc of circle of a given radius
 *
 * So implement a control using an external (gyroscop) measure of the
 * yaw rate to adjust the speeds to make that better
 *
 * inputs:
 *   gyro - data from IDS
 *   t - current time
 *   vel_reference  - desired linear velocity
 *   yawr_reference - desired angular velocity
 *   yawr_measure   - mesured angular velocity
 *   yaw_measure    - mesured angular position
 * outputs
 *   yawr_command - corrected angular velocity to be sent to the robot
 *
 * also access the gyroAsserv member of the IDS directly.
 */

void
control_yaw(rmp400_gyro_asserv *gyro,
    double t, double vel_reference, double yawr_reference,
	    double yawr_measure, double yaw_measure, double *yawr_command)
{
	*yawr_command = gyro->prev_command;

	if (gyro->first) {
		*yawr_command = yawr_reference;
		gyro->first = 0;
		gyro->enabled = 1;
		gyro->jump_t = 0.;
	} else {
		double dt = t - gyro->prev_t;

		if (yawr_reference == 0.0) {
			if (vel_reference != 0.0) {
				// straight -> directly asserv on angle
				if (gyro->straight) {
					if (yawr_measure == gyro->prev_measure)
						return;

					// PI controller on angle commanded in speed
					// I is needed to remove static error
					// D is not needed because only used
					//   to go straight (small errors)
					double error_s = gyro->straight_angle-yaw_measure;
					if (error_s > M_PI) error_s -= 2*M_PI;
					if (error_s < -M_PI) error_s += 2*M_PI;
					gyro->integral_s += error_s * (t-gyro->prev_t);
					*yawr_command =
						rmp400_kp_gyro_theta*dt * error_s +
						rmp400_ki_gyro_theta*dt * gyro->integral_s;
				} else {
					*yawr_command = 0.0;
					gyro->straight = 1;
					gyro->straight_angle = yaw_measure;
					gyro->integral_s = 0.0;
				}
			} else {
				// do nothing, command (0,0) is correctly respected
				*yawr_command = 0.0;
				gyro->straight = 0;
			}
		} else {
			const double ref_ratio = 1.5;
			const double noise_th = 0.1;

			// if changing sign or increasing from around 0
			// then directly apply a priori command
			if ((yawr_reference*gyro->prev_reference < 0) ||
			    ((fabs(gyro->prev_reference) < noise_th)
				&& (fabs(yawr_reference) >= noise_th*ref_ratio))) {
				*yawr_command = yawr_reference;
				gyro->jump_t = t;
			}
			// if changing more than 50%
			// then directly apply extrapolated command
			if ((fabs(gyro->prev_reference) >= noise_th) &&
			    (yawr_reference*gyro->prev_reference > 0) &&
			    ((yawr_reference/gyro->prev_reference > ref_ratio)
				|| (gyro->prev_reference/yawr_reference > ref_ratio))) {
				*yawr_command = gyro->prev_command * yawr_reference
				    / gyro->prev_reference;
				gyro->jump_t = t;
			}
			// otherwise if delay from last jump has elapsed
			// P controller on speed commanded in speed
			if (t-gyro->jump_t >= rmp400_delay_gyro_omega) {
				if (yawr_measure == gyro->prev_measure) return;

				*yawr_command = gyro->prev_command +
				    rmp400_kp_gyro_omega*dt
				    * (yawr_reference-yawr_measure);
			} else if (t != gyro->jump_t) {

				// if we are in the lock period
				// and we didn't just reenter it,
				// then we have to follow the reference changes
				if (fabs(gyro->prev_reference) < noise_th
				    || fabs(yawr_reference - gyro->prev_reference) < 1e-10)
					*yawr_command = gyro->prev_command;
				else
					*yawr_command = gyro->prev_command
					    * yawr_reference / gyro->prev_reference;
			}
			gyro->straight = 0;
		}
	}

	//*yawr_command = yawr_reference; // TEST remove asserv

	gyro->prev_reference = yawr_reference;
	gyro->prev_measure = yawr_measure;
	gyro->prev_command = *yawr_command;
	gyro->prev_t = t;
}
