/*
 * Copyright (c) 2009-2017 CNRS/LAAS
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

#include <sys/types.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <rmp/rmpLib.h>
#include <fe/ftdi-emergency.h>

#include "acrmp400.h"

#include "rmp400_c_types.h"

#include "codels.h"
#include "rmp400_Log.h"

#ifdef notyet
int
rmp440LogPose(struct rmp440_log_str *log, const or_pose_estimator_state *pose)
{

	if (fprintf(log->out, "%lf\t"
		"%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t%2.6g\t%2.6g\t%2.6g\n",
		pose->ts.sec + pose->ts.nsec*1.0E-9,
		pose->vel._value.vx, pose->vel._value.wz,
		pose->pos._value.x, pose->pos._value.y, pose->pos._value.z,
		pose->pos._value.qw, pose->pos._value.qx, 
		pose->pos._value.qy, pose->pos._value.qz,
		pose->vel._value.vx, pose->vel._value.vy, pose->vel._value.vz,
		pose->vel._value.wx, pose->vel._value.wy, pose->vel._value.wz) 
	    < 0) {
		printf ("%s: cannot log in %s\n", __func__, log->fileName);
		return -1;
	}

	return 0;
}
#endif


int
rmp400LogData(const struct rmp400_log_str *log,
    const or_pose_estimator_state *pose,
    const rmp400_gyro *gyro,
    const rmp400_gyro_asserv *gyro_asserv,
    const struct cmd_str *cmd,
    const rmp400_data_str rs_data[2])
{
  
  
	if (fprintf(log->out, "%lf\t"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g\t%d"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%d\t%d"
	       "\t%d\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g\t%.6g"
	       "\t%d\t%d"
	       "\t%d\t%.6g"
	       "\t%.6g\t%.6g"
	       "\t%.6g\t%.6g"
	       "\n",
				/* timestamps */
	       pose->ts.sec + pose->ts.nsec*1.0E-9, 
				/* the KVH gyro */
	       gyro->gyroTheta,
	       gyro->gyroOmega,
	       cmd->vReference, cmd->vCommand,
	       cmd->wReference, cmd->wCommand,
	       gyro_asserv->straight, gyro_asserv->straight_angle,
				/* the robot first axle */
	       rs_data[0].integrated_yaw,
	       rs_data[0].yaw_rate,
	       rs_data[0].pitch_angle,
	       rs_data[0].pitch_rate,
	       rs_data[0].roll_angle,
	       rs_data[0].roll_rate,
	       rs_data[0].rw_velocity, // invert left/right
	       rs_data[0].lw_velocity,
	       rs_data[0].integrated_right_wheel, // invert left/right
	       rs_data[0].integrated_left_wheel,
	       rs_data[0].integrated_fore_aft,
	       rs_data[0].right_torque, // invert left/right
	       rs_data[0].left_torque,
	       rs_data[0].servo_frames,
	       rs_data[0].operational_mode,
	       rs_data[0].controller_gain_schedule,
	       rs_data[0].ui_voltage,
	       rs_data[0].powerbase_voltage,
	       rs_data[0].battery_charge,
	       rs_data[0].velocity_command,
	       rs_data[0].turn_command,
				/* the robot second axle */
	       rs_data[1].integrated_yaw,
	       rs_data[1].yaw_rate,
	       rs_data[1].pitch_angle,
	       rs_data[1].pitch_rate,
	       rs_data[1].roll_angle,
	       rs_data[1].roll_rate,
	       rs_data[1].rw_velocity, // invert left/right
	       rs_data[1].lw_velocity,
	       rs_data[1].integrated_right_wheel, // invert left/right
	       rs_data[1].integrated_left_wheel,
	       rs_data[1].integrated_fore_aft,
	       rs_data[1].right_torque, // invert left/right
	       rs_data[1].left_torque,
	       rs_data[1].servo_frames,
	       rs_data[1].operational_mode,
	       rs_data[1].controller_gain_schedule,
	       rs_data[1].ui_voltage,
	       rs_data[1].powerbase_voltage,
	       rs_data[1].battery_charge,
	       rs_data[1].velocity_command,
	       rs_data[1].turn_command) < 0)
    {
      printf("%s: cannot log in %s\n", 
	  __func__, log->fileName);
      return -1;
    }
  
  return 0;
}
