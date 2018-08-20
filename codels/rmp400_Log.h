/*
 * Copyright (c) 2017 CNRS/LAAS
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
#ifndef _RMP400_LOG_H
#define _RMP400_LOG_H

typedef struct rmp400_log_str {
	FILE *out;
	char *fileName;
} rmp400_log_str;

#define rmp400_log_header \
	       "#date" \
	       "\tgyroAngle\tgyroRate" \
	       "\tsent_velocity_reference\tsent_velocity_command" \
	       "\tsent_turn_reference\tsent_turn_command" \
	       "\tstraight\tstraight_angle" \
	       "\tintegrated_yaw1\tyaw_rate1" \
	       "\tpitch_angle1[broken]\tpitch_rate1" \
	       "\troll_angle1\troll_rate1" \
	       "\tlw_velocity1\trw_velocity1" \
	       "\tintegrated_left_wheel1\tintegrated_right_wheel1" \
	       "\tintegrated_fore_aft1\tleft_torque1\tright_torque1" \
	       "\tservo_frames1\toperational_mode1" \
	       "\tcontroller_gain_schedule1\tui_voltage1" \
	       "\tpowerbase_voltage1\tbattery_charge1" \
	       "\tvelocity_command1[broken]\tturn_command1[broken]" \
	       "\tintegrated_yaw2\tyaw_rate2" \
	       "\tpitch_angle2[broken]\tpitch_rate2" \
	       "\troll_angle2\troll_rate2" \
	       "\tlw_velocity2\trw_velocity2" \
	       "\tintegrated_left_wheel2\tintegrated_right_wheel2" \
	       "\tintegrated_fore_aft2\tleft_torque2\tright_torque2" \
	       "\tservo_frames2\toperational_mode2" \
	       "\tcontroller_gain_schedule2\tui_voltage2" \
	       "\tpowerbase_voltage2\tbattery_charge2" \
	       "\tvelocity_command2[broken]\tturn_command2[broken]"

extern int rmp400LogPose(const struct rmp400_log_str *log, const or_pose_estimator_state *pose);
extern int rmp400LogData(const struct rmp400_log_str *log,
			 const or_pose_estimator_state *pose,
			 const rmp400_gyro *gyro,
			 const rmp400_gyro_asserv *gyro_asserv,
			 const struct cmd_str *cmd,
			 const rmp400_data_str data[2]);

#endif
