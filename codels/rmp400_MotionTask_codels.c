#include "acrmp400.h"

#include "rmp400_c_types.h"


/* --- Task MotionTask -------------------------------------------------- */


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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
}


/** Codel odoAndAsserv of task MotionTask.
 *
 * Triggered by rmp400_odo.
 * Yields to rmp400_ether, rmp400_pause_odo, rmp400_end.
 * Throws rmp400_emergency_stop.
 */
genom_event
odoAndAsserv(const rmp400_io *rmp,
             const rmp400_kinematics_str *kinematics,
             const rmp400_var_params *var_params,
             const rmp400_log_str *log,
             const rmp400_Joystick *Joystick, GYRO_DATA **gyroId,
             FE_STR **fe, or_genpos_cart_state *robot,
             or_genpos_cart_config_var *var, or_genpos_cart_speed *ref,
             rmp400_max_accel *max_accel, rmp400_feedback **rs_data,
             rmp400_mode *rs_mode, rmp400_gyro *gyro,
             rmp400_gyro_asserv *gyro_asserv, const rmp400_Pose *Pose,
             const rmp400_Status *Status,
             const rmp400_StatusGeneric *StatusGeneric,
             const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
}


/** Codel endOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp400_end.
 * Yields to rmp400_ether.
 * Throws rmp400_emergency_stop.
 */
genom_event
endOdoAndAsserv(rmp400_io **rmp, rmp400_feedback **rs_data,
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
rmp400InitStart(const char device[32], rmp400_io **rmp, FE_STR **fe,
                rmp400_feedback **rs_data, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_init_main;
}

/** Codel rmp400InitMain of activity Init.
 *
 * Triggered by rmp400_init_main.
 * Yields to rmp400_pause_init_main, rmp400_ether.
 * Throws rmp400_emergency_stop, rmp400_already_initialized,
 *        rmp400_malloc_error, rmp400_rmplib_error.
 */
genom_event
rmp400InitMain(rmp400_io **rmp, FE_STR **fe, rmp400_feedback **rs_data,
               rmp400_mode *rs_mode, rmp400_dynamic_str *dynamics,
               rmp400_kinematics_str *kinematics,
               const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_pause_init_main;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_js_main;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_pause_js_main;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
}
