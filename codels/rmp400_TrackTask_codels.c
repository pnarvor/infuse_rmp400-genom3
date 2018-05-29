#include "acrmp400.h"

#include "rmp400_c_types.h"


/* --- Task TrackTask --------------------------------------------------- */


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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_track_main;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_pause_track_main;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp400_ether;
}
