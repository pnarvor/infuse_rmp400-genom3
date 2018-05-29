#include "acrmp400.h"

#include "rmp400_c_types.h"


/* --- Activity Track --------------------------------------------------- */

/** Validation codel trackControl of activity Track.
 *
 * Returns genom_ok.
 * Throws rmp400_not_connected, rmp400_port_not_found, rmp400_bad_ref,
 * rmp400_cmd_stop_track, rmp400_motors_off, rmp400_emergency_stop,
 * rmp400_power_cord_connected.
 */
genom_event
trackControl(const rmp400_io *rmp, const rmp400_feedback *rs_data,
             const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
log_stop(rmp400_log_str **log, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}
