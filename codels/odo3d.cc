#include <MTI-clients/MTI.h>

extern "C" {
#include <rmp/rmpLib.h>
}

#include "acrmp400.h"
#include "rmp400_c_types.h"
#include "orMathLib.h"


/* Odometry z correction */
#define RMP400_ZODOCOR_ACCTHRES 0.75 /* acceleration threshold (factor of RMP400_ACCEL_LIN_MAX) to disable z update (m/s2) */
#define RMP400_ZODOCOR_LATENCY  4.0  /* how long it is disabled after last accel above threshold (s) */
#define RMP400_ZODOCOR_AVGPITCHSIZE 5 /* number of pitch samples to average for value to use when update is disabled */
#define RMP400_ZODOCOR_AVGVELSIZE 5 /* number of pitch samples to average for value to use when update is disabled */
#define RMP_DEFAULT_MAXIMUM_DECEL       3.923 /* Copied from rmp440.h in rmp440-libs */


// To replace POM_EULER struct in rmp440-genom2
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} PomEuler;

bool readMTI(MTI_DATA** mtiHandle, rmp400_mti_inertial_data* data)
{
    if(!mtiHandle)
        return false;
    MTI* mtiHandleP = (MTI*)*mtiHandle;
    if(!mtiHandleP)
        return false;

    return mtiHandleP->read((INERTIAL_DATA*)data, false);
    //return mtiHandleP->read((INERTIAL_DATA*)data, true);
}



bool rmp400odo3d(MTI_DATA** mtiHandle, rmp400_mti* mti,
    or_genpos_cart_state* robot, or_genpos_cart_3dstate* robot3d,
    rmp400_odometry_mode* odoMode, double period)
{
	int i;
	static double mpitch = 0.0, oldPitch = -1.0;
	static int pausePitch = 0;
	static double lastVel = 0.0;
	static double odoPitch = 0.0;
	static double lastLargeAccelTime = 0.0;
	static double history_pitch[2*RMP400_ZODOCOR_AVGPITCHSIZE] = {0.};
	static int history_pitch_i = 0;
	static double history_vel[RMP400_ZODOCOR_AVGVELSIZE] = {0.};
	static int history_vel_i = 0;
	double vel, accel;
	PomEuler attitude;

	/* 3D odometry */
	if (*odoMode == rmp400_odometry_3d) {
		if (!readMTI(mtiHandle, &mti->data)) {
			printf("switching back to odo2D\n");
            *odoMode = rmp400_odometry_2d;
		} else
		{
            attitude.roll  = DEG_TO_RAD(mti->data.euler[0]);
            attitude.pitch = DEG_TO_RAD(mti->data.euler[1]);
            attitude.yaw   = DEG_TO_RAD(mti->data.euler[2]);

			/* pause pitch updates if acceleration is too large */
			history_pitch[history_pitch_i++] = attitude.pitch;
			if (history_pitch_i >= 2*RMP400_ZODOCOR_AVGPITCHSIZE) history_pitch_i = 0;
			history_vel[history_vel_i++] = robot->v;
			if (history_vel_i >= RMP400_ZODOCOR_AVGVELSIZE) history_vel_i = 0;

			vel = 0.;
			for(i = 0; i < RMP400_ZODOCOR_AVGVELSIZE; ++i)
				vel += history_vel[i];
			vel /= RMP400_ZODOCOR_AVGVELSIZE;
			
			accel = (vel - lastVel) / period;
			lastVel = vel;

			if (fabs(accel) > RMP400_ZODOCOR_ACCTHRES*RMP_DEFAULT_MAXIMUM_DECEL)
			{
				lastLargeAccelTime = 0.0;
				if (!pausePitch)
				{
					/* Compute pitch value to use next during the pitch update pause.
					 * We don't use the latest RMP400_ZODOCOR_AVGPITCHSIZE values because they are
					 * often already contaminated by the acceleration (and decreasing the
					 * acceleration threshold would be too sensitive to noise).
					 * We average the RMP400_ZODOCOR_AVGPITCHSIZE values before to filter the noise.
					 */
					odoPitch = 0.0;
					for(i = 0; i < RMP400_ZODOCOR_AVGPITCHSIZE; ++i)
						odoPitch += history_pitch[(history_pitch_i+i)%(2*RMP400_ZODOCOR_AVGPITCHSIZE)];
					odoPitch /= (2*RMP400_ZODOCOR_AVGPITCHSIZE);
					pausePitch = 1;
				}
			} else
			{
				if (pausePitch)
				{
					lastLargeAccelTime += period;
					if (lastLargeAccelTime > RMP400_ZODOCOR_LATENCY)
						pausePitch = 0;
				}
				if (!pausePitch)
					odoPitch = attitude.pitch;
			}
		}
	}
	if (*odoMode == rmp400_odometry_2d) {
		mpitch = 0;
		oldPitch = -1;
        attitude.roll  = 0.0;
        attitude.pitch = 0.0;
        attitude.yaw   = 0.0;
	}           
	robot3d->pitch = attitude.pitch;
	robot3d->roll = attitude.roll;
	/* XXXX For now keep the position in the Z=0 frame */
	robot3d->theta  = robot->theta;
	robot3d->xRob = robot->xRob;
	robot3d->yRob = robot->yRob;
	robot3d->zRob += - (robot->v * period) * sin(odoPitch);
	robot3d->xRef = robot->xRef;
	robot3d->yRef = robot->yRef;
	robot3d->zRef = 0.0;
	robot3d->v = robot->v;
	robot3d->vt = 0.0;
	robot3d->w = robot->w;

    return true;
}
