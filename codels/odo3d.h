#ifndef DEF_ODO3d_H
#define DEF_ODO3d_H

extern bool readMTI(MTI_DATA** mtiHandle, rmp400_mti_inertial_data* data);
extern bool rmp400odo3d(MTI_DATA** mtiHandle, rmp400_mti* mti,
    or_genpos_cart_state* robot, or_genpos_cart_3dstate* robot3d,
    rmp400_odometry_mode* odoMode, double period);

#endif
