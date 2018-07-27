# Bunch of rmp400 control aliases for control by joystick just call 
rmp400-connect-joystick() {
    rosservice call /rmp400/connect_port {'Joystick','/joystick/device/Logitech_Logitech_Cordless_Rumb'}
}
rmp400-connect-jtrack () {
    rosservice call /joystick_to_twist/connect_port {'joystick','/joystick/device/Logitech_Logitech_Cordless_Rumb'}
    rosservice call /rmp400/connect_port {'cmd_vel_Infuse','/joystick_to_twist/twist'}
}
rmp400-init() {
    rosaction call /rmp400/Init {}
}
rmp400-gyro() {
    rosaction call /rmp400/Gyro '{params: {mode: {value: 1}, port: "/dev/ttyS0", type: {value: 2}, latitude: 43.56192, woffset: 0.0 }}'
}
rmp400-update-gyro-bias() {
    rosaction call /rmp400/GyroBiasUpdate ''$1''
}
rmp440-mti() {
    rosaction call /rmp440/InitMTI '{params: {mode: {value: 2}, port: "/dev/ttyS3", outputMode: 6, outputFormat: 4}}'
}
rmp440-toggle-odometry-mode() {
    rosaction call /rmp440/ToggleOdometryMode {}
}
rmp400-joystick-on() {
    rosaction call /rmp400/JoystickOn {}
}
rmp400-toggle-track-mode() {
    rosservice call /rmp400/toggleInfuseTrackMode {}
}
rmp400-track-mode() {
    rosaction call /rmp400/Track {}
}

rmp400-init-track-mode() {
    rmp400-connect-jtrack
    rmp400-gyro
    rmp400-update-gyro-bias 500
    rmp400-toggle-track-mode
    rmp400-init
    rmp400-track-mode
}
rmp400-init-joystick-mode() {
    rmp400-connect-joystick
    rmp400-gyro
    rmp400-update-gyro-bias 500
    rmp400-init
    rmp400-joystick-on
}
rmp400-odo2d() {
    rmp400-connect-joystick
    rmp400-gyro
    rmp400-update-gyro-bias 5000
    rmp400-init
    rmp400-joystick-on
}
rmp400-odo3d() {
    rmp400-connect-joystick
    rmp400-gyro
    rmp400-update-gyro-bias 5000
    rmp400-mti
    rmp400-toggle-odometry-mode
    rmp400-init
    rmp400-joystick-on
}


