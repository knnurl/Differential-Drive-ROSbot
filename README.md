# Differential-Drive-ROSbot
This repo is going to include files and docs for a robot run and teleoperated via ROS Noetic


 * Two wheel differential drive robot driver using rosserial
 * This sketch demonstrates the motion control using ROS and the ESP32
 * Driver on ESP32 takes data from ROS and sends it to BLDC hub motor drivers via PWM,
 *   then send back various feedback data such as odometry.
