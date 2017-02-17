# Crazyflie_optitrack_hover
Autonomous hovering of Crazyflie with Optitrack (NESL- UCLA)


## FAST INFO

######  General way to publish message

rostopic pub -r 50  crazyflie/cmd_vel geometry_msgs/Twist  '{linear:  {x: 10, y: -13 , z: 45500.0}, angular: {x: 00.0,y: 0.0,z: 0.0}}'
ctrl Z to break

rostopic pub -l  crazyflie/goal geometry_msgs/PoseStamped '{header: {frame_id: "/worldFrame"}, pose: {position:  {x: 1, y: -1 , z: 1.0}, orientation: {x: 00.0,y: 0.0,z: 0.0}}}'


## Spcific for this project.

######  Call the serice to start the flight:

rosservice call /crazyflie/takeoff 
rosservice call /crazyflie/land

######  This is to set the position on the fly:

rosparam set /crazyflie/positiongoal/y 1

###### PID parameters tunnable (depends on the crazyflie)
PIDs:
  X:
    kp: 6.0
    kd: 1.0
    ki: 1.0
    minOutput: -30.0
    maxOutput: 30
    integratorMin: -1
    integratorMax: 1
    bias: 0
  Y:
    kp: -3.0
    kd: 1.0
    ki: 1.0
    minOutput: -100.0
    maxOutput: 100.0
    integratorMin: -1
    integratorMax: 1
    bias: 0.5
  Z:
    kp: 700.0
    kd: 50.0
    ki: 7.0
    minOutput: -15000.0
    maxOutput: 10000.0
    integratorMin: -1000.0
    integratorMax: 500.0
    bias: 43500
  Yaw:
    kp: -2.0
    kd: -.2.0
    ki: 0.0
    minOutput: -20.0
    maxOutput: 20
    integratorMin: 0.0
    integratorMax: 0.0
    bias: 0
    
/ means the root of the current drive;
./ means the current directory;
../ means the parent of the current directory.
