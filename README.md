# EW309_FY23_Turret_Nucleo
Nucleo L432KC Control Board for EW309 Nerf Gun Turret

![20221230_113939_resized](https://user-images.githubusercontent.com/5246863/210094853-12e5ea91-9030-4626-8e50-d254bd821977.jpg)
Picture of the populated printed circuit board above

 This project uses a Nucleo L432KC based microcontroller to control a Nerf Gun on a yaw/pitch turret for the Weapons, Robotics, and Control Engineering Departrment EW309 course at the US Naval Academy.  The board has two motor control ports that each provide a digital PWM output, with two direction contorl signals, power, and ground.  These are used to interface to an L298 H-Bridge motor driver (or other compatible H-Bridge motor driver) to control the dual axis turret.  There are also two MOSFET drivers for powering the motors that drive the spinners to project the ball on the nerf gun, and the belt driven feed motor that moves the nerf balls through the spinners to project them.  Position data on the gun/turret is provided by a BNO-055 inertial measurement unit (IMU) which is capable of providing yaw and pitch angles directly as Euler angles in radians or degrees.
 
 ![Schematic](https://user-images.githubusercontent.com/5246863/210096393-63ad9af6-b74b-4e90-86b4-8053dc039929.PNG)
Electrical schematic above

  The example program provided was done in the Arduino IDE - 20221216_EW309_Turret_Nucleo_L432KC_BNO055_MotConL298_PID.ino
  
  The PCB and Schematic files are also provided and were done using Express PCB Plus
  
