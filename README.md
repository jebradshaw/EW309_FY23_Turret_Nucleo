# EW309_FY23_Turret_Nucleo
Nucleo L432KC Control Board for EW309 Nerf Gun Turret

![20221230_113939_resized](https://user-images.githubusercontent.com/5246863/210094853-12e5ea91-9030-4626-8e50-d254bd821977.jpg)
Picture of the populated printed circuit board above

 This project uses a Nucleo L432KC based microcontroller to control a Nerf Gun on a yaw/pitch turret for the Weapons, Robotics, and Control Engineering Departrment EW309 course at the US Naval Academy.  The board has two motor control ports that each provide a digital PWM output, with two direction contorl signals, power, and ground.  These are used to interface to an L298 H-Bridge motor driver (or other compatible H-Bridge motor driver) to control the dual axis turret.  There are also two MOSFET drivers for powering the motors that drive the spinners to project the ball on the nerf gun, and the belt driven feed motor that moves the nerf balls through the spinners to project them.  Position data on the gun/turret is provided by a BNO-055 inertial measurement unit (IMU) which is capable of providing yaw and pitch angles directly as Euler angles in radians or degrees.
 
![EW309_Nucleo_Turret_Schematic_20221005](https://user-images.githubusercontent.com/5246863/216617690-2f975659-58e2-4f0f-b66c-b6aa46c2bea6.png)
Electrical schematic above

  The example program provided was done in the Arduino IDE - 20230119_EW309_NucleoL432KC_JoesTest.ino
  
  Nucleo can be reprogrammed without compilation by dragging the .bin file to the Nucleo drive
  
  The PCB and Schematic files are also provided and were done using Express PCB Plus
  
![ExpressPCBsnapshot_Top](https://user-images.githubusercontent.com/5246863/214311960-8675de82-d033-4f10-a195-b6982a7e78f9.JPG)
  Top Printed Circuit Board Express PCB Layout
  
  
![ExpressPCBsnapshot_Bottom](https://user-images.githubusercontent.com/5246863/214312068-1767d38f-51f8-480b-b7b5-29cbe34a29fe.JPG)
  Bottom Printed Circuit Board Express PCB Layout
