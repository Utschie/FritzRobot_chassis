# FritzRobot_firmware
### 2023-10-26 update
add static filter to gyro's x and y axis and raised the threshold to 0.003. If the threshold value smaller than 0.003, there would be long-term shift on gyroscope when the chassis is static.
### 2023-10-25 update
Fixed the number of bits and format of the USBVCom's output to make it easier to read in the host computer via the serial driver. And added the speed of each wheel and the overall speed of the body.
![avatar](./pictures/Vcom_output2.png)
### 2023-10-16 update
re-tune the pid parameters since the accurater encoder, the new parameters: Kp=1.17,Ki=0.103
### 2023-10-14 update
switched to larger moment motor and accurater GMR encoder, therefore changed the value of ry (wheelbase) and nPpr(pulse per round).
![avatar](./pictures/chassis.jpg)
### 2023-10-12 update
re-distribute the  GPIO pins and TIM to release the Ethernet peripheral's pins  
### 2023-10-08 update
added extended kalman filter to estimate oreientation and output the acceleration, Angular velocity and quaternion through USBVcom. 
### 2023-10-05 update
added static filter to gyro's z-axis, if in 0.5s the std of z-axis<0, they imu.wz are set to 0.0, 
### 2023-10-03 update
IMU raw data sent out. The sampling rate of gyro and accel is 200hz, the DLPF for gyro and accel are both set to 92hz, the range of gyro is 250 dps, and accel is 4g.
![avatar](./pictures/accel.png)
![avatar](./pictures/gyro.png)
### 2023-09-25 update
McNamee's Wheel Forward and Reverse Kinematics Solved
### 2023-09-24 update
reorganize the codes, define new data type and functions to make the code more modular and readable
### 2023-09-19 update
PI control motor，Ki=3.0 and Kp=45. The control result shows like，
![avatar](./pictures/encoder_control_result.png)
### 2023-09-17 update
receive formatted char from VCP and save the speed control value in float.
### 2023-09-12 update
encoder's readings sent to USB VCOM
### 2023-09-09 update
pwm and encoder must use two different TIM，problems would happen when pwm output and encoder uses different channel on the same TIM.
