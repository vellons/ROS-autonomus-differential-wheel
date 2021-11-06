# Arduino <-> ROS

Arduino and ROS communicate using serial.

To test serial communication try test_serial_led.py on Raspberry and test_serial_led.ino on Arduino.

My robot has two motors without encoders. It's controlled by and Arduino Uno and Arduino Motors Shield.


If the serial communication work you can choose between two programs to load:
 - **rosserial_cmd.ino**
 - **rosserial_cmd_imu_auto_correct.ino**: this use an IMU (BNO055) to go straight and self-correct.
 
Both program use /cmd_vel topic data in this way:
 
```c
void cmd_vel_callback(const geometry_msgs::Twist &twist_msg) {
	// Map values at [-1 .. 1]
	x = max(min(twist_msg.linear.x, 1.0f), -1.0f);
	z = max(min(twist_msg.angular.z, 1.0f), -1.0f);

	l = (x - z) / 2;
	r = (x + z) / 2;

	speed_SX = map_pwm(fabs(l), SPEED_MIN_SX, SPEED_MAX_SX);
	speed_DX = map_pwm(fabs(r), SPEED_MIN_DX, SPEED_MAX_DX);
	if (speed_SX < 0) speed_SX = 0;
	if (speed_SX > 255) speed_SX = 255;
	if (speed_DX < 0) speed_DX = 0;
	if (speed_DX > 255) speed_DX = 255;
	
	...
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &cmd_vel_callback);
```
