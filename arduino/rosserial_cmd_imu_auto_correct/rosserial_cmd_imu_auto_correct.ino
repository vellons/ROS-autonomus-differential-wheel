#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define DEBUG 0

// PIN - LEFT MOTOR (Arduino motor shield - L298P)
#define PIN_DIRECTION_SX 12
#define PIN_BRAKE_SX 9
#define PIN_SPEED_SX 3

// PIN - RIGHT MOTOR (Arduino motor shield - L298P)
#define PIN_DIRECTION_DX 13
#define PIN_BRAKE_DX 8
#define PIN_SPEED_DX 11

#define SPEED_MIN_SX 80
#define SPEED_MIN_DX 80
#define SPEED_MAX_SX 251
#define SPEED_MAX_DX 255
#define K_BUS 6

int speed_SX = SPEED_MIN_SX;
int speed_DX = SPEED_MIN_DX;

float x = 0, z = 0, l = 0, r = 0;
int busIni = 0, bus = 0, busErr = 0, busErrAss = 0;
char robot_status = 0;
unsigned int t = 0;
int max_speed = 0;

ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event;


void motor_forward() {
  digitalWrite(PIN_DIRECTION_SX, HIGH); // Direction SX forward
  digitalWrite(PIN_DIRECTION_DX, HIGH); // Direction DX forward

  digitalWrite(PIN_BRAKE_SX, LOW); // Brake SX disable
  digitalWrite(PIN_BRAKE_DX, LOW); // Brake DX disable

  analogWrite(PIN_SPEED_SX, speed_SX);
  analogWrite(PIN_SPEED_DX, speed_DX);
}

void motor_stop() {
  digitalWrite(PIN_BRAKE_SX, HIGH); // Brake SX
  digitalWrite(PIN_BRAKE_DX, HIGH); // Brake DX

  analogWrite(PIN_SPEED_SX, 0);
  analogWrite(PIN_SPEED_DX, 0);
}

void read_imu() {
  bno.getEvent(&event);
  bus = event.orientation.x;
}

void calc_err() {
  // Remove initial offset
  if (bus > busIni) {
    bus = bus - busIni;
  } else {
    bus = 359 - (busIni - bus);
  }

  if (bus < 180) {
    busErr = bus;
  } else {
    busErr = bus - 359;
  }
  busErrAss = busErr;
  if (busErrAss <= 0) busErrAss *= -1;
}

void correct_pwm_forward() {
  max_speed = max(speed_SX, speed_DX);
  if (busErr <= 0) {
    speed_DX = max_speed - (busErrAss * K_BUS);
    speed_SX = max_speed;
    if (speed_DX < 0) speed_DX = 0;
  } else if (busErr > 0) {
    speed_SX = max_speed - (busErrAss * K_BUS);
    speed_DX = max_speed;
    if (speed_SX < 0) speed_SX = 0;
  }
}

int map_pwm(float x, float out_min, float out_max) {
  return x * (out_max - out_min) + out_min;
}

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

  if (DEBUG == 1) {
    char log_msg[15];
    sprintf(log_msg, "PWM: %d - %d", speed_SX, speed_DX);
    nh.logwarn(log_msg);
  }

  if (x <= 0.05) {
    robot_status = 0;
    motor_stop();
  } else if (z <= 0.03 && z >= -0.03) { // Check IMU to go straight
    robot_status = 1;
    read_imu();
    busIni = bus;
    calc_err();
    correct_pwm_forward();
    t = millis();
    motor_forward();
  } else {
    robot_status = 2;
    motor_forward();
  }
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &cmd_vel_callback);


void setup() {
  pinMode(PIN_DIRECTION_SX, OUTPUT);
  pinMode(PIN_BRAKE_SX, OUTPUT);
  pinMode(PIN_SPEED_SX, OUTPUT);
  pinMode(PIN_DIRECTION_DX, OUTPUT);
  pinMode(PIN_BRAKE_DX, OUTPUT);
  pinMode(PIN_SPEED_DX, OUTPUT);
  motor_stop();

  nh.initNode();
  nh.subscribe(subCmdVel);

  // Wait until you are actually connected to ROS master
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.logwarn("Arduino connected");

  if (!bno.begin()) {
    nh.logfatal("IMU error");
    while (1);
  }
  delay(500);
  read_imu();
  delay(500);
  read_imu();
  busIni = bus;
  t = millis();
}


void loop() {
  nh.spinOnce();
  delay(1);
  if (robot_status == 1 && millis() - t > 125) {
    read_imu();
    calc_err();
    correct_pwm_forward();
    motor_forward();
  }
}
