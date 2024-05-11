// 3 wheeled Holonomic Drive

/*
           /\
          /  \
         /    \
        /      \
       /        \
      /    ^     \
     /     |      \
    /      |r      \
   /       |        \
  /        |         \
 /         |          \
/__________|___________\
     L/2        L/2

*/
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <cmath>
#include <array> 
//imports
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"


#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

//Motor Driver 1
#define EN_A 26
#define IN1 16 
#define IN2 4
#define IN3 13
#define IN4 12
#define EN_B 25

//Motor Driver 2
// #define EN_C 14 
// #define IN5 13
// #define IN6 12
#define EN_C 27
#define IN5 15
#define IN6 2


// Setting PWM properties 
const int freq = 100000;
const int resolution = 8;


// Define the number of steps per revolution of the encoder
const uint16_t STEPS_PER_REV = 48;

// Declaring input pins for three N20 DC Motors
#define MOTOR1_PHASE_A 5
#define MOTOR1_PHASE_B 18

#define MOTOR2_PHASE_A 19
#define MOTOR2_PHASE_B 21

#define MOTOR3_PHASE_A 22
#define MOTOR3_PHASE_B 23

volatile int16_t motor1Pos = 0;
volatile int16_t motor2Pos = 0;
volatile int16_t motor3Pos = 0;
unsigned int motor1_encoder_counter = 0;


/*
--------------------------------------------
--------------------------------------------
        KINEMATICS AND ODOMETRY
--------------------------------------------
--------------------------------------------
*/

// Robot Parameters

float wheel_radius = 0.036;        //36mm      // Wheel Radius in meters
float robot_radius = 0.08;               // Robot Radius in meters

const float MOTOR_MAX_RPM = 60;		// Maximum RPM of the motors


// Kinematics Variables
float wheels_angular_vel_vec[3] = {0, 0, 0};   // Angular Velocities of the wheels
           // Beta constant

// Odometry Variables
float x = 0;        // X position of the robot
float y = 0;        // Y position of the robot
float theta = 0;    // Orientation of the robot

float x_dot = 0;    // X velocity of the robot
float y_dot = 0;    // Y velocity of the robot
float theta_dot = 0;// Angular velocity of the robot

double dt = 0.0;    // Time step

float motor_angular_velocities[3] = {0, 0, 0}; // Angular velocities of the motors in rad/s which will come from encoders

/**
 * @brief Performs Runge-Kutta integration to update the position and orientation of the robot.
 *
 * This function updates the position (x, y) and orientation (theta) of the robot using Runge-Kutta integration.
 * It calculates the new position and orientation based on the current velocities (x_dot, y_dot, theta_dot)
 * and the time step (dt).
 */
void IntegrateByRungeKutta() {
    
    double theta_bar = theta + (theta_dot * dt / 2);

    x = x + (x_dot * cos(theta_bar) - y_dot * sin(theta_bar)) * dt;
    y = y + (x_dot * sin(theta_bar) + y_dot * cos(theta_bar)) * dt;
    theta = theta + theta_dot * dt;
}

/**
 * @brief Performs Euler integration to update the position and orientation of the robot.
 *
 * This function updates the position (x, y) and orientation (theta) of the robot using Euler integration.
 * It calculates the new position and orientation based on the current velocities (x_dot, y_dot, theta_dot)
 * and the time step (dt).
 */
void IntegrateByEuler(){
    x = x + (x_dot * cos(theta) - y_dot * sin(theta)) * dt;
    y = y + (x_dot * sin(theta) + y_dot * cos(theta)) * dt;
    theta = theta + theta_dot * dt;
}


/**
 * @brief Calculates the angular velocities of the wheels based on the linear and angular velocities of the robot.
 *
 * This function calculates the angular velocities of the wheels based on the linear and angular velocities of the robot.
 * The linear and angular velocities are passed as arguments to the function.
 *
 * @param linear_x The linear velocity of the robot in the x-direction.
 * @param linear_y The linear velocity of the robot in the y-direction.
 * @param angular_z The angular velocity of the robot about the z-axis.
 */
std::array<double, 4> eulerToQuaternion(double roll, double pitch, double yaw) {
    // Compute the quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    std::array<double, 4> quaternion;
    quaternion[0] = cr * cp * cy + sr * sp * sy; // w
    quaternion[1] = sr * cp * cy - cr * sp * sy; // x
    quaternion[2] = cr * sp * cy + sr * cp * sy; // y
    quaternion[3] = cr * cp * sy - sr * sp * cy; // z

    return quaternion;
}

/**
 * Drives motor 1 with the specified PWM value.
 * 
 * @param pwm The PWM value to set for motor 1, 2 or 3. Positive values drive the motor forward, while negative values drive it backward.
 */
void DriveMotor1(int pwm){
  if(pwm > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(0, abs(pwm));
}

void DriveMotor2(int pwm){
  if(pwm > 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(1, abs(pwm));
}

void DriveMotor3(int pwm){
  if(pwm > 0){
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  else{
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }
  ledcWrite(2, abs(pwm));
  // delay(500);
}


void debugMotorSpeedsIncrement() {
    for(int i = 255; i >= 80; i=i-10){
    DriveMotor1(i);
    Serial.print("M1-PWM: ");
    Serial.println(i);
    delay(1000);

  }

  DriveMotor1(0);

  for(int i = 255; i >= 80; i=i-10){
    DriveMotor2(i);
    Serial.print("M2-PWM: ");
    Serial.println(i);
    delay(1000);

  }

  DriveMotor2(0);

  for(int i = 255; i >= 80; i=i-10){
    DriveMotor3(i);
    Serial.print("M3-PWM: ");
    Serial.println(i);
    delay(1000);

  }

  DriveMotor3(0);

  //for loop now in negative pwm
  for(int i = 80; i <= 255; i=i+10){
    DriveMotor1(-i);
    Serial.print("M1-PWM: ");
    Serial.println(-i);
    delay(1000);

  }

  DriveMotor1(0);

  for(int i = 80; i <= 255; i=i+10){
    DriveMotor2(-i);
    Serial.print("M2-PWM: ");
    Serial.println(-i);
    delay(1000);

  }
  DriveMotor2(0);

  for(int i = 80; i <= 255; i=i+10){
    DriveMotor3(-i);
    Serial.print("M3-PWM: ");
    Serial.println(-i);
    delay(1000);

  }

  DriveMotor3(0);

}

void debugMotorDirection() {
  DriveMotor1(255);
  delay(1000);
  DriveMotor1(-255);
  delay(1000);
  DriveMotor1(0);
  delay(1000);

  DriveMotor2(255);
  delay(1000);
  DriveMotor2(-255);
  delay(1000);
  DriveMotor2(0);
  delay(1000);

  DriveMotor3(255);
  delay(1000);
  DriveMotor3(-255);
  delay(1000);
  DriveMotor3(0);
  delay(1000);
}
/*
--------------------------------------------
--------------------------------------------
		ROS2 PUBLISHER AND SUBSCRIBER
--------------------------------------------
--------------------------------------------
*/

// Micro-ROS Objects
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher;
nav_msgs__msg__Odometry msg_odometry;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;
rclc_executor_t executor_sub;


#define LED_PIN LED_BUILTIN		// LED Pin


// Function Prototypes
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Error Loop
void error_loop()
{	
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Timer Callback - Publishes the heartbeat
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    double covariance_pose[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-9, 0, 0, 0, 0,
    0, 0, 1e-9, 0, 0, 0,
    0, 0, 0, 1e-9, 0, 0,
    0, 0, 0, 0, 1e-9, 0,
    0, 0, 0, 0, 0, 1e-9
    };

    double covariance_twist[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-9, 0, 0, 0, 0,
    0, 0, 1e-9, 0, 0, 0,
    0, 0, 0, 1e-9, 0, 0,
    0, 0, 0, 0, 1e-9, 0,
    0, 0, 0, 0, 0, 1e-9
    };

    int64_t now = esp_timer_get_time();
    std::array<double, 4> quaternion_angle = eulerToQuaternion(0, 0, theta);

    // Publish the Odometry message
    msg_odometry.header.stamp.sec = now / 1000000;                  // Convert microseconds to seconds
    msg_odometry.header.stamp.nanosec = (now % 1000000) * 1000;     // Get the remaining microseconds and convert to nanoseconds
    // msg_odometry.header.frame_id = "odom";
    rosidl_runtime_c__String__assign(&msg_odometry.header.frame_id, "odom"); // Use this instead of the above line 

    // msg_odometry.child_frame_id = "base_link";
    rosidl_runtime_c__String__assign(&msg_odometry.child_frame_id, "base_link"); // Use this instead of the above line

    // Set the position and orientation of the robot
    msg_odometry.pose.pose.position.x = x;
    msg_odometry.pose.pose.position.y = y;
    msg_odometry.pose.pose.position.z = 0;
;

    msg_odometry.pose.pose.orientation.w = quaternion_angle[0];
    msg_odometry.pose.pose.orientation.x = quaternion_angle[1];
    msg_odometry.pose.pose.orientation.y = quaternion_angle[2];
    msg_odometry.pose.pose.orientation.z = quaternion_angle[3];

    for (int i = 0; i < 36; ++i) { msg_odometry.pose.covariance[i] = covariance_pose[i]; }

    
    msg_odometry.twist.twist.linear.x = x_dot;
    msg_odometry.twist.twist.linear.y = y_dot;
    msg_odometry.twist.twist.linear.z = 0;

    msg_odometry.twist.twist.angular.x = 0;
    msg_odometry.twist.twist.angular.y = 0;
    msg_odometry.twist.twist.angular.z = theta_dot;

    for (int i = 0; i < 36; ++i) { msg_odometry.twist.covariance[i] = covariance_twist[i]; }


    RCSOFTCHECK(rcl_publish(&publisher, &msg_odometry, NULL));

  }
}




// Subscription Callback - Receives the Twist message and calculates the angular velocities of the wheels and gives the pwm values to the motors
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg_twist = (const geometry_msgs__msg__Twist *)msgin;

    // Initialize the velocities 
    double linear_x = msg_twist->linear.x; 
    double linear_y = msg_twist->linear.y;
    double angular_z = msg_twist->angular.z;

    //Serial print linear x y and ang z
    Serial.print("Linear x: ");
    Serial.print(linear_x);
    Serial.print("\t");
    Serial.print("Linear y: ");
    Serial.print(linear_y); 
    Serial.print("\t");
    Serial.print("Angular z: ");
    Serial.println(angular_z);

    double sqrt3o2 = 0.86602540378;
    //robot radius = 0.08m
    //wheel radius = 0.036m
    // Calculate the angular velocities of the wheels
    double u1 = ((robot_radius*angular_z) + linear_y )/wheel_radius;      //57.7778
    double u2 = ((robot_radius*angular_z) + (-0.5*linear_y) + (-sqrt3o2*linear_x))/wheel_radius;  //
    double u3 = ((robot_radius*angular_z) + (-0.5*linear_y) + (sqrt3o2*linear_x))/wheel_radius;  //

    int m1 = map(abs(u1), 0, 60.00, 100, 255) * (u1 > 0 ? 1 : -1);
    int m2 = map(abs(u2), 0, 60.00, 100, 255) * (u2 > 0 ? 1 : -1);
    int m3 = map(abs(u3), 0, 60.00, 100, 255) * (u3 > 0 ? 1 : -1);

   // Add an if statement if the value is equal to 0, set it to 0
   if (u1==0.00) { m1 = 0; }
    if (u2==0.00) { m2 = 0; }
    if (u3==0.0) { m3 = 0; }


    DriveMotor3(m3);
    DriveMotor2(m2);
    DriveMotor1(m1);
   
   
    
    // Serial print m1,m2,m3 and u1, u2, u3 values 
    Serial.print("m1: ");
    Serial.print(m1);
    Serial.print("\t");
    Serial.print("u1: ");
    Serial.println(u1);
    Serial.print("m2: ");
    Serial.print(m2);
    Serial.print("\t");
    Serial.print("u2: ");
    Serial.println(u2);
    Serial.print("m3: ");
    Serial.print(m3);
    Serial.print("\t");
    Serial.print("u3: ");
    Serial.println(u3);
     
    // digitalWrite(LED_PIN, (msg_twist->linear.x == 0) ? LOW : HIGH);
}

String motor1_sign = "p";

void motor1_encoder_callback() {
  motor1_encoder_counter++;

  if (digitalRead(MOTOR1_PHASE_B) == HIGH) {
    motor1_sign = "p";
  }

  else {
    motor1_sign = "n";
  }

  Serial.println(motor1_sign + String(motor1_encoder_counter));
}
void setup()
{
    // Initialize the motor control pins as outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(EN_C, OUTPUT);

    // configure LED PWM functionalitites
    ledcSetup(0, freq, resolution);
    ledcSetup(1, freq, resolution);
    ledcSetup(2, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(EN_A, 0);
    ledcAttachPin(EN_B, 1);
    ledcAttachPin(EN_C, 2);

    // Ensure the motor control pins start low
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    
    //Encoder pins
    pinMode(MOTOR1_PHASE_B, INPUT);

    // attachInterrupt(digitalPinToInterrupt(MOTOR1_PHASE_A), motor1_encoder_callback, RISING);

    Serial.begin(115200);
    // set_microros_serial_transports(Serial);


    IPAddress agent_ip(10, 42, 0, 1);
    size_t agent_port = 8888;

    char ssid[] = "adyansh";
    char psk[]= "12345678";


    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_xiao_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odometry"));

  // create timer, called every 10 ms to publish heartbeat
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA));

}

void driveForward(){
  DriveMotor1(0);
  DriveMotor2(255);
  DriveMotor3(255);
  delay(2000);

}

void driveBackward(){
  DriveMotor1(0);
  DriveMotor2(-255);
  DriveMotor3(-255);
  delay(2000);

}


void Rotate(){
  DriveMotor1(255);
  DriveMotor2(255);
  DriveMotor3(255);
  delay(2000);

}

void loop()
{
  delay(100);
  
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));

  // driveBackward();
  // driveForward();
  // Rotate();

  // debugMotorDirection();

}




/* -------------------------
----------------------------
        ENCODER CODE
----------------------------
----------------------------*/

// // Interrupt service routines for handling encoder pulses
// void IRAM_ATTR ISR_MOTOR1() {
//   static bool oldStateA = false;
//   bool newStateA = digitalRead(MOTOR1_PHASE_A);
//   bool StateB = digitalRead(MOTOR1_PHASE_B);

//   if (oldStateA && !newStateA) {
//     if (StateB) {
//       motor1Pos++;
//     } else {
//       motor1Pos--;
//     }
//   }

//   oldStateA = newStateA;
// }

// void IRAM_ATTR ISR_MOTOR2() {
//   static bool oldStateA = false;
//   bool newStateA = digitalRead(MOTOR2_PHASE_A);
//   bool StateB = digitalRead(MOTOR2_PHASE_B);

//   if (oldStateA && !newStateA) {
//     if (StateB) {
//       motor2Pos++;
//     } else {
//       motor2Pos--;
//     }
//   }

//   oldStateA = newStateA;
// }

// void IRAM_ATTR ISR_MOTOR3() {
//   static bool oldStateA = false;
//   bool newStateA = digitalRead(MOTOR3_PHASE_A);
//   bool StateB = digitalRead(MOTOR3_PHASE_B);

//   if (oldStateA && !newStateA) {
//     if (StateB) {
//       motor3Pos++;
//     } else {
//       motor3Pos--;
//     }
//   }

//   oldStateA = newStateA;
// }

// // Constants
// const float WHEEL_CIRCUMFERENCE = M_PI * 0.0315; // circumference of the wheel in meters
// const float COUNTS_PER_REVOLUTION = STEPS_PER_REV * 2; // considering both edges of square wave
// const float TIME_INTERVAL_US = 10000; // time interval for counting the encoder ticks

// // Variables
// uint32_t prevEncoder1Count = 0;
// uint32_t prevEncoder2Count = 0;
// uint32_t prevEncoder3Count = 0;

// float angularVelocity1 = 0.0;
// float angularVelocity2 = 0.0;
// float angularVelocity3 = 0.0;

// volatile signed long diffEncoder1Count = 0;
// volatile signed long diffEncoder2Count = 0;
// volatile signed long diffEncoder3Count = 0;

// unsigned long prevTime = 0; // Newly added line

// void setup() {
//   Serial.begin(115200);

//   // Initializing input pins as pullups
//   pinMode(MOTOR1_PHASE_A, INPUT_PULLUP);
//   pinMode(MOTOR1_PHASE_B, INPUT_PULLUP);

//   pinMode(MOTOR2_PHASE_A, INPUT_PULLUP);
//   pinMode(MOTOR2_PHASE_B, INPUT_PULLUP);

//   pinMode(MOTOR3_PHASE_A, INPUT_PULLUP);
//   pinMode(MOTOR3_PHASE_B, INPUT_PULLUP);

//   // Register interrupt handlers
//   attachInterrupt(digitalPinToInterrupt(MOTOR1_PHASE_A), ISR_MOTOR1, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(MOTOR2_PHASE_A), ISR_MOTOR2, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(MOTOR3_PHASE_A), ISR_MOTOR3, CHANGE);

// }

// void loop() {
//   // Count the total counts for each wheel
//   static uint32_t currTime = micros();
//   if (micros() - prevTime >= TIME_INTERVAL_US) {
//     prevTime = currTime;

//     uint32_t currEncoder1Count = motor1Pos;
//     uint32_t currEncoder2Count = motor2Pos;
//     uint32_t currEncoder3Count = motor3Pos;

//     // Update encoder counters for next iteration
//     motor1Pos = 0;
//     motor2Pos = 0;
//     motor3Pos = 0;

//     // Calculate angular velocity for each wheel
//     angularVelocity1 = (COUNTS_PER_REVOLUTION / (currEncoder1Count)) * (WHEEL_CIRCUMFERENCE / TIME_INTERVAL_US);
//     angularVelocity2 = (COUNTS_PER_REVOLUTION / (currEncoder2Count)) * (WHEEL_CIRCUMFERENCE / TIME_INTERVAL_US);
//     angularVelocity3 = (COUNTS_PER_REVOLUTION / (currEncoder3Count)) * (WHEEL_CIRCUMFERENCE / TIME_INTERVAL_US);

//     // Print angular velocity for each wheel
//     Serial.print("Total Ticks 1: ");
//     Serial.print(currEncoder1Count);
//     Serial.print("\tTotal Ticks 2: ");
//     Serial.print(currEncoder2Count);
//     Serial.print("\tTotal Ticks 3: ");
//     Serial.println(currEncoder3Count);

//     Serial.print("Angular Velocity 1: ");
//     Serial.print(angularVelocity1, 2);
//     Serial.print("\tAngular Velocity 2: ");
//     Serial.print(angularVelocity2, 2);
//     Serial.print("\tAngular Velocity 3: ");
//     Serial.println(angularVelocity3, 2);
//     delay(300);

//     // Save previous encoder counts
//     prevEncoder1Count = currEncoder1Count;
//     prevEncoder2Count = currEncoder2Count;
//     prevEncoder3Count = currEncoder3Count;
//   }
// }