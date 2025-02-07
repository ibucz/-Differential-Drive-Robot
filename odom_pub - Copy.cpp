#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define IN1 6  // Left motor forward
#define IN2 7  // Left motor backward
#define IN3 9  // Right motor forward
#define IN4 10 // Right motor backward
#define ENA 5  // Enable pin for the left motor
#define ENB 8  // Enable pin for the right motor

#define IR_SENSOR_PIN A0      // Analog pin connected to IR sensor
#define DISTANCE_THRESHOLD 40 // Distance threshold in cm

// Ultrasonic sensor pins
#define TRIG_PIN 11 // Trigger pin
#define ECHO_PIN 12 // Echo pin

// Encoder pins
#define LEFT_ENCODER_PIN 2   // Left encoder pin
#define RIGHT_ENCODER_PIN 3  // Right encoder pin

volatile long leftEncoderCount = 0;   // Left encoder count
volatile long rightEncoderCount = 0;  // Right encoder count

ros::NodeHandle nh;

// Define publishers for sensor data
std_msgs::Int32 ir_distance_msg;
ros::Publisher ir_distance_pub("ir_distance", &ir_distance_msg);

std_msgs::Int32 ultrasonic_distance_msg;
ros::Publisher ultrasonic_distance_pub("ultrasonic_distance", &ultrasonic_distance_msg);

std_msgs::Int32 left_encoder_msg;
ros::Publisher left_encoder_pub("left_encoder_count", &left_encoder_msg);

std_msgs::Int32 right_encoder_msg;
ros::Publisher right_encoder_pub("right_encoder_count", &right_encoder_msg);

void leftEncoderISR() {
  leftEncoderCount++;
  left_encoder_msg.data = leftEncoderCount;
  left_encoder_pub.publish(&left_encoder_msg);
}

void rightEncoderISR() {
  rightEncoderCount++;
  right_encoder_msg.data = rightEncoderCount;
  right_encoder_pub.publish(&right_encoder_msg);
}

void onTwist(const geometry_msgs::Twist &msg) {
  // Enable both motors initially
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Stop motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Move forward
  if (msg.linear.x > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, HIGH);
  }
  // Move backward
  else if (msg.linear.x < 0) {
    digitalWrite(IN2, HIGH);
    digitalWrite(IN4, HIGH);
  }
  // Turn right
  else if (msg.angular.z < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN4, HIGH);
  }
  // Turn left
  else if (msg.angular.z > 0) {
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
  } else {
    // Stop motors
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", onTwist);

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IR_SENSOR_PIN, INPUT); // Set IR sensor pin as input

  // Encoder pins as inputs
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  // Attach interrupt for encoder pins
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initially disable both motors
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  nh.initNode();
  nh.subscribe(sub);

  // Advertise ROS publishers
  nh.advertise(ir_distance_pub);
  nh.advertise(ultrasonic_distance_pub);
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
}

long readUltrasonicDistance() {
  // Send a 10us pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin and calculate distance
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convert to centimeters

  return distance;
}

void publishSensorData() {
  // Read distance from IR sensor
  int irDistance = analogRead(IR_SENSOR_PIN) * 0.49; // Convert sensor reading to centimeters
  ir_distance_msg.data = irDistance;
  ir_distance_pub.publish(&ir_distance_msg);

  // Read distance from ultrasonic sensor
  long ultrasonicDistance = readUltrasonicDistance();
  ultrasonic_distance_msg.data = ultrasonicDistance;
  ultrasonic_distance_pub.publish(&ultrasonic_distance_msg);
}

void loop() {
  nh.spinOnce();

  // Publish sensor data
  publishSensorData();

  // Print IR sensor distance to the serial monitor
  Serial.print("IR Sensor Distance: ");
  Serial.print(ir_distance_msg.data);
  Serial.println(" cm");

  // Print ultrasonic sensor distance to the serial monitor
  Serial.print("Ultrasonic Sensor Distance: ");
  Serial.print(ultrasonic_distance_msg.data);
  Serial.println(" cm");

  // Print encoder counts to the serial monitor
  Serial.print("Left Encoder Count: ");
  Serial.println(leftEncoderCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightEncoderCount);

  delay(100); // Delay for readability
}