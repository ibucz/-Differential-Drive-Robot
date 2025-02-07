#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;


#define L 0.33
#define R 0.05
// Define pin constants

const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 6;
const int IN4 = 7;
const int speedL = 10;
const int speedR = 5;
const int irSensorPin = A0; // Assuming the IR sensor is connected to analog pin 0
const int obstacleThreshold = 900; // Adjust this threshold based on sensor readings
const int buzzerPin = 11; // Define the buzzer pin
volatile unsigned int counter, counter1, pulses1, pulses2; // Declare pulses1 and pulses2
int rpm, rpm2;

// Define ROS publishers
geometry_msgs::Point32 L_encoder_msg;
geometry_msgs::Point32 r_encoder_msg;
std_msgs::Bool obstacle_detected_msg;
ros::Publisher l_enc_pub("encoderL", &L_encoder_msg);
ros::Publisher r_enc_pub("encoderR", &r_encoder_msg);
ros::Publisher obstacle_pub("obstacle_detected", &obstacle_detected_msg);

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Do something with the received Twist message (e.g., control the robot)
  float vel = msg.linear.x;
  float omega = msg.angular.z;

  float VR = (2 * vel + omega * L) / (2 * R);
  float VL = (2 * vel - omega * L) / (2 * R);

  //-----right motor------
  if (VR < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(speedR, abs(VR));
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(speedR, VR);
  }

  //-----left motor------
  if (VL < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(speedL, abs(VL));
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(speedL, VL);
  }
}

// Define ROS subscriber
geometry_msgs::Twist cmd_vel_msg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

void setup() {
  Serial.begin(57600);
  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
  nh.advertise(obstacle_pub);
  nh.subscribe(cmd_vel_sub);

  // Set up pins
  for (int i = 5; i <= 11; i++) {
    pinMode(i, OUTPUT);
  }

  // Set up buzzer pin
  pinMode(buzzerPin, OUTPUT);

  attachInterrupt(0, countpulse, RISING);
  attachInterrupt(1, countpulse1, RISING); // Change interrupt number for the second encoder
}

void loop() {
  // Check for obstacle
  if (analogRead(irSensorPin) < obstacleThreshold) {
    obstacle_detected_msg.data = true;
    obstacle_pub.publish(&obstacle_detected_msg);
    stopp();
    // Activate buzzer alarm
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
    delay(200); // Alarm duration
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
    delay(50);
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
    delay(200); // Alarm duration
    digitalWrite(buzzerPin, LOW); // Turn off buzzerdigitalWrite(buzzerPin, HIGH); // Turn on buzzer
    delay(50); // Alarm duration
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
    backward();

    // Change direction randomly
    int random_direction = random(0, 2); // Generate random number between 0 and 1
    switch (random_direction) {
      case 0:
        left();
        break;
      case 1:
        right();
        break;
      default:
        stopp();
    }
  } else {
    obstacle_detected_msg.data = false;
    obstacle_pub.publish(&obstacle_detected_msg);
    forward();
  }

  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
    rpm = (0.375*counter / 20) * 60;
    rpm2 = (1.714285714*counter1 / 20) * 60;
    counter = 0;
    counter1 = 0;
    previousMillis += 1000;
  }

  Serial.print("Speed: ");
  Serial.println(rpm);
  Serial.println(" rps");
  Serial.print("Speed2: ");
  Serial.println(rpm2);
  Serial.println(" rps");

  // Publish encoder values
  L_encoder_msg.x = pulses1;
  r_encoder_msg.x = pulses2;
  l_enc_pub.publish(&L_encoder_msg);
  r_enc_pub.publish(&r_encoder_msg);

  delay(10);
}

void forward() {
  // Move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 250);
  analogWrite(speedR, 250);
}

void backward() {
  // Move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(speedL, 250);
  analogWrite(speedR, 250);
  delay(500);
}

void left() {
  // Turn left
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 0);
  analogWrite(speedR, 250);

  delay(1000);
}

void right() {
  // Turn right
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  analogWrite(speedL, 250);
  analogWrite(speedR, 0);

  delay(1000);
}

void stopp() {
  // Stop the vehicle
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(speedL, LOW);
  digitalWrite(speedR, LOW);
}

void countpulse() {
  counter++;
  pulses1++; // Increment pulses1 for the left encoder
}

void countpulse1() {
  counter1++;
  pulses2++; // Increment pulses2 for the right encoder
}

