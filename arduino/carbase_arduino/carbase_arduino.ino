#include <due_can.h>
#include <ros2arduino.h>
#include <Servo.h>

#include "MecanumWheelDriver.h"

#define MOTOR_NUM 4
#define XRCEDDS_PORT Serial

using namespace ros2;

// Motor driver
MecanumWheelDriver motor_driver(-1.0, -1.0, -1.0);

// RM motor
const int MAX_OUTPUT = 16384;
int rm_id = 0;
int rm_speed[MOTOR_NUM] = { 0, 0, 0, 0 }, rm_speed_error[MOTOR_NUM] = { 0, 0, 0, 0 }, rm_last_speed_error[MOTOR_NUM] = { 0, 0, 0, 0};
int rm_target_speed[MOTOR_NUM] = { 0, 0, 0, 0 };
int rm_output[MOTOR_NUM] = { 0, 0, 0, 0 };

// PID - Speed
const double MAX_SO = 1000.0;
double rm_sk[MOTOR_NUM][3] = {{ 0.75, 0.0, 2.75 }, { 0.75, 0.0, 2.75 }, { 0.75, 0.0, 2.75 }, { 0.75, 0.0, 2.75 }}; //original 1.5, 0.0, 2.75
double rm_so[MOTOR_NUM][3] = {{ 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }}; 

// Servo
const double ROTATION_SPEED = 0.1;
const int SERVO_MIN = 0;
const int SERVO_MAX = 80;
Servo left_servo;
Servo right_servo;
double servo_rot; 

void PIDSpeedCalculate(int id) {
  rm_last_speed_error[id] = rm_speed_error[id];

  rm_speed_error[id] = rm_target_speed[id] - rm_speed[id];
  rm_speed_error[id] = rm_speed[id] > 32768 ? rm_speed_error[id] + 65536 : rm_speed_error[id];

  rm_so[id][0] = rm_sk[id][0] * rm_speed_error[id];
  rm_so[id][1] += (rm_sk[id][1] * rm_speed_error[id]);
  rm_so[id][1] = constrain(rm_so[id][1], -MAX_SO, MAX_SO);
  rm_so[id][2] = rm_sk[id][2] * (rm_speed_error[id] - rm_last_speed_error[id]);

  rm_output[id] = rm_so[id][0] + rm_so[id][1] + rm_so[id][2];
  rm_output[id] = constrain(rm_output[id], -MAX_OUTPUT, MAX_OUTPUT);
}

// CAN bus447
CAN_FRAME tx_msg, rx_msg;

void sendCANFrame() {
  for(int i = 0; i < MOTOR_NUM; i++) {
    tx_msg.data.byte[2 * i] = rm_output[i] >> 8;
    tx_msg.data.byte[2 * i + 1] = rm_output[i];
  }

  Can0.sendFrame(tx_msg);
}

// ROS
void carbaseSubscriberCallback(geometry_msgs::Twist twist, void* arg) {
  (void)(arg);
 
  // angular x
  if (twist.angular.x > 0.5){
    servo_rot += ROTATION_SPEED;
  }
  if (twist.angular.x < -0.5){
    servo_rot -= ROTATION_SPEED;
  }
  servo_rot = constrain(SERVO_MIN, (int)servo_rot, SERVO_MAX);
  left_servo.write(100 + (int)servo_rot);
  right_servo.write(180 - (int)servo_rot);
  
  // angular y clmp open/close
  if (twist.angular.y > 0.5){
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
  } else if (twist.angular.y < -0.5){
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }
  // linear z clamp height
  if (twist.linear.z > 0.5){
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
  } else if (twist.linear.z < -0.5){
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
  }

  motor_driver.getMovement(rm_target_speed, twist.linear.x, twist.linear.y, twist.angular.z);
}


class CarbaseController : public ros2::Node {
  private:
    Subscriber<geometry_msgs::Twist> *carbase_subscriber;

  public:
    CarbaseController() : Node("Carbase_controller_arduino_node") {
      this->carbase_subscriber = this->createSubscriber<geometry_msgs::Twist>("carbase_cmd", (ros2::CallbackFunc)carbaseSubscriberCallback, nullptr);
      
    } 
};

void setup() {
  XRCEDDS_PORT.begin(115200);
  while(!XRCEDDS_PORT);

  for (int i = 2; i < 10; i++){
    pinMode(i, OUTPUT);
  }
  servo_rot = 0;
  left_servo.attach(10);
  right_servo.attach(11);
  left_servo.write(100 + servo_rot);
  right_servo.write(180 - servo_rot);
  
  init(&XRCEDDS_PORT);

  Can0.begin(CAN_BPS_1000K);

  tx_msg.id = 0x200;
  tx_msg.length = 8;

}

void loop() {
  static CarbaseController carbase_controller;

  ros2::spin(&carbase_controller);

  Can0.watchFor();
  Can0.read(rx_msg);

  if(rx_msg.id > 0x200 && rx_msg.id <= (0x200 + MOTOR_NUM)) {
    rm_id = rx_msg.id - 0x201;

    rm_speed[rm_id] = rx_msg.data.byte[2] << 8 | rx_msg.data.byte[3];

    PIDSpeedCalculate(rm_id);

    sendCANFrame();
  }
}
