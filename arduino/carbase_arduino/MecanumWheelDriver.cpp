#include "MecanumWheelDriver.h"

MecanumWheelDriver::MecanumWheelDriver(double lx, double ly, double wheel_radius) {
  this->lx = lx > 0.0 ? lx : 4.0;
  this->ly = ly > 0.0 ? ly : 2.0;
  this->wheel_radius = wheel_radius > 0.0 ? wheel_radius : 5.5;

  for(int i = 0; i < this->NUM_OF_WHEEL; i++) {
    this->carbase_matrix[i][0] = 1;
    this->carbase_matrix[i][1] = 1;
    this->carbase_matrix[i][2] = 1;
  }

  this->carbase_matrix[0][1] *= -1;
  this->carbase_matrix[0][2] *= -1;
  this->carbase_matrix[2][2] *= -1;
  this->carbase_matrix[3][1] *= -1;
}

void MecanumWheelDriver::getMovement(int* movement, double linear_x, double linear_y, double angular_z) {
  static double move_vel = 0.0;

  for(int i = 0; i < this->NUM_OF_WHEEL; i++) {
    move_vel = (this->carbase_matrix[i][0] * linear_x + this->carbase_matrix[i][1] * linear_y + this->carbase_matrix[i][2] * angular_z) / this->wheel_radius;
    move_vel *= 5000;
    move_vel = constrain(move_vel, -this->MAX_RPM, this->MAX_RPM);

    if(i % 2 == 1)
      move_vel *= -1;

    movement[i] = (int)move_vel;
  }
}
