#ifndef MECANUM_WHEEL_DRIVER
#define MECANUM_WHEEL_DRIVER

#define DEGREE_2_RADIAN 0.0174532925f
#define M_PI            3.14159265358979323846f

#include "Arduino.h"

using namespace std;

class MecanumWheelDriver {
  private:
    const static int MAX_RPM = 10000;
    const static int NUM_OF_WHEEL = 4;

    double lx, ly, wheel_radius;
    double carbase_matrix[NUM_OF_WHEEL][NUM_OF_WHEEL - 1];

  public:
    MecanumWheelDriver(double, double, double);

    void getMovement(int*, double, double, double);
};


#endif
