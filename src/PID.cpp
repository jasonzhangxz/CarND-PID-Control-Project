#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  p_error = 0;
  i_error = 0;
  d_error = 0;

  pre_cte = 0;

  err_sum = 0;
  counter = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = cte - pre_cte;
  i_error += cte;
  pre_cte = cte;

  err_sum += cte;

}

double PID::TotalError() {
  return Kp*p_error+Ki*i_error+Kd*d_error;
}

double PID::AverageError() {
  return err_sum/counter;
}
