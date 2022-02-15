#include "KalmanFilter1D.h"

KalmanFilter1D::KalmanFilter1D(double init_angle)
{
    this->angle = init_angle;
    this->K = 0.0;
    this->P = 0.0;
    this->Q_angle = 0.001;
    this->R_measure = 0.03;
}

void KalmanFilter1D::set_state_covariance(double a)
{
    this->Q_angle = a;
}
void KalmanFilter1D::set_measurement_covariance(double a)
{
    this->R_measure = a;
}
double KalmanFilter1D::get_state(double x, double x_dot, double dt)
{
    angle = angle + x_dot*dt;
    P = P + Q_angle*dt;
    double y = x - angle;
    double S = P + R_measure;
    K = P/S;
    angle = angle + K*y;
    P = (1-K)*P;
    return angle;
}