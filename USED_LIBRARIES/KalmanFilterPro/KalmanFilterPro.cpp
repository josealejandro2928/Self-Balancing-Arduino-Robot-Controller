#include "KalmanFilterPro.h"

KalmanFilterPro::KalmanFilterPro(float init_angle)
{
    this->angle = init_angle;
    this->K[0] = 0.0;
    this->K[1] = 0.0;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            this->P[i][j] = 0.0;
        }
    }
    this->Q_angle = 0.007;
    this->Q_gyro_bias = 0.03;
    this->R_measure = 0.05;
    this->bias = 0.0;
    this->rate = 0.0;
}

void KalmanFilterPro::set_state_covariance(float var_state, float var_input)
{
    this->Q_angle = var_state;
    this->Q_gyro_bias = var_input;
}

void KalmanFilterPro::set_measurement_covariance(float var_measurement)
{
    this->R_measure = var_measurement;
}

float KalmanFilterPro::get_state(float x, float x_dot, float dt)
{
    rate = x_dot - bias;
    angle = angle + rate * dt;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + this->Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += this->Q_gyro_bias * dt;

    float y = x - angle;
    float S = P[0][0] + R_measure;

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return this->angle;
}