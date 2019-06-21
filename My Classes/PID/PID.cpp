
extern "C"
{
#include "math.h"
}

#include "PID.h"

//@Default costructor for Arduino PWM motor control
PID::PID()
{
    this->Kc = 5.0;
    this->Kd = 1.25;
    this->Ki = 0.5;
    this->max_output = 255;
    this->threshold = 0.0;
    this->reset();
}

//@Constructor for a given constant set of pid parameters and a max output signal
PID::PID(double iKc, double iKi, double iKd, double ithreshold, double imax_output)
{
    this->Kc = iKc;
    this->Kd = iKd;
    this->Ki = iKi;
    this->max_output = imax_output;
    this->threshold = ithreshold;
}
void PID::reset()
{
    this->error = 0.0;
    this->l_error = 0.0;
    this->l2_error = 0.0;
    this->sum_error = 0.0;
    this->l_u_signal = 0.0;
    this->u_signal = 0.0;
}
double PID::constraint(double a, double b, double x)
{
    if (x > a && x < b)
        return x;
    if (x <= a)
        return a;
    else
        return b;
}

void PID::update_state()
{
    l2_error = l_error;
    l_error = error;
    sum_error = constraint(-100, 100, (sum_error + error));
    l_u_signal = u_signal;
}

//@ getting the signal in order to control our variable,
//@ dt is the time interval between the last compute and now
double PID::classic_compute(double setpoint, double variable, double dt)
{
    error = setpoint - variable;
    if (abs(error) <= threshold)
    {
        return 0.0;
    }
    double P = Kc * error;
    double I = Ki * sum_error * dt;
    double D = Kd * (error - l_error) / dt;
    double output = P + I + D;
    u_signal = constraint(-1.0 * max_output, max_output, output);
    ///////////////////////////////update the state////////////////////////////////////
    update_state();
    return u_signal;
}

double PID::positional_compute(double setpoint, double variable)
{
    error = setpoint - variable;
    double P = Kc * error;
    double I = Ki * error;
    double D = Kd * (error - l_error);
    double output = P + I + D + l_u_signal;
    u_signal = constraint(-1.0 * max_output, max_output, output);
    ///////////////////////////////update the state////////////////////////////////////
    update_state();
    return u_signal;
}
double PID::velocity_compute(double setpoint, double variable)
{
    error = setpoint - variable;
    double P = Kc * (error - l_error);
    double I = Ki * error;
    double D = Kd * ((error - 2 * l_error + l2_error));
    double output = P + I + D + l_u_signal;
    u_signal = constraint(-1.0 * max_output, max_output, output);
    ///////////////////////////////update the state////////////////////////////////////
    update_state();
    return u_signal;
}
void PID::set_constant(double a, double b, double c)
{
    this->Kc = a;
    this->Ki = b;
    this->Kd = c;
}