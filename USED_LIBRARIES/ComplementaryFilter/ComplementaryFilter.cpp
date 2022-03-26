#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(){
    this->A = 0.0;
    this->B = 0.0;
    this->variable = 0.0;
}

ComplementaryFilter::ComplementaryFilter(float A,float B,float x){
    this->A = A;
    this->B = B;
    this->variable = x;
}

float ComplementaryFilter::get_measurement(float x,float x_dot,float dt){
    this->variable = this->A*(this->variable + x_dot*dt)+this->B*x;
    return variable;
}

void ComplementaryFilter::set_constants(float a1,float a2){
    this->A = a1;
    this->B = a2;
}