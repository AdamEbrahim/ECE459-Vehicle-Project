#include <SimplePID.h>
#include <cmath>

SimplePID::SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {

}

void SimplePID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
}

// A function to compute the control signal; value = curr pos of motor, target = target pos of motor 
void SimplePID::evalu(int value, int target, float deltaT, int& pwr, int& dir, int a){
    // error
    int e;
    if (a == 0) { //right motor
        e = target - value;
    } else { //left motor
        e = value - target;
    }

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
        pwr = umax;
    }

    // motor direction
    dir = 1;
    if(u<0){
        dir = -1;
    }

    // store previous error
    eprev = e;
}

float SimplePID::angleWrap(float radians) {
    while(radians > M_PI){
        radians -= 2 * M_PI;
    }
    while(radians < -M_PI){
        radians += 2 * M_PI;
    }
    return radians;
}

float SimplePID::angleWrapDeg(float degrees) {
    while(degrees > 180){
        degrees -= 360;
    }
    while(degrees < -180){
        degrees += 360;
    }
    return degrees;
}


void SimplePID::evaluAngle(float value, float target, float deltaT, int& pwr, int& dir){
    // error
    float e;
    // if (a == 0) { //right motor
        e = target - value;
    // } else { //left motor
    //     e = value - target;
    // }

    e = angleWrapDeg(e);

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
        pwr = umax;
    }

    // motor direction
    dir = 1;
    if(u<0){
        dir = -1;
    }

    // store previous error
    eprev = e;
}