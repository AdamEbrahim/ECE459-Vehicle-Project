#ifndef SIMPLEPID_H
#define SIMPLEPID_H

#include <cmath>

class SimplePID {
    private:
        float kp, kd, ki, umax; // Parameters
        float eprev, eintegral; // Storage
        float angleWrap(float radians);
        float angleWrapDeg(float degrees);

    public:
        SimplePID();

        void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);

        void evalu(int value, int target, float deltaT, int &pwr, int &dir, int a);
        void evaluAngle(float value, float target, float deltaT, int &pwr, int &dir);
};

#endif