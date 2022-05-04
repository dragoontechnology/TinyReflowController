#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

// Unit Conversions:
#define CC_TO_GRAMS 0.730  // for gasoline.  Ranges from ~0.72 to 0.74
#define GRAMS_TO_CC 1.370  



float limf(float value, float min, float max);
int limi(int value,int min,int max);
int IIRfilteri(int newVal, int oldVal, int weight);
double IIRfilterd(double newVal, double oldVal, int weight);
float IIRfilterf(float newVal, float oldVal, int weight);

class MovingAverage {
public: 
    MovingAverage(unsigned int filterSize);
    double run(double val);
private: 
    unsigned int _filterSize;
    unsigned int _filterArray[20];
    int _filterIndex = 0;
    double _sum = 0;
};

class RateLimitd{
public:
    RateLimitd(float rate);
    double limit(double newVal);
private:
    float _rate;
    unsigned int _oldMillis;
    double _oldVal = 0;
};

class RateLimiti{
public:
    RateLimiti(float rate);
    int limit(int newVal);
private:
    float _rate;
    unsigned int _oldMillis;
    double _oldVal = 0;
};

class PID {
public: 
    PID(float KP, float KI, float KD, float min = 0.0, float max = 1000.0);
    float calculate(float value, float setpoint);
    float KP,KI,KD,min,max,I;
private: 
    float _lastValue,_old;
    int32_t _lastTime = 0;
};

/**
 * @brief Calculates time to run a block of code 100 times in ms. 
 * 
 */
class Profiling {
public:
    void Profile_start(){start_micros = micros();};  // put at start of block being inspected
    void Profile_stop(); // put at end of block being inspected. Updages run time average.
    uint16_t Profile_time();  // average time to run block in integer microseconds
private:
    uint32_t start_micros = 0;
    float avg_time = 0.0;  // running average IIR, microseconds to run the block.
};
Profiling Profiler;

#endif
