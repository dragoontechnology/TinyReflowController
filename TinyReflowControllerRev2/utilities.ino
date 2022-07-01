/**
 * @file helper.ino
 * @author Carlos Murphy (carlosmurphy@dragoon.tech)
 * @brief Contains useful functions
 * @version 0.1
 * @date 2021-05-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include "Arduino.h"
#include "utilities.h"

/**
 * @brief limits the input to the given min max
 * 
 * @param value float value to be limited
 * @param min float minimum allowable value
 * @param max float maximum allowable value
 * @return float 
 */
float limf(float value, float min, float max){
  if (value < min){
    return min;
  }
  else if (value > max){
    return max;
  }
  return value;
}


/**
 * @brief limits the input to the given min max
 * 
 * @param value double value to be limited
 * @param min double minimum allowable value
 * @param max double maximum allowable value
 * @return double 
 */
double limd(double value, double min, double max){
  if (value < min){
    return min;
  }
  else if (value > max){
    return max;
  }
  return value;
}

/**
 * @brief limits the input to the given min max
 * 
 * @param value value to be limited
 * @param min minimum allowable value
 * @param max maximum allowable value
 * @return int 
 */
int limi(int value,int min,int max){
  if (value < min){
    return min;
  }
  else if (value > max){
    return max;
  }
  return value;
}

/**
 * @brief Simple Infinite Impulse Response Filter for int
 * 
 * @param newVal value we are going to filter
 * @param oldVal previously measured value
 * @param weight int 0-100 weight we want to give new values 0 = do not use new values, 100 = trust new value 100%
 * @return int post filtering value
 */
int IIRfilteri(int newVal, int oldVal, int weight){
  float newBit = ((float(weight)/100) * float(newVal));
  float oldBit = ((1-(float(weight)/100)) * float(oldVal));
  return (int(newBit + oldBit));
}

/**
 * @brief Simple Infinite Impulse Response Filter for doubles
 * 
 * @param newVal value we are going to filter
 * @param oldVal previously measured value
 * @param weight int 0-100 weight we want to give new values 0 = do not use new values, 100 = trust new value 100%
 * @return double post filtering value
 */
double IIRfilterd(double newVal, double oldVal, int weight){
  double newBit = ((double(weight)/100) * newVal);
  double oldBit = ((1-(double(weight)/100)) * oldVal);
  return (newBit + oldBit);
}

/**
 * @brief Simple Infinite Impulse Response Filter for floats
 * 
 * @param newVal value we are going to filter
 * @param oldVal previously measured value
 * @param weight int 0-100 weight we want to give new values 0 = do not use new values, 100 = trust new value 100%
 * @return float post filtering value
 */
float IIRfilterf(float newVal, float oldVal, int weight){
  return (((float(weight)/100) * newVal) + ((1-(float(weight)/100)) * oldVal));
}

/**
 * @brief Construct a new Moving Average:: Moving Average object
 * 
 * @param filterSize int, the length of the moving average window
 */
MovingAverage::MovingAverage(unsigned int filterSize){
  this->_filterSize = filterSize;
}

/**
 * @brief Run the moving average by inputting one new value
 * 
 * @param val double, new value to add to average
 * @return double Average of the last "filter size" samples input
 */
double MovingAverage::run(double val){
  _sum -= _filterArray[_filterIndex];
  _filterArray[_filterIndex] = val;
  _sum += val;
  _filterIndex = (_filterIndex + 1) % _filterSize;
  return (_sum/_filterSize);
}

/**
 * @brief Construct a new Rate Limitd:: Rate Limitd object
 * 
 * @param rate unsigned int - units/ms which is the maximum +-rate of the inputs change
 */
RateLimitd::RateLimitd(float rate){
  _rate = rate;
  _oldMillis = millis();
}

/**
 * @brief Takes a new value and checks that it's change is less than the given limit +-
 * 
 * @param newVal double - new value to check against rate limit
 * @return double, the input if within limits else the maximum limit allowed
 */
double RateLimitd::limit(double newVal){
  unsigned int _timeDelta = millis() - _oldMillis;
  double _currentRate = (newVal - _oldVal)/_timeDelta;
  if (_currentRate > _rate){
    return (_oldVal + (_rate*_timeDelta));
  }
  else if (_currentRate < (-1 * _rate)){
    return (_oldVal - (_rate*_timeDelta));
  }
  else{
    return newVal;
  }
}

/**
 * @brief Construct a new Rate Limiti:: Rate Limiti object
 * 
 * @param rate float - units/ms which is the maximum +-rate of the inputs change
 */
RateLimiti::RateLimiti(float rate){
  _rate = rate;
  _oldMillis = millis();
}

/**
 * @brief Takes a new value and checks that it's change is less than the given limit +-
 * 
 * @param newVal int - new value to check against rate limit
 * @return double, the input if within limits else the maximum limit allowed
 */
int RateLimiti::limit(int newVal){
  unsigned int _timeDelta = millis() - _oldMillis;
  float _currentRate = (newVal - _oldVal)/_timeDelta;
  if (_currentRate > _rate){
    return int((_oldVal + (_rate*_timeDelta)));
  }
  else if (_currentRate < (-1 * _rate)){
    return int((_oldVal - (_rate*_timeDelta)));
  }
  else{
    return newVal;
  }
}

/**
 * @brief Construct a new PID::PID object
 * 
 * @param P proportional  term
 * @param I integral term
 * @param D derivative term
 * @param mininum minimum output
 * @param maximum maximum output
 */
PID::PID(float P, float I, float D, float mininum, float maximum){
  KP = P;
  KI = I;
  KD = D;
  min = mininum;
  max = maximum;
}

float PID::calculate(float value, float setpoint){
  float _Prop;
  float _D;
  float command = 0.0;
  float deltaTime = float(millis() - _lastTime) / 1000.0;
  //protect against divide by zero
  if (deltaTime <= 0.0){
    deltaTime = 0.00000001;
  }
  float error = setpoint - value;

  _Prop = KP * error;
  if (deltaTime < 0.5){
    I = I + deltaTime*KI*error;
  }// don't iterate I if deltaTime is too big.  Otherwise the first run can be large.

  I = constrain(I, min, max);
  _D = KD * (1.0/deltaTime) * (_lastValue - value);

  command = constrain(_Prop+I+_D, min, max);
  _lastValue = value;
  _lastTime = millis();
  return command;
}//PID::calculate()


// calculate running IIR average of 100 cycles in ms.
void Profiling::Profile_stop(){
  uint32_t time = 0; 

  time = micros() - start_micros;

  avg_time = avg_time*0.99 + float(time)*0.01;


  //avg_time = float(micros() - start_micros)*0.01 + avg_time*0.99;
}// stoptime

// return current average block time cycles, in us
uint16_t Profiling::Profile_time(){
  return avg_time;
}// profile_time
