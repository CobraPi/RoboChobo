#ifndef _UTILS_H
#define _UTILS_H

#include "math.h"
#include <Arduino.h>

float alphaFilter(float currentValue, float previousValue, float alpha);

int inputExponential(int expo, long int value, int inputMin, int inputMax);

#endif // _UTILS_H
