#ifndef AXIS9_SENSOR_H_
#define AXIS9_SENSOR_H_

#include <Message.hpp>
#include "SbVector3.h"

typedef struct Axis9SensorType
{
    vector3_t acc;
    vector3_t gyro;
    vector3_t mag;
} axis9sensor_t;

typedef sb::Message<axis9sensor_t> Axis9Sensor;

#endif