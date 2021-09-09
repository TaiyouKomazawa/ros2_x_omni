#ifndef SB_VECTOR_3_H_
#define SB_VECTOR_3_H_

#include <Message.hpp>

typedef struct Vector3Type
{
    float x;
    float y;
    float th;
} vector3_t;

typedef sb::Message<vector3_t> SbVector3;

#endif