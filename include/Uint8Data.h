#ifndef UINT8_DATA_H_
#define UINT8_DATA_H_

#include <Message.hpp>

typedef struct Uint8DataType
{
    uint8_t c;
} uint8data_t;

typedef sb::Message<uint8data_t> Uint8Data;

#endif