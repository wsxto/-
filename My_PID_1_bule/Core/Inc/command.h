#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"
uint8_t CommandBuffer_Write(uint8_t *data, uint8_t length);
uint8_t CommandBuffer_GetCommand(uint8_t *command);
#endif // COMMAND_H
