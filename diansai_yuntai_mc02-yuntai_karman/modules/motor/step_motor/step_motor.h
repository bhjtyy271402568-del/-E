#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>
#include <stdbool.h>



// 步进电机控制命令
void step_sendcmd(uint8_t addr, uint8_t *cmd, uint16_t len);

#endif  