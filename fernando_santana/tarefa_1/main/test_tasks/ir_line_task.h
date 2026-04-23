#ifndef IR_LINE_TASK_H
#define IR_LINE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include "driver/gpio.h"

// Macros baseadas no esquemático oficial
#define IR_SENSOR_1_GPIO  GPIO_NUM_15 // IR_OUT1 (Ex: Mais à esquerda)
#define IR_SENSOR_2_GPIO  GPIO_NUM_16 // IR_OUT2
#define IR_SENSOR_3_GPIO  GPIO_NUM_17 // IR_OUT3 (Centro)
#define IR_SENSOR_4_GPIO  GPIO_NUM_18 // IR_OUT4
#define IR_SENSOR_5_GPIO  GPIO_NUM_8  // IR_OUT5 (Ex: Mais à direita)

// Inicializa os pinos
void ir_line_init(void);

// Lê o estado bruto dos sensores
uint8_t ir_line_read_raw(void);

portTASK_FUNCTION(IR_Line_Task, arg);

#endif // IR_LINE_TASK_H