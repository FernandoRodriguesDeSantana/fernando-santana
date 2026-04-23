/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "wheel.h"
#include "threeeyes_task.h"
#include "wheel_task.h"
#include "imu_task.h"
#include <inttypes.h>
// Para  a implementação da Queue
#include "freertos/queue.h"
// Implementação do dicionário de estados "nav_types.h"
#include "nav_types.h"
// Implementação da biblioteca do sensor de linha
#include "ir_line_task.h"

#define THREE_EYES_TASK
#define IMU_TASK
#define WHEEL_CTRL_TASK

extern void IR_Line_Task(void *arg);
extern void Control_Task(void *arg);

void app_main(void)
{
    wheel_Init();
    QueueHandle_t fila_navegacao = xQueueCreate(10, sizeof(nav_msg_t));
    
    if(fila_navegacao == NULL)
    {
        ESP_LOGE("MAIN", "Falta de memoria RAM na criação da fila!");
        return;
    }

    // Para guardar o handle da task do sensor de linha (ir_line_task.c)
    TaskHandle_t handle_ir_task = NULL;

    // Criação da task do sensor de linha
    xTaskCreate(IR_Line_Task, 
                "ir_line", 
                configMINIMAL_STACK_SIZE*3, 
                (void*) fila_navegacao, 
                5, 
                &handle_ir_task);
    
    // Preparação da mensagem para o ultrassom
    static system_context_t contexto;
    contexto.fila_navegacao = fila_navegacao;
    contexto.handle_ir_task = handle_ir_task;

    // xTaskCreate(Control_Task, "Control_Task", 4096, (void *)fila_navegacao, 4, NULL);

    #ifdef THREE_EYES_TASK
        xTaskCreate(Threeeyes,
                    "threeeyes",
                    configMINIMAL_STACK_SIZE*3,
                    &contexto,
                    5,
                    NULL);
    #endif

    #ifdef IMU_TASK
        xTaskCreate(IMU_Task,
                    "imu",
                    configMINIMAL_STACK_SIZE*3,
                    NULL,
                    5,
                    NULL);
    #endif

    #ifdef WHEEL_CTRL_TASK
        xTaskCreate(wheel_ctrl,
                    "wheel",
                    configMINIMAL_STACK_SIZE*3,
                    (void*) fila_navegacao,
                    5,
                    NULL);
    #endif
}