#include "threeeyes_task.h"
#include "nav_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

const static char *TAG = "three_eyes";

portTASK_FUNCTION(Threeeyes, args)
{
    ThreeEyes_Init();   // Inicializa o timer de captura do MCPWM para os 3 sensores
	//ThreeEyes_DisableLeft();
    //ThreeEyes_DisableRight();
    ultrasonic_value_t sensor[3];   // Estrutura para guardar os resultados (distância)
    char *near_sensor_name; // Para debug: nome do sensor que viu o objeto mais perto
    char *sensor_name[] = {"left", "middle", "right"};

    system_context_t *contexto = (system_context_t *) args; // Recebe o malote de contexto

    if (contexto == NULL) {
        ESP_LOGE(TAG, "Erro: Malote de contexto não recebido!");
        vTaskDelete(NULL);
    }

    nav_msg_t mensagem_emergencia;
    mensagem_emergencia.estado_atual = OBSTACULO_DETECTADO; 

    bool obstaculo_presente = false;

	while(1)
	{
        // Envia o gatilho (Trigger) e espera pelo retorno (Echo) usando interrupções
        ThreeEyes_TrigAndWait(portMAX_DELAY);
        ThreeEyes_Read(&sensor[0], &sensor[1], &sensor[2]);
        
        uint32_t min_ticks = 0xFFFFFFFF; 
        near_sensor_name = "none";

        // Varre os 3 sensores para descobrir qual tem o objeto mais próximo
        for ( int i = 0; i < 3; i++ )
        {
            if (sensor[i].isUpdated == pdTRUE && sensor[i].tof_ticks < min_ticks) 
            {
                min_ticks = sensor[i].tof_ticks;
                near_sensor_name = sensor_name[i];
            }
        }

        float distance = (min_ticks * (1000000.0 / esp_clk_apb_freq())) / 58.0;
        ESP_LOGI(TAG, "The sensor with the nearest detected object was: %s (Distance: %.2f cm)", near_sensor_name, distance);
        //printf("The sensor with the nearest detected object was: %s (Distance: %"PRIu32" ticks)\n", near_sensor_name, min_ticks);
        
        if(distance <= 10)
        {
            if(obstaculo_presente == false)
            {
                obstaculo_presente = true;
                ESP_LOGW(TAG, "OBSTÁCULO em %s a %.1f cm! Suspendendo Visão.", near_sensor_name, distance);

                // Interrompe a task do sensor de linha
                if(contexto->handle_ir_task != NULL)
                {
                    vTaskSuspend(contexto->handle_ir_task);
                }

                if(contexto->fila_navegacao != NULL)
                {
                    xQueueReset(contexto->fila_navegacao);
                    xQueueSendToFront(contexto->fila_navegacao, &mensagem_emergencia, 0);
                }
            }
        }
        else
        {
            if(obstaculo_presente == true)
            {
                obstaculo_presente = false;
                ESP_LOGI(TAG, "Caminho livre! Retomando Visão.");

                // Acordando a task do sensor de linha
                if(contexto->handle_ir_task != NULL)
                {
                    vTaskResume(contexto->handle_ir_task);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
	
}