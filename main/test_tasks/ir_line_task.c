#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "nav_types.h"
#include "ir_line_task.h"
#include "freertos/queue.h"
#include "esp_err.h"

static const char *TAG = "IR_Line"; // Etiqueta para logs no terminal

// Definição física dos pinos conforme o esquemático
#define INFRA_RED_VERY_LEFT_GPIO  GPIO_NUM_8
#define INFRA_RED_LEFT_GPIO       GPIO_NUM_18
#define INFRA_RED_MIDDLE_GPIO     GPIO_NUM_17
#define INFRA_RED_RIGHT_GPIO      GPIO_NUM_16
#define INFRA_RED_VERY_RIGHT_GPIO GPIO_NUM_15

// Máscaras binárias para identificar cada combinação de sensores ativos
#define IR_LINE_VERY_VERY_LEFT  (1ULL << INFRA_RED_VERY_LEFT_GPIO)
#define IR_LINE_VERY_LEFT       ((1ULL << INFRA_RED_VERY_LEFT_GPIO) | (1ULL << INFRA_RED_LEFT_GPIO))
#define IR_LINE_LEFT            (1ULL << INFRA_RED_LEFT_GPIO)
#define IR_LINE_LEFT_MIDDLE     ((1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO))
#define IR_LINE_MIDDLE          (1ULL << INFRA_RED_MIDDLE_GPIO)
#define IR_LINE_RIGHT_MIDDLE    ((1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO))
#define IR_LINE_RIGHT           (1ULL << INFRA_RED_RIGHT_GPIO)
#define IR_LINE_VERY_RIGHT      ((1ULL << INFRA_RED_RIGHT_GPIO) | (1ULL << INFRA_RED_VERY_RIGHT_GPIO))
#define IR_LINE_VERY_VERY_RIGHT (1ULL << INFRA_RED_VERY_RIGHT_GPIO)

// Máscara total para configurar todos os pinos IR como entrada de uma vez
#define INFRA_RED_OUT_GPIO_MASK ((uint64_t)((1ULL << INFRA_RED_VERY_LEFT_GPIO) | (1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO) | (1ULL << INFRA_RED_VERY_RIGHT_GPIO)))

portTASK_FUNCTION(IR_Line_Task, arg)
{
    QueueHandle_t fila_navegacao = (QueueHandle_t) arg; // Resgata a fila do argumento
    nav_msg_t mensagem_envio;   // Estrutura para o envio do estado

    robot_state_t ultimo_estado_conhecido = LINHA_CENTRO;   // Memória para quando perder a linha

    // Configuração técnica dos pinos GPIO
    gpio_config_t ir_line_config = {
            .pin_bit_mask = INFRA_RED_OUT_GPIO_MASK,    // Aplica a máscara dos 5 pinos
            .mode = GPIO_MODE_INPUT,    // Define como entrada
            .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Sem resistor de pull-down
            .pull_up_en = GPIO_PULLUP_DISABLE,  // Sem resistor de pull-up (hardware já possui)
            .intr_type = GPIO_INTR_DISABLE  // Sem interrupções
    };

    gpio_config( &ir_line_config ); // Variável para armazenar o valor bruto dos pinos

    uint64_t gpioValue;

    ESP_LOGI(TAG, "Tarefa de leitura IR iniciada!");

    while (1) {
        // Lê o estado de cada pino e desloca para sua posição binária original
        gpioValue = (uint64_t)gpio_get_level(INFRA_RED_VERY_LEFT_GPIO) << INFRA_RED_VERY_LEFT_GPIO |
                (uint64_t)gpio_get_level(INFRA_RED_LEFT_GPIO) << INFRA_RED_LEFT_GPIO|
                (uint64_t)gpio_get_level(INFRA_RED_MIDDLE_GPIO) << INFRA_RED_MIDDLE_GPIO|
                (uint64_t)gpio_get_level(INFRA_RED_RIGHT_GPIO) << INFRA_RED_RIGHT_GPIO|
                (uint64_t)gpio_get_level(INFRA_RED_VERY_RIGHT_GPIO) << INFRA_RED_VERY_RIGHT_GPIO;
        
        // INVERSÃO CRUCIAL: O sensor envia 0 no preto. Inverteu-se para que o código entenda 1 como "vi a linha"
        gpioValue = ~gpioValue;

        // Limpa bits indesejados (outros pinos do ESP32 que não são do sensor)
        gpioValue &= INFRA_RED_OUT_GPIO_MASK;

        // Classifica a leitura bruta em um dos 9 estados de navegação
        switch(gpioValue)
        {
        case IR_LINE_VERY_VERY_LEFT:
            mensagem_envio.estado_atual = LINHA_EXTREMA_ESQUERDA;
            ultimo_estado_conhecido = LINHA_EXTREMA_ESQUERDA;
            break;
        case IR_LINE_VERY_LEFT:
            mensagem_envio.estado_atual = LINHA_MUITO_ESQUERDA;
            ultimo_estado_conhecido = LINHA_MUITO_ESQUERDA;
            break;
        case IR_LINE_LEFT:
            mensagem_envio.estado_atual = LINHA_ESQUERDA;
            ultimo_estado_conhecido = LINHA_ESQUERDA;
            break;
        case IR_LINE_LEFT_MIDDLE:
            mensagem_envio.estado_atual = LINHA_CENTRO_ESQUERDA;
            ultimo_estado_conhecido = LINHA_CENTRO_ESQUERDA;
            break;
        case IR_LINE_MIDDLE:
            mensagem_envio.estado_atual = LINHA_CENTRO;
            ultimo_estado_conhecido = LINHA_CENTRO;
            break;
        case IR_LINE_RIGHT_MIDDLE:
            mensagem_envio.estado_atual = LINHA_CENTRO_DIREITA;
            ultimo_estado_conhecido = LINHA_CENTRO_DIREITA;
            break;
        case IR_LINE_RIGHT:
            mensagem_envio.estado_atual = LINHA_DIREITA;
            ultimo_estado_conhecido = LINHA_DIREITA;
            break;
        case IR_LINE_VERY_RIGHT:
            mensagem_envio.estado_atual = LINHA_MUITO_DIREITA;
            ultimo_estado_conhecido = LINHA_MUITO_DIREITA;
            break;
        case IR_LINE_VERY_VERY_RIGHT:
            mensagem_envio.estado_atual = LINHA_EXTREMA_DIREITA;
            ultimo_estado_conhecido = LINHA_EXTREMA_DIREITA;
            break;
        default:
            // Se não houver leitura válida (linha perdida), mantém a última direção
            mensagem_envio.estado_atual = ultimo_estado_conhecido;
            break;
        }

        // Envia o estado decidido para a tarefa dos motores através da fila
        if(fila_navegacao != NULL) {
            xQueueSend(fila_navegacao, &mensagem_envio, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}