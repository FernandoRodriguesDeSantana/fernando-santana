#ifndef NAV_TYPES_H
#define NAV_TYPES_H

// Estrutura de estados que o robô pode assumir baseado no sensor de linha
typedef enum {
    LINHA_PERDIDA = 0,      // Nenhum sensor detectou a linha preta
    LINHA_EXTREMA_ESQUERDA, // Apenas o sensor 1 detectou a linha preta
    LINHA_MUITO_ESQUERDA,   // Sensores 1 e 2 juntos
    LINHA_ESQUERDA,         // Sensor 2 sozinho
    LINHA_CENTRO_ESQUERDA,  // Sensores 2 e 3 juntos
    LINHA_CENTRO,           // Sensor 3 (Meio) sozinho
    LINHA_CENTRO_DIREITA,   // Sensores 3 e 4 juntos
    LINHA_DIREITA,          // Sensor 4 sozinho
    LINHA_MUITO_DIREITA,    // Sensores 4 e 5 juntos
    LINHA_EXTREMA_DIREITA,  // Sensor 5 sozinho
    OBSTACULO_DETECTADO
}   robot_state_t;

// Estrutura da mensagem enviada através da Fila (FreeRTOS Queue)
typedef struct {
    robot_state_t estado_atual;
} nav_msg_t;

// Contexto do sistema para não se usar variáveis globais, considerando que a função xTaskCreate aceita apenas um argumento
typedef struct {
    QueueHandle_t fila_navegacao;   // Handle da fila para comunicação
    TaskHandle_t handle_ir_task;    // Handle da tarefa IR para suspensão/retomada
} system_context_t;

#endif // NAV_TYPES_H