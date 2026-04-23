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
#include "nav_types.h"
#include "freertos/queue.h"

const static char *TAG = "wheels";

portTASK_FUNCTION(wheel_ctrl, arg)
{
  // Resgatando a fila recebida da função main.c
  QueueHandle_t fila_navegacao = (QueueHandle_t) arg;

  if(fila_navegacao == NULL)
  { 
    ESP_LOGE(TAG, "Fila não recebida. Encerrando task dos motores");
    vTaskDelete(NULL);
  }

  nav_msg_t mensagem_recebida;

  uint32_t power_left_wheel, power_right_wheel; 

  // Escala de velocidades suavizadas
  const uint32_t VEL_MAX    = BDC_MCPWM_DUTY_TICK_MAX * 0.55;
  const uint32_t VEL_ALTA   = BDC_MCPWM_DUTY_TICK_MAX * 0.50; 
  const uint32_t VEL_MEDIA  = BDC_MCPWM_DUTY_TICK_MAX * 0.45; 
  const uint32_t VEL_BAIXA  = BDC_MCPWM_DUTY_TICK_MAX * 0.40; 
  const uint32_t VEL_RE     = BDC_MCPWM_DUTY_TICK_MAX * 0.40; 
  const uint32_t VEL_PARADO = 0;

  int pL = 0, pR = 0;

  // Memória do estado anterior para o freio
  robot_state_t estado_anterior = LINHA_CENTRO;

  while(1)
  {     
    // Aguarda indefinidamente (portMAX_DELAY) até que uma nova mensagem chegue na fila
    if(xQueueReceive(fila_navegacao, &mensagem_recebida, portMAX_DELAY) == pdTRUE)
    {
      // Freio de transição de estados: Se o robô mudou de estado, dá um pequeno choque de freio
      if(mensagem_recebida.estado_atual != estado_anterior)
      {
          wheel_SetRawSpeed(WHEEL_FORWARD, 0, WHEEL_FORWARD, 0);
          vTaskDelay(pdMS_TO_TICKS(15));  // Delay curto para amortecer a inércia física
      }

      // Atualiza a memória
      estado_anterior = mensagem_recebida.estado_atual;

      // Lógica Diferencial: Define o comportamento de cada roda para cada um dos 9 estados
      switch(mensagem_recebida.estado_atual)
      {
        case LINHA_CENTRO:
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_ALTA, WHEEL_FORWARD, VEL_ALTA);
          break;
        
        case LINHA_CENTRO_ESQUERDA: 
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_MEDIA, WHEEL_FORWARD, VEL_ALTA);
          break;

        case LINHA_ESQUERDA: 
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_BAIXA, WHEEL_FORWARD, VEL_MAX);
          break;

        case LINHA_MUITO_ESQUERDA: // Curva acentuada (Acelera mais a roda de fora)
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_BAIXA, WHEEL_FORWARD, VEL_MAX);
          break;

        case LINHA_EXTREMA_ESQUERDA: // Curva de 90 graus (Gira no próprio eixo com Ré)
          wheel_SetRawSpeed(WHEEL_REVERSE, VEL_RE, WHEEL_FORWARD, VEL_MAX);
          break;

        case LINHA_CENTRO_DIREITA: 
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_ALTA, WHEEL_FORWARD, VEL_MEDIA);
          break;

        case LINHA_DIREITA: // Saiu um pouco mais para a direita
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_MAX, WHEEL_FORWARD, VEL_BAIXA);
          break;

        case LINHA_MUITO_DIREITA: // Curva acentuada para a direita
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_MAX, WHEEL_FORWARD, VEL_BAIXA);
          break;

        case LINHA_EXTREMA_DIREITA: // Curva de 90 graus para a direita (Gira no eixo com Ré)
          wheel_SetRawSpeed(WHEEL_FORWARD, VEL_MAX, WHEEL_REVERSE, VEL_RE);
          break;

        case LINHA_PERDIDA:
        case OBSTACULO_DETECTADO:
          wheel_SetRawSpeed(WHEEL_STOP, VEL_PARADO, WHEEL_STOP, VEL_PARADO);
          break;
      }
    }

    wheel_GetEndoderPulses(&pL, &pR);
    // ESP_LOGI(TAG, "Left encoder: %d\tRight encoder: %d\r\n", pL, pR);
          
    wheel_GetPower(&power_left_wheel, &power_right_wheel);
    // Descomente se quiser continuar monitorando a proteção ADC no terminal:
    // printf("Left ADC: %" PRIu32 "; \t Right ADC: %" PRIu32 ".\n", power_left_wheel, power_right_wheel);
          
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}