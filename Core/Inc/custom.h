#ifndef __CUSTOM_H
#define __CUSTOM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

// --- FINTO HARDWARE LED ---
typedef enum {
  LED2 = 0,
  LED_CUSTOM = 1
} Led_TypeDef;

#define BSP_LED_Init(x)
#define BSP_LED_On(x)     HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET)
#define BSP_LED_Off(x)    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET)
#define BSP_LED_Toggle(x) HAL_GPIO_TogglePin(GPIOB, LED1_Pin)

// --- FINTO HARDWARE BOTTONE ---
typedef enum {
  BUTTON_KEY = 0
} Button_TypeDef;

#define BSP_PB_Init(...)
#define BSP_PB_GetState(...) 0

// --- FINTO HARDWARE SERIALE (COM Port) ---
typedef enum {
  COM1 = 0
} COM_TypeDef;

#define BSP_COM_Init(...)

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_H */
