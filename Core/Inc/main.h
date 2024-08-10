/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOA
#define AMP_ENABLE_Pin GPIO_PIN_3
#define AMP_ENABLE_GPIO_Port GPIOA
#define NEO_Pin GPIO_PIN_0
#define NEO_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_7
#define SD_DETECT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct {
    uint8_t  ChunkID[4];        // "RIFF"
    uint32_t ChunkSize;         // Size of the rest of the file
    uint8_t  Format[4];         // "WAVE"
    uint8_t  Subchunk1ID[4];    // "fmt "
    uint32_t Subchunk1Size;     // 16 for PCM
    uint16_t AudioFormat;       // PCM = 1
    uint16_t NumChannels;       // Mono = 1, Stereo = 2
    uint32_t SampleRate;        // 44100, 48000, etc.
    uint32_t ByteRate;          // SampleRate * NumChannels * BitsPerSample/8
    uint16_t BlockAlign;        // NumChannels * BitsPerSample/8
    uint16_t BitsPerSample;     // 8 bits = 8, 16 bits = 16
    uint8_t  Subchunk2ID[4];    // "data"
    uint32_t Subchunk2Size;     // Number of bytes in the data
} WAV_HeaderTypeDef;

typedef enum
{
	MAX_ERROR = 0,  	/* 0 */
	MAX_SUCCESS, 		/* 1 */
	MAX_MOUNTERR, 		/* 2 */
	MAX_OPENERR, 		/* 3 */
	MAX_READHEADERERR, 	/* 4 */
	MAX_READERR, 		/* 5 */
	MAX_HALERR, 		/* 6 */
	MAX_BUFFERFAIL		/* 7 */

} MAXRESULT;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
