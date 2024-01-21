/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMPXX80.h"
#include "lcd_i2c.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature_f;
float temperaturaReferencyjna =  27.5;

char aktualnaTemperatura_ch[24];

int32_t pressure;
int32_t counter = 0;

const float Tp = 1.0f;
const float U_saturation = 1.f;
const uint32_t max_pulse = 999;

float pulse = 1.f;
uint16_t pulseOut = 0;

char odczyt[1];

struct PID
{
float U;
float Kp;
float Ki;
float Kd;
float UP, UD, UI;
float previousIn;
float error;
float previousError;
float Tp;
};

struct PID pid;

struct lcd_disp disp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calcPID(float zadanaTemperatura, float aktualnaTemperatura, struct PID *PID)
{
//uchyb
PID->error = zadanaTemperatura - aktualnaTemperatura;
//sygnały sterujące
PID->UP = PID->Kp * PID->error;
PID->UI = PID->Ki * PID->Tp / 2.0 * (PID->error + PID->previousError) + PID->previousIn;
PID->UD = PID-> Kd *(PID->error - PID->previousError) / PID->Tp;

PID->previousError = PID->error;
PID->previousIn = PID->UI;
//sygnał sterujący
PID->U = PID->UP + PID->UI + PID->UD;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //CZUJNIK BPM280
  BMP280_Init(&hspi1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
  //PID
  pid.U = 0;
  pid.Kp = 0.1152;
  pid.Ki = 0.0002159;
  pid.Kd = -0.1479;
  pid.UP = 0;
  pid.UD = 0;
  pid.UI = 0;
  pid.error = 0;
  pid.previousIn = 0;
  pid.previousError = 0;
  pid.Tp = 1.f;

  //wyswietlacz LCd

  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance == TIM3)
{
	BMP280_ReadTemperatureAndPressure(&temperature_f, &pressure);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseOut);


	if(temperaturaReferencyjna < 20.0)
		{
		temperaturaReferencyjna = 20.0;
		}
	else if(temperaturaReferencyjna > 50.0)
		{
		temperaturaReferencyjna = 50.0;
		}

	calcPID(temperaturaReferencyjna, temperature_f, &pid);

	pulse = htim1.Init.Period * pid.U;

	if(pulse < 0.0)
		{
		pulseOut = 0;
		}
	else if(pulse > htim1.Init.Period)
		{
		pulseOut = htim1.Init.Period;
		}
	else
		{
		pulseOut = (uint16_t) pulse;
		}

	sprintf(aktualnaTemperatura_ch, "%f : %f \n\r", temperature_f, temperaturaReferencyjna);
	HAL_UART_Transmit(&huart3, (uint8_t *)aktualnaTemperatura_ch,
	sizeof(aktualnaTemperatura_ch)-1, 1000);

	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);


			switch(odczyt[0])
			{
				case 'up':
				{
					temperaturaReferencyjna += 0.5;
					break;
				}
				case 'down':
				{
					temperaturaReferencyjna -= 0.5;
					break;
				}
			}
		HAL_UART_Receive_IT(&huart3, odczyt, 1);

	}
}

void update_and_display_data(float aktualnaTemperatura, long temperaturaZadana) {
    // Inicjalizacja struktury LCD_Display
    LCD_Display disp;

    // Aktualizacja pierwszej linii
    sprintf((char *)disp.f_line, "Temp. %.2f C", aktualnaTemperatura);
    lcd_display(&disp);

    // Aktualizacja drugiej linii
    sprintf((char *)disp.s_line, "Temp. %.2f Pa", temperaturaZadana);
    lcd_display(&disp);

    // Opóźnienie na pół sekundy
    HAL_Delay(500);
}

void control_fan(float aktualnaTemperatura, float temperaturaZadana) {
    // Sprawdzenie warunku
    if (aktualnaTemperatura > temperaturaZadana) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
    } else {

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    }

    // Opóźnienie 1000 ms (1 sekunda)
    HAL_Delay(500);
}
void adjustTemperature() {
    static int prev_count = 0;
    int count = __HAL_TIM_GET_COUNTER(&htim1);


        if (count > prev_count) {
        	temperaturaZadana = temperaturaZadana + 1;
        }

        if (prev_count > count) {
        	temperaturaZadana = temperaturaZadana - 1;
        }

        prev_count = count;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
