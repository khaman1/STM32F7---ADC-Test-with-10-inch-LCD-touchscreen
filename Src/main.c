/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "adc.h"
/* USER CODE BEGIN Includes */
#include "stm32746g_sdram.h"
#include "stm32746g_LCD.h"
#include "GT811.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint32_t ADC_Value[100];
uint32_t ad1,ad2;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TS_StateTypeDef  TS_State={0};
uint8_t TS_flag ;
uint8_t TouchPoit;
TS_StateTypeDef  TS_BKState;
uint8_t value = 0;
uint16_t i;
uint32_t PointColor[]={LCD_COLOR_BLUE,LCD_COLOR_GREEN,LCD_COLOR_RED,LCD_COLOR_MAGENTA,LCD_COLOR_YELLOW};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void BSP_LCD_DisplayANumber(uint16_t Xpos, uint16_t Ypos, uint16_t number, uint16_t clear_chars)
{
	int n[100],j;
	int cnt = 0;
	int tmp = number;
	
	while(tmp)
	{
		n[cnt] = tmp%10 + 48;
		tmp 		/= 10;
		cnt++;
	}
	
	
	
	
	for(j=0; j<cnt; j++)
		BSP_LCD_DisplayChar(Xpos + 18*(cnt-1-j), Ypos, n[j]);
	
	for(j=cnt; j<clear_chars; j++)
		BSP_LCD_DisplayChar(Xpos + 18*j, Ypos, ' ');
}












#define DEBOUNCE_LIMIT 10
#define ADC_SAMPLES		 2000







/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
//  MX_I2C4_Init();
	MX_ADC1_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	/* Program the SDRAM external device */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
  BSP_SDRAM_Init();   
	BSP_LCD_Init();
	GT811_Init();
	
	BSP_LCD_SetLayerVisible(1,DISABLE);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(BSP_LCD_GetBackColor());
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0,0, (uint8_t*)"Screen Test - Kha Man",CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawHLine(0, 30, 1024);	
	
	
	
	uint8_t	MODE = 0;
	uint8_t debounceCNT = 6;
	
	
	uint32_t avg = 0;
	uint16_t adc_counter = 0;
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(TS_flag == 1)
		{
			GT811_GetState(&TS_State);	
			if(TS_State.touchDetected != 0)
			{
				
				// Clear
				TouchPoit = TS_BKState.touchDetected;
				for(i = 0;i < 5;i++)
				{
					if(TouchPoit & 0x01)
					{
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_FillCircle(TS_BKState.touchX[i],TS_BKState.touchY[i],20);
						//BSP_LCD_DrawVLine(TS_BKState.touchX[i], 55, 580);
						//BSP_LCD_DrawHLine(5, TS_BKState.touchY[i], 1004);	
					}
					TouchPoit >>= 1;
				}
				
				
				// Redraw
				TouchPoit = TS_State.touchDetected;
				for(i = 0;i < 5;i++)
				{
					if(TouchPoit & 0x01)
					{
						if(TS_State.touchY[i] <  75)TS_State.touchY[i] =  75;
						if(TS_State.touchY[i] > 580)TS_State.touchY[i] = 580;
						if(TS_State.touchX[i] <  20)TS_State.touchX[i] =  20;
						if(TS_State.touchX[i] > 1004)TS_State.touchX[i] = 1004;
					
						BSP_LCD_SetTextColor(PointColor[i]);
						BSP_LCD_FillCircle(TS_State.touchX[i],TS_State.touchY[i],20);
						
						
						
						//BSP_LCD_DrawVLine(TS_State.touchX[i], 55, 580);
						//BSP_LCD_DrawHLine(5, TS_State.touchY[i], 1004);
						TS_BKState.touchX[i] = TS_State.touchX[i];
						TS_BKState.touchY[i] = TS_State.touchY[i];
					}
					TouchPoit >>= 1;
				}
				
				// Draw Button
				
				
				if(TS_State.touchX[0] > 840 && TS_State.touchX[0] < 1000
					 && TS_State.touchY[0] > 500 && TS_State.touchY[0] < 550)
				{
					if(debounceCNT >= DEBOUNCE_LIMIT)
					{
						debounceCNT = 0;
						MODE = !MODE;
						
						avg = 0;
						adc_counter = 0;
						
						if(MODE)
							BSP_LCD_DisplayStringAt(0,200, (uint8_t*)"HELLO1",CENTER_MODE);
						else
							BSP_LCD_DisplayStringAt(0,200, (uint8_t*)"HELLO2",CENTER_MODE);
					}
				
					/////////// Calculate ADC average
					
				}
				
				
				debounceCNT++;
				
				BSP_LCD_DisplayStringAt(50,520, (uint8_t*)"BUTTON",RIGHT_MODE);
				BSP_LCD_DrawHLine(850, 510, 132);	
				BSP_LCD_DrawHLine(850, 550, 132);	
				BSP_LCD_DrawVLine(850, 510, 41);	
				BSP_LCD_DrawVLine(982, 510, 41);	
				
				
				
				
				
				
				
				
				
				
				
				
				TS_BKState.touchDetected = TS_State.touchDetected;
			}
			TS_flag = 0;
		}
		
		////////// GET ADC
		for(i = 0,ad1 =0,ad2=0; i < 100;)
		{
			ad1 += ADC_Value[i++];
			ad2 += ADC_Value[i++];
		}
		ad1 /= 61;
		ad2 /= 61;
		
		ad1 -= 15; // OFFSET
		
		BSP_LCD_DisplayANumber(80,300,ad1, 5);
		
		// Average
		if(adc_counter == 0)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED1_Pin);	
			adc_counter++;
		}
		else if(adc_counter < ADC_SAMPLES)
		{
			avg += ad1;
			adc_counter++;
		}
		else if(adc_counter == ADC_SAMPLES)
		{
			adc_counter++; // To get out of the next loop
			
			avg /= ADC_SAMPLES;
			BSP_LCD_DisplayANumber(80,400,ad1, 5);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED1_Pin);
		}
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_ActivateOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 3;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * @}
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
	{
		TS_flag = 1;
		printf("ok\r\n");
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
