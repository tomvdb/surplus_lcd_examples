/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t getUs(void) {

uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
register uint32_t ms, cycle_cnt;
do {
ms = HAL_GetTick();
cycle_cnt = SysTick->VAL;
} while (ms != HAL_GetTick());
return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

 

void delayUs(uint16_t micros) {
uint32_t start = getUs();
while (getUs()-start < (uint32_t) micros) {
 __ASM volatile ("nop");
}
}



void SPI_Send( char Data )
{
		uint8_t dt = Data;
	
		HAL_GPIO_WritePin(LCD_STB_GPIO_Port, LCD_STB_Pin, GPIO_PIN_RESET);
	
//    device.write(Data); 
		HAL_SPI_Transmit(&hspi1, &dt, 1, 500);
	
		HAL_GPIO_WritePin(LCD_STB_GPIO_Port, LCD_STB_Pin, GPIO_PIN_SET);
}

void SPI_Send_37uS_Delay(char Data)
{
    SPI_Send(Data);
    delayUs(37);
}


void LCD_Init (void)
{
    HAL_Delay(50);

    // This does the basic setup of the controller:
    SPI_Send_37uS_Delay(0x38);  //function set
    SPI_Send_37uS_Delay(0x3C);  //function set
    SPI_Send_37uS_Delay(0x40); //set cgram 0

    // R bias
    SPI_Send_37uS_Delay(0x06);

    SPI_Send_37uS_Delay(0x38);  
    SPI_Send_37uS_Delay(0x06);
    SPI_Send_37uS_Delay(0x0C);
    SPI_Send_37uS_Delay(0x40); 
    SPI_Send_37uS_Delay(0x3C);
    SPI_Send_37uS_Delay(0x80);
    SPI_Send_37uS_Delay(0x00);

    // This sets the icon characters all off:
    SPI_Send_37uS_Delay(0x38); 
    SPI_Send_37uS_Delay(0x40);
    SPI_Send_37uS_Delay(0x3C); 
    SPI_Send_37uS_Delay(0x85); 
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
}

void LCD_Write( char Line , char *Data )
{
    uint8_t i;

    SPI_Send_37uS_Delay(0x38);

    if ( Line == 1 ) 
        SPI_Send_37uS_Delay(0x80);
    else
        SPI_Send_37uS_Delay(0x80 + 0x40);

    SPI_Send_37uS_Delay(0x3C);
    SPI_Send_37uS_Delay(0x8F);
    SPI_Send(0x00);

    for ( i = 0 ; i < 15 ; i++ )
        {
            if ( Data[i] == 0 )
            break;
            SPI_Send(Data[i]);
        }

    for ( ; i < 15 ; i++ )
        SPI_Send(' ');
}

const uint8_t Sig_Tab[6][6] = {  {0x04,0x00,0x00,0x00,0x00,0x00},
							{0x04,0x04,0x00,0x00,0x00,0x00},
							{0x04,0x04,0x04,0x00,0x00,0x00},
							{0x04,0x04,0x04,0x04,0x00,0x00},
							{0x04,0x04,0x04,0x04,0x04,0x00},
							{0x04,0x04,0x04,0x04,0x04,0x04} };

const uint8_t Batt_Tab[6][6] = { {0x10,0x00,0x00,0x00,0x00,0x00},
							{0x10,0x00,0x00,0x10,0x00,0x00},
							{0x10,0x00,0x10,0x10,0x00,0x00},
							{0x10,0x10,0x10,0x10,0x00,0x00},
							{0x10,0x10,0x10,0x10,0x10,0x00},
							{0x10,0x10,0x10,0x10,0x10,0x10} }; 
								
void LCD_Icons( uint8_t Batt , uint8_t Sig, uint8_t plug, uint8_t down, uint8_t up, uint8_t phone, uint8_t env )
{
SPI_Send_37uS_Delay(0x38);
SPI_Send_37uS_Delay(0x40);
SPI_Send_37uS_Delay(0x3C);
SPI_Send_37uS_Delay(0x85);

SPI_Send_37uS_Delay( Sig_Tab[Sig][0] + Batt_Tab[Batt][0]  ); 
SPI_Send_37uS_Delay( Sig_Tab[Sig][1] + Batt_Tab[Batt][1] + plug );//plug
SPI_Send_37uS_Delay( Sig_Tab[Sig][2] + Batt_Tab[Batt][2] + down ); //call down
SPI_Send_37uS_Delay( Sig_Tab[Sig][3] + Batt_Tab[Batt][3] + up ); //call up
SPI_Send_37uS_Delay( Sig_Tab[Sig][4] + Batt_Tab[Batt][4] + phone  ); //phone
SPI_Send_37uS_Delay( Sig_Tab[Sig][5] + Batt_Tab[Batt][5] + env ); //envelope

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();

	LCD_Init();
	LCD_Write(1,"hello world!" );
	LCD_Icons(5, 5, 0x08, 0x08, 0x08, 0x08, 0x08);
	
  /* USER CODE BEGIN 2 */

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USB init function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_STB_GPIO_Port, LCD_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LCD_STB_Pin */
  GPIO_InitStruct.Pin = LCD_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
