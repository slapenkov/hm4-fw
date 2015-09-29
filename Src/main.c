/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER Includes @ definitions */
#define MODBUS_SERIAL_BAUD 115200 //set speed
#define MODBUS_SWITCH USART_1
#define MODBUS_SERIAL_RX_BUFFER_SIZE  256
#define MODBUS_ADDRESS 0x01 //set slave address

#include "modbus.h"

/* USER Includes END*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef mbtim;

UART_HandleTypeDef mbuart;

/* USER CODE BEGIN PV */

static int16_t hold_regs[] = { 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006,
		0x0008, 0x0009, 0x000A };
static int16_t event_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);

int main(void) {

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_TIM7_Init();
	MX_USART1_UART_Init();

	/* Initialization of modbus layer */
	modbus_init();

	/* Infinite loop */
	while (1) {
		/* wait for modbus request */
		while (!modbus_kbhit()) {
		};

		//request processing
		//check address against our address, 0 is broadcast
		if ((modbus_rx.address == MODBUS_ADDRESS) || modbus_rx.address == 0) {
			switch (modbus_rx.func) {
			case FUNC_READ_HOLDING_REGISTERS:
				if (FALSE)	        //test for register address correct
					modbus_exception_rsp(MODBUS_ADDRESS, modbus_rx.func,
							ILLEGAL_DATA_ADDRESS);
				else {
					if (modbus_rx.func == FUNC_READ_HOLDING_REGISTERS)
						modbus_read_holding_registers_rsp(MODBUS_ADDRESS,
								(modbus_rx.data[3] * 2),
								hold_regs + modbus_rx.data[1]);

					event_count++;
				}
				break;
			case FUNC_WRITE_SINGLE_REGISTER:
				if (FALSE)
					modbus_exception_rsp(MODBUS_ADDRESS, modbus_rx.func,
							ILLEGAL_DATA_ADDRESS);
				else {
					//the registers are stored in little endian format
					//hold_regs[modbus_rx.data[1]] = make16(modbus_rx.data[3],modbus_rx.data[2]);
					//select action rather to corresponding register
					switch (modbus_rx.data[1]) {
					case 0x00:
					case 0x01:
					case 0x02:
					case 0x03:
					case 0x04:
					case 0x05:
					case 0x06:
					case 0x07:
					case 0x08:
						hold_regs[modbus_rx.data[1]] = make16(modbus_rx.data[2],
								modbus_rx.data[3]); //save value
						break;
					};

					modbus_write_single_register_rsp(MODBUS_ADDRESS,
							make16(modbus_rx.data[0], modbus_rx.data[1]),
							make16(modbus_rx.data[2], modbus_rx.data[3]));
				}
				break;
			case FUNC_WRITE_MULTIPLE_REGISTERS:
				if (FALSE)
					modbus_exception_rsp(MODBUS_ADDRESS, modbus_rx.func,
							ILLEGAL_DATA_ADDRESS);
				else {
					uint8_t i, j;

					for (i = 0, j = 5; i < modbus_rx.data[4] >> 1; ++i, j += 2)
					//hold_regs[i] = make16(modbus_rx.data[j+1],modbus_rx.data[j]);
							{
						//select action rather to corresponding register
						switch (modbus_rx.data[1] + i) {
						case 0x00:
						case 0x01:
						case 0x02:
						case 0x03:
						case 0x04:
						case 0x05:
						case 0x06:
						case 0x07:
						case 0x08:
						case 0x09:
							hold_regs[modbus_rx.data[1] + i] = make16(
									modbus_rx.data[j], modbus_rx.data[j + 1]); //save value
							break;
						};
					};

					modbus_write_multiple_registers_rsp(MODBUS_ADDRESS,
							make16(modbus_rx.data[0], modbus_rx.data[1]),
							make16(modbus_rx.data[2], modbus_rx.data[3]));

					event_count++;
				}
				break;

				//TODO Add read-write registers function

			default:    //We don't support the function, so return exception
				modbus_exception_rsp(MODBUS_ADDRESS, modbus_rx.func,
						ILLEGAL_FUNCTION);
			}
		}
	}
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

//TODO Configure SysTick timer speed
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000); //sys Tick config

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	HAL_ADC_Init(&hadc1);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void) {

	DAC_ChannelConfTypeDef sConfig;

	/**DAC Initialization
	 */
	hdac.Instance = DAC;
	HAL_DAC_Init(&hdac);

	/**DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

	/**DAC channel OUT2 config
	 */
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);

}

/* TIM7 init function */
void MX_TIM7_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	mbtim.Instance = TIM7;
	mbtim.Init.Prescaler = 32;
	mbtim.Init.CounterMode = TIM_COUNTERMODE_UP;
	mbtim.Init.Period = 4000;
	mbtim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&mbtim);

	/*sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	 HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);*/

}

/* USART1 init function */
void MX_USART1_UART_Init(void) {

	mbuart.Instance = USART1;
	mbuart.Init.BaudRate = MODBUS_SERIAL_BAUD;
	mbuart.Init.WordLength = UART_WORDLENGTH_8B;
	mbuart.Init.StopBits = UART_STOPBITS_1;
	mbuart.Init.Parity = UART_PARITY_NONE;
	mbuart.Init.Mode = UART_MODE_TX_RX;
	mbuart.Init.HwFlowCtl = UART_HWCONTROL_RTS;
	mbuart.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&mbuart);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE()
	;
	__GPIOA_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pins : PC0 PC1 PC2 PC3
	 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
