/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define G_MS_INICIAL 0
#define MODO_RUN   0
#define MODO_PAUSA 1
#define ADC_MAX 4095
#define THR_INICIAL 70   // umbral inicial en %
#define HISTERESIS  5    // la alarma apaga cuando pct < thr - 5
#define RX_TAM    128   // tamano del buffer circular de entrada
#define TX_TAM    512   // tamano del buffer circular de salida
#define LINEA_MAX 32    // maximo de caracteres por linea (sin el terminador)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t g_ms = G_MS_INICIAL;   // milisegundos (lo suma TIM4)
volatile uint32_t g_flanco = 0;   // la ISR lo pone en 1 cuando el botón cambia

uint32_t modo = MODO_RUN;   // despues de cada reinicio arranca en RUN
uint32_t ms_actividad = 0;               // cuenta para el led de actividad
uint32_t boton_estable = 0;       // 1 = presionado, 0 = suelto (ya confirmado)
uint32_t cuenta_cambio = 0;       // ms que lleva la lectura distinta del estado confirmado
uint32_t cont_pulsaciones = 0;    // pulsaciones validas
uint32_t evento_boton = 0;        // lo pone el boton, lo consume main

uint32_t adc_valor = 0;        // ultima lectura del adc (0 a 4095)
uint32_t muestra_valida = 0;   // 1 = ya hay al menos una lectura
uint32_t ms_adc = 0;           // cuenta para muestrear cada 100 ms
uint32_t cont_muestras = 0;    // cuantas lecturas van (prueba de tiempo)

uint32_t pct = 0;   // porcentaje de la ultima lectura (0 a 100)
uint32_t thr = THR_INICIAL;   // umbral actual
uint32_t alarma = 0;          // 1 = alarma encendida

// uart: buffer circular de entrada
uint8_t rx_buf[RX_TAM];
volatile uint32_t rx_head = 0;   // donde escribe la ISR
volatile uint32_t rx_tail = 0;   // donde lee main
uint8_t rx_byte;                 // aqui llega cada caracter
uint32_t rx_errores = 0;         // errores de la uart
uint32_t rx_overflow = 0;        // veces que se lleno el buffer

// uart: buffer circular de salida
uint8_t tx_buf[TX_TAM];
volatile uint32_t tx_head = 0;       // donde escribe main
volatile uint32_t tx_tail = 0;       // donde lee la ISR
volatile uint32_t tx_len = 0;        // tamano del bloque que se esta mandando
volatile uint32_t tx_enviando = 0;   // 1 = hay un bloque saliendo
uint32_t tx_perdidos = 0;            // caracteres descartados por buffer lleno

// uart: linea que se esta armando
char linea[LINEA_MAX + 1];       // +1 para el '\0'
uint32_t linea_len = 0;          // cuantos caracteres lleva
uint32_t descartando = 0;        // 1 = linea muy larga, ignoro hasta el LF

// telemetria
uint32_t ms_telemetria = 0;      // cuenta para mandar cada 1000 ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void reiniciar_alarma(void);
void boton_actualizar(void);
uint32_t tomar_flanco(void);
void leer_adc(void);
void evaluar_alarma(void);
uint32_t adc_a_pct(uint32_t valor);

void poner_pausa(void);
void poner_run(void);
uint32_t rx_leer(uint8_t *c);
uint32_t armar_linea(uint8_t c);
void tx_iniciar(void);
void enviar(const char *texto);
void enviar_estado(void);
uint32_t leer_numero(const char *s, uint32_t *valor);
void procesar_linea(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);   // arranca el tick de 1 ms
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   // empieza a escuchar 1 byte
	// tiempo de muestreo largo para el potenciometro (61.5 ciclos)
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	// calibracion del adc (se hace una vez al arrancar)
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	uint32_t h_tick = g_ms;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Bloque que corre una vez por cada ms
		if ((uint32_t) (g_ms - h_tick) >= 1U) {
			h_tick = h_tick + 1U;

			// led de actividad, cambia cada 500 ms
			ms_actividad++;
			if (ms_actividad >= 500) {
				ms_actividad = 0;
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}

			boton_actualizar();
			// una muestra cada 100 ms del adc(sigue en RUN y en PAUSA)
			ms_adc++;
			if (ms_adc >= 100) {
				ms_adc = 0;
				leer_adc();
			}

			// telemetria cada 1000 ms (sigue en RUN y en PAUSA)
			ms_telemetria++;
			if (ms_telemetria >= 1000) {
				ms_telemetria = 0;
				enviar_estado();
			}
			// una pulsacion valida cambia entre RUN y PAUSA
			if (evento_boton == 1) {
				evento_boton = 0;
				if (modo == MODO_RUN) {
					poner_pausa();
				} else {
					poner_run();
				}
			}
		}

		// uart: leer lo que llego y armar lineas
		uint8_t c;
		while (rx_leer(&c) == 1) {
			uint32_t r = armar_linea(c);

			if (r == 1) {
				procesar_linea();   // linea completa: ejecuta el comando
			} else if (r == 2) {
				enviar("ERR\r\n");  // linea muy larga
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_TIM34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Convierte la lectura del adc (0 a 4095) a porcentaje (0 a 100) con redondeo
uint32_t adc_a_pct(uint32_t valor) {
	// se usa 32 bits: 100 * 4095 = 409500, no cabe en 16 bits
	return (100 * valor + ADC_MAX / 2) / ADC_MAX;
}
// ISR del boton
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		g_flanco = 1;
	}
}

// Lee la nota y la borra sin que la ISR se meta en medio
uint32_t tomar_flanco(void) {
	__disable_irq();
	uint32_t hubo = g_flanco;
	g_flanco = 0;
	__enable_irq();
	return hubo;
}

// Se llama una vez por ms
void boton_actualizar(void) {
	// 1 = presionado (en la placa presionado lee 0)
	uint32_t lectura = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);

	// Si hubo un flanco, reinicia la espera de 30 ms
	if (tomar_flanco() == 1) {
		cuenta_cambio = 0;
	}

	// Antirrebote
	if (lectura == boton_estable) {
		cuenta_cambio = 0;
	} else {
		cuenta_cambio++;
		if (cuenta_cambio >= 30) {
			cuenta_cambio = 0;
			boton_estable = lectura;

			if (boton_estable == 1) {   // pulsacion confirmada
				cont_pulsaciones++;
				evento_boton = 1;
			}
			// la liberacion tambien se confirma aqui, antes de aceptar otra
		}
	}
}

// Se llama sola cada 1 ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		g_ms = g_ms + 1;
	}
}

// Lee el adc una vez, la conversion dura unos microsegundos
void leer_adc(void) {
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
		adc_valor = HAL_ADC_GetValue(&hadc1);
		pct = adc_a_pct(adc_valor);
		muestra_valida = 1;
		cont_muestras++;
		evaluar_alarma();
	}
	HAL_ADC_Stop(&hadc1);
}

// Decide si la alarma enciende o apaga, con histeresis
void evaluar_alarma(void) {
	if (muestra_valida == 0) {
		return;   // todavia no hay lectura
	}
	if (modo != MODO_RUN) {
		return;   // en pausa se maneja en la etapa 8
	}

	if (pct >= thr) {
		alarma = 1;
	} else if (pct + HISTERESIS < thr) {
		alarma = 0;   // pct < thr - 5, escrito asi para no restar sin signo
	}
	// en la banda intermedia no se toca: conserva su estado

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState) alarma);
}

// Reinicia la decision de la alarma con la ultima muestra (sin histeresis)
void reiniciar_alarma(void) {
	if (muestra_valida == 0) {
		return;   // espera la primera muestra, evaluar_alarma la atiende
	}
	if (pct >= thr) {
		alarma = 1;
	} else {
		alarma = 0;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState) alarma);
}

// Pasa a PAUSA: la alarma se apaga
void poner_pausa(void) {
	modo = MODO_PAUSA;
	alarma = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

// Pasa a RUN. Si ya estaba en RUN no cambia nada (conserva la alarma)
void poner_run(void) {
	if (modo == MODO_RUN) {
		return;
	}
	modo = MODO_RUN;
	reiniciar_alarma();   // al reanudar se decide de nuevo
}

// ISR de la uart: llego un byte
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		uint32_t siguiente = (rx_head + 1) % RX_TAM;

		if (siguiente == rx_tail) {
			rx_overflow++;   // buffer lleno, se pierde el byte
		} else {
			rx_buf[rx_head] = rx_byte;
			rx_head = siguiente;
		}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   // listo para el siguiente
	}
}

// Si la uart tiene un error (por ejemplo overrun)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		rx_errores++;
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   // volver a escuchar
	}
}

// ISR de la uart: termino de salir un bloque
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		tx_tail = (tx_tail + tx_len) % TX_TAM;   // ese bloque ya salio
		tx_enviando = 0;
		tx_iniciar();                            // si queda algo, lo manda
	}
}

// Saca un byte del buffer. Devuelve 1 si habia, 0 si estaba vacio
uint32_t rx_leer(uint8_t *c) {
	if (rx_tail == rx_head) {
		return 0;
	}
	*c = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RX_TAM;
	return 1;
}

// Mete un caracter a la linea. Solo el LF termina la linea (el CR se ignora,
// asi LF y CRLF funcionan igual).
// Devuelve: 0 = nada, 1 = linea completa, 2 = linea muy larga
uint32_t armar_linea(uint8_t c) {
	if (c == '\r') {
		return 0;   // el CR de un CRLF no hace nada
	}

	// fin de linea (LF)
	if (c == '\n') {
		if (descartando == 1) {
			descartando = 0;
			linea_len = 0;
			return 2;   // la linea era muy larga: se descarta completa
		}
		if (linea_len > 0) {
			linea[linea_len] = '\0';   // cierra el texto
			linea_len = 0;
			return 1;
		}
		return 0;   // linea vacia: se ignora
	}

	// caracter normal
	if (descartando == 1) {
		return 0;   // la linea se paso, ignoro hasta el LF
	}
	if (linea_len < LINEA_MAX) {
		linea[linea_len] = (char) c;
		linea_len++;
	} else {
		descartando = 1;   // llego el caracter 33
	}
	return 0;
}

// Manda por la uart el bloque que haya en el buffer de salida.
// Solo arranca si no hay otro bloque saliendo.
// (quien la llame desde main debe apagar las interrupciones un instante)
void tx_iniciar(void) {
	if (tx_enviando == 1 || tx_tail == tx_head) {
		return;   // ya hay uno saliendo, o no hay nada que mandar
	}

	uint32_t h = tx_head;
	uint32_t len;
	if (h > tx_tail) {
		len = h - tx_tail;          // todo seguido hasta head
	} else {
		len = TX_TAM - tx_tail;     // hasta el final del arreglo (luego da la vuelta)
	}

	tx_len = len;
	tx_enviando = 1;
	if (HAL_UART_Transmit_IT(&huart2, &tx_buf[tx_tail], len) != HAL_OK) {
		tx_enviando = 0;   // no pudo arrancar, se reintenta en el siguiente enviar
	}
}

// Manda un texto por la uart SIN esperar: lo copia al buffer y sigue
void enviar(const char *texto) {
	for (uint32_t i = 0; texto[i] != '\0'; i++) {
		uint32_t siguiente = (tx_head + 1) % TX_TAM;

		if (siguiente == tx_tail) {
			tx_perdidos++;   // buffer lleno, descarto lo que sobra
			break;
		}
		tx_buf[tx_head] = (uint8_t) texto[i];
		tx_head = siguiente;
	}

	__disable_irq();
	tx_iniciar();
	__enable_irq();
}

// Arma la linea de estado y la manda. La usan la telemetria y STATUS
void enviar_estado(void) {
	const char *nombre = "RUN";
	if (modo == MODO_PAUSA) {
		nombre = "PAUSE";
	}

	char resp[100];
	snprintf(resp, sizeof(resp),
			"t=%lu;ADC=%lu;PCT=%lu;THR=%lu;MODE=%s;ALARM=%lu\r\n",
			(unsigned long) (g_ms - G_MS_INICIAL), (unsigned long) adc_valor,
			(unsigned long) pct, (unsigned long) thr, nombre,
			(unsigned long) alarma);
	enviar(resp);
}

// Revisa que s sea solo digitos y devuelve su valor en *valor.
// Devuelve: 0 = bien, 1 = no es un numero limpio
uint32_t leer_numero(const char *s, uint32_t *valor) {
	if (s[0] == '\0') {
		return 1;   // vacio
	}

	uint32_t v = 0;
	for (uint32_t i = 0; s[i] != '\0'; i++) {
		if (s[i] < '0' || s[i] > '9') {
			return 1;   // letra, punto decimal, signo, etc
		}
		if (v < 1000) {
			v = v * 10 + (s[i] - '0');   // si ya paso de 1000 no sigo, no desborda
		}
	}
	*valor = v;
	return 0;
}

// Ejecuta el comando que esta en linea
void procesar_linea(void) {
	// parte la linea en palabras (ignora espacios al inicio y al final)
	char *cmd = strtok(linea, " \t");
	char *arg = strtok(NULL, " \t");
	char *extra = strtok(NULL, " \t");

	if (cmd == NULL) {
		return;   // solo habia espacios
	}

	if (strcmp(cmd, "STATUS") == 0) {
		if (arg != NULL) {
			enviar("ERR\r\n");
		} else {
			enviar_estado();
		}
	}

	else if (strcmp(cmd, "THR") == 0) {
		uint32_t n;
		if (arg == NULL || extra != NULL) {
			enviar("ERR\r\n");
		} else if (leer_numero(arg, &n) != 0 || n < 10 || n > 95) {
			enviar("ERR\r\n");   // texto, decimal o fuera de 10..95
		} else {
			thr = n;
			if (modo == MODO_RUN) {
				reiniciar_alarma();   // en pausa la alarma sigue apagada
			}
			char resp[24];
			snprintf(resp, sizeof(resp), "OK THR=%lu\r\n", (unsigned long) n);
			enviar(resp);
		}
	}

	else if (strcmp(cmd, "START") == 0) {
		if (arg != NULL) {
			enviar("ERR\r\n");
		} else {
			poner_run();
			enviar("OK RUN\r\n");
		}
	}

	else if (strcmp(cmd, "STOP") == 0) {
		if (arg != NULL) {
			enviar("ERR\r\n");
		} else {
			poner_pausa();
			enviar("OK PAUSE\r\n");
		}
	}

	else {
		enviar("ERR\r\n");   // comando desconocido
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
