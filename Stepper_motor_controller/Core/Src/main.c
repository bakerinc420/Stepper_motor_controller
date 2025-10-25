/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/apps/httpd.h"
#include <String.h>
#include <stdlib.h>
#include <Stdbool.h>
extern struct netif gnetif;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool LD1ON = false;
bool LD2ON = false;
int steps = 0;
int direction = 0;
int speed;
int current_speed = 50;
int K = 0;
int virtual_steps = 6400;
bool steps_received = false;
bool direction_received = false;
bool motorDirection;
bool speed_received = false;

bool calibration_mode = false;
bool calibration_in_progress = false;
int calibration = 0;
int calibration_target_steps = 200; // Количество шагов для калибровки
int calibration_direction = 1; // Направление калибровки (1 - по часовой, 0 - против)

osMessageQId motorCommandQueue;
osMutexId motorMutex;
bool motorBusy = false;

typedef struct {
    int direction;
    int steps;
    int speed;
} MotorCommand_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int map_speed_to_delay(int speed);
void Stepper_motor_with_ramp(int direction, int steps, int target_speed_percent, int accel_steps, int decel_steps, int virtual_steps);
int Virtual_Steps(int virtual_steps);
void Steps_Calibration(int direction, int steps);

const char* LedCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const tCGI LedCGI = { "/leds.cgi", LedCGIhandler };

const char* MotorCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const tCGI MotorCGI = { "/motor.cgi", MotorCGIhandler };

const char* CalibrationCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const tCGI CalibrationCGI = { "/calibration.cgi", CalibrationCGIhandler };

tCGI theCGItable[4];

// just declaring SSI handler function [* SSI #1 *]
u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen);
// [* SSI #2 *]
#define numSSItags 8

// [* SSI #3 *]
char const *theSSItags[numSSItags] = { "tag1", "tag2","tag3", "tag4", "tag5" , "tag6", "tag7", "tag8"};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */
void StartMotorTask(void const * argument);
void StartButtonTask(void const * argument);
osThreadId motorTaskHandle;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void usDelay(int useconds) //1 sec = 10000
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  while(__HAL_TIM_GET_COUNTER(&htim2) < useconds)
  {

    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
      break;
    }
  }
}


const char* LedCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	uint32_t i = 0;
	if (iIndex == 0) {
		HAL_GPIO_WritePin(GPIOB, Led2_Pin, GPIO_PIN_RESET);
		LD2ON = false;
		HAL_GPIO_WritePin(GPIOB, Led3_Pin, GPIO_PIN_RESET);
		LD1ON = false;
	}

	for (i = 0; i < iNumParams; i++) {
		if (strcmp(pcParam[i], "led") == 0)
		{
			if (strcmp(pcValue[i], "1") == 0) {
				HAL_GPIO_WritePin(GPIOB, Led3_Pin, GPIO_PIN_SET);
				LD1ON = true;
			}
			else if (strcmp(pcValue[i], "2") == 0) {
				HAL_GPIO_WritePin(GPIOB, Led2_Pin, GPIO_PIN_SET);
				LD2ON = true;
			}
		}
	}
	return "/index.shtml";
}

const char* MotorCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	uint32_t i = 0;
	MotorCommand_t motorCmd = {0};

    for (i = 0; i < iNumParams; i++) {
        if (strcmp(pcParam[i], "direction") == 0) {
            motorCmd.direction = atoi(pcValue[i]);
            direction = motorCmd.direction;
        }
        else if (strcmp(pcParam[i], "numberInput") == 0) {
            motorCmd.steps = atoi(pcValue[i]);
            steps = motorCmd.steps;
        }
        else if (strcmp(pcParam[i], "speed") == 0) {
            motorCmd.speed = atoi(pcValue[i]);
            speed = motorCmd.speed;
            current_speed = motorCmd.speed;
        } else if(strcmp(pcParam[i], "VirtualSteps") == 0){
			virtual_steps = atoi(pcValue[i]);
			K = virtual_steps / 6400;
			if (K == 1) { K = 0; }
        }
    }

    // Если получены все параметры - отправляем команду в очередь
	if (motorCmd.steps > 0 && motorCmd.steps <= 1000) {
		if (osMutexWait(motorMutex, 0) == osOK) {
			if (motorBusy == false) {
				motorBusy = true;
				osMutexRelease(motorMutex);

				MotorCommand_t *cmd_copy = pvPortMalloc(sizeof(MotorCommand_t));
				if (cmd_copy != NULL) {
					*cmd_copy = motorCmd;

					if (osMessagePut(motorCommandQueue, (uint32_t) cmd_copy, 0) != osOK) {
						vPortFree(cmd_copy);
						osMutexWait(motorMutex, osWaitForever);
						motorBusy = false;
						osMutexRelease(motorMutex);
					}
				} else {
					osMutexWait(motorMutex, osWaitForever);
					motorBusy = false;
					osMutexRelease(motorMutex);
				}
			} else {
				osMutexRelease(motorMutex);
			}
		}
	}
    return "/index.shtml";
}

const char* CalibrationCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
    uint32_t i = 0;

    for (i = 0; i < iNumParams; i++) {
        if (strcmp(pcParam[i], "calibration") == 0) {
            if (strcmp(pcValue[i], "start") == 0) {
                // Запуск калибровки
                if (osMutexWait(motorMutex, 0) == osOK) {
                    if (!motorBusy && !calibration_in_progress) {
                        calibration_mode = true;
                        calibration_in_progress = true;
                        osMutexRelease(motorMutex);

                        // Отправляем команду калибровки в очередь
                        MotorCommand_t *calib_cmd = pvPortMalloc(sizeof(MotorCommand_t));
                        if (calib_cmd != NULL) {
                            calib_cmd->direction = 1;
                            calib_cmd->steps = calibration;
                            calib_cmd->speed = 100; // Медленная скорость для калибровки

                            if (osMessagePut(motorCommandQueue, (uint32_t) calib_cmd, 0) != osOK) {
                                vPortFree(calib_cmd);
                                calibration_in_progress = false;
                            }
                        }
                    } else {
                        osMutexRelease(motorMutex);
                    }
                }
            }
            else if (strcmp(pcValue[i], "stop") == 0) {
                // Остановка калибровки
                calibration_mode = false;
                calibration_in_progress = false;
            }
        }
    }
    return "/index.shtml";
}

void Steps_Calibration(int direction, int steps){

	if (direction == 1){ //по часовой
		calibration += steps;
		while (calibration >= 200){
			calibration -= 200;
		}

	} else if (direction == 0){ // против часовой
		calibration -= steps;
		while (calibration < 0){
			calibration += 200;
		}
	}
}

void myCGIinit(void) {
	theCGItable[0] = LedCGI;
	theCGItable[1] = MotorCGI;
	theCGItable[2] = CalibrationCGI;
	http_set_cgi_handlers(theCGItable, 3);
}

u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen) {
	if (iIndex == 0) {
		if (LD1ON == false) {
			char myStr1[] = "<input value=\"1\" name=\"led\" type=\"checkbox\">";
			strcpy(pcInsert, myStr1);
			return strlen(myStr1);
		}
		else if (LD1ON == true) {
			char myStr1[] = "<input value=\"1\" name=\"led\" type=\"checkbox\" checked>";
			strcpy(pcInsert, myStr1);
			return strlen(myStr1);
		}
	}

	else if (iIndex == 1)
	{
		if (LD2ON == false) {
			char myStr2[] = "<input value=\"2\" name=\"led\" type=\"checkbox\">";
			strcpy(pcInsert, myStr2);
			return strlen(myStr2);
		} else if (LD2ON == true) {
			char myStr2[] = "<input value=\"2\" name=\"led\" type=\"checkbox\" checked>";
			strcpy(pcInsert, myStr2);
			return strlen(myStr2);
		}
	} else if (iIndex == 2) {

		if (direction == 0){
			char myStr3[] = "<option value=\"0\">Против часовой стрелки 0</option>"
							"<option value=\"1\">По часовой стрелке 1</option>";
			strcpy(pcInsert, myStr3);
			return strlen(myStr3);
		} else if (direction == 1){
			char myStr3[] = "<option value=\"1\">По часовой стрелке 1</option>"
							"<option value=\"0\">Против часовой стрелки 0</option>";
			strcpy(pcInsert, myStr3);
			return strlen(myStr3);
		}

    }  else if (iIndex == 3){
    	char myStr4[200];
    	snprintf(myStr4, sizeof(myStr4), "<input type=\"range\" min=\"0\" max=\"100\" value=\"%d\"  id=\"speed\" name=\"speed\"><br>" "<div>Скорость: <span id=\"speedValue\">%d</span>%%</div>", current_speed, current_speed);
    	strcpy(pcInsert,myStr4);
    	return strlen(myStr4);

    }  else if (iIndex == 4) {

    	char myStr5[] = "<input type=\"number\" id=\"numberInput\" name=\"numberInput\" min=\"0\" max=\"1000\" step=\"1\" placeholder=\"0-1000\" required>";
    	strcpy(pcInsert, myStr5);
    	return strlen(myStr5);
    }	else if (iIndex == 5) {
        // Статус мотора
        osMutexWait(motorMutex, 0);
        if (motorBusy == true) {
            char myStr6[] = "<div style='color: orange;'>Мотор занят</div>";
            strcpy(pcInsert, myStr6);
            osMutexRelease(motorMutex);
            return strlen(myStr6);
        } else {
            char myStr6[] = "<div style='color: green;'>Мотор готов</div>";
            strcpy(pcInsert, myStr6);
            osMutexRelease(motorMutex);
            return strlen(myStr6);
        }
    }	else if(iIndex == 6){ //virtual steps
		char myStr7[120];
		snprintf(myStr7, sizeof(myStr7), "<input type=\"number\" id=\"VirtualSteps\" name=\"VirtualSteps\" min=\"6400\" max=\"1000000\" value=\"%d\" required>", virtual_steps);
		strcpy(pcInsert, myStr7);
		return strlen(myStr7);

    }	else if (iIndex == 7) {
        // Статус калибровки
        if (calibration_in_progress) {
            char myStr8[] = "<div style='color: orange;'>Калибровка выполняется...</div>";
            strcpy(pcInsert, myStr8);
            return strlen(myStr8);
        } else {
            char myStr8[] = "<div>Длинное нажатие кнопки для калибровки</div>";
            strcpy(pcInsert, myStr8);
            return strlen(myStr8);
        }
    }

	return 0;
}


void mySSIinit(void) {
	http_set_ssi_handler(mySSIHandler, (char const**) theSSItags,
	numSSItags);
}

void Stepper_motor_with_ramp(int direction, int steps, int target_speed_percent, int accel_steps, int decel_steps, int virtual_steps) {
    HAL_GPIO_WritePin(GPIOG, ENABLE_PG3_Pin, GPIO_PIN_SET);

    switch (direction) {
        case 1:
            HAL_GPIO_WritePin(GPIOC, CW_CCW_DIR_Pin, GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(GPIOC, CW_CCW_DIR_Pin, GPIO_PIN_RESET);
            break;
    }

    if (accel_steps + decel_steps > steps) {
        accel_steps = steps / 2;
        decel_steps = steps - accel_steps;
    }
    int const_steps = steps - accel_steps - decel_steps;

    int target_delay = map_speed_to_delay(target_speed_percent); // 100% = минимальная задержка (быстро), 0% = максимальная задержка (медленно)
    int start_delay = map_speed_to_delay(30); // Начинаем с 30% скорости (медленно) // УСТАНАВЛИВАЕМ НАЧАЛЬНУЮ ЗАДЕРЖКУ ДЛЯ РАЗГОНА

    if (start_delay <= target_delay) {
        start_delay = target_delay + 10; // Добавляется запас для плавности //  начальная задержка больше целевой для плавного разгона
    }

    // === ФАЗА 1: ПЛАВНЫЙ РАЗГОН ===
    for (int i = 0; i < accel_steps * 16 * 2; i++) {
    	int result  = Virtual_Steps(virtual_steps);
    	if (result != 0){
    		return;
    	}
        HAL_GPIO_TogglePin(GPIOC, CLK_STEP_Pin);
        // Линейно уменьшаем задержку от start_delay до target_delay
        int current_delay = start_delay - ((start_delay - target_delay) * i) / (accel_steps * 16 * 2);// ВЫЧИСЛЯЕМ ТЕКУЩУЮ ЗАДЕРЖКУ ДЛЯ ПЛАВНОГО РАЗГОНА

        if (current_delay < target_delay) {
            current_delay = target_delay; // Защита от перескока (задержка не должна быть меньше целевой)
        }

        usDelay(current_delay);
    }

    // === ФАЗА 2: ДВИЖЕНИЕ С ПОСТОЯННОЙ СКОРОСТЬЮ ===
    for (int i = 0; i < const_steps * 16 * 2; i++) {
    	int result  = Virtual_Steps(virtual_steps);
		if (result != 0) {
			return;
		}
        HAL_GPIO_TogglePin(GPIOC, CLK_STEP_Pin);
        usDelay(target_delay);
    }

    // === ФАЗА 3: ПЛАВНОЕ ТОРМОЖЕНИЕ ===
    for (int i = 0; i < decel_steps * 16 * 2; i++) {
    	int result  = Virtual_Steps(virtual_steps);
		if (result != 0) {
			return;
		}
        HAL_GPIO_TogglePin(GPIOC, CLK_STEP_Pin);
        int current_delay = target_delay + ((start_delay - target_delay) * i) / (decel_steps * 16 * 2); // ВЫЧИСЛЯЕМ ТЕКУЩУЮ ЗАДЕРЖКУ ДЛЯ ПЛАВНОГО ТОРМОЖЕНИЯ
        usDelay(current_delay);
    }

    HAL_GPIO_WritePin(GPIOC, CLK_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, ENABLE_PG3_Pin, GPIO_PIN_RESET);
    Steps_Calibration(direction, steps);
}


int map_speed_to_delay(int speed) {
    int min_delay = 1;   // 1 мс - максимальная скорость
    int max_delay = 100; // 100 мс - минимальная скорость

    int delay_range = max_delay - min_delay;
    return max_delay - (speed * delay_range / 100);
}

int Virtual_Steps(int virtual_steps) {
	int counter  = 0;
	for (int i = 0; i < virtual_steps; i++){
		counter += 1;
		usDelay(1);
	}
	return 0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	osMutexDef(motorMutex);
	motorMutex = osMutexCreate(osMutex(motorMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	osMessageQDef(motorCommandQueue, 5, MotorCommand_t);
	motorCommandQueue = osMessageCreate(osMessageQ(motorCommandQueue), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 512);
	motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

	osThreadDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 256);
	osThreadCreate(osThread(buttonTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led3_Pin|Led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ENABLE_Pin|ENABLE_PG3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CLK_STEP_Pin|CW_CCW_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led3_Pin Led2_Pin */
  GPIO_InitStruct.Pin = Led3_Pin|Led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_Pin ENABLE_PG3_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|ENABLE_PG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_STEP_Pin CW_CCW_DIR_Pin */
  GPIO_InitStruct.Pin = CLK_STEP_Pin|CW_CCW_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartMotorTask(void const *argument) {
	MotorCommand_t receivedCmd;
	osEvent event;

	for (;;) {
		// ожидание команды из очереди
		event = osMessageGet(motorCommandQueue, osWaitForever);

		if (event.status == osEventMessage) {
			receivedCmd = *(MotorCommand_t*) event.value.p;
			if (calibration_mode) {
				// Медленная и плавная калибровка
				Stepper_motor_with_ramp(0, receivedCmd.steps, receivedCmd.speed, receivedCmd.steps / 2, receivedCmd.steps / 2, K);

				// Завершение калибровки
				osMutexWait(motorMutex, osWaitForever);
				calibration_mode = false;
				calibration_in_progress = false;
				motorBusy = false;
				osMutexRelease(motorMutex);

				// Визуальная индикация завершения калибровки
				HAL_GPIO_WritePin(GPIOB, Led2_Pin | Led3_Pin, GPIO_PIN_RESET);
				for (int i = 0; i < 3; i++) {
					HAL_GPIO_WritePin(GPIOB, Led3_Pin, GPIO_PIN_SET);
					osDelay(300);
					HAL_GPIO_WritePin(GPIOB, Led3_Pin, GPIO_PIN_RESET);
					osDelay(300);
				}
			} else {
				if (receivedCmd.steps == 0) {
					printf("ERROR: Received zero steps! Check CGI handler.\n");
					continue;
				}

				int actual_speed = receivedCmd.speed;
				int steps = receivedCmd.steps;
				int direction = receivedCmd.direction;

				if (steps < 20) {
					if (actual_speed > 15) {
						actual_speed = 15;
					}
					Stepper_motor_with_ramp(direction, steps, actual_speed, 0,
							0, K);
				} else if (steps >= 20 && steps < 50) {
					if (actual_speed > 50) {
						actual_speed = 50;
					}
					Stepper_motor_with_ramp(direction, steps, actual_speed, 5,
							5, K);
				} else if (steps >= 50 && steps <= 100) {
					if (actual_speed > 80) {
						actual_speed = 80;
					}
					Stepper_motor_with_ramp(direction, steps, actual_speed, 15,
							15, K);
				} else {
					if (actual_speed < 75) {
						actual_speed = 75;
					}
					Stepper_motor_with_ramp(direction, steps, actual_speed, 50,
							35, K);
				}

				// Освобождаем мотор
				osMutexWait(motorMutex, osWaitForever);
				motorBusy = false;
				osMutexRelease(motorMutex);
			}
			vPortFree(event.value.p);
		}

		// Небольшая задержка для других задач
		osDelay(100);
	}
}
////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void StartButtonTask(void const * argument) {
    uint32_t last_button_state = 0;
    uint32_t button_press_time = 0;
    const uint32_t long_press_delay = 3000; // 3 секунды для длинного нажатия

    for (;;) {
        uint32_t current_button_state = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);

        // Обнаружение нажатия кнопки
        if (current_button_state == GPIO_PIN_SET && last_button_state == GPIO_PIN_RESET) {
            // Кнопка нажата - запоминаем время
            button_press_time = osKernelSysTick();
        }

        // Обнаружение отпускания кнопки
        if (current_button_state == GPIO_PIN_RESET && last_button_state == GPIO_PIN_SET) {
            uint32_t press_duration = osKernelSysTick() - button_press_time;

            // Длинное нажатие (более 3 секунд) - запуск калибровки
            if (press_duration >= long_press_delay) {
                if (osMutexWait(motorMutex, 0) == osOK) {
                    if (motorBusy == false && calibration_in_progress == false) {
                        calibration_mode = true;
                        calibration_in_progress = true;
                        osMutexRelease(motorMutex);

                        // Визуальная индикация начала калибровки
                        HAL_GPIO_WritePin(GPIOB, Led2_Pin|Led3_Pin, GPIO_PIN_SET);

                        // Отправляем команду калибровки
                        MotorCommand_t *calib_cmd = pvPortMalloc(sizeof(MotorCommand_t));
                        if (calib_cmd != NULL) {
                            calib_cmd->direction = 1;
                            calib_cmd->steps = calibration;
                            calib_cmd->speed = 100;

                            if (osMessagePut(motorCommandQueue, (uint32_t) calib_cmd, 0) != osOK) {
                                vPortFree(calib_cmd);
                                calibration_in_progress = false;
                                HAL_GPIO_WritePin(GPIOB, Led2_Pin|Led3_Pin, GPIO_PIN_RESET);
                            }
                        }
                    } else {
                        osMutexRelease(motorMutex);
                        // Индикация ошибки - мигание светодиодами
                        for (int i = 0; i < 5; i++) {
                            HAL_GPIO_TogglePin(GPIOB, Led2_Pin);
                            osDelay(200);
                        }
                    }
                }
            }
        }

        last_button_state = current_button_state;
        osDelay(50); // Проверяем состояние кнопки каждые 50мс
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  HAL_TIM_Base_Start(&htim2);
  netif_set_link_up(&gnetif);
  netif_set_up(&gnetif);
  httpd_init();
  myCGIinit();
  mySSIinit();
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	//HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_TogglePin(GPIOB,LD2_Pin);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
