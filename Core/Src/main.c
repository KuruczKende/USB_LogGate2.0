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
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp/board_api.h"
#include "tusb.h"
#include "class/hid/hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// English
#define LANGUAGE_ID 0x0409
#define BUF_COUNT   4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t u16SegDispValue = 0u;
uint8_t u8SegDispDigit = 0u;
const uint32_t au32SegMasks[16] = { 0b01111011000000000000000000000000,
                                    0b00000011000000000000000000000000,
                                    0b01110010100000000000000000000000,
                                    0b00110011100000000000000000000000,
                                    0b00001011100000000000000000000000,
                                    0b00111001100000000000000000000000,
                                    0b01111001100000000000000000000000,
                                    0b00100011000000000000000000000000,
                                    0b01111011100000000000000000000000,
                                    0b00111011100000000000000000000000,
                                    0b01101011100000000000000000000000,
                                    0b01011001100000000000000000000000,
                                    0b01111000000000000000000000000000,
                                    0b01010011100000000000000000000000,
                                    0b01111000100000000000000000000000,
                                    0b01101000100000000000000000000000};


tusb_desc_device_t desc_device;

uint8_t buf_pool[BUF_COUNT][64];
uint8_t buf_owner[BUF_COUNT] = { 0 }; // device address that owns buffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void tusb_time_delay_ms_api(uint32_t ms){
  const uint32_t time_ms = HAL_GetTick();
  while ((HAL_GetTick() - time_ms) < ms) {}
}
void led_blinking_task(void);
extern void cdc_app_task(void);
extern void hid_app_task(void);

void vSetSegDisp(uint16_t u16Value);
void vShowSegDisp(void);
uint32_t board_millis(){return HAL_GetTick();}
void board_led_write(bool boVal){HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,boVal?GPIO_PIN_SET:GPIO_PIN_RESET);}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_OTG_HS_HCD_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // init device stack on configured roothub port
  tusb_rhport_init_t host_init = {
    .role = TUSB_ROLE_HOST,
    .speed = TUSB_SPEED_FULL
  };
  tusb_init(BOARD_TUH_RHPORT, &host_init);

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_FULL
  };
  //tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t u32Last = HAL_GetTick();
  while (1)
  {
    tuh_task(); // tinyusb device task
    led_blinking_task();
    cdc_app_task();
    hid_app_task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*if(HAL_GetTick() - u32Last > 250){
      u32Last = HAL_GetTick();
      vSetSegDisp(u16SegDispValue + 1);
    }//*/
    vShowSegDisp();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

void tuh_mount_cb(uint8_t dev_addr) {
  // application set-up
  printf("A device with address %d is mounted\r\n", dev_addr);
}

void tuh_umount_cb(uint8_t dev_addr) {
  // application tear-down
  printf("A device with address %d is unmounted \r\n", dev_addr);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < 1000) return; // not enough time
  start_ms += 1000;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

























void vAlterSegDisp(uint16_t u16Reset, uint16_t u16Set){
  u16SegDispValue &= ~u16Reset;
  u16SegDispValue |= u16Set;
}
void vSetSegDisp(uint16_t u16Value)
{
  u16SegDispValue = u16Value;
}

void vShowSegDisp(void)
{
  uint8_t u8SegValue;
  switch (u8SegDispDigit)
  {
  case 1u:
    GPIOE->BSRR = 0x00347F88;
    u8SegDispDigit = 2u;
    u8SegValue = u16SegDispValue >> 8u;
    break;
  case 2u:
    GPIOE->BSRR = 0x002C7F90;
    u8SegDispDigit = 3u;
    u8SegValue = u16SegDispValue >> 4u;
    break;
  case 3u:
    GPIOE->BSRR = 0x001C7FA0;
    u8SegDispDigit = 0u;
    u8SegValue = u16SegDispValue;
    break;
  default:
    GPIOE->BSRR = 0x00387F84;
    u8SegDispDigit = 1u;
    u8SegValue = u16SegDispValue >> 12u;
    break;
  }
  GPIOE->BSRR = au32SegMasks[u8SegValue&0x0F];
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
