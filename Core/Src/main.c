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
#include "i2c.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "SH1106.h"

/* SH1106 width in pixels */
#ifndef SH1106_WIDTH
#define SH1106_WIDTH            128
#endif
/* SH1106 LCD height in pixels */
#ifndef SH1106_HEIGHT
#define SH1106_HEIGHT           64
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUZZ_ON()   HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET)
#define BUZZ_OFF()  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RFID_BUF_SIZE  512
#define EPC_MAX_CHARS 128   // đủ cho EPC 64 byte hex
#define RFID_FRAME_TIMEOUT_MS 10

uint8_t rfid_rx;

uint8_t rfid_buf[RFID_BUF_SIZE];
volatile uint16_t rfid_len = 0;
volatile uint32_t rfid_last_rx_tick = 0;

const uint8_t rfid_start_cmd[] = {
    0x5A, 0x00, 0x01, 0x02, 0x10, 0x00, 0x05,
    0x00, 0x00, 0x00, 0x01, 0x01, 0xF4, 0x87
};

char last_epc[EPC_MAX_CHARS + 1] = "EMPTY";
uint32_t last_epc_tick = 0;

uint32_t buzz_next_tick = 0;
#define EPC_EMPTY_TIMEOUT_MS 2000  // 2 giây không có EPC → empty
#define READER_ACTIVE_TIMEOUT_MS 100   // 100ms không có byte UART -> coi là reader ngừng


#define BUZZ_BEEP_MS  80      // thời gian bíp (ms)
uint32_t buzz_off_tick = 0;   // thời điểm tắt còi
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Chuyển mảng byte sang chuỗi HEX ASCII
static void bytes_to_hex(const uint8_t *in, uint8_t len, char *out)
{
    const char hex[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < len; i++)
    {
        out[i * 2]     = hex[(in[i] >> 4) & 0x0F];
        out[i * 2 + 1] = hex[in[i] & 0x0F];
    }
    out[len * 2] = '\0';
}

static void OLED_Show_EPC_If_New(const char *epc)
{
    if (epc == NULL || epc[0] == '\0')
        return;

    // EPC giống EPC cũ → không redraw
    if (strcmp(epc, last_epc) == 0)
        {
    	last_epc_tick = HAL_GetTick();
    	return;
        }

    // Lưu EPC mới
    strncpy(last_epc, epc, EPC_MAX_CHARS);
    last_epc[EPC_MAX_CHARS] = '\0';
    last_epc_tick = HAL_GetTick();
    //BUZZ_ON();                               // bật còi
    buzz_off_tick = HAL_GetTick() + BUZZ_BEEP_MS;

    // ===== VẼ OLED =====
    SH1106_Clear();

    SH1106_GotoXY(0, 0);
    SH1106_Puts("EPC:", &Font_7x10, 1);

    uint8_t y = 12;
    const uint8_t chars_per_line = 18;

    while (*epc && y <= 54)
    {
        char buf[19];
        uint8_t i = 0;

        while (*epc && i < chars_per_line)
            buf[i++] = *epc++;

        buf[i] = '\0';

        SH1106_GotoXY(0, y);
        SH1106_Puts(buf, &Font_7x10, 1);

        y += 12;
    }

    SH1106_UpdateScreen();
}

static void OLED_Show_Empty_If_Needed(void)
{
    // Nếu đang empty rồi → không redraw
    if (strcmp(last_epc, "EMPTY") == 0)
        return;

    strcpy(last_epc, "EMPTY");
    BUZZ_OFF();  // Không có tag → tắt còi


    SH1106_Clear();

    SH1106_GotoXY(0, 0);
    SH1106_Puts("EPC:", &Font_7x10, 1);

    SH1106_GotoXY(0, 12);
    SH1106_Puts("empty", &Font_7x10, 1);

    SH1106_UpdateScreen();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rfid_rx, 1);   // Reader
  SH1106_Init();
  SH1106_Clear();
  SH1106_UpdateScreen();
  OLED_Show_Empty_If_Needed();
  BUZZ_OFF();   // đảm bảo lúc bật nguồn còi tắt
  SH1106_GotoXY(0, 0);
  SH1106_Puts("EPC: akjwhdui198", &Font_7x10, 1);
  SH1106_UpdateScreen();
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // ===== RFID -> EPC -> PC =====
	    if (rfid_len > 9 && (HAL_GetTick() - rfid_last_rx_tick) > RFID_FRAME_TIMEOUT_MS)
	    {
	        /*
	          Layout theo bạn mô tả:
	          Byte 0..6  : header (bỏ)
	          Byte 7..8  : EPC length (00 XX)
	          Byte 9..   : EPC data
	        */

	        uint8_t epc_len = rfid_buf[8];  // ví dụ: 0x0C = 12

	        // kiểm tra an toàn
	        if (epc_len > 0 && (9 + epc_len) <= rfid_len)
	        {
	            uint8_t *epc_ptr = &rfid_buf[9];

	            char epc_str[EPC_MAX_CHARS + 1];
	            bytes_to_hex(epc_ptr, epc_len, epc_str);
	            OLED_Show_EPC_If_New(epc_str);
	        }

	        // reset buffer raw RFID
	        rfid_len = 0;
	    }
	    // Nếu quá lâu không có EPC mới → hiển thị empty
	    if ((HAL_GetTick() - last_epc_tick) > EPC_EMPTY_TIMEOUT_MS)
	    {
	        OLED_Show_Empty_If_Needed();
	    }

	    // ===== CÒI PHỤ THUỘC DATA UART =====
	    uint8_t reader_active = 0;

	    if (HAL_GetTick() - rfid_last_rx_tick < READER_ACTIVE_TIMEOUT_MS)
	    {
	        reader_active = 1;   // Reader còn gửi dữ liệu
	    }
	    else
	    {
	        reader_active = 0;   // Reader đã ngừng
	    }

	    if (reader_active)
	    {
	        if (HAL_GetTick() >= buzz_next_tick)
	        {
	            //BUZZ_ON();
	            buzz_off_tick  = HAL_GetTick() + BUZZ_BEEP_MS;
	            buzz_next_tick = HAL_GetTick() + 200; // tốc độ bíp
	        }
	    }
	    else
	    {
	        BUZZ_OFF();          // <<< TẮT NGAY
	        buzz_off_tick = 0;
	    }

	    // tắt còi sau mỗi bíp
	    if (buzz_off_tick != 0 && HAL_GetTick() >= buzz_off_tick)
	    {
	        BUZZ_OFF();
	        buzz_off_tick = 0;
	    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();


    // ===== RFID -> STM32 =====
    if (huart->Instance == USART2)
    {
        if (rfid_len < RFID_BUF_SIZE)
        {
            rfid_buf[rfid_len++] = rfid_rx;
        }
        rfid_last_rx_tick = now;

        HAL_UART_Receive_IT(&huart2, &rfid_rx, 1);
    }
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
