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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include <stdio.h>
#include <string.h>

#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "Fonts/fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
///* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NB_OF_VAR];
//uint16_t VarDataTab[NB_OF_VAR] = {'Z','A','P','I','S',' ','Z',' ','1','0',':','0','6','0','5'}; // n+1
//uint8_t VarDataTabRead[NB_OF_VAR];
//uint16_t VarIndex,VarDataTmp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
///* Direct printf to output somewhere */
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//
//#ifndef __UUID_H
//#define __UUID_H
////#define STM32_UUID ((uint32_t *)0x1FF0F420)
//#define STM32_UUID ((uint32_t *)UID_BASE)
//#endif //__UUID_H

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// --------- PROSTE DANE EKRANU ----------
typedef struct {
  int   speed_kmh;   // np. 23
  int   power_w;     // np. 312
  float voltage_v;   // np. 41.8
  int   assist;      // PAS level, np. 3
  float trip_km;     // np. 12.4
  int   time_hh;     // np. 10
  int   time_mm;     // np. 06
} UiData;

// ---------- POMOCNICZE RYSOWANIE ------------
static void UI_DrawBattery(int x, int y, int w, int h, uint8_t pct)
{
  if(pct > 100) pct = 100;
  if(pct > 100) pct = 100;
  if(w < 8) w = 8;
  if(h < 8) h = 8;

  // obudowa
  GFX_DrawRectangle(x, y, w-2, h, WHITE);
  // „bolczyk” plusa po prawej
  GFX_DrawFillRectangle(x + w - 2, y + h/3, 2, h/3, WHITE);

  // wypełnienie wewnątrz
  int inner_w = (w-2) - 4;       // margines 2 piksele z każdej strony
  int inner_h = h - 4;
  int fill_w  = (inner_w * pct) / 100;
  GFX_DrawFillRectangle(x+2, y+2, fill_w, inner_h, WHITE);
}

static void UI_DrawLabelValue(int x, int y, const char* label, const char* value)
{
  char buf[20];
  // label
  GFX_SetFontSize(1);
  snprintf(buf, sizeof(buf), "%s", label);
  GFX_DrawString(x, y, buf, WHITE, 0);

  // value pod spodem
  GFX_SetFontSize(1);
  GFX_DrawString(x, y + (GFX_GetFontHeight()*1) + 2, (char*)value, WHITE, 0);
}

static int UI_TextWidthPx(const char* s)
{
  // szerokość pojedynczego znaku = font[1] * size, spacja = +1
  // (funkcje dostępne w Twojej bibliotece GFX)
  return (int)strlen(s) * ( (int)GFX_GetFontWidth() * (int)GFX_GetFontSize() + 1 );
}

static void UI_DrawCenteredText(int center_x, int y, const char* s)
{
  int w = UI_TextWidthPx(s);
  int x = center_x - w/2;
  if(x < 0) x = 0;
  GFX_DrawString(x, y, (char*)s, WHITE, 0);
}

// -------------- GŁÓWNY EKRAN ---------------
static void UI_DrawFirstScreen(const UiData* d)
{
  // tło
  SSD1306_Clear(BLACK);

  // USTAW CZCIONKĘ (wstaw prawdziwą nazwę z Fonts/fonts.h)
  // Przykłady często spotykane: Font5x7, Font_5x7, font5x7
  extern const uint8_t font_8x5[];    // <--- jeśli u Ciebie ma inną nazwę, podmień!
  GFX_SetFont(font_8x5);

  // Górny pasek: PAS + godzina + bateria
  char buf[24];

  // PAS (lewy górny)
  snprintf(buf, sizeof(buf), "PAS %d", d->assist);
  GFX_SetFontSize(1);
  GFX_DrawString(2, 1, buf, WHITE, 0);



//  // Godzina (środek góry)
//  snprintf(buf, sizeof(buf), "%02d:%02d", d->time_hh, d->time_mm);
//  UI_DrawCenteredText(SSD1306_LCDWIDTH/2, 1, buf);

  // Bateria (prawy górny)
  // Załóżmy, że 42.0V ≈ 100%, 36.0V ≈ 0% dla 10S Li-ion (przykład)
  int pct = (int)((d->voltage_v - 36.0f) * (100.0f / (42.0f - 36.0f)));
  if(pct < 0) pct = 0; if(pct > 100) pct = 100;
  UI_DrawBattery(SSD1306_LCDWIDTH - 26, 0, 26, 10, (uint8_t)pct);

  // Separator
  GFX_DrawFastHLine(0, 12, SSD1306_LCDWIDTH, WHITE);

  // Duża prędkość w centrum
  char spd[8];
  snprintf(spd, sizeof(spd), "%d", d->speed_kmh);
  GFX_SetFontSize(4);                 // x2 – duże cyfry (Twoje GFX tak powiększa) :contentReference[oaicite:3]{index=3}
  UI_DrawCenteredText(SSD1306_LCDWIDTH/2, 14, spd);

  // Jednostka "km/h" pod spodem mniejsza
  GFX_SetFontSize(1);
  UI_DrawCenteredText(SSD1306_LCDWIDTH/2, 18, spd + GFX_GetFontHeight()*2 + 2);
  GFX_DrawString(SSD1306_LCDWIDTH/2 + 30, 18 + GFX_GetFontHeight()*2 + 2, "km/h", WHITE, 0);

  // Dolny blok (trzy kolumny): PWR / VOLT / TRIP
  GFX_DrawFastHLine(0, SSD1306_LCDHEIGHT-20, SSD1306_LCDWIDTH, WHITE);

  // Kolumna 1: PWR
  char val[20];
  snprintf(val, sizeof(val), "%d W", d->power_w);
  UI_DrawLabelValue(2, SSD1306_LCDHEIGHT-18, "PWR", val);

  // Kolumna 2: VOLT (środek)
  snprintf(val, sizeof(val), "%.1f V", d->voltage_v);
  int mid_x = SSD1306_LCDWIDTH/2 - 20;
  UI_DrawLabelValue(mid_x, SSD1306_LCDHEIGHT/6 -18, 0, val);

  // Kolumna 3: TRIP (prawa)
  snprintf(val, sizeof(val), "%.1f km", d->trip_km);
  UI_DrawLabelValue(SSD1306_LCDWIDTH-48, SSD1306_LCDHEIGHT-18, "TRIP", val);

  // Wyświetl
  SSD1306_Display();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init(&hi2c1);

  SSD1306_Clear(BLACK);

  // USTAW CZCIONKĘ (TAK SAMO jak w UI_DrawFirstScreen!)
  // podmień jeśli u Ciebie nazywa się inaczej
  GFX_SetFont(font_8x5);
  GFX_SetFontSize(1);
  // przykładowe dane do pierwszego ekranu
  UiData d = {
    .speed_kmh = 12,
    .power_w   = 312,
    .voltage_v = 41.8f,
    .assist    = 3,
    .trip_km   = 12.4f,
    .time_hh   = 10,
    .time_mm   = 6
  };

  UI_DrawFirstScreen(&d);
  GFX_SetFontSize(2);
  SSD1306_Display();

//  char uart2Data[24] = "Connected to UART Two\r\n";
//     /*
//      * Output to uart2
//      * use screen or putty or whatever terminal software
//      * 8N1 115200
//      */
//     HAL_UART_Transmit(&huart2, (uint8_t *)&uart2Data,sizeof(uart2Data), 0xFFFF);
//
//   	printf("\r\n");
//
//   	printf("Scanning I2C bus:\r\n");
//  	HAL_StatusTypeDef result;
//   	uint8_t i;
//   	for (i=1; i<128; i++)
//   	{
//   	  /*
//   	   * the HAL wants a left aligned i2c address
//   	   * &hi2c1 is the handle
//   	   * (uint16_t)(i<<1) is the i2c address left aligned
//   	   * retries 2
//   	   * timeout 2
//   	   */
//   	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
//   	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
//   	  {
//   		  printf("."); // No ACK received at that address
//   	  }
//   	  if (result == HAL_OK)
//   	  {
//   		  printf("0x%X", i); // Received an ACK at that address
//   	  }
//   	}
//   	printf("\r\n");
//

//
//
//  HAL_FLASH_Unlock();
//
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  /* EEPROM Init */
//  if( EE_Init() != EE_OK)
//  {
//    Error_Handler();
//  }
//
//  // Fill EEPROM variables addresses
//  for(VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++)
//  {
//	  VirtAddVarTab[VarIndex-1] = VarIndex;
//  }
//
//
//  // Store Values in EEPROM emulation
//  HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);
//
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//	{
//	  /* Sequence 1 */
//	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK)
//	  {
//		Error_Handler();
//	  }
//	}
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  // Read values
//  HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//  {
//	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
//	  {
//		  Error_Handler();
//	  }
//  }
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
//  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
//  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
//
//
//  // Store revert Values in EEPROM emulation
//  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
//  HAL_UART_Transmit(&huart2, "Store revert values\n\r", 21, 100);
//
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//	{
//	  /* Sequence 1 */
//	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[NB_OF_VAR-VarIndex-1])) != HAL_OK)
//	  {
//		Error_Handler();
//	  }
//	}
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  // Read values
//  HAL_UART_Transmit(&huart2, "Read revert values\n\r", 20, 100);
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//  {
//	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
//	  {
//		  Error_Handler();
//	  }
//  }
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  HAL_UART_Transmit(&huart2, "Read revert table: ", 19, 100);
//  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
//  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
//
//  // Store Values in EEPROM emulation
//  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
//  HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);
//
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//	{
//	  /* Sequence 1 */
//	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK)
//	  {
//		Error_Handler();
//	  }
//	}
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  // Read values
//  HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
//  {
//	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
//	  {
//		  Error_Handler();
//	  }
//  }
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//  HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
//  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
//  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
//
//
//
//    for(uint8_t i = 0; i < 30; i++)
//    {
//  	  SSD1306_DrawPixel(10+i, 10, WHITE);
//    }

//    GFX_DrawLine(10, 20, 39, 20, WHITE);
//
//    GFX_DrawLine(10, 30, 39, 50, WHITE);
//
//    GFX_DrawRoundRectangle(50, 10, 40, 60, 5, WHITE);



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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART2 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
//
//  return ch;
//}

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
