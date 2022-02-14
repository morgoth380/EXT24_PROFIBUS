/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "VPC3+S.h"   

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

uint8_t do_MAC_Reset(void);
uint8_t do_GO_LEAVE_DATA_EX(void);
uint8_t do_Baudrate_Detect(void);
uint8_t do_WD_DP_MODE_TIMEOUT(void);
uint8_t do_User_Timer_Clock(void);
uint8_t do_NEW_GC_COMMAND(void);
uint8_t do_NEW_SSA_DATA(void);
uint8_t do_Prm(void);
uint8_t do_Cfg(void);
uint8_t do_Diag(void);
uint8_t do_DX_OUT(void);
uint8_t do_DXB_Link_Error(void);
uint8_t do_NEW_Ext_Prm_Data(void);
uint8_t do_DXB_Out(void);
uint8_t do_Poll_End_Ind(void);
uint8_t do_FDL_Inf(void);

static uint8_t (*pHandlerProfi[])(void) = {
    do_MAC_Reset,
    do_GO_LEAVE_DATA_EX,
    do_Baudrate_Detect,
    do_WD_DP_MODE_TIMEOUT,
    do_User_Timer_Clock,
    do_DXB_Link_Error,
    do_NEW_Ext_Prm_Data,
    do_DXB_Out,
    do_NEW_GC_COMMAND,
    do_NEW_SSA_DATA,
    do_Cfg,
    do_Prm,
    do_Diag,
    do_DX_OUT,
    do_Poll_End_Ind,
    do_FDL_Inf,
};



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint16_t addr;
  uint16_t addr2;
  uint16_t regVal;
  uint16_t VPC3_Status;
  uint16_t boudrateStatus;
  uint16_t interuptStatus;
  uint16_t debugCmpResult;
  uint16_t intRequestReg;
  uint16_t intRequestRegAck;
  uint16_t i;
  uint16_t resetMask;
  uint16_t checkMask;
  uint16_t val;
  static uint16_t intRequestArray[15] = {0};
  static uint16_t j = 0;
  
  #pragma pack(push,1)
  uint8_t debugWrBuf[10];
  uint8_t debugRdBuf[10];
  #pragma pack(pop)
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  
  #warning После реализации связи по UART вызвать initVPC3 при получении profibus-адреса
  initVPC3(1);

  while (1)
  {
    
    addr = GET_VPC_ADR(isreg.rd.status_L);
    readVPC3(&regVal, addr, sizeof(regVal));
    boudrateStatus = (regVal >> 8) & 0x0F;
    
    if(boudrateStatus == 0x0F){
      boudrateStatus++;
      boudrateStatus--;
    }else{
        //Пока обрабатываем событие в режиме проверки статусов
        addr = GET_VPC_ADR(int_req1);
        intRequestReg = 0;
        readVPC3(&intRequestReg, addr, sizeof(intRequestReg));

        /*Для каждого бита, установленного в слове запросов прерываний,
         вызываем соответствующую функцию */
        
        if((intRequestReg != 0) && (j < 15)){
          intRequestArray[j++] = intRequestReg;
        }
        
        
        intRequestRegAck = intRequestReg;
        i = 0;
        while(intRequestReg){
          checkMask = 1 << i;
          if(intRequestReg & checkMask){
            if(pHandlerProfi[i]){
              pHandlerProfi[i]();
            }
          }
          resetMask = ~checkMask;
          intRequestReg &= resetMask;
          i++;
        }
        
        addr = GET_VPC_ADR(isreg.wr.int_ack1);
        writeVPC3(addr, &intRequestRegAck, sizeof(uint16_t));
        
        if(intRequestRegAck != 0){ //если было прерывание
          val = EOI;
          addr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
          writeVPC3(addr, &val, sizeof(uint8_t));
        }
     }
    /*
    debugWrBuf[0] = 0;
    debugWrBuf[1] = 1;
    debugWrBuf[2] = 2;
    debugWrBuf[3] = 3;
    debugWrBuf[4] = 4;
    debugWrBuf[5] = 5;
    debugWrBuf[6] = 6;
    debugWrBuf[7] = 7;
    debugWrBuf[8] = 8;
    debugWrBuf[9] = 9;
    
    debugRdBuf[0] = 0;
    debugRdBuf[1] = 0;
    debugRdBuf[2] = 0;
    debugRdBuf[3] = 0;
    debugRdBuf[4] = 0;
    debugRdBuf[5] = 0;
    debugRdBuf[6] = 0;
    debugRdBuf[7] = 0;
    debugRdBuf[8] = 0;
    debugRdBuf[9] = 0;
    
    addr = GET_DP_BUFFERS(Din_Buf1);

    writeVPC3(addr, &debugWrBuf[0], sizeof(debugWrBuf));
    readVPC3(&debugRdBuf[0], addr, sizeof(debugRdBuf));
    debugCmpResult = memcmp(&debugWrBuf[0], &debugRdBuf[0], sizeof(debugRdBuf));
    if(debugCmpResult != 0){
      debugCmpResult++;
      debugCmpResult--;
    }
    */
    
    /*
    //Текущие запросы прерываний
    addr2 = GET_VPC_ADR(int_req1);
    readVPC3(&interuptStatus, addr2, sizeof(interuptStatus));
    if(interuptStatus != 12304U){
      interuptStatus++;
      interuptStatus--;
    }
*/
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

