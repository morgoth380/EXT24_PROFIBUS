#include "stm32f1xx_hal.h"
#include "VPC3+S.h" 
#include "string.h"
#include "spi.h"


#warning Определиться со временем!
#define TIMEOUT_VAL 300


/**
  * @brief  Отпрвка данных в микросхему VPC3+S
  * @param  regAddr: адрес регистра VPC3+S
  * @param  pTxVal: указатель на отправляемые данные
  * @retval Результат операции
  */
uint16_t writeVPC3(uint16_t regAddr, const void *pTxVal, uint32_t sz)
{
  uint16_t SPI_dataLength;
  uint8_t WrBuf[MAX_DOUT_BUF] = {0}; //Локальный буфер для отправки
  
  if(sz == 0){
    return 1;
  }
  
  WrBuf[0] = (sz == 1) ? WRITE_BYTE_INSTRUCTION : WRITE_ARRAY_INSTRUCTION; //Код операции
  WrBuf[1] = (regAddr >> 8) & 0x00FFU;                  //Старший байт адреса
  WrBuf[2] = regAddr & 0x00FFU;                         //Младший байт адреса
  
  memcpy((uint8_t *)&WrBuf[3], (uint8_t *)pTxVal, sz); //Копируем значения для передачи
  SPI_dataLength = 3 + sz;                             //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Transmit(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High 
  
  return 0;
}

/**
  * @brief  Чтение данных с VPC3+S
  * @param  pRxVal: указатель куда считывать данные
  * @param  regAddr: адрес регистра VPC3+S
  * @retval Результат операции
  */
uint16_t readVPC3(void *pRxVal, uint16_t regAddr, uint32_t sz)
{
  uint8_t WrBuf[MAX_DOUT_BUF] = {0}; //Локальный буфер для отправки
  uint16_t SPI_dataLength;
  
  if(sz == 0){
    return 1;
  }
  
  WrBuf[0] = (sz == 1) ? READ_BYTE_INSTRUCTION : READ_ARRAY_INSTRUCTION; //Код операции
  WrBuf[1] = (regAddr >> 8) & 0x00FFU;                  //Старший байт адреса
  WrBuf[2] = regAddr & 0x00FFU;                         //Младший байт адреса
  
  SPI_dataLength = 3 + sz;                             //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Receive(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High
  
  memcpy((uint8_t *)pRxVal, (uint8_t *)&WrBuf[3], sz); //Копируем считанные значения
  
  return 0;
}