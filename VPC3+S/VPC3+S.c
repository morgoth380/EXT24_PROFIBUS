#include "stm32f1xx_hal.h"
#include "VPC3+S.h" 
#include "string.h"
#include "spi.h"


#warning Определиться со временем!
#define TIMEOUT_VAL 300


/**
  * @brief  Инициализация микросхемы VPC3+S
  * @param slaveAddr: Адрес устройства. Параметр от верхнего уровня 
  * @param  
  * @retval
  */

void initVPC3(uint16_t slaveAddr)
{

  uint8_t DebugVar = 0;

  uint16_t Add;
  uint8_t N_Diag;
  uint8_t prmVal;
  uint16_t regAddr;

  //pDpSystem = &sDpSystemChannel1;

  //Переводим в состояние Offline
  prmVal = GO_OFFLINE;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  //Сброс запросов прерываний
  prmVal = 0;
  regAddr = GET_VPC_ADR(int_req1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  prmVal = 0;
  regAddr = GET_VPC_ADR(int_req2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  //Разрешение прерывания от User timer clock
  prmVal = USER_TIMER_CLOCK;
  regAddr = GET_VPC_ADR(isreg.wr.int_mask_L);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Остальные прерывания пока запрещаем
  prmVal = 0;
  regAddr = GET_VPC_ADR(isreg.wr.int_mask_H);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Поддержка режимов SYNC и FREEZE
  #warning Вероятно, что поддержка этих режимов должна управляться параметром
  prmVal = (SYNC_SUPPORTED + FREEZE_SUPPORTED);
  regAddr = GET_VPC_ADR(mode_reg0_L);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //1) The User_Time_Clock-Interrupt occurs every 10 ms
  //2) DP_Mode is enabled
  //3) Special clear mode. VPC3+S will accept data telegrams with data unit = 0
  prmVal = (USER_TIMEBASE + DP_MODE + SET_EXT_PRM_SUPP + SPEC_CLEAR_MODE);
  regAddr = GET_VPC_ADR(mode_reg0_H);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg3);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Установка адреса
  prmVal = slaveAddr;
  regAddr = GET_VPC_ADR(slave_addr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));


  //Отладка. Проверка адреса
  regAddr = GET_VPC_ADR(slave_addr); 
  readVPC3(&DebugVar, regAddr, sizeof(uint8_t));

  prmVal = (2000 & 0xFF);
  regAddr = GET_VPC_ADR(user_wd_value_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = (2000 >> 8);
  regAddr = GET_VPC_ADR(user_wd_value_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = RES_USER_WD;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal)); //Сброс пользовательского сторожевого таймера

  prmVal = 0;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mintsdr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal)); 

  #warning Определиться  с длиной буфера 
  prmVal = LEN_DOUT_BUF;
  regAddr = GET_VPC_ADR(len_dout_buf);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (uint16_t) GET_DP_BUFFERS(Dout_Buf1) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(dout_buf_ptr_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Dout_Buf2) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(dout_buf_ptr_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Dout_Buf3) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(dout_buf_ptr_3);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  #warning Определиться с длиной буфера
  prmVal = LEN_DIN_BUF;
  regAddr = GET_VPC_ADR(len_din_buf);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Din_Buf1) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(din_buf_ptr_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Din_Buf2) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(din_buf_ptr_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Din_Buf3) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(din_buf_ptr_3);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_DIAG_BUF;
  regAddr = GET_VPC_ADR(len_diag_buf_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Diag_Buf1) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(diag_buf_ptr_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_DIAG_BUF;
  regAddr = GET_VPC_ADR(len_diag_buf_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Diag_Buf2) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(diag_buf_ptr_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_CNTRL_BUF;
  regAddr = GET_VPC_ADR(len_ctrl_buf_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //prmVal = Len_Cntrl_Buf;
  //adrReg = GET_VPC_ADR(len_ctrl_buf_2);
  //writeVPC3(adrReg, &prmVal, sizeof(prmVal));		//+++

  prmVal = 0;
  regAddr = GET_VPC_ADR(aux_buf_sel);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Aux_Buf1) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(aux_buf_ptr_1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Aux_Buf2) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(aux_buf_ptr_2);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 5;
  regAddr = GET_VPC_ADR(len_ssa_buf);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(SSA_Data) & 0x7FF;

  prmVal = 0;
  regAddr = GET_VPC_ADR(ssa_buf_ptr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_PRM_DATA;
  regAddr = GET_VPC_ADR(len_prm_data);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Prm_Data) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(prm_buf_ptr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_CFG_BUF;
  regAddr = GET_VPC_ADR(r_len_cfg_data);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Cfg_Buf) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(cfg_buf_ptr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Initialization of SAP list for PRM data and DPV1
  Add = (int) GET_DP_BUFFERS(SAP_List) & 0x7FF;
  prmVal = Add >> 3; //адрес сегмента
  regAddr = GET_VPC_ADR(fdl_sap_list_ptr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
///////////////////////////////////////////////////////////
  /*
  adrReg = (int) GET_DP_BUFFERS(SAP_List.SAP49);
  prmVal = 49; //SAP 49
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));

  adrReg++;
  prmVal = 0x7F;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));

  adrReg++;
  prmVal = 0x7F;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));

  adrReg++;
  prmVal = 0x0B;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));

  adrReg++;
  Add = (int) GET_DP_BUFFERS(Ind_Buff0);
  prmVal = Add >> 3;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));
  adrReg++;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));

  adrReg++;
  Add = (int) GET_DP_BUFFERS(Res_Buff0);
  prmVal = Add >> 3;
  writeVPC3(adrReg, &prmVal, sizeof(prmVal));
*/
  ////////////////////////////////////////////////////////
  regAddr = (int) GET_DP_BUFFERS(SAP_List.SAP51) & 0x7FF;
  prmVal = 51; //SAP 51
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
/////////////////////////////////////////////////////////
  regAddr++;
  prmVal = 0x7F;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  prmVal = 0x7F;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  prmVal = 0x51;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  Add = (int) GET_DP_BUFFERS(Ind_Buff0);
  prmVal = Add >> 3;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  Add = (int) GET_DP_BUFFERS(Ind_Buff1);
  prmVal = Add >> 3;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  Add = (int) GET_DP_BUFFERS(Res_Buff0);
  prmVal = Add >> 3;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
////////////////////////////////////////////////////////////
  regAddr = (int) GET_DP_BUFFERS(SAP_List.SAPFF) & 0x7FF;
  prmVal = 0xFF; //End of SAPlist
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  regAddr++;
  prmVal = 0;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  regAddr++;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  regAddr++;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
////////////////////////////////////////////////////////////
  //Initialization of Ind & Resp Buff's
  regAddr = (int) GET_DP_BUFFERS(Ind_Buff0);
  prmVal = 0x00;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  prmVal = 164;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr = (int) GET_DP_BUFFERS(Res_Buff0);
  prmVal = 0x00;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  prmVal = 164;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr = (int) GET_DP_BUFFERS(Ind_Buff1);
  prmVal = 0x00;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr++;
  prmVal = 164;
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  //End of initialization of SAP list
/////////////////////////////////////////////////////////////

  //p_DP_Buf->Cfg_Buf[0]=0x00;		//2 PKW
  //p_DP_Buf->Cfg_Buf[1]=0xF1;              //2 PZD
  prmVal = 0x00;
  regAddr = GET_DP_BUFFERS(Cfg_Buf[0]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  prmVal = 0xF1;
  regAddr = GET_DP_BUFFERS(Cfg_Buf[1]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = LEN_CFG_BUF;
  regAddr = GET_VPC_ADR(len_read_cfg_data);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  Add = (int) GET_DP_BUFFERS(Read_Cfg_Buf) & 0x7FF;
  prmVal = Add >> 3;
  regAddr = GET_VPC_ADR(read_cfg_buf_ptr);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0xF3;
  regAddr = GET_DP_BUFFERS(Read_Cfg_Buf[0]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  prmVal = 0xF1;
  regAddr = GET_DP_BUFFERS(Read_Cfg_Buf[1]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0xFF;
  regAddr = GET_VPC_ADR(real_no_add_change);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

#warning определиться с идентификатором устройства
  prmVal = IDENT0;
  regAddr = GET_VPC_ADR(ident_low);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = IDENT1;
  regAddr = GET_VPC_ADR(ident_high);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0x1E;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.wd_baud_ctrl);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0;
  regAddr = GET_DP_BUFFERS(Diag_Buf1[0]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  prmVal = 2;
  regAddr = GET_DP_BUFFERS(Diag_Buf1[6]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = 0;
  regAddr = GET_DP_BUFFERS(Diag_Buf2[0]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  prmVal = 2;
  regAddr = GET_DP_BUFFERS(Diag_Buf2[6]);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  regAddr = GET_VPC_ADR(ctrl_reg.rd.new_diag_buf_cmd); 
  readVPC3(&N_Diag, regAddr, sizeof(N_Diag));

  prmVal = START_SPC3;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s); //Старт модуля. Переход из состояния Offline state в Passive_Idle (ожидание приема от мастера)
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
}


/**
  * @brief  Отправка данных в микросхему VPC3+S
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