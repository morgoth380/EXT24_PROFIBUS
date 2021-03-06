#include "stm32f1xx_hal.h"
#include "VPC3+S.h" 
#include "string.h"
#include "spi.h"


#warning Определиться со временем!
#define TIMEOUT_VAL 300

uint16_t getOutputBufPnt(void);
uint16_t getInputBufPnt(void);

uint8_t cnfTelegramChecking(uint8_t *pToCfgData, uint8_t bCfgLength);
DP_ERROR_CODE VPC3_CalculateInpOutpLength( uint8_t *pToCfgData, uint8_t bCfgLength);

VPC3_SYSTEM_STRUC  sDpSystemChannel1;   /**< Глобальная структура управления протоколом. */

VPC3_SYSTEM_STRUC *pDpSystem;

uint8_t N_Din; // номер следующего выходного буфера для формирования данных к мастеру

#warning Пока этот объект глобальный, может его нужно перенести в какой-то супер объект управления протоколом
PzdiPzdo_Type PzdiPzdo;

//Массив указателей на обработчики прерываний
/*
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
*/

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

  pDpSystem = &sDpSystemChannel1;

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
  prmVal = USER_TIMER_CLOCK | BAUDRATE_RETECT;
  regAddr = GET_VPC_ADR(isreg.wr.int_mask_L); //0x04
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Остальные прерывания пока запрещаем
  prmVal = 0;
  regAddr = GET_VPC_ADR(isreg.wr.int_mask_H); //0x05
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  //Поддержка режимов SYNC и FREEZE
  #warning Вероятно, что поддержка этих режимов должна управляться параметром
  prmVal = (SYNC_SUPPORTED + FREEZE_SUPPORTED) + INT_POL;
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

#warning определиться с временем контроля
  prmVal = /*0x1E*/0xFF;
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

  //Отладка - До перевода VPC3 в работу проверим статус VPC3
  regAddr = GET_VPC_ADR(isreg.rd.status_L);
  readVPC3(&prmVal, regAddr, sizeof(prmVal));
  
  prmVal = START_SPC3;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s); //Старт модуля. Переход из состояния Offline state в Passive_Idle (ожидание приема от мастера)
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));
  
  //Отладка - Сразу проверяем состояние VPC3
  regAddr = GET_VPC_ADR(isreg.rd.status_L);
  readVPC3(&prmVal, regAddr, sizeof(prmVal));  
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

/**
  * @brief  Обработка прерывания от VPC3+S
  * @param  None
  * @retval None
  */
void VPC3_InterruptProcessing(void)
{
    uint16_t regAddr;
    uint16_t intRequestReg = 0;
    uint16_t intReg = 0;
    uint16_t i;
 
    /*
    //Считываем слово текущих запросов прерываний
    regAddr = GET_VPC_ADR(int_req1);
    readVPC3(&intRequestReg, regAddr, sizeof(intRequestReg));
    
    if((intRequestReg & (1 << 2)) != 0){
      intRequestReg++;
      intRequestReg--;
    }
    
   
#warning тут скорее всего дичь. Во первых почему эти прерывания игнорируются, а во вторых тут возможна установка лишнего бита
    if (((intRequestReg & 0x0C00U) == NEW_PRM_DATA) || ((intRequestReg & 0x0C00) == NEW_CFG_DATA)) {
      intRequestReg ^= NEW_CFG_DATA;
      intRequestReg ^= NEW_PRM_DATA;
    }

    //Для каждого бита, установленного в слове запросов прерываний,
    //вызываем соответствующую функцию
    i = 0;
    while(intRequestReg){
      if(intRequestReg & (1 << i)){
        if(pHandlerProfi[i]){
          pHandlerProfi[i]();
        }
      }
      intRequestReg &= ~(1 << i);
      i++;
    }*/
    
}


#warning Разобраться что делает эта функция
uint8_t do_MAC_Reset(void)
{
  uint8_t val;
  uint16_t modeRegAddr = 0;

  val = START_VPC3;
  modeRegAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
  writeVPC3(modeRegAddr, &val, sizeof(val));

  return 0;
}

uint8_t do_GO_LEAVE_DATA_EX(void)
{
  uint16_t regAddr;
  uint8_t VPC3State;
  
  regAddr = GET_VPC_ADR(isreg.rd.status_L);      //регистр статуса
  readVPC3(&VPC3State, regAddr, sizeof(uint8_t));
  
  //если вышли из состояния обмена сброс VPC3
  if((VPC3State & MASK_DP_STATE) != DATA_EX){
    #warning тут нужно вставить реальный адрес, полученный от верхнего уровня
    initVPC3(3); 
  }

  return 0;
}

uint8_t do_Baudrate_Detect(void)
{
  return 0;
}

uint8_t do_WD_DP_MODE_TIMEOUT(void)
{
  return 0;
}

uint8_t do_User_Timer_Clock(void)
{
  
  uint16_t regAddr;
  uint8_t prmVal;

  prmVal = RES_USER_WD;
  regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  prmVal = USER_TIMER_CLOCK;
  regAddr = GET_VPC_ADR(isreg.wr.int_ack1);
  writeVPC3(regAddr, &prmVal, sizeof(prmVal));

  return 0;
}

uint8_t do_DXB_Link_Error(void)
{
  return 0;
}

uint8_t do_NEW_Ext_Prm_Data(void)
{
  return 0;
}


uint8_t do_DXB_Out(void)
{
  return 0;
}

uint8_t do_NEW_GC_COMMAND(void)
{
  return 0;
}


uint8_t do_NEW_SSA_DATA(void)
{
  return 0;
}


#warning Эта функция по факту в техасе вроде как не вызывалась. Проверить как вызывалась альтернативная функция.
#warning Скорее всего ее нужно заменить на ту что по факту вызвалась в техасовском проекте
uint8_t do_Prm(void)
{
    uint8_t prmVal;
    uint16_t regAddr;
    uint16_t bufAddr;
    uint8_t Aux[32];

    // параметры сети Profibus DP
    bufAddr = GET_DP_BUFFERS(Prm_Data[0]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[0] = prmVal;

    bufAddr = GET_DP_BUFFERS(Prm_Data[1]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[1] = prmVal;
    regAddr = GET_VPC_ADR(user_wd_value_1);
    writeVPC3(regAddr, &prmVal, sizeof(prmVal));

    bufAddr = GET_DP_BUFFERS(Prm_Data[2]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[2] = prmVal;
    regAddr = GET_VPC_ADR(user_wd_value_2);
    writeVPC3(regAddr, &prmVal, sizeof(prmVal));

    bufAddr = GET_DP_BUFFERS(Prm_Data[3]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[3] = prmVal;
    regAddr = GET_VPC_ADR(ctrl_reg.wr.mintsdr);
    writeVPC3(regAddr, &prmVal, sizeof(prmVal));

    bufAddr = GET_DP_BUFFERS(Prm_Data[4]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
   //ExtPrm[4] = prmVal;
    bufAddr = GET_DP_BUFFERS(Prm_Data[5]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[5] = prmVal;
    bufAddr = GET_DP_BUFFERS(Prm_Data[6]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[6] = prmVal;

    bufAddr = GET_DP_BUFFERS(Prm_Data[7]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[7] = prmVal;

    bufAddr = GET_DP_BUFFERS(Prm_Data[8]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[8] = prmVal;

    bufAddr = GET_DP_BUFFERS(Prm_Data[9]);
    readVPC3(&prmVal, bufAddr, sizeof(prmVal));
    //ExtPrm[9] = prmVal;

    bufAddr = GET_DP_BUFFERS(Aux_Buf1);
    readVPC3(&Aux, bufAddr, 32);

    regAddr = GET_VPC_ADR(ctrl_reg.rd.user_prm_data_ok);
    readVPC3(&prmVal, regAddr, sizeof(prmVal));

    return prmVal;
}

/**
  * @brief  Обработка прерывания события конфигурации
  * @param  None
  * @retval None
  */
uint8_t do_Cfg(void)
{
  uint8_t len_cfg_data;
  uint8_t prmVal;
  uint8_t cnfCheckStatus;
  uint16_t regAddr;
  uint16_t buffAddr;
  DP_ERROR_CODE calcDataLen;
  uint8_t ok = 1;
  uint8_t aFmt[LEN_CFG_BUF]; //массив конфигурационных данных 
  
  //Длина принятой телеграммы
  regAddr = GET_VPC_ADR(r_len_cfg_data);                  //Адрес регистра с длиной конфигурационной телеграммы
  readVPC3(&len_cfg_data, regAddr, sizeof(len_cfg_data)); //Читаем длину кофигурационной телеграммы
  
  buffAddr = GET_DP_BUFFERS(Cfg_Buf[0]);
  readVPC3(&aFmt[0], buffAddr, len_cfg_data);                   //считывание массива конфигурационных данных
  cnfCheckStatus = cnfTelegramChecking(&aFmt[0], len_cfg_data); //проверка конфигурационных данных на соответствие GSD
  
  if(!cnfCheckStatus){ //конф. телеграмма ок?
    calcDataLen = VPC3_CalculateInpOutpLength( &aFmt[0], len_cfg_data ); //Расчет длины IO
    if(calcDataLen == DP_OK){
      regAddr = GET_VPC_ADR(len_dout_buf);

      prmVal = pDpSystem->bOutputDataLength;
      writeVPC3(regAddr, &prmVal, sizeof(uint8_t)); //Настройка

      regAddr = GET_VPC_ADR(len_din_buf);
      prmVal = pDpSystem->bInputDataLength;
      writeVPC3(regAddr, &prmVal, sizeof(uint8_t)); //Настройка

      prmVal = En_Change_Cfg_Puffer;
      regAddr = GET_VPC_ADR(ctrl_reg.wr.mode_reg1_s);
      writeVPC3(regAddr, &prmVal, sizeof(prmVal));

      regAddr = GET_VPC_ADR(ctrl_reg.rd.new_din_buf_cmd);
      readVPC3(&N_Din, regAddr, sizeof(N_Din));
    }else{
       ok = 0;
    }
  }else{
     ok = 0;
  }
  
  return 0;  
}

#warning Эта функция по факту в техасе вроде как не вызывалась. Проверить как вызывалась альтернативная функция.
#warning Скорее всего ее нужно заменить на ту что по факту вызвалась в техасовском проекте
uint8_t do_Diag(void)
{
  uint8_t Diag[LEN_DIAG_BUF];
  uint8_t NDiag;
  uint16_t regAddr;
  uint16_t bufAddr;

  uint8_t Aux[32];
  bufAddr = GET_DP_BUFFERS(Aux_Buf1);
  readVPC3(&Aux, bufAddr, 32);

//------------выбор буфера для диагностики--------------------------
  regAddr = GET_VPC_ADR(ctrl_reg.rd.diag_buffer_sm);
  readVPC3(&NDiag, regAddr, sizeof(NDiag));

//--------------формирование буфера диагностики---------------------
  Diag[0] = 0x08;  // 0 байт (0 - Ext_Diag; 1 - Stat_Diag; 2 - Ext_Diag_Overf)
  Diag[1] = 0x0C;
  Diag[2] = 0x00;
  Diag[3] = 0x02;
  Diag[4] = IDENT1;
  Diag[5] = IDENT0;
  Diag[6] = 0x02;
  Diag[7] = 0x01;

  if ((NDiag &= 3)) {
          bufAddr = GET_DP_BUFFERS(Diag_Buf1);
  } else {
          bufAddr = GET_DP_BUFFERS(Diag_Buf2);
  };

  writeVPC3(bufAddr, &Diag[0], 8);
  //-------------изменить буфер диагностики------------------------
  regAddr = GET_VPC_ADR(ctrl_reg.rd.new_diag_buf_cmd);

  readVPC3(&NDiag, regAddr, sizeof(NDiag));

  return NDiag;   
}

/**
  * @brief  Обработка прерывания события Data_Exchange telegram
  * @param  None
  * @retval None
  */
#warning В техасовском проекте эта обработка вызывалась не в прерывании
#warning Почему тут опрашиваются не запросы а флаги?
uint8_t do_DX_OUT(void)
{
  uint16_t pp1;
  uint16_t regAddr;
  uint16_t IR_Status;
  uint16_t pToOutputBuffer;
  uint16_t pToInputBuffer;
  uint16_t cmpStatus;
  uint16_t cmpAddrStatus;
  uint16_t i, j;
  uint16_t prmVal;
  uint16_t PZDI[LEN_DIN_BUF];
  uint8_t outputBuffer[LEN_DOUT_BUF];
  static uint16_t prevPZDO[LEN_DOUT_BUF] = {0};
  static uint16_t aPrevPZDOadr[LEN_DOUT_BUF] = {0}; //предыдущие адреса записываемых в привод параметров
  uint16_t PZDO[DOUT_DATA_LENGTH];
  uint32_t tempPZDI[DIN_DATA_LENGTH];
  uint8_t DinCmd;
  
  regAddr = GET_VPC_ADR(isreg.rd.int_reg1);         //Текущие флаги прерываний
  readVPC3(&IR_Status, regAddr, sizeof(IR_Status));  
  
  //Проверка флага прерывания по обмену данными
  if(IR_Status & DX_OUT){
    pToOutputBuffer = getOutputBufPnt(); //указатель на буфер с данными от ПЛК
    if(pToOutputBuffer){
      #warning Реализовать инициализацию pDpSystem->bOutputDataLength в конфигурационной телеграмме
      readVPC3(&outputBuffer[0], pToOutputBuffer, pDpSystem->bOutputDataLength);
      
      for(i = 0, j = 0; j < pDpSystem->bOutputDataLength / 2; i = j * 2){
        PZDO[j] = (outputBuffer[i] << 8) + outputBuffer[i + 1];
        j++;
      }
      
      cmpStatus = memcmp(&PZDO[0], &prevPZDO[0], pDpSystem->bOutputDataLength);
      cmpAddrStatus = memcmp(&PzdiPzdo.PZDO.aPZDOadr[0], &aPrevPZDOadr[0], pDpSystem->bOutputDataLength); //проверка изменения адресов
      
      if((cmpStatus != 0) || (cmpAddrStatus != 0)){ //!если содержимое буфера изменилось или адрес изменился, пишем в систему параметров ПЧ
        for(i = 0; i < pDpSystem->bOutputDataLength / 2; i++){
          if( (PZDO[i] != prevPZDO[i]) || (PzdiPzdo.PZDO.aPZDOadr[i] != aPrevPZDOadr[i])){ //!пишем если значение параметра изменилось
          #warning тут пишем данные в буфер который далее передается на врхний уровень
          #warning Скорее всего этот цикл не нужен. Нужно просто передать данные на верхний уровен. Этот цикл нужен был только для проекта Техаса 
          }
        }
        memcpy(&prevPZDO[0], &PZDO[0], pDpSystem->bOutputDataLength);
        memcpy(&aPrevPZDOadr[0], &PzdiPzdo.PZDO.aPZDOadr[0], pDpSystem->bOutputDataLength);
      }
    }
    
    regAddr = GET_VPC_ADR(isreg.wr.int_ack2); //Квитируем прерывание получения телеграммы Data_Exchange telegram
    prmVal = DX_OUT;
    writeVPC3(regAddr, &prmVal, sizeof(prmVal)); //квитирование прерывания
    
    /*****************Считывание параметров из ПЧ в ПЛК*****************/
    pToInputBuffer = getInputBufPnt(); //!адрес доступного буфера для отправки данных в ПЛК
    if(pToInputBuffer){
      
#warning Следующий цикл не нужен. Он нужен был в техасе. Нужно просто отправлять принятые от CP24 данные
      for(i = 0; i < pDpSystem->bInputDataLength / 2; i++){ //формируем массив из системы параметров ПЧ
        //read_PKW(PzdiPzdo.PZDI.aPZDIadr[i], &tempPZDI[i], &pp1); //массив для отправки в ПЛК
      }

#warning Этот цикл тоже не нужен
      /**Значение из s32 в u16***/
      for(i = 0; i < pDpSystem->bInputDataLength / 2; i++){
        PZDI[i] = (uint16_t)tempPZDI[i]; //массив для отправки в ПЛК тип u16
      }

      #warning Зачем swap?
      for(i = 0; i < pDpSystem->bInputDataLength / 2; i++){
        PZDI[i] = (PZDI[i] << 8) | ((uint16_t)0x00FF & (PZDI[i] >> 8));
      }

      writeVPC3(pToInputBuffer, &PZDI[0], pDpSystem->bInputDataLength); //запись значений параметров в буфер для отправки в ПЛК

      regAddr = GET_VPC_ADR(ctrl_reg.rd.new_din_buf_cmd); //09h - регистр, считывание которого делает буфер доступным для отправки в ПЛК
      readVPC3(&DinCmd, regAddr, sizeof(uint8_t));
    }
    
  }
  
  return 0; 
}

uint8_t do_Poll_End_Ind(void)
{
  return 0;
}

uint8_t do_FDL_Inf(void)
{
  return 0;
}

/**
  * @brief  Получение указателя на буфер с данными
  * @param  
  * @retval Указатель на буфер с полученными от ПЛК данными
  */
uint16_t getOutputBufPnt(void){
  uint16_t pToOutputBuffer;
  uint16_t regAddr;
  uint8_t DoutSegAddr;
  uint8_t U_BufferAssignment; //текущий номер указателя на буфер данных

  regAddr = GET_VPC_ADR(ctrl_reg.rd.dout_buffer_sm);
  readVPC3(&U_BufferAssignment, regAddr, sizeof(U_BufferAssignment));

  //Проверяем какой из регистров содержит адрес буфера с данными
  switch(U_BufferAssignment & 0x30){
  case 0x10: //dout_buf_ptr_1
          regAddr = GET_VPC_ADR(dout_buf_ptr_1); //адрес регистра с сегементным адресом буфера 1
          readVPC3(&DoutSegAddr, regAddr, sizeof(uint8_t));
          pToOutputBuffer = DoutSegAddr << 3;   //Переводим адрес сегмента в абсолютный адрес
          break;
  case 0x20: //dout_buf_ptr_2
          regAddr = GET_VPC_ADR(dout_buf_ptr_2); //адрес параметра с указателем на буфер 2
          readVPC3(&DoutSegAddr, regAddr, sizeof(uint8_t));
          pToOutputBuffer = DoutSegAddr << 3;
          break;
  case 0x30: //dout_buf_ptr_3
          regAddr = GET_VPC_ADR(dout_buf_ptr_3); //адрес параметра с указателем на буфер 3
          readVPC3(&DoutSegAddr, regAddr, sizeof(uint8_t));
          pToOutputBuffer = DoutSegAddr << 3;
          break;
  default:
          pToOutputBuffer = VPC3_NULL_PTR;
          break;
  }

  return(pToOutputBuffer);
}


/**
  * @brief  Получение указателя на буфер для отправки данных в ПЛК
  * @param  
  * @retval Указатель на буфер куда можно писать данные для отправки в ПЛК
  */
uint16_t getInputBufPnt(void){
  uint8_t  U_BufferAssignment; //текущий номер указателя на буфер данных
  uint8_t DinSegAddr;
  uint16_t regAddr;
  uint16_t pToInputBuffer;

  regAddr = GET_VPC_ADR(ctrl_reg.rd.din_buffer_sm);           //
  readVPC3(&U_BufferAssignment, regAddr, sizeof(U_BufferAssignment)); //Номер текущего доступного пользователю буфера

  switch(U_BufferAssignment & 0x30){
  case 0x10:
          regAddr = GET_VPC_ADR(din_buf_ptr_1);            //Адрес параметра с указателем на буфер 1
          readVPC3(&DinSegAddr, regAddr, sizeof(uint8_t)); //Читаем адрес сегмента
          pToInputBuffer = DinSegAddr << 3;                //Переводим адрес сегмента в абсолютный адрес
          break;
  case 0x20:
          regAddr = GET_VPC_ADR(din_buf_ptr_2);            //адрес параметра с указателем на буфер 2
          readVPC3(&DinSegAddr, regAddr, sizeof(uint8_t)); //Читаем адрес сегмента
          pToInputBuffer = DinSegAddr << 3;                //Переводим адрес сегмента в абсолютный адрес
          break;
  case 0x30:
          regAddr = GET_VPC_ADR(din_buf_ptr_3); //адрес параметра с указателем на буфер 3
          readVPC3(&DinSegAddr, regAddr, sizeof(uint8_t));
          pToInputBuffer = DinSegAddr << 3;
          break;
  default:
          pToInputBuffer = VPC3_NULL_PTR;
          break;
  }
  return(pToInputBuffer);
}

/**
 * @fn - cnfTelegramChecking
 * @brief - Проверка конфигурационных байт на соответствие GSD
 * @param   - pToCfgData указатель на буфер с конфигурационными данными
 * @param   - bCfgLength длина буфера
 * @return  - 0 - если данные корректные, 1 - в противном случае
 */
uint8_t cnfTelegramChecking(uint8_t *pToCfgData, uint8_t bCfgLength){
  if( ((pToCfgData[0] >> 4) == 0x0D) || ((pToCfgData[0] >> 4) == 0x0E) || ((pToCfgData[0] >> 4) == 0x05) || ((pToCfgData[0] >> 4) == 0x06) ){ //проверка соответствия конф. данных GSD-файлу
          return(0);
  }else{
          return(1);
  }
}

/**
 * @fn    - VPC3_CalculateInpOutpLength
 * @brief - Расчет длины входных/выходных данных по конфигурационной телеграмме
 * @param - pToCfgData указатель на буфер с конфигурационными данными
 * @param - bCfgLength длина буфера с конфигурационными данными
 * @return  - DP_OK, если расчет ОК
 * @return  - DP_CFG_LEN_ERROR, если некорректная длина конфигурационно телеграммы
 * @return  - DP_CALCULATE_IO_ERROR, если ошибка длины входных/выходных данных
 * @return  - DP_CFG_FORMAT_ERROR, ошибка в формате телеграммы
 */
DP_ERROR_CODE VPC3_CalculateInpOutpLength( uint8_t *pToCfgData, uint8_t bCfgLength)
{
   DP_ERROR_CODE bError;
   uint8_t       bSpecificDataLength;
   uint8_t       bTempOutputDataLength;
   uint8_t       bTempInputDataLength;
   uint8_t       bLength;
   uint8_t       bCount;

   bError = DP_OK;
   bTempOutputDataLength = 0;
   bTempInputDataLength  = 0;

   if( (bCfgLength > 0) && (bCfgLength <= LEN_CFG_BUF) ){
      for( ; bCfgLength > 0; bCfgLength -= bCount )
      {
         bCount = 0;

         if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT ) //биты 4 и 5 конфигурационного байта
         {
            /* general identifier format */
            bCount++;
            /* pToCfgData points to "Configurationbyte", CFG_BF means "CFG_IS_BYTE_FORMAT" */
            bLength = (uint8_t)( ( *pToCfgData & VPC3_CFG_BF_LENGTH) + 1 ); //длина данных

            if( *pToCfgData & VPC3_CFG_BF_OUTP_EXIST ) //если телеграмма касается выходных данных
            {
               bTempOutputDataLength += ( *pToCfgData & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength; //байт или слово
            } /* if( *pToCfgData & VPC3_CFG_BF_OUTP_EXIST ) */

            if( *pToCfgData & VPC3_CFG_BF_INP_EXIST ) //если телеграмма касается входных данных
            {
               bTempInputDataLength += ( *pToCfgData & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength; //байт или слово
            } /* if( *pToCfgData & VPC3_CFG_BF_INP_EXIST ) */

            pToCfgData++;
         } /* if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT ) */
         else
         {
            /* pToCfgData points to the headerbyte of "special identifier format */
            /* CFG_SF means "CFG_IS_SPECIAL_FORMAT" */
            if( *pToCfgData & VPC3_CFG_SF_OUTP_EXIST )
            {
               bCount++;    /* next byte contains the length of outp_data */
               bLength = (uint8_t)( ( *( pToCfgData + bCount ) & VPC3_CFG_SF_LENGTH ) + 1 );

               bTempOutputDataLength += ( *( pToCfgData + bCount ) & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_SF_OUTP_EXIST ) */

            if( *pToCfgData & VPC3_CFG_SF_INP_EXIST )
            {
               bCount++;  /* next byte contains the length of inp_data */
               bLength = (uint8_t)( ( *( pToCfgData + bCount ) & VPC3_CFG_SF_LENGTH ) + 1 );

               bTempInputDataLength += ( *( pToCfgData + bCount ) & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_SF_INP_EXIST ) */

            bSpecificDataLength = (uint8_t)( *pToCfgData & VPC3_CFG_BF_LENGTH );

            if( bSpecificDataLength != 15 )
            {
               bCount = (uint8_t)( bCount + 1 + bSpecificDataLength );
               pToCfgData = pToCfgData + bCount;
            } /* if( bSpecificDataLength != 15 ) */
            else
            {
               /* specific data length = 15 not allowed */
               pDpSystem->bInputDataLength  = 0xFF;
               pDpSystem->bOutputDataLength = 0xFF;
               bError = DP_CFG_FORMAT_ERROR;
            } /* else of if( bSpecificDataLength != 15 ) */
         } /* else of if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT ) */
      } /* for( ; bCfgLength > 0; bCfgLength -= bCount ) */

      if( ( bCfgLength != 0 ) || ( bTempInputDataLength > LEN_DIN_BUF ) || ( bTempOutputDataLength > LEN_DOUT_BUF ) ) //а может быть и меньше???
      {
         pDpSystem->bInputDataLength  = 0xFF;
         pDpSystem->bOutputDataLength = 0xFF;

         bError = DP_CALCULATE_IO_ERROR;
      } /* if( ( bCfgLength != 0 ) || ( bTempInputDataLength > DIN_BUFSIZE ) || ( bTempOutputDataLength > DOUT_BUFSIZE ) ) */
      else
      {
         pDpSystem->bInputDataLength  = bTempInputDataLength;
         pDpSystem->bOutputDataLength = bTempOutputDataLength;
         bError = DP_OK;
      } /* else of if( ( bCfgLength != 0 ) || ( bTempInputDataLength > DIN_BUFSIZE ) || ( bTempOutputDataLength > DOUT_BUFSIZE ) ) */
   } /* if( ( bCfgLength > 0 ) && ( bCfgLength <= CFG_BUFSIZE ) ) */
   else //некорректная длина конфигурационных данных:
   {
      pDpSystem->bInputDataLength  = 0xFF;
      pDpSystem->bOutputDataLength = 0xFF;
      bError = DP_CFG_LEN_ERROR;
   } /* else of if( ( bCfgLength > 0 ) && ( bCfgLength <= CFG_BUFSIZE ) ) */

   return bError;
} /* DP_ERROR_CODE VPC3_CalculateInpOutpLength( MEM_UNSIGNED8_PTR pToCfgData, uint8_t bCfgLength ) */