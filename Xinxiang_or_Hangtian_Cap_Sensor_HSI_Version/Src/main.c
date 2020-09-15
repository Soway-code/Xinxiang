#include "main.h"
#include "Picocap_app.h"
#include "adc_app.h"
#include "dac_bsp.h"
#include "iwdg_bsp.h"
#include "ModBus.h"
#include "In_Memory_app.h"
#include "tim_bsp.h"

#define APPLICATION_ADDRESS     (uint32_t)0x08003000

#define VECTOR_TABLE_SIZE       48 * 4

#if   (defined ( __CC_ARM ))
__IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
__no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#endif // (defined ( __CC_ARM ))



/* ϵͳʱ������ */
void SystemClock_Config(void);


int main(void)
{
    /* F0ϵ��û���ж���������ӳ��Ĵ��������ԣ�����ʹ����һ�ַ����������ж�������
    0x20000000��ʼ���ڴ�ռ��У��ٽ�����ڴ��ַ��0x00000000������ж����������ӳ�� */
    memcpy((void*)VectorTable, (void*)APPLICATION_ADDRESS, VECTOR_TABLE_SIZE); 
    /* ��ϵͳ�ڴ�ӳ�䵽0x00000000 */
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
    
    uint32_t                            FilterResult;               //�˲���Ľ��
    uint32_t                            PCap_PWM_Period_Old;        //��һ��PWM���ڵ�ֵ
    static DataFilterParam              FilterParam;                //�����˲���Ҫ�Ĳ����ṹ��
    static PCap_DataConvert_Param       DataConvert_Param;          //PCap������ת����Ҫ�Ĳ����ṹ��
    static PCap_DataConvert_Out_Param   DataConvert_Out;            //PCapת����������ݵĽṹ��
#ifdef USING_ADC_TEMPER_SENSOR    
    static ADC_TemperParam_TypeDef      ADC_TemperParam;            //ADC�¶ȴ�����Ҫ�Ĳ����ṹ��
    static ADC_TemperOut_TypeDef        ADC_TemperOut;              //ADC�¶ȴ���������ݵĽṹ��
#endif // USING_ADC_TEMPER_SENSOR 
    static ModBusBaseParam_TypeDef      ModBusBaseParam;            //ModBus����Ļ��������ṹ
    static ModBus_Device_Param          ModBus_Device;              //ModBus�����豸�Ľṹ��
    uint32_t                    PCap_Tick_Start  = HAL_GetTick();   //���㳬ʱ����ʼֵ������Pcap��ʱ�ɼ�
    __IO uint32_t               Auto_Up_Tick_Start = HAL_GetTick(); //���㳬ʱ����ʼֵ������ModBus��ʱ�Զ��ϴ�
    uint32_t                            temp_buf;
    float                               temp_res_value;
    
    /* Ϊ ModBus ��� ADC �� Pcap �豸���� */
#ifdef USING_ADC_TEMPER_SENSOR 
    ModBus_Device.ADC_TemperParam         = &ADC_TemperParam;
    ModBus_Device.ADC_TemperOut           = &ADC_TemperOut;
#endif // USING_ADC_TEMPER_SENSOR 
    ModBus_Device.DataFilter              = &FilterParam;
    ModBus_Device.PCap_DataConvert        = &DataConvert_Param; 
    ModBus_Device.PCap_DataConvert_Out    = &DataConvert_Out;            
    
/**************************************** ��ʼ��HAL���ϵͳʱ�� ****************************************/
    HAL_Init();                                         //HAL���ʼ��
    SystemClock_Config();                               //ϵͳʱ������
/*******************************************************************************************************/


/******************************************* ��ʼ������ģ�� ********************************************/
    InMemory_SystemParam_Check();                       //����ڲ��豸����    
    DataFilterParam_Init(&FilterParam, DATA_BUF_MAX);   //�˲�������ʼ��
    DataConvertParam_Init(&DataConvert_Param);          //PCap����ת��������ʼ��
#ifdef USING_ADC_TEMPER_SENSOR 
    Sensor_ADC_TemperParam_Init(&ADC_TemperParam);      //ADC�¶ȴ��������ʼ��
#endif // USING_ADC_TEMPER_SENSOR 
    PCap_Init();                                        //PCap��ʼ��    
#ifdef USING_ADC_TEMPER_SENSOR 
    Sensor_ADC_Init();                                  //ADC��ʼ��
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_DAC
    BSP_DAC_Init();                                     //DAC��ʼ��
#endif // USING_DAC
#ifdef USING_PWM
    TIM_DevObj_Init();                                  //��ʱ���豸�����ʼ��
    tim_dev[TIM3_INDEX].tim_pwm_init(0, 8121, 4061);    //��ʱ��3��ʼ��
    tim_dev[TIM3_INDEX].tim_pwm_start(TIM_CHANNEL_2);   //��ʱ��3 PWM ����
    tim_dev[TIM3_INDEX].tim_set_pwm(TIM_CHANNEL_2, 
                                    DataConvert_Out.PCap_PWM_Value, 
                                    DataConvert_Out.PCap_PWM_Value / 2);
#endif // USING_PWM
    BSP_IWDG_Init();                                    //�������Ź���ʼ��
    ModBus_Init(&ModBusBaseParam);                      //ModBus��ʼ��(�������ڳ�ʼ��)
/*******************************************************************************************************/
    
    while (1)
    {
/********************************************* ���Ź�ι�� **********************************************/
        BSP_IWDG_Refresh();
/*******************************************************************************************************/
        
        
/********************************************* ModBus���� **********************************************/
        //�������ݸ�����
        if(Sensor_USART_Get_RX_Updata_Flag())       
        {
            //ModBus��Ϣ֡����
            ModbusHandle(&ModBusBaseParam, &ModBus_Device);
            //����������ݸ��±�־
            Sensor_USART_Clear_RX_Updata_Flag();    
            //������ ModBus ��Ϣ���Զ��ϴ���ʱ�ӵ�ǰʱ�俪ʼ���¼�ʱ
            Auto_Up_Tick_Start = HAL_GetTick();       
        }                
        //����Զ��ϴ�ʱ�䲻Ϊ 0 ���Զ��ϴ���ʱʱ�䵽
        else if((ModBusBaseParam.AutoUpload != 0)   
            && ((ModBusBaseParam.AutoUpload * AUTOUPLOAD_CYCLE) <= (HAL_GetTick() - Auto_Up_Tick_Start)))
        {
            //Modbus֡�Զ��ϴ�
            ModbusAutoUpload(&ModBusBaseParam, &ModBus_Device);
            //��¼��ǰʱ��Ϊ��һ���Զ��ϴ����¼�ʱ
            Auto_Up_Tick_Start = HAL_GetTick();       
        }
/*******************************************************************************************************/
        
        
/*********************************** Pcap�ɼ����˲�������ת������� ************************************/
        //��ʱʱ�䵽�ɼ�����ֵ
        if(HAL_GetTick() - PCap_Tick_Start > PCAP_COLLECT_CYCLE)    
        {
            //��ȡPCap�������ݲ��жϷ���״̬,�ɹ�״̬������˲�������ת��
            if(Sensor_PCap_GetResult(RESULT_REG1_ADDR, 
                                    &ModBus_Device.PCap_DataConvert_Out->PCap_ResultValue, 
                                    1) == OP_SUCCESS)
            {
                //�����˲����ж��Ƿ�ɹ�
                if(Sensor_DataFilter(&FilterParam, 
                                    ModBus_Device.PCap_DataConvert_Out->PCap_ResultValue, 
                                    &FilterResult) == OP_SUCCESS)
                {
                    //��ֵת��
                    Sensor_PCap_DataConvert(&DataConvert_Param, 
                                            FilterResult, 
                                            &DataConvert_Out);
                    
/*********************************************** DAC���� ***********************************************/
#ifdef USING_DAC                                       
                    if(DataConvert_Param.CapDA_ClibEn != CLIB_ENABLE)
                    {
                        DataConvert_Out.PCap_DA_Value = Sensor_PCap_DA_Convert(&DataConvert_Param, 
                                                                            DataConvert_Out.LiquidHeightAD);  
                    }                        
                    //���δʹ��DA�궨 �� DAֵ�����˲���DAת��    
                    if(DataConvert_Out.PCap_DA_Value != BSP_Get_DAC(DA_CHANNEL_2))
                    {                                                                                          
                        //DAת��,ʹ��ͨ��2
                        BSP_DAC_Convert(DataConvert_Out.PCap_DA_Value, DA_CHANNEL_2); 
                    }                  
#endif // USING_DAC
/*******************************************************************************************************/
                    
/*********************************************** PWM���� ***********************************************/
#ifdef USING_PWM
                    if(DataConvert_Param.CapPWM_ClibEn != CLIB_ENABLE)
                    {
                        DataConvert_Out.PCap_PWM_Value = Sensor_PCap_PWM_Convert(&DataConvert_Param, 
                                                                                DataConvert_Out.LiquidHeight);
                    }                    
                    //���δʹ��PWM�궨 �� PWMֵ�����˲���PWMת��    
                    if(DataConvert_Out.PCap_PWM_Value != PCap_PWM_Period_Old)
                    { 
                        tim_dev[TIM3_INDEX].tim_set_pwm(TIM_CHANNEL_2, 
                                                        1000000000 / DataConvert_Out.PCap_PWM_Value, 
                                                        1000000000 / DataConvert_Out.PCap_PWM_Value / 2);
                        PCap_PWM_Period_Old = DataConvert_Out.PCap_PWM_Value;
                    }
#endif // USING_PWM
/*******************************************************************************************************/                    
                }
            }
/******************************************* PCap PT1000���� *******************************************/
            //��ȡPCap PT1000���ݲ��жϷ���״̬,�ɹ�״̬�������ֵת��
            if(Sensor_PCap_GetResult(RESULT_REG10_ADDR, 
                                    &temp_buf, 
                                    1) == OP_SUCCESS)
            {            
                temp_res_value = PCap_GetTemp((float)temp_buf / 2097152.0);
                DataConvert_Out.PCap_Temper_Value = (temp_res_value + 273.1 + 0.05) * 10;
            }
            //��¼��ǰʱ��Ϊ��һ�βɼ�����ֵ���¼�ʱ
            PCap_Tick_Start = HAL_GetTick();      
        }
/*******************************************************************************************************/    
        
/*******************************************************************************************************/
        
        
/*********************************************** ADC���� ***********************************************/
#ifdef USING_ADC_TEMPER_SENSOR
        //�ж�ADC�Ƿ񱻸���
        if(Sensor_ADC_Get_Updata_Flag() == UPDATA_OK)
        {
            //��ȡADת�������¶�ֵ
            ADC_TemperOut.TemperInAir = (Sensor_ADC_Get_TemperData() + 273.1 + 0.5) * 10;
            //���ADC���±�־,����ADCת��
            Sensor_ADC_Clean_Updata_Flag();
        }          
#endif // USING_ADC_TEMPER_SENSOR
/*******************************************************************************************************/        
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

