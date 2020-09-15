/**@file        In_Memory_app.c
* @brief        ��д�ڲ��洢����Ӧ��
* @details      �ڲ��洢���ж�ȡ��д��ϵͳ�ڲ�����
* @author       ���
* @date         2020-08-31
* @version      V1.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author  <th>Description
* <tr><td>2020/08/31  <td>1.0.0    <td>���  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

#include "In_Memory_app.h"


/** ϵͳ�������� [ ModBus���� + �˲�ϵ�� + PCap����ת������ + ADC�¶ȴ������ + ��ʼֵ��־ ]
������ */

const ModBusBaseParam_TypeDef   Default_ModBusBaseParam = {         //ModBus����Ļ��������ṹ
    65,                         ///< �豸��ַ
    USART_BAUDRATE_9600_CODE,   ///< ���ڲ����ʴ���
    USART_PARITY_NONE_CODE,     ///< ������żУ�����
    0,                          ///< �Զ��ϴ�����
    0,                          ///< ���ģʽ
    FREEZE_DISABLE,             ///< �豸����
    IN_MEMORY_WR_DISABLE,       ///< �ڲ��洢��дʹ��
    NULL,                       ///< ModBus����/���մ���ṹ��
#ifdef USING_RT_THREAD_OS
    NULL,                       ///< ���ڷ�����
#else
    0,
#endif // USING_RT_THREAD_OS
    
/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM
    0,                          ///< ���������־
    0,                          ///< �����ȴ�ʱ��
#endif // BOOT_PROGRAM
    /** ModBus�ص����� */
    NULL
};
const uint32_t Default_FilterFactor = 4;                             ///< �˲�ϵ��
const PCap_DataConvert_Param    Default_DataConvert_Param = {       //PCap������ת����Ҫ�Ĳ����ṹ��
    COMPENSATE_DISABLE,         ///< ������ʹ��
    1135,                       ///< �߶����� 1135 mm
    0,                          ///< ����ADֵ���
    20900,                      ///< ����ADֵ�¿̶�
    47000,                      ///< ����ADֵ�Ͽ̶�
    65535,                      ///< ����ADֵ����
#ifdef USING_DAC
    428,                        ///< ����DAֵ���
    0,                          ///< ����DAֵ�¿̶�
    0,                          ///< ����DAֵ�Ͽ̶�
    3824,                       ///< ����DAֵ������
    CLIB_DISABLE,               ///< ����DA�궨��ʹ��
#endif // USING_DAC
#ifdef USING_PWM
    5910,                       ///< ����PWM 0 ��
    5000,                       ///< ����PWM 1 ��
    4380,                       ///< ����PWM 2 ��
    3840,                       ///< ����PWM 3 ��
    3420,                       ///< ����PWM 4 ��
    2810,                       ///< ����PWM 5 ��
    0,                          ///< ����PWM 0 ��߶�ֵ
    335,                        ///< ����PWM 1 ��߶�ֵ
    535,                        ///< ����PWM 2 ��߶�ֵ
    735,                        ///< ����PWM 3 ��߶�ֵ
    935,                        ///< ����PWM 4 ��߶�ֵ
    1135,                       ///< ����PWM 5 ��߶�ֵ
    CLIB_DISABLE,               ///< ����PWM�궨ʹ��
#endif // USING_PWM
    1655630,                    ///< �������
    2426863,                    ///< ����������
    1.0,                        ///< ��������ϵ��K
    0.0,                        ///< ��������ϵ��B
};
#ifdef USING_ADC_TEMPER_SENSOR
const ADC_TemperParam_TypeDef   Default_ADC_TemperParam = {         //ADC�¶ȴ�����Ҫ�Ĳ����ṹ��
    1.0,                        ///< �¶�1����ϵ��K1
    0.0,                        ///< �¶�1����ϵ��B1
    1.0,                        ///< �¶�2����ϵ��K2
    0.0,                        ///< �¶�2����ϵ��B2
    0,                          ///< �¶�DAֵ���
    4095,                       ///< �¶�DAֵ������
    4095,                       ///< �¶�DAֵ����
};
#endif // USING_ADC_TEMPER_SENSOR
const uint8_t   System_Param_Flag = SYSTEMPARAM_IS_PROGRAMED;       ///< д���ʼֵ��־


/**@brief       �ڲ�Flashϵͳ�������,�����ֲ�һ�µĲ���,���½� ȱʡֵ д���ڲ�Flash
* @return       ����ִ�н��
* - None
*/
void InMemory_SystemParam_Check(void)
{
    uint16_t Cnt;
    uint8_t Check_Sta = 0;
    uint8_t Cur_Param_Bak1;

    for(Cnt = 0; Cnt < DEVICE_PARAM_LEN; Cnt++)
    {
        //����1
        Cur_Param_Bak1 = InMemory_Read_OneByte((SYSTEM_PARAM_BAK1_BASE_ADDRESS + Cnt));
        //�豸�����뱸��1��ͬ
        if(InMemory_Read_OneByte((SYSTEM_PARAM_BASE_ADDRESS + Cnt)) != Cur_Param_Bak1)
        {
            Check_Sta = 1;
        }
        //����1����
        if(Check_Sta)
        {
            break;
        }
    }

    if(InMemory_Read_OneByte(SYSPARAM_FLAG_BASE_ADDRESS) != SYSTEMPARAM_IS_PROGRAMED)
    {
        Check_Sta = 1;
    }
    if(Check_Sta == 0 && InMemory_Read_OneByte(SYSPARAM_FLAG_BAK1_BASE_ADDRESS) != SYSTEMPARAM_IS_PROGRAMED)
    {
        Check_Sta = 1;
    }
    //������������д��ȱʡϵͳ����
    if(Check_Sta)
    {
        //д����ϵͳ����ȱʡֵ
        //дModBus����
        InMemory_Write2T_MultiBytes(MODBUS_PARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_ModBusBaseParam, 
                                    MODBUS_PARAM_LEN);
        //д�˲�ϵ��
        InMemory_Write2T_MultiBytes(FILTERFACTOR_BASE_ADDRESS, 
                                    (uint8_t *)&Default_FilterFactor, 
                                    FILTERFACTOR_PARAM_LEN);
        //дPCap����ת������
        InMemory_Write2T_MultiBytes(PCAP_PARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_DataConvert_Param, 
                                    PCAP_PARAM_LEN);
#ifdef USING_ADC_TEMPER_SENSOR
        //дADC�¶ȴ������
        InMemory_Write2T_MultiBytes(ADC_TEMPERPARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_ADC_TemperParam, 
                                    ADC_TEMPER_PARAM_LEN);
#endif // USING_ADC_TEMPER_SENSOR
        //д��ʼֵ��־
        InMemory_Write2T_MultiBytes(SYSPARAM_FLAG_BASE_ADDRESS, 
                                    (uint8_t *)&System_Param_Flag, 
                                    sizeof(System_Param_Flag));
    }
}


/**@brief       ���ڲ��洢��ָ��λ��д1���ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    WrData : Ҫд�������;
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Write_OneByte(uint16_t RWAddr, uint8_t WrData)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Write_OneByte(RWAddr, WrData);
#elif defined(__IN_FLASH_H)
    return InFlash_Write_OneByte(RWAddr, WrData);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       ���ڲ��洢��ָ��λ�ö�1���ֽ�
* @param[in]    RWAddr : ����ʼ��ַ
* @return       ����ִ�н��
* - 1���ֽ�����
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Read_OneByte(uint16_t RWAddr)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Read_OneByte(RWAddr);
#elif defined(__IN_FLASH_H)
    return InFlash_Read_OneByte(RWAddr);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}
/**@brief       ���ڲ��洢��ָ��λ��д����ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    pWrbuf : Ҫд������ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Write_MultiBytes(uint16_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Write_MultiBytes(RWAddr, pWrbuf, Wrlen);
#elif defined(__IN_FLASH_H)
    return InFlash_Write_MultiBytes(RWAddr, pWrbuf, Wrlen);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       ���ڲ��洢��ָ��λ�ö�����ֽ�
* @param[in]    RWAddr : ����ʼ��ַ
* @param[in]    pWrbuf : Ҫ��ȡ�����ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
void InMemory_Read_MultiBytes(uint16_t RWAddr, uint8_t *pRdbuf, uint16_t Rdlen)
{
#if defined(__IN_EEPROM_H)
    InEEPROM_Read_MultiBytes(RWAddr, pRdbuf, Rdlen);
#elif defined(__IN_FLASH_H)
    InFlash_Read_MultiBytes(RWAddr, pRdbuf, Rdlen);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       ��STM32F072xx�ڲ�Flashָ��λ��д����ֽ��ұ���1��
* @param[in]    FlashAddr : д��ʼ��ַ
* @param[in]    pWrbuf : Ҫд������ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
*/
uint8_t InMemory_Write2T_MultiBytes(uint16_t FlashAddr, const uint8_t *pWrbuf, uint16_t Wrlen)
{
    //����״̬
    uint8_t Err;
    //д״̬
    uint8_t Wrsta;

    Err = OP_SUCCESS;
    Wrsta = OP_SUCCESS;

    //ϵͳ�����洢��
    Wrsta = InMemory_Write_MultiBytes(FlashAddr, pWrbuf, Wrlen);
    if(OP_SUCCESS != Wrsta)
    {
        Err = OP_FAILED;
    }
    //ϵͳ����������1
    Wrsta = InMemory_Write_MultiBytes(FlashAddr + SYSTEM_PARAM_BAK1_BASE_ADDRESS, pWrbuf, Wrlen);
    if(OP_SUCCESS != Wrsta)
    {
        Err = OP_FAILED;
    }

    return Err;
}

/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
int check_device_param(void)
{
    InMemory_SystemParam_Check();           // ����ڲ�ϵͳ����
    
    return RT_EOK;
}
INIT_BOARD_EXPORT(check_device_param);
#endif // USING_RT_THREAD_OS

