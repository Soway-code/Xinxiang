/**@file        Param_Init.c
* @brief        �����豸��ģ������в����ĳ�ʼ��
* @details      Ϊ�����豸��ģ����������в������г�ʼ������
* @author       ���
* @date         2020-09-22
* @version      V1.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/09/22  <td>1.0.0    <td>���    <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

#include "Param_Init.h"


/**@brief       ModBus������ʼ��
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��
* @return       ����ִ�н��
* - None
*/
void ModBus_Param_Init(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    InMemory_Read_MultiBytes(MODBUS_PARAM_BASE_ADDRESS, 
                            (uint8_t *)ModBusBaseParam, 
                            sizeof(ModBusBaseParam_TypeDef));               //��ModBus����
}

/**@brief       ��ʼ�������˲���Ҫ�Ĳ����ṹ
* @param[in]    Filterfactor_CountMax : �˲�ϵ�����ۼ���������ֵ,�˲�ϵ���ڸ�8λ,
* �ۼ���������ֵ�ڵ�8λ;
* @param[out]   FilterParam : �����˲���Ҫ�Ĳ����ṹָ��;
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax)
{        
    InMemory_Read_MultiBytes(FILTERFACTOR_BASE_ADDRESS, 
                            (uint8_t *)&FilterParam->FilterFactor, 
                            sizeof(FilterParam->FilterFactor));      //�˲�ϵ��
    SwitchCurFilter(FilterParam->FilterFactor, FilterParam);         //�л��˲�ˮƽ
    FilterParam->InputCountMax = Filterfactor_CountMax; 
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMax),   //CapMax��Ա��Ե�ַ
                            (uint8_t *)&FilterParam->InputRangeMax, 
                            sizeof(FilterParam->InputRangeMax));      //��������ֵ
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMin),   //CapMin��Ա��Ե�ַ
                            (uint8_t *)&FilterParam->InputRangeMin, 
                            sizeof(FilterParam->InputRangeMin));      //�������ֵ
}


/**@brief       ��ʼ������ת����Ҫ�Ĳ����ṹ
* @param[out]   DataConvert_Param : ����ת����Ҫ�Ĳ����ṹָ��;
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param)
{   
    InMemory_Read_MultiBytes(PCAP_PARAM_BASE_ADDRESS, 
                            (uint8_t *)DataConvert_Param, 
                            sizeof(PCap_DataConvert_Param));      //����ʹ�ܱ�־        
}

/**@brief       ��ʼ���¶�ת����Ҫ�Ĳ����ṹ
* @param[out]   ADC_TemperParam : �¶�ת����Ҫ�Ĳ����ṹָ��; 
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam)
{    
    InMemory_Read_MultiBytes(ADC_TEMPERPARAM_BASE_ADDRESS, 
                            (uint8_t *)ADC_TemperParam, 
                            sizeof(ADC_TemperParam_TypeDef));
    ADC_TemperParam->TempDARange = ADC_TemperParam->ADC_Temper_Calib.TempDAMax
                                    - ADC_TemperParam->ADC_Temper_Calib.TempDAMin;
}
