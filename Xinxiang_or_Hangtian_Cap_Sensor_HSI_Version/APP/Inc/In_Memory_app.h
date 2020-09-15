/**@file        In_Memory_app.h
* @details      In_Memory_app.c��ͷ�ļ�,�������ڲ��洢��Ӧ�õ�API����
* @author       ���
* @date         2020-08-31
* @version      V1.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Maintainer  <th>Description
* <tr><td>2020/08/31  <td>1.0.0    <td>���      <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/
#ifndef __IN_MEMORY_APP_H
#define __IN_MEMORY_APP_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

#if defined(STM32F0)
#include "In_Flash.h"
#elif defined(STM32L0)
#include "In_EEPROM.h"
#endif // defined(STM32F0) or defined(STM32L0)
#include "TypeConvert.h"
#include "adc_app.h"
#include "Picocap_app.h"
#include "ModBus.h"


/*************************** ϵͳ�������ڲ��洢���е����ڵ�ַ ***************************/
/** ModBus���� */
#define MODBUS_PARAM_BASE_ADDRESS       ((uint32_t)0)                                              
/** �˲�ϵ�� */
#define FILTERFACTOR_BASE_ADDRESS       ((uint32_t)(MODBUS_PARAM_BASE_ADDRESS + sizeof(ModBusBaseParam_TypeDef)))      
/** PCap����������Ϊ��4�ֽڶ��룬���� + sizeof(void*) */
#define PCAP_PARAM_BASE_ADDRESS         ((uint32_t)(FILTERFACTOR_BASE_ADDRESS + sizeof(void*)))      
/** ADC�¶�ת������ */
#define ADC_TEMPERPARAM_BASE_ADDRESS    ((uint32_t)(PCAP_PARAM_BASE_ADDRESS + sizeof(PCap_DataConvert_Param)))      
/** ϵͳд���ʼֵ��־ */
#define SYSPARAM_FLAG_BASE_ADDRESS      ((uint32_t)(ADC_TEMPERPARAM_BASE_ADDRESS + sizeof(ADC_TemperParam_TypeDef)))   

#define MODBUS_PARAM_BAK1_BASE_ADDRESS      ((uint32_t)(MODBUS_PARAM_BASE_ADDRESS + 0x800))
#define FILTERFACTOR_BAK1_BASE_ADDRESS      ((uint32_t)(FILTERFACTOR_BASE_ADDRESS + 0x800))
#define PCAP_PARAM_BAK1_BASE_ADDRESS        ((uint32_t)(PCAP_PARAM_BASE_ADDRESS + 0x800))
#define ADC_TEMPERPARAM_BAK1_BASE_ADDRESS   ((uint32_t)(ADC_TEMPERPARAM_BASE_ADDRESS + 0x800))
#define SYSPARAM_FLAG_BAK1_BASE_ADDRESS     ((uint32_t)(SYSPARAM_FLAG_BASE_ADDRESS + 0x800))

#define MODBUS_PARAM_BAK2_BASE_ADDRESS      ((uint32_t)(MODBUS_PARAM_BAK1_BASE_ADDRESS + 0x800))
#define FILTERFACTOR_BAK2_BASE_ADDRESS      ((uint32_t)(FILTERFACTOR_BAK1_BASE_ADDRESS + 0x800))
#define PCAP_PARAM_BAK2_BASE_ADDRESS        ((uint32_t)(PCAP_PARAM_BAK1_BASE_ADDRESS + 0x800))
#define ADC_TEMPERPARAM_BAK2_BASE_ADDRESS   ((uint32_t)(ADC_TEMPERPARAM_BAK1_BASE_ADDRESS + 0x800))
#define SYSPARAM_FLAG_BAK2_BASE_ADDRESS     ((uint32_t)(SYSPARAM_FLAG_BAK1_BASE_ADDRESS + 0x800))

/** ϵͳ��������ַ����ʼ��ַ�� */
#define SYSTEM_PARAM_BASE_ADDRESS           MODBUS_PARAM_BASE_ADDRESS
#define SYSTEM_PARAM_BAK1_BASE_ADDRESS      MODBUS_PARAM_BAK1_BASE_ADDRESS
#define SYSTEM_PARAM_BAK2_BASE_ADDRESS      MODBUS_PARAM_BAK2_BASE_ADDRESS

/***************************** �豸����ռ�ô洢�ռ�ĳ��� ****************************/                                                                      
#define MODBUS_PARAM_LEN                sizeof(ModBusBaseParam_TypeDef)         ///< ���������ܳ���   
#define FILTERFACTOR_PARAM_LEN          sizeof(void*)
#define PCAP_PARAM_LEN                  sizeof(PCap_DataConvert_Param)
#ifdef USING_ADC_TEMPER_SENSOR
#define ADC_TEMPER_PARAM_LEN            sizeof(ADC_TemperParam_TypeDef)
#else
#define ADC_TEMPER_PARAM_LEN            0
#endif // #ifdef USING_ADC_TEMPER_SENSOR
/** �豸�����ܳ��ȣ�1�ֽ��˲�ϵ�� + PCap����ת������ + ADC�¶ȴ������ + 1�ֽڳ�ʼֵ��־ */
#define DEVICE_PARAM_LEN                (FILTERFACTOR_PARAM_LEN + \
                                        PCAP_PARAM_LEN + \
                                        ADC_TEMPER_PARAM_LEN + \
                                        1) 
#define SYSTEM_PARAM_LEN                (MODBUS_PARAM_LEN + DEVICE_PARAM_LEN)   ///< ϵͳ�����ܳ���   

/********************************** �ڲ��洢��д���־ֵ *********************************/   
#define SYSTEMPARAM_IS_PROGRAMED        0xAA            ///< д���ʼֵ��־


/********************************** �ڲ��洢�������궨�� *********************************/   
#if defined(__IN_FLASH_H)
#define IN_MEMORY_WR_ENABLE             IN_FLASH_WR_ENABLE
#define IN_MEMORY_WR_DISABLE            IN_FLASH_WR_DISABLE
#define IN_MEMORY_ERR_MAX               WRITE_FLASH_ERR_MAX
#elif defined(__IN_EEPROM_H)
#define IN_MEMORY_WR_ENABLE             IN_EEPROM_WR_ENABLE
#define IN_MEMORY_WR_DISABLE            IN_EEPROM_WR_DISABLE
#define IN_MEMORY_ERR_MAX               WRITE_EEPROM_ERR_MAX
#endif // defined(__IN_FLASH_H) or defined(__IN_EEPROM_H)



/**@brief       �ڲ�Flashϵͳ�������,�����ֲ�һ�µĲ���,���½� ȱʡֵ д���ڲ�Flash
* @return       ����ִ�н��
* - None
*/
void InMemory_SystemParam_Check(void);

/**@brief       ���ڲ��洢��ָ��λ��д1���ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    WrData : Ҫд�������;
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Write_OneByte(uint16_t RWAddr, uint8_t WrData);

/**@brief       ���ڲ��洢��ָ��λ�ö�1���ֽ�
* @param[in]    RWAddr : ����ʼ��ַ
* @return       ����ִ�н��
* - 1���ֽ�����
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Read_OneByte(uint16_t RWAddr);

/**@brief       ���ڲ��洢��ָ��λ��д����ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    pWrbuf : Ҫд������ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
uint8_t InMemory_Write_MultiBytes(uint16_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen);

/**@brief       ���ڲ��洢��ָ��λ�ö�����ֽ�
* @param[in]    RWAddr : ����ʼ��ַ
* @param[in]    pWrbuf : Ҫ��ȡ�����ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - None
* @note         ������ͨ�������ڲ�Flash��EEPROM����APIʵ�ֵ�
*/
void InMemory_Read_MultiBytes(uint16_t RWAddr, uint8_t *pRdbuf, uint16_t Rdlen);

/**@brief       ��STM32F072xx�ڲ�Flashָ��λ��д����ֽ��ұ���1��
* @param[in]    FlashAddr : д��ʼ��ַ
* @param[in]    pWrbuf : Ҫд������ݻ���ָ��;
* @param[in]    Wrlen : ���ݳ���
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
*/
uint8_t InMemory_Write2T_MultiBytes(uint16_t FlashAddr, const uint8_t *pWrbuf, uint16_t Wrlen);

#ifdef __cplusplus
}
#endif
#endif // __IN_MEMORY_APP_H
