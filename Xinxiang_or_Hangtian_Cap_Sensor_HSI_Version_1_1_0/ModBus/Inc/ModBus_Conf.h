/**@file        Modbus_Conf.h
* @details      Modbus_Conf.c��ͷ�ļ�,�����˵��ݴ������궨��API����,������
* �豸�����ṹ��
* @author       ׯ��Ⱥ
* @date         2020-07-20
* @version      V2.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author  <th>Maintainer  <th>Description
* <tr><td>2020/07/20  <td>2.0.0    <td>ׯ��Ⱥ  <td>���      <td>����ӵĳ������(ָ��ִ�в���)
* </table>
*
**********************************************************************************
*/

#ifndef __MODBUS_CONF_H
#define __MODBUS_CONF_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "Picocap_app.h"
#include "adc_app.h"
#include "dac_bsp.h"
#include "In_Memory_app.h"
/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifndef USING_RT_THREAD_OS
#include "tim_bsp.h"
#endif // USING_RT_THREAD_OS


/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
#include "rtconfig.h"

#if defined(APP_USING_MODBUS_RTU)
#define USING_MODBUS_RTU            ///< ʹ��ModBus RTUЭ��
#elif defined(APP_USING_MODBUS_ASCII)
#define USING_MODBUS_ASCII          ///< ʹ��ModBus ASCIIЭ��
#endif // defined(APP_USING_MODBUS_RTU) or defined(APP_USING_MODBUS_ASCII)

#if defined(APP_SUBCODE_IS_DEVADDR)
#define SUBCODE_IS_DEVADDR          ///< ����ModBus����Ϊ�豸��ַ����������Ĭ��Ϊ0
#endif // defined(APP_SUBCODE_IS_DEVADDR)

#else

//#define USING_MODBUS_RTU            ///< ʹ��ModBus RTUЭ��, ����Ҫ�� usart_app.h �ﶨ��USING_UART_TIMEOUT
#define USING_MODBUS_ASCII          ///< ʹ��ModBus ASCIIЭ��, ����Ҫ�� usart_app.h �ﶨ��USING_CHARMATCH
#define SUBCODE_IS_DEVADDR          ///< ����ModBus����Ϊ�豸��ַ����������Ĭ��Ϊ0(�� DEFAULT_SUBCODE)

#endif // USING_RT_THREAD_OS

#define AUTOUPLOAD_CYCLE                1000            ///< �Զ��ϴ�����,��λ ms

#ifndef RECEIVE_SIZE
#define RECEIVE_SIZE                    128             ///< ���ջ���Ĵ�С
#endif // RECEIVE_SIZE

#ifndef SEND_SIZE
#define SEND_SIZE                       128             ///< ���ͻ����С
#endif // SEND_SIZE

#define DAC_VALUE_MAX                   4095        ///< DAC���ֵ

#ifndef SUBCODE_IS_DEVADDR
#define DEFAULT_SUBCODE                 0
#endif  // SUBCODE_IS_DEVADDR

/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM

#define RESPONSE_ERR_NONE   0     //��Ӧ�ɹ�
#define RESPONSE_REC_ERR    1     //���մ���
#define RESPONSE_LRC_ERR    2     //У�������

#define ERASE_FLAG          0x0C
#define ERASE_FLAG_NONE     0xFF

#define UPGRADE_FLAG        0x0C
#define UPGRADE_FLAG_NONE   0xFF

#endif // BOOT_PROGRAM


#define ADDR_ERASEFLAG      0x17FE
#define ADDR_UPGRADEFLAG    0x17FF


#define CALIB_CAPMIN_FLAG               0x01                ///< �궨��������־
#define CALIB_CAPMAX_FLAG               0x10                ///< �궨�������ȱ�־
#define CALIB_CAPEOC_FLAG               0x11                ///< �����ݽ�����־

#define CALIB_CAPADMIN_FLAG             0x01                ///< �궨����AD����־
#define CALIB_CAPADLOW_FLAG             0x02                ///< �궨����AD�¿̶ȱ�־
#define CALIB_CAPADHIH_FLAG             0x10                ///< �궨����AD�Ͽ̶ȱ�־
#define CALIB_CAPADMAX_FLAG             0x20                ///< �궨����AD���ȱ�־
#define CALIB_CAPADEOC_FLAG             0x33                ///< �궨����AD������־

#define CALIB_CAPDAMIN_FLAG             0x01                ///< �궨����DA����־
#define CALIB_CAPDALOW_FLAG             0x02                ///< �궨����DA�¿̶ȱ�־
#define CALIB_CAPDAHIGH_FLAG            0x10                ///< �궨����DA�Ͽ̶ȱ�־
#define CALIB_CAPDAMAX_FLAG             0x20                ///< �궨����DA���ȱ�־
#define CALIB_CAPDAEOC_FLAG             0x33                ///< �궨����DA������־

#define CALIB_TEMPDAMIN_FLAG            0x01                ///< �궨�¶�DA����־
#define CALIB_TEMPDAMAX_FLAG            0x10                ///< �궨�¶�DA���ȱ�־
#define CALIB_TEMPDAEOC_FLAG            0x11                ///< �궨�¶�DA������־

#ifdef USING_PWM

#define CALIB_CAPPWM0_FLAG              0x01                ///< �궨���� PWM 0���־
#define CALIB_CAPPWM1_FLAG              0x02                ///< �궨���� PWM 1���־
#define CALIB_CAPPWM2_FLAG              0x04                ///< �궨���� PWM 2���־
#define CALIB_CAPPWM3_FLAG              0x08                ///< �궨���� PWM 3���־
#define CALIB_CAPPWM4_FLAG              0x10                ///< �궨���� PWM 4���־
#define CALIB_CAPPWM5_FLAG              0x20                ///< �궨���� PWM 5���־
#define CALIB_CAPPWMEOC_FLAG            0x3F                ///< �궨���� PWM ������־

#define CALIB_CAPPWM_HIGH0_FLAG         0x01                ///< �궨���� PWM 0��߶ȱ�־
#define CALIB_CAPPWM_HIGH1_FLAG         0x02                ///< �궨���� PWM 1��߶ȱ�־
#define CALIB_CAPPWM_HIGH2_FLAG         0x04                ///< �궨���� PWM 2��߶ȱ�־
#define CALIB_CAPPWM_HIGH3_FLAG         0x08                ///< �궨���� PWM 3��߶ȱ�־
#define CALIB_CAPPWM_HIGH4_FLAG         0x10                ///< �궨���� PWM 4��߶ȱ�־
#define CALIB_CAPPWM_HIGH5_FLAG         0x20                ///< �궨���� PWM 5��߶ȱ�־
#define CALIB_CAPPWM_HIGH_EOC_FLAG      0x3F                ///< �궨���� PWM �߶Ƚ�����־

#endif // USING_PWM


/** ModBus�����豸�Ĳ����ṹ,�ɸ��ݲ�ͬ�Ĳ�Ʒ�����ɾ����Ա */
typedef struct {
    DataFilterParam *DataFilter;
    PCap_DataConvert_Param *PCap_DataConvert;
    PCap_DataConvert_Out_Param *PCap_DataConvert_Out; 
    ADC_TemperParam_TypeDef *ADC_TemperParam;
    ADC_TemperOut_TypeDef   *ADC_TemperOut;
}ModBus_Device_Param;


/**@brief       ���ݱ궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_Cap_Calibration(void *arg);

/**@brief       ����ADֵ�궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_CapAD_Calibration(void *arg);

/**@brief       ����DAֵ�궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_CapDAOut_Calibration(void *arg);

/**@brief       �¶�DAֵ�궨����
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_TempDAOut_Calibration(void *arg);

#ifdef USING_PWM
/**@brief       ����PWMֵ�궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_CapPWMOut_Calibration(void *arg);


#endif // USING_PWM

#ifdef __cplusplus
}
#endif
#endif // __MODBUS_CONF_H
