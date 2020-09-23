/**@file        Picocap_app.h
* @details      Picocap_app.c��ͷ�ļ�,������PCapӦ�õĺ궨��,������PCapӦ�õ�API����
* @author       ���
* @date         2020-04-30
* @version      V1.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/04/30  <td>1.0.0    <td>���    <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

#ifndef __PICOCAP_APP_H
#define __PICOCAP_APP_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "Picocap.h"


#define PCAP_COLLECT_CYCLE              100     ///< PCap�ɼ����ڣ���λ ms

#define DATA_BUF_MAX                    10      ///< PCap����ԭʼ����ֵ�Ļ�������

#define DATA_2ND_FILTER_BUF_MAX         10      ///< PCap�����˲���������

#define DATA_1ST_FILTER_BUF_MAX         96      ///< PCapһ���˲���������

#define PCAP_DAC_MIN_VALUE              0       ///< PCap DAC�����Сֵ

#define PCAP_DAC_MAX_VALUE              4095    ///< PCap DAC������ֵ

#define PCAP_ADC_MIN_VALUE              0       ///< PCap ADC�����Сֵ

#define PCAP_ADC_MAX_VALUE              65535   ///< PCap ADC������ֵ

#ifdef USING_PWM
#define PCAP_PWM_MIN_VALUE              2730    ///< PCap PWM�����Сֵ

#define PCAP_PWM_MAX_VALUE              5990    ///< PCap PWM������ֵ

#define PCAP_PWM_HIGH_MIN_VALUE         0       ///< PCap PWM�߶���Сֵ

#define PCAP_PWM_HIGH_MAX_VALUE         1135    ///< PCap PWM�߶����ֵ
#endif // USING_PWM

#define TempA	(3.9083E-3)  //0.0039083
#define TempB	(-5.775E-7)  //-0.0000005775
#define R2Value	(1000)

/** �����˲���Ҫ�Ĳ����ṹ */
typedef struct { 
    uint8_t FilterFactor;               ///< �˲�ϵ��  
    uint8_t FilterStart;                ///< �˲���ʼ��־
    uint8_t InputCountMax;              ///< �ۼ���������ֵ������ΪDATA_BUF_MAX    
    uint8_t FilterBufMax;               ///< �˲���󻺴�
    uint16_t FilterCycle;               ///< �˲�����
    uint32_t InputRangeMax;             ///< ���뷶Χ���ֵ
    uint32_t InputRangeMin;             ///< ���뷶Χ��Сֵ 
}DataFilterParam;

/** PCap AD�궨ֵ */
typedef struct {
    uint16_t CapADMin;                  ///< ����ADֵ���
    uint16_t CapADLow;                  ///< ����ADֵ�¿̶�
    uint16_t CapADHigh;                 ///< ����ADֵ�Ͽ̶�
    uint16_t CapADMax;                  ///< ����ADֵ������
}CapAD_CalibDef;

/** PCap DA�궨ֵ */
typedef struct {
    uint16_t CapDAMin;                  ///< ����DAֵ���
    uint16_t CapDALow;                  ///< ����DAֵ�¿̶�
    uint16_t CapDAHigh;                 ///< ����DAֵ�Ͽ̶�
    uint16_t CapDAMax;                  ///< ����DAֵ������    
}CapDA_CalibDef;

/** PCap PWM�궨ֵ */
typedef struct {
    uint16_t CapPWM0;                   ///< ����PWM 0 ��
    uint16_t CapPWM1;                   ///< ����PWM 1 ��
    uint16_t CapPWM2;                   ///< ����PWM 2 ��
    uint16_t CapPWM3;                   ///< ����PWM 3 ��
    uint16_t CapPWM4;                   ///< ����PWM 4 ��
    uint16_t CapPWM5;                   ///< ����PWM 5 ��
}CapPWM_CalibDef;

/** PCap PWM�߶ȱ궨ֵ */
typedef struct {
    uint16_t CapPWM_High0;              ///< ����PWM 0 ��߶�ֵ
    uint16_t CapPWM_High1;              ///< ����PWM 1 ��߶�ֵ
    uint16_t CapPWM_High2;              ///< ����PWM 2 ��߶�ֵ
    uint16_t CapPWM_High3;              ///< ����PWM 3 ��߶�ֵ
    uint16_t CapPWM_High4;              ///< ����PWM 4 ��߶�ֵ
    uint16_t CapPWM_High5;              ///< ����PWM 5 ��߶�ֵ    
}CapPWM_High_CalibDef;

/** PCap ���ݱ궨ֵ */
typedef struct {
    uint32_t CapMin;                    ///< �������
    uint32_t CapMax;                    ///< ����������
}Cap_CalibDef;
    
/** PCap������ת����Ҫ�Ĳ����ṹ */
typedef struct {
    uint8_t CompenEn;                   ///< ����ʹ��
    uint16_t HeightRange;               ///< �߶�����
    CapAD_CalibDef CapAD_Calib;         ///< PCap AD�궨ֵ
#ifdef USING_DAC
    CapDA_CalibDef CapDA_Calib;         ///< PCap DA�궨ֵ �Լ� �궨ʹ��
    uint8_t CapDA_ClibEn;               ///< ����DA�궨ʹ��
#endif // USING_DAC
#ifdef USING_PWM
    CapPWM_CalibDef CapPWM_Calib;       ///< PCap PWM�궨ֵ
    CapPWM_High_CalibDef CapPWM_High_Calib; ///< PCap PWM�߶ȱ궨ֵ
    uint8_t CapPWM_ClibEn;              ///< ����PWM�궨ʹ��
#endif // USING_PWM
    Cap_CalibDef Cap_Calib;             ///< PCap ���ݱ궨ֵ
    float Correct_K;                    ///< ��������ϵ��K
    float Correct_B;                    ///< ��������ϵ��B
}PCap_DataConvert_Param;

/** PCapת����������ݵĽṹ */
typedef struct {
    uint16_t LiquidHeightAD;            ///< Һλ�߶�ADֵ
    uint16_t LiquidHeight_Percentage;   ///< Һλ�߶Ȱٷֱ�
    uint32_t LiquidHeight;              ///< Һλ�߶�
    uint32_t PCap_ResultValue;          ///< PCapԭʼֵ
#ifdef USING_DAC
    uint16_t PCap_DA_Value;             ///< Pcap DAֵ
#endif // USING_DAC
#ifdef USING_PWM
    uint16_t PCap_PWM_Value;            ///< Pcap PWMֵ
#endif // USING_PWM
    uint16_t PCap_Temper_Value;         ///< Pcap �¶�ֵ
}PCap_DataConvert_Out_Param;

/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
#include <rtthread.h>

#define PCAP_DEVICE_NAME        "pcap"

struct rt_pcap_device_obj {
    struct rt_device            dev;
    DataFilterParam             DataFilter;
    PCap_DataConvert_Param      PCap_DataConvert;
    PCap_DataConvert_Out_Param  PCap_DataConvert_Out;
};
#endif // USING_RT_THREAD_OS


/**@brief       �л��˲�ˮƽ����
* @param[in]    FiltFactor : �˲�ϵ��;
* @param[in]    FilterParam : ָ���˲������ṹ��;
* @return       ����ִ�н��
* - None
* @note         �˲�ˮƽ��9�������ִӵ͵��߶�Ӧ�˲���ȴӵ͵���
*/
void SwitchCurFilter(uint8_t FiltFactor, DataFilterParam *FilterParam);

/**@brief       ��ȡPCapԭʼ�ɼ�ֵ
* @param[in]    reg_addr : ����Ĵ����ĵ�ַ;
* @param[out]   PCap_Result : ����PCap��������;
* @param[in]    Read_Cnt : ��ȡ�ĵ��ݸ���;
* @return       ����ִ�н��
* - OP_SUCCESS(�����ɹ�)
* - OP_FAILED(����ʧ��)
*/
uint8_t Sensor_PCap_GetResult(uint8_t reg_addr, uint32_t *PCap_Result, uint8_t Read_Cnt);

/**@brief       ���ݰ�ָ���˲����������˲�
* @param[in]    FilterParam : ָ���˲������ṹ��;
* @param[in]    InputValue : ���������;
* @param[out]   OutputValue : ������ݵ�ָ��;
* @return       ����ִ�н��
* - OP_SUCCESS(�����ɹ�)
* - OP_FAILED(����ʧ��)
* @note         ʹ�ñ�����ǰ,�Ƚ�FilterParam��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
uint8_t Sensor_DataFilter(DataFilterParam *FilterParam, uint32_t InputValue, uint32_t *OutputValue);

/**@brief       ������ֵת����Һλ�߶Ⱥ�ADֵ
* @param[in]    DataConvert_Param : ָ�����Ʋ����ṹ��;
* @param[in]    Cap_Value : ���������(����ֵ);
* @param[out]   DataConvert_Out : ������ݵĲ����ṹ��ָ��
* @return       ����ִ�н��
* - None
* @note         ʹ�ñ�����ǰ,�Ƚ�DataConvert_Param��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
void Sensor_PCap_DataConvert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_Value, 
                                PCap_DataConvert_Out_Param *DataConvert_Out);

#ifdef USING_DAC
/**@brief       �������ADֵת����DAֵ
* @param[in]    DataConvert_Param : ָ�����Ʋ����ṹ��;
* @param[in]    Cap_AD_Value : ���������(ADֵ);
* @return       ����ִ�н��
* - DAֵ
* @note         ʹ�ñ�����ǰ,�Ƚ�DataConvert_Param��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
uint16_t Sensor_PCap_DA_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_AD_Value);
#endif // USING_DAC

#ifdef USING_PWM
/**@brief       ���ݰ�ָ�����Ʋ�����������ת��
* @param[in]    DataConvert_Param : ָ�����Ʋ����ṹ��;
* @param[in]    LiquidHeight : ���������(Һλ�߶�ֵ);
* @return       ����ִ�н��
* - PWMֵ
* @note         ʹ�ñ�����ǰ,�Ƚ�DataConvert_Param��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
uint16_t Sensor_PCap_PWM_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t LiquidHeight);
#endif // USING_PWM

/**@brief       ��ȡ��Pcap���������������ο�����ı�ֵת������¶�ֵ
* @param[in]    R_ratio : ����������ο�����ı�ֵ
* @return       ����ִ�н��
* - �¶�ֵ; 
*/
float PCap_GetTemp(float R_ratio);

#ifdef __cplusplus
}
#endif
#endif // __PICOCAP_APP_H
