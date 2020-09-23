/**@file        Picocap_app.c
* @brief        PCapоƬӦ�ü������ݴ���
* @details      ��ȡPCapоƬ,�Զ��������ݽ����˲�,����ת��
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

#include "Picocap_app.h"
#include "VariaType.h"
#include "TypeConvert.h"
#include "math.h"


/**@brief       �л��˲�ˮƽ����
* @param[in]    FiltFactor : �˲�ϵ��;
* @param[in]    FilterParam : ָ���˲������ṹ��;
* @return       ����ִ�н��
* - None
* @note         �˲�ˮƽ��9�������ִӵ͵��߶�Ӧ�˲���ȴӵ͵���
*/
void SwitchCurFilter(uint8_t FiltFactor, DataFilterParam *FilterParam)
{  
    switch(FiltFactor)
    {
    case 0:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 1;
    break;

    case 1:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 12;
    break;

    case 2:
        FilterParam->FilterBufMax   = 2;
        FilterParam->FilterCycle    = 12;
    break;

    case 3:
        FilterParam->FilterBufMax   = 3;
        FilterParam->FilterCycle    = 12;
    break;

    case 4:
        FilterParam->FilterBufMax   = 4;
        FilterParam->FilterCycle    = 15;
    break;

    case 5:
        FilterParam->FilterBufMax   = 5;
        FilterParam->FilterCycle    = 24;
    break;

    case 6:
        FilterParam->FilterBufMax   = 6;
        FilterParam->FilterCycle    = 30;
    break;

    case 7:
        FilterParam->FilterBufMax   = 8;
        FilterParam->FilterCycle    = 30;
    break;

    case 8:
        FilterParam->FilterBufMax   = 8;
        FilterParam->FilterCycle    = 60;
    break;

    case 9:
        FilterParam->FilterBufMax   = 10;
        FilterParam->FilterCycle    = 96;
    break;

    default:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 12;
    break;
    }

    FilterParam->FilterStart = 0;
}

/**@brief       ��ȡPCapԭʼ�ɼ�ֵ
* @param[in]    reg_addr : ����Ĵ����ĵ�ַ;
* @param[out]   PCap_Result : ����PCap��������;
* @param[in]    Read_Cnt : ��ȡ����ĸ���;
* @return       ����ִ�н��
* - OP_SUCCESS(�����ɹ�)
* - OP_FAILED(����ʧ��)
*/
uint8_t Sensor_PCap_GetResult(uint8_t reg_addr, uint32_t *PCap_Result, uint8_t Read_Cnt)
{
    uint32_t PCap_Status;
    uint8_t Result = OP_FAILED;    
    uint8_t i;
    
    //��ȡPCap״̬
    PCap_Status = PCap_Res_Sta();
    if(PCap_Status & PCAP_RUNBIT)
    {
        for(i = 0; i < Read_Cnt; i++)
        {
            PCap_Result[i] = PCap_Res_Data(reg_addr + i);           
        }
        Result = OP_SUCCESS;
    }
    if(reg_addr <= RESULT_REG7_ADDR)
    {
        PCap_Measure();
    }
            
    return Result;
}

/**@brief       ���ݰ�ָ���˲����������˲�
* @param[in]    FilterParam : ָ���˲������ṹ��;
* @param[in]    InputValue : ���������;
* @param[out]   OutputValue : ������ݵ�ָ��;
* @return       ����ִ�н��
* - OP_SUCCESS(�����ɹ�)
* - OP_FAILED(����ʧ��)
* @note         ʹ�ñ�����ǰ,�Ƚ�FilterParam��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
uint8_t Sensor_DataFilter(DataFilterParam *FilterParam, uint32_t InputValue, uint32_t *OutputValue)
{
    static uint32_t Data_1st_FilterBuf[DATA_1ST_FILTER_BUF_MAX];        //һ���˲�����
    static uint32_t Data_1st_FilterBufBak[DATA_1ST_FILTER_BUF_MAX];     //һ���˲����汸��
    static uint32_t Data_2nd_FilterBuf[DATA_2ND_FILTER_BUF_MAX];        //�����˲�����
    static uint32_t Data_2nd_FilterBufBak[DATA_2ND_FILTER_BUF_MAX];     //�����˲����汸��
    static uint32_t DataResBuf[DATA_BUF_MAX];                           //����ԭʼ����ֵ�Ļ���
    static uint8_t DataResBuf_Count;
    uint8_t i;
    uint32_t Data_Avg1;
    uint32_t Data_Avg2;
    uint32_t Data_Avg3;
    uint64_t Data_FilterSum;                                            //�˲������ۼƺ�
    
    if(InputValue > FilterParam->InputRangeMax)
    {
        InputValue = FilterParam->InputRangeMax;
    }
    if(InputValue < FilterParam->InputRangeMin)
    {
        InputValue = FilterParam->InputRangeMin;
    }
    
    if(FilterParam->FilterStart == 0)
    {
        DataResBuf_Count = 0;
        for(i = 0; i < DATA_1ST_FILTER_BUF_MAX; i++)
        {
            Data_1st_FilterBuf[i] = InputValue;
        }
        for(i = 0; i < DATA_2ND_FILTER_BUF_MAX; i++)
        {
            Data_2nd_FilterBuf[i] = InputValue;
        }
        FilterParam->FilterStart = 1;
        
        return OP_FAILED;
    }
    
    DataResBuf[DataResBuf_Count++] = InputValue;
    if(DataResBuf_Count < DATA_BUF_MAX)
    {
        return OP_FAILED;
    }
    else
    {
        DataResBuf_Count = 0;
    }
    
    Data_Avg1 = GetDelExtremeAndAverage(DataResBuf, DATA_BUF_MAX, 2, 5);
    
    if(FilterParam->FilterCycle < 2)
    {
        *OutputValue = Data_Avg1;
        return OP_SUCCESS;
    }
    else
    {
        for(i = 0; i < FilterParam->FilterCycle - 1; i++)
        {
            Data_1st_FilterBuf[i] = Data_1st_FilterBuf[i + 1];
        }
        Data_1st_FilterBuf[i] = Data_Avg1;
        
        if(FilterParam->FilterCycle >= 3)
        {
            for(i = 0; i < FilterParam->FilterCycle; i++)
            {
                Data_1st_FilterBufBak[i] = Data_1st_FilterBuf[i];
            }
            Data_Avg2 = GetDelExtremeAndAverage(Data_1st_FilterBufBak, 
                                            FilterParam->FilterCycle, 
                                            FilterParam->FilterCycle / 3, 
                                            FilterParam->FilterCycle / 3);
        }
        else
        {
            Data_FilterSum = 0;                                                               
            for(i = 0; i < FilterParam->FilterCycle; i++)
            {
                Data_FilterSum += Data_1st_FilterBuf[i];
            }           
            Data_Avg2 = Data_FilterSum / FilterParam->FilterCycle;        //�����˲������ƽ��ֵ
        }
        
        if(FilterParam->FilterBufMax < 2)            
        {
            *OutputValue = Data_Avg2;
            return OP_SUCCESS;
        }
        else
        {
            for(i = 0; i < FilterParam->FilterBufMax - 1; i++)
            {
                Data_2nd_FilterBuf[i] = Data_2nd_FilterBuf[i + 1];
            }
            Data_2nd_FilterBuf[i] = Data_Avg2;
            
            if(FilterParam->FilterBufMax >= 3)
            {
                for(i = 0; i < FilterParam->FilterBufMax; i++)
                {
                    Data_2nd_FilterBufBak[i] = Data_2nd_FilterBuf[i];
                }
                Data_Avg3 = GetDelExtremeAndAverage(Data_2nd_FilterBufBak, 
                                                FilterParam->FilterBufMax, 
                                                FilterParam->FilterBufMax / 3, 
                                                FilterParam->FilterBufMax / 3);
            }
            else
            {
                Data_FilterSum = 0;                                                               
                for(i = 0; i < FilterParam->FilterBufMax; i++)
                {
                    Data_FilterSum += Data_2nd_FilterBuf[i];
                }           
                Data_Avg3 = Data_FilterSum / FilterParam->FilterBufMax;        //�����˲������ƽ��ֵ
            }
            
            *OutputValue = Data_Avg3;
            return OP_SUCCESS;
        }       
    }    
}

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
                                PCap_DataConvert_Out_Param *DataConvert_Out)
{
    float PCap_Result;
    float LiquidHeightRate;

    //����ʹ��,����K,Bֵ
    if(DataConvert_Param->CompenEn == COMPENSATE_ENABLE)        
    {
        PCap_Result = Cap_Value * DataConvert_Param->Correct_K 
                                + DataConvert_Param->Correct_B;
    }
    else if(DataConvert_Param->CompenEn == COMPENSATE_DISABLE)        
    {
        PCap_Result = Cap_Value;
    }
    //������ݱ���
    LiquidHeightRate = (float)(PCap_Result - DataConvert_Param->Cap_Calib.CapMin) 
                                / (float)(DataConvert_Param->Cap_Calib.CapMax - DataConvert_Param->Cap_Calib.CapMin);
    
    if(1.0 < LiquidHeightRate)
    {
        LiquidHeightRate = 1.0;
    }
    else if (0.0 > LiquidHeightRate)
    {
        LiquidHeightRate = 0.0;
    }
    //ת����0--65535��Χ�ڵ�ADֵ
    DataConvert_Out->LiquidHeightAD = (uint16_t)(LiquidHeightRate * PCAP_ADC_MAX_VALUE);
    //ת����Һλ�߶�
    DataConvert_Out->LiquidHeight = (uint32_t)(LiquidHeightRate 
                                            * DataConvert_Param->HeightRange);      
    DataConvert_Out->LiquidHeight_Percentage = (uint16_t)(LiquidHeightRate * 1000);
}

#ifdef USING_DAC
/**@brief       �������ADֵת����DAֵ
* @param[in]    DataConvert_Param : ָ�����Ʋ����ṹ��;
* @param[in]    Cap_AD_Value : ���������(ADֵ);
* @return       ����ִ�н��
* - DAֵ
* @note         ʹ�ñ�����ǰ,�Ƚ�DataConvert_Param��ʼ��(��������ÿһ����Ա�����ʵ�ֵ),����������ʹ��
*/
uint16_t Sensor_PCap_DA_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_AD_Value)
{
    uint16_t Output_DA_Value;
    float PCap_Result;
    float Rate;
    
    //����ʹ��,����K,Bֵ
    if(DataConvert_Param->CompenEn == COMPENSATE_ENABLE)        
    {
        PCap_Result = Cap_AD_Value * DataConvert_Param->Correct_K 
                                + DataConvert_Param->Correct_B;
    }
    else if(DataConvert_Param->CompenEn == COMPENSATE_DISABLE)        
    {
        PCap_Result = Cap_AD_Value;
    }
    
    Rate = (float)(PCap_Result - DataConvert_Param->CapAD_Calib.CapADMin) 
                    / (float)(DataConvert_Param->CapAD_Calib.CapADMax - DataConvert_Param->CapAD_Calib.CapADMin);
    
    if(1.0 < Rate)
    {
        Rate = 1.0;
    }
    else if (0.0 > Rate)
    {
        Rate = 0.0;
    }
    
    //���û��4���ֵ����ֱ�Ӽ���DA���ֵ
    if((0x0000 == DataConvert_Param->CapDA_Calib.CapDALow) || (0x0000 == DataConvert_Param->CapDA_Calib.CapDAHigh)
        || (0x0000 == DataConvert_Param->CapAD_Calib.CapADLow) || (0x0000 == DataConvert_Param->CapAD_Calib.CapADHigh))
    {
        Output_DA_Value = (uint16_t)(Rate * (DataConvert_Param->CapDA_Calib.CapDAMax 
                                            - DataConvert_Param->CapDA_Calib.CapDAMin)) 
                                            + DataConvert_Param->CapDA_Calib.CapDAMin;
    }
    else
    {                 
        //ADֵ�����AD����
        if(Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADMin)  
        {
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMin;
        }
        //ADֵ�����AD���¿̶�AD֮��
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADLow) 
            && (Cap_AD_Value) >= DataConvert_Param->CapAD_Calib.CapADMin)
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADMin)
                / (float)(DataConvert_Param->CapAD_Calib.CapADLow - DataConvert_Param->CapAD_Calib.CapADMin);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDALow - DataConvert_Param->CapDA_Calib.CapDAMin;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                            + DataConvert_Param->CapDA_Calib.CapDAMin;
        }
        //ADֵ���¿̶�ADֵ���Ͽ̶�ADֵ֮��
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADHigh) 
            && (Cap_AD_Value >= DataConvert_Param->CapAD_Calib.CapADLow))
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADLow) 
                / (float)(DataConvert_Param->CapAD_Calib.CapADHigh - DataConvert_Param->CapAD_Calib.CapADLow);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAHigh - DataConvert_Param->CapDA_Calib.CapDALow;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                            + DataConvert_Param->CapDA_Calib.CapDALow;
        }
        //ADֵ���Ͽ̶�ADֵ��������ADֵ֮��
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADMax) 
            && (Cap_AD_Value >= DataConvert_Param->CapAD_Calib.CapADHigh))
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADHigh) 
                / (float)(DataConvert_Param->CapAD_Calib.CapADMax - DataConvert_Param->CapAD_Calib.CapADHigh);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMax - DataConvert_Param->CapDA_Calib.CapDAHigh;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                                    + DataConvert_Param->CapDA_Calib.CapDAHigh;
        }                                            
        //ADֵ������������
        else                                                       
        {
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMax;
        }
    } 
    
    return Output_DA_Value;
}
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
                                uint32_t LiquidHeight)
{
    uint16_t PCapPWM_OutValue;
    float Rate;
    
        //Һλ�߶� < 0��߶�
    if(LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)
    {
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0;                                        
    }
    //Һλ�߶��� 0�� ~ 1��߶� ֮��
    if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High0 
        && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High1)
    {
        Rate = (float)((float)LiquidHeight / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0 - DataConvert_Param->CapPWM_Calib.CapPWM1;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0 - (uint16_t)(Rate * PCapPWM_OutValue);
                                        
    }
    //Һλ�߶��� 1�� ~ 2��߶� ֮��
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High2)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High1));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM1 - DataConvert_Param->CapPWM_Calib.CapPWM2;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM1 - (uint16_t)(Rate * PCapPWM_OutValue);
    }
    //Һλ�߶��� 2�� ~ 3��߶� ֮��
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High3)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High2));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM2 - DataConvert_Param->CapPWM_Calib.CapPWM3;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM2 - (uint16_t)(Rate * PCapPWM_OutValue);
    }   
    //Һλ�߶��� 3�� ~ 4��߶� ֮��
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High4)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High4 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High3));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM3 - DataConvert_Param->CapPWM_Calib.CapPWM4;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM3 - (uint16_t)(Rate * PCapPWM_OutValue);
    } 
    //Һλ�߶��� 4�� ~ 5��߶� ֮��
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High4
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High5)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High4 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High5 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High4));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM4 - DataConvert_Param->CapPWM_Calib.CapPWM5;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM4 - (uint16_t)(Rate * PCapPWM_OutValue);
    } 
    //Һλ�߶� >= 5��߶�
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High5)
    {
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM5;
    } 
    
    return PCapPWM_OutValue;
}
#endif // USING_PWM

/**@brief       ��ȡ��Pcap���������������ο�����ı�ֵת������¶�ֵ
* @param[in]    R_ratio : ����������ο�����ı�ֵ
* @return       ����ִ�н��
* - �¶�ֵ; 
*/
float PCap_GetTemp(float R_ratio)
{
	return (-1 * TempA + sqrt(TempA * TempA - 4 * TempB * (1 - R_ratio))) / (2 * TempB);
}

/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
#include <board.h>

static struct rt_pcap_device_obj pcap_device_obj;       //Pcap�豸����

/**@brief       Pcap�豸��ʼ��,��������rt_device_registerע���,��
* rt_device_init��rt_device_open����
* @param[in]    dev : �豸���
* @return       ����ִ�н��
* - RT_EOK : �豸��ʼ���ɹ�; 
*/
static rt_err_t pcap_init(rt_device_t dev)
{    
    rt_thread_mdelay(1000);
    PCap_Init();
    
    return RT_EOK;
}

/**@brief       Pcap�豸��,��������rt_device_registerע���,��
* rt_device_open����
* @param[in]    dev : �豸���
* @param[in]    oflag : �豸����ģʽ��־
* @return       ����ִ�н��
* - RT_EOK : �豸��ʼ���ɹ�; 
* @note         ���ﲻʹ���豸��,��ֻ����һ��RT_EOK
*/
static rt_err_t  pcap_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/**@brief       Pcap�豸�ر�,��������rt_device_registerע���,��
* rt_device_close����
* @param[in]    dev : �豸���
* @return       ����ִ�н��
* - RT_EOK:�豸�رճɹ�; 
* @note         ���ﲻʹ���豸�ر�,��ֻ����һ��RT_EOK
*/
static rt_err_t  pcap_close(rt_device_t dev)
{
    return RT_EOK;
}

/**@brief       Pcap�豸������,��������rt_device_registerע���,��
* rt_device_read����
* @param[in]    dev : �豸���
* @param[in]    pos : ��ȡ����ƫ����;
* @param[out]   buffer : �ڴ滺����ָ�룬��ȡ�����ݽ��ᱻ�����ڻ�������;
* @param[in]    size : ��ȡ���ݵĴ�С
* @return       ����ִ�н��
* - size : �������ݵ�ʵ�ʴ�С��������ַ��豸�����ش�С���ֽ�Ϊ
* ��λ������ǿ��豸�����صĴ�С�Կ�Ϊ��λ;
* - 0 : ��Ҫ��ȡ��ǰ�̵߳� errno ���жϴ���״̬
*/
static rt_size_t pcap_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if(Sensor_PCap_GetResult(pos, buffer, size) != OP_SUCCESS)
    {
        return 0;
    }    
    
    return size;
}

/**@brief       Pcap�豸����,��������rt_device_registerע���,��
* rt_device_control����
* @param[in]    dev : �豸���
* @param[in]    cmd : ��������֣��������ͨ�����豸�����������;
* @param[in]    arg : ���ƵĲ���
* @return       ����ִ�н��
* - RT_EOK:����ִ�гɹ�;
* @note         ���ﲻʹ���豸����,��ֻ����һ��RT_EOK
*/
static rt_err_t  pcap_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}


/**@brief       cap�豸������ʼ��,ע��Pcap�豸
* @return       ����ִ�н��
* - int����ֵ(RT_EOK)
* @note         ������ʹ��RT-Thread���Զ���ʼ�����INIT_DEVICE_EXPORT
* ����ִ��,ϵͳ��λ���Զ���ʼ��
*/
static int pcap_device_init(void)
{
    pcap_device_obj.dev.type     = RT_Device_Class_Miscellaneous;
    pcap_device_obj.dev.init     = pcap_init;
    pcap_device_obj.dev.open     = pcap_open;
    pcap_device_obj.dev.close    = pcap_close;
    pcap_device_obj.dev.read     = pcap_read;
    pcap_device_obj.dev.write    = RT_NULL;
    pcap_device_obj.dev.control  = pcap_control;    
    
    pcap_device_obj.dev.user_data = &pcap_device_obj;
    
    DataFilterParam_Init(&pcap_device_obj.DataFilter, DATA_BUF_MAX);    // �˲�������ʼ��
    DataConvertParam_Init(&pcap_device_obj.PCap_DataConvert);           // PCap����ת��������ʼ��
    
    rt_device_register(&pcap_device_obj.dev, PCAP_DEVICE_NAME,          // ע��Pcap�豸
                        RT_DEVICE_FLAG_RDWR
                        | RT_DEVICE_FLAG_STANDALONE);
    
    return RT_EOK;
}
INIT_DEVICE_EXPORT(pcap_device_init);

#endif // USING_RT_THREAD_OS

