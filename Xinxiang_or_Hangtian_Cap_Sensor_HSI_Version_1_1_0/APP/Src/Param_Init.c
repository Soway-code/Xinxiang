/**@file        Param_Init.c
* @brief        各个设备或模块的运行参数的初始化
* @details      为所有设备或模块所需的运行参数进行初始化操作
* @author       杨春林
* @date         2020-09-22
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/09/22  <td>1.0.0    <td>杨春林    <td>创建初始版本
* </table>
*
**********************************************************************************
*/

#include "Param_Init.h"


/**@brief       ModBus参数初始化
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体
* @return       函数执行结果
* - None
*/
void ModBus_Param_Init(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    InMemory_Read_MultiBytes(MODBUS_PARAM_BASE_ADDRESS, 
                            (uint8_t *)ModBusBaseParam, 
                            sizeof(ModBusBaseParam_TypeDef));               //读ModBus参数
}

/**@brief       初始化数据滤波需要的参数结构
* @param[in]    Filterfactor_CountMax : 滤波系数和累计输入的最大值,滤波系数在高8位,
* 累计输入的最大值在低8位;
* @param[out]   FilterParam : 数据滤波需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax)
{        
    InMemory_Read_MultiBytes(FILTERFACTOR_BASE_ADDRESS, 
                            (uint8_t *)&FilterParam->FilterFactor, 
                            sizeof(FilterParam->FilterFactor));      //滤波系数
    SwitchCurFilter(FilterParam->FilterFactor, FilterParam);         //切换滤波水平
    FilterParam->InputCountMax = Filterfactor_CountMax; 
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMax),   //CapMax成员相对地址
                            (uint8_t *)&FilterParam->InputRangeMax, 
                            sizeof(FilterParam->InputRangeMax));      //电容满点值
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMin),   //CapMin成员相对地址
                            (uint8_t *)&FilterParam->InputRangeMin, 
                            sizeof(FilterParam->InputRangeMin));      //电容零点值
}


/**@brief       初始化数据转换需要的参数结构
* @param[out]   DataConvert_Param : 数据转换需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param)
{   
    InMemory_Read_MultiBytes(PCAP_PARAM_BASE_ADDRESS, 
                            (uint8_t *)DataConvert_Param, 
                            sizeof(PCap_DataConvert_Param));      //补偿使能标志        
}

/**@brief       初始化温度转换需要的参数结构
* @param[out]   ADC_TemperParam : 温度转换需要的参数结构指针; 
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam)
{    
    InMemory_Read_MultiBytes(ADC_TEMPERPARAM_BASE_ADDRESS, 
                            (uint8_t *)ADC_TemperParam, 
                            sizeof(ADC_TemperParam_TypeDef));
    ADC_TemperParam->TempDARange = ADC_TemperParam->ADC_Temper_Calib.TempDAMax
                                    - ADC_TemperParam->ADC_Temper_Calib.TempDAMin;
}
