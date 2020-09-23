/**@file        Param_Init.h
* @details      Param_Init.c的头文件,声明了各个设备或模块的运行参数的初始化函数
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
#ifndef __PARAM_INIT_H
#define __PARAM_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "In_Memory_app.h"
#include "ModBus.h"
#include "Picocap_app.h"
#include "adc_app.h"

/**@brief       ModBus参数初始化
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体
* @return       函数执行结果
* - None
*/
void ModBus_Param_Init(ModBusBaseParam_TypeDef *ModBusBaseParam);

/**@brief       初始化数据滤波需要的参数结构
* @param[in]    Filterfactor_CountMax : 滤波系数和累计输入的最大值,滤波系数在高8位,
* 累计输入的最大值在低8位;
* @param[out]   FilterParam : 数据滤波需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax);

/**@brief       初始化数据转换需要的参数结构
* @param[out]   DataConvert_Param : 数据转换需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param);

/**@brief       初始化温度转换需要的参数结构
* @param[out]   ADC_TemperParam : 温度转换需要的参数结构指针; 
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam);

#ifdef __cplusplus
}
#endif
#endif // __PARAM_INIT_H
