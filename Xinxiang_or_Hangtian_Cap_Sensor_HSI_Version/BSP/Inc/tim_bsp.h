/**@file        tim_bsp.h
* @details      tim_bsp.c��ͷ�ļ�
* @author       ���
* @date         2020-08-13
* @version      V1.0.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/04/27  <td>1.0.0    <td>���    <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

#ifndef __TIM_BSP_H
#define __TIM_BSP_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef struct _Tim_DevDef {
    void        (*tim_init)(uint32_t prescaler, uint32_t period);
    void        (*tim_pwm_init)(uint32_t prescaler, uint32_t period, uint32_t pulse);
    uint32_t    (*tim_pwm_start)(uint32_t channel);
    void        (*tim_set_pwm)(uint32_t channel, uint32_t period, uint32_t pulse);
    uint32_t    (*tim_base_start)(void);
}Tim_DevDef;

/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifndef USING_RT_THREAD_OS

//#define BSP_USING_TIM2
#define BSP_USING_TIM3
//#define BSP_USING_TIM7

#define MAX_PERIOD 65535
#define MIN_PERIOD 3
#define MIN_PULSE 2

#endif // USING_RT_THREAD_OS

enum
{
#ifdef BSP_USING_TIM1
    TIM1_INDEX,
#endif // BSP_USING_TIM1
#ifdef BSP_USING_TIM2
    TIM2_INDEX,
#endif // BSP_USING_TIM2
#ifdef BSP_USING_TIM3
    TIM3_INDEX,
#endif // BSP_USING_TIM3
#ifdef BSP_USING_TIM4
    TIM4_INDEX,
#endif // BSP_USING_TIM4
#ifdef BSP_USING_TIM5
    TIM5_INDEX,
#endif // BSP_USING_TIM5
#ifdef BSP_USING_TIM6
    TIM6_INDEX,
#endif // BSP_USING_TIM6
#ifdef BSP_USING_TIM7
    TIM7_INDEX,
#endif // BSP_USING_TIM7
#ifdef BSP_USING_TIM8
    TIM8_INDEX,
#endif // BSP_USING_TIM8
#ifdef BSP_USING_TIM9
    TIM9_INDEX,
#endif // BSP_USING_TIM9
#ifdef BSP_USING_TIM10
    TIM10_INDEX,
#endif // BSP_USING_TIM10
#ifdef BSP_USING_TIM11
    TIM11_INDEX,
#endif // BSP_USING_TIM11
#ifdef BSP_USING_TIM12
    TIM12_INDEX,
#endif // BSP_USING_TIM12
#ifdef BSP_USING_TIM13
    TIM13_INDEX,
#endif // BSP_USING_TIM13
#ifdef BSP_USING_TIM14
    TIM14_INDEX,
#endif // BSP_USING_TIM14
#ifdef BSP_USING_TIM15
    TIM15_INDEX,
#endif // BSP_USING_TIM15
#ifdef BSP_USING_TIM16
    TIM16_INDEX,
#endif // BSP_USING_TIM16
#ifdef BSP_USING_TIM17
    TIM17_INDEX,
#endif // BSP_USING_TIM17
    TIM_INDEX_TOTAL
};

/** Ĭ��Ԥ��Ƶֵ��ȡֵ��Χ��0x0~0xFFFF��ʵ�ʷ�ƵΪ DEFAULT_PRESCALER + 1 */
#define TIM3_DEFAULT_PRESCALER  0       
/** Ĭ������ֵ��ȡֵ��Χ��0x0~0xFFFF(TIM2��ȡ��0x0~0xFFFFFFFF������Ŀ����÷�Χ
�Բο��ֲ�˵��Ϊ׼)��ʵ������Ϊ DEFAULT_PERIOD + 1 */
#define TIM3_DEFAULT_PERIOD     8121    

extern Tim_DevDef tim_dev[TIM_INDEX_TOTAL];

void TIM_DevObj_Init(void);

#ifdef __cplusplus
}
#endif
#endif // __TIM_BSP_H
