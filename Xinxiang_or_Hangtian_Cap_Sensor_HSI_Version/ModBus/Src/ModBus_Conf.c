/**@file        Modbus_Conf.c
* @brief        Modbus �����봦������
* @details      ����λ�����͵�ָ����н�������Ӧ,���й�������Զ��ϴ��ĳ�����붼��д�ڱ��ļ�
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

#include "ModBus_Conf.h"

#if defined(USING_MODBUS_RTU)
#include "ModBus_RTU.h"
#elif defined(USING_MODBUS_ASCII)
#include "ModBus_ASCII.h"
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)


#ifdef __PICOCAP_APP_H

/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifndef BOOT_PROGRAM
static uint32_t Calib_CapMin;                   ///< �궨�������ֵ
static uint32_t Calib_CapMax;                   ///< �궨��������ֵ
#ifdef USING_ADC_TEMPER_SENSOR
static uint16_t Calib_TempDAMin;                ///< �궨�¶�DA���ֵ
static uint16_t Calib_TempDAMax;                ///< �궨�¶�DA������
#endif
#ifdef USING_DAC
static uint16_t Calib_DAMin;                    ///< �궨����DA���ֵ
static uint16_t Calib_DALow;                    ///< �궨����DA�¿̶�
static uint16_t Calib_DAHigh;                   ///< �궨����DA�Ͽ̶�
static uint16_t Calib_DAMax;                    ///< �궨����DA������
static uint8_t CapDA_ClibFlag = CALIB_CLEAR;    ///< �궨����DA��־
#endif // USING_DAC
#ifdef USING_PWM
static uint16_t Calib_PWM0;                     ///< �궨���� PWM 0��ֵ
static uint16_t Calib_PWM1;                     ///< �궨���� PWM 1��ֵ
static uint16_t Calib_PWM2;                     ///< �궨���� PWM 2��ֵ
static uint16_t Calib_PWM3;                     ///< �궨���� PWM 3��ֵ
static uint16_t Calib_PWM4;                     ///< �궨���� PWM 4��ֵ
static uint16_t Calib_PWM5;                     ///< �궨���� PWM 5��ֵ
static uint8_t  CapPWM_ClibFlag = CALIB_CLEAR;  ///< �궨���� PWM ��־
#endif // USING_PWM


const uint8_t Company_Name[] = {9, '7','3','8','8','3','0','6','7','6'};
const uint8_t Product_Code[] = {20, 'S','F','C','G','2','0','L','-','1','1','3','5','A',
                                '-','M','2','-','M','E','P'};
const uint8_t Hardware_Version[] = {7, 'H','V','2','.','0','.','0'};
const uint8_t Software_Version[] = {7, 'S','V','2','.','0','.','0'};
const uint8_t Device_ID[] = {11, '2','0','2','0','0','8','1','7','0','0','1'};
const uint8_t Customer_Code[] = {7, 'K','1','5','0','0','1','6'};

//Modbus �������³�ʼ���ص�����
static int MB_USART_ReInit(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//Modbus �豸����ص�����
static int MB_Freeze(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//ϵͳ��λ�ص�����
static int MB_System_Reset(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//�޵�ַ��鷢������
static int MB_SendData_NoCheck(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//��������
static uint8_t BankUp_Data(void);

//�ָ���������
static uint8_t Factory_Data_Reset(void);



/**@brief       Modbus ��Ԥ����,�ӽ��յ�ԭʼ��������ȡ����ϢID�ͼĴ�������,���豸��ַ�͹�������뷢�ͻ�����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[out]   ReadAddr : ��ȡ��ַ(��ϢID)
* @param[out]   RegNum : �Ĵ�������
* @return       ����ִ�н��
* - None
* @note         �Ĵ�����ַ����Խ��
*/
static void ModBus_ReadPreHandle(   ModBusBaseParam_TypeDef *ModBusBaseParam, 
                                    uint16_t *ReadAddr, 
                                    uint16_t *RegNum, 
                                    uint8_t *Err_Status)
{
    *Err_Status = OP_SUCCESS;
    
    if(ReadAddr != NULL)
    {
        //�Ĵ�����ַ���ֽ�
        *ReadAddr = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2];
        *ReadAddr <<= 8;
        //�Ĵ�����ַ���ֽ�
        *ReadAddr |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
#if defined(SUBCODE_IS_DEVADDR)
        if((*ReadAddr >> 8) != ModBusBaseParam->Device_Addr)
        {
            *Err_Status = OP_FAILED;
        }        
#else
        if((*ReadAddr >> 8) != DEFAULT_SUBCODE)
        {
            *Err_Status = OP_FAILED;
            return;
        }
#endif // defined(SUBCODE_IS_DEVADDR)
        *ReadAddr &= 0x00FF;
    }

    if(RegNum != NULL)
    {
        //�Ĵ������ݸ��ֽ�
        *RegNum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
        *RegNum <<= 8;
        //�Ĵ����������ֽ�
        *RegNum |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    }
    
    //�����豸��ַ��������
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            2);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 2;
}

/**@brief       Modbus дԤ����,�ӽ��յ�ԭʼ��������ȡ����ϢID,���豸��ַ��
* �����롢��ϢID����Ϣ���ȴ��뷢�ͻ�����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[out]   WriteAddr : д���ַ(��ϢID)
* @param[out]   RegNum : �Ĵ�������
* @return       ����ִ�н��
* - None
* @note         �Ĵ�����ַ����Խ��
*/
static void ModBus_WritePreHandle(  ModBusBaseParam_TypeDef *ModBusBaseParam, 
                                    uint16_t *WriteAddr, 
                                    uint16_t *RegNum, 
                                    uint8_t *Err_Status)
{
    *Err_Status = OP_SUCCESS;
    
    if(WriteAddr != NULL)
    {
        //�Ĵ�����ַ���ֽ�
        *WriteAddr = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2];
        *WriteAddr <<= 8;
        //�Ĵ�����ַ���ֽ�
        *WriteAddr |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
#if defined(SUBCODE_IS_DEVADDR)
        if((*WriteAddr >> 8) != ModBusBaseParam->Device_Addr)
        {
            *Err_Status = OP_FAILED;
        }        
#else
        if((*WriteAddr >> 8) != DEFAULT_SUBCODE)
        {
            *Err_Status = OP_FAILED;
            return;
        }
#endif // defined(SUBCODE_IS_DEVADDR)
        *WriteAddr &= 0x00FF;
    }

    if(RegNum != NULL)
    {
        //�Ĵ������ݸ��ֽ�
        *RegNum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
        *RegNum <<= 8;
        //�Ĵ����������ֽ�
        *RegNum |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    }

    //����Ӧ���ݴ��뷢�ͻ���
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            6);

    ModBusBaseParam->ModBus_TX_RX.Send_Len = 6;
}

/**@brief       Modbus 03��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc03(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //�Ĵ�������
    uint16_t DataBuf;
    //�Ĵ�����ַ
    uint16_t ReadAddr;
    //�Ĵ�������
    uint16_t RegNum;
    //����״̬
    uint8_t Err_Status;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //��Ԥ����
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //���ʵ�ַ������Ч��Χ��
    if( ReadAddr < HOLDING_REG_REGION1_BGEIN 
        || (ReadAddr + RegNum) > (HOLDING_REG_REGION1_END + 1)
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }
    if(Err_Status != OP_SUCCESS && ReadAddr != 0x0030)
    {
        ReadAddr = 0xFFFF;
    }

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr++, ReadAddr++)
    {
        switch(ReadAddr)
        {
            case 0x0030:
                //�豸��ַ
                DataBuf = ModBusBaseParam->Device_Addr;
                if(BROADCASTADDR == ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0])
                {
                    ModBusBaseParam->ModBus_CallBack = MB_SendData_NoCheck;
                }
            break;
                    
            case 0x0031:
                //������
                DataBuf = ModBusBaseParam->BaudRate;
            break;
                    
            case 0x0032:
                //��żУ��
                DataBuf = ModBusBaseParam->Parity;
            break;

            case 0x0033:
                //��
                DataBuf = 0;
            break;

            case 0x0034:
                //����ʹ��
                DataBuf = Device_Param->PCap_DataConvert->CompenEn;
            break;

            case 0x0035:
                //�˲�ϵ��
                if(Device_Param->DataFilter->FilterFactor == 0)        //ʵʱ
                {
                    DataBuf = 1;
                }
                else if(Device_Param->DataFilter->FilterFactor == 4)   //ƽ��
                {
                    DataBuf = 2;
                }
                else if(Device_Param->DataFilter->FilterFactor == 9)   //ƽ��
                {
                    DataBuf = 3;
                }
            break;

            case 0x0036:
                //�Զ��ϴ�����
                DataBuf = (ModBusBaseParam->AutoUpload / 10) + 1;
            break;

            case 0x0037:
                //��������ϵ��K
                DataBuf = (uint16_t)(Device_Param->PCap_DataConvert->Correct_K * 100.0);
            break;

            case 0x0038:
                //��������ϵ��B
                DataBuf = (uint16_t)(Device_Param->PCap_DataConvert->Correct_B + 100.0);
            break;

            case 0x0039:
                //λ�ñ�����ֵ1 ����
                DataBuf = 0;
            break;
            
            case 0x003A:
                //�ز�1 ����
                DataBuf = 0;
            break;
            
            case 0x003B:
                //λ�ñ�����ֵ2 ����
                DataBuf = 0;
            break;
            
            case 0x003C:
                //�ز�2 ����
                DataBuf = 0;
            break;
            
            case 0x003D:
                //ת����λ ����
                DataBuf = 0;
            break;
            
            case 0x003E:
                //Pcap����ʱ�� ����
                DataBuf = 0;
            break;

            case 0x003F:
                //����
                DataBuf = Device_Param->PCap_DataConvert->HeightRange * 10;
            break;

//            case 0x0040:
//                //��
//                DataBuf = 0;
//            break;

//            case 0x0041:
//                //ȼ��ѡ�� ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0042:
//                //����������״ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0047:
//                //����ʱ�䷧ֵ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0048:
//                //��������ֵ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0049:
//                //©��ʱ�䷧ֵ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x004A:
//                //©������ֵ ����
//                DataBuf = 0;
//            break;
            
            case 0x0060:
                //�����ʽ
                DataBuf = ModBusBaseParam->Output_Mode;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                //���ʵ�ַ��Ч
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> 8);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)DataBuf;       
    }
}

/**@brief       Modbus 04��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc04(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //�Ĵ�����ַ
    uint16_t ReadAddr;
    //�Ĵ�������
    uint16_t RegNum;
    //����״̬
    uint8_t Err_Status;
    //�Ĵ�������
    uint32_t DataBuf;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //��Ԥ����
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //�Ĵ�����ַ��Ч
    if( Err_Status != OP_SUCCESS 
        || (ReadAddr > INPUT_REG_REGION1_END
        && (ReadAddr < INPUT_REG_REGION2_BEGIN || ReadAddr > INPUT_REG_REGION2_END))
        || (ReadAddr & 0x01) != 0 || (RegNum & 0x01) != 0
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }
    
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr += 2, ReadAddr += 2)
    {
        switch(ReadAddr)
        {
            case 0x0000:
                //Һλ�߶�ADֵ
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeightAD;
            break;
                
            case 0x0002:
                //Һ���¶� ����
                DataBuf = 0;
            break;
                
            case 0x0004:
                //�����¶�
                DataBuf = (uint32_t)Device_Param->PCap_DataConvert_Out->PCap_Temper_Value;
            break;
            
//            case 0x0006:
//                //������ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0008:
//                //©���� ����
//                DataBuf = 0;
//            break;
//            
//            case 0x000A:
//                //�������� ����
//                DataBuf = 0;
//            break;
            
            case 0x000C:
                //Һλ�ٷֱ�
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeight_Percentage;
            break;
            
            case 0x000E:
                //Һλ�߶�
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeight * 10;
            break;
    #ifdef USING_DAC        
            case 0x0010:
                //ҺλDAֵ
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_DA_Value;
            break;
    #endif // USING_DAC
    #ifdef USING_PWM
            case 0x0012:
                //ҺλPWMֵ
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_PWM_Value;
            break;
    #endif // USING_PWM
            case 0x0080:
                //PCapԭʼֵ
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
            break;
            
//            case 0x0082:
//                //PCap״̬ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0084:
//                //PCapд�̼�״̬ ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0086:
//                //Һλ������� ����
//                DataBuf = 0;
//            break;
//            
//            case 0x0088:
//                //Һλ�������� ����
//                DataBuf = 0;
//            break;
//            
//            case 0x008A:
//                //������� ����
//                DataBuf = 0;
//            break;
//            
//            case 0x008C:
//                //������ ����
//                DataBuf = 0;
//            break;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }

        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 24);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 16);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 8);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)DataBuf;        
    }
}

/**@brief       Modbus 05��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc05(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //�Ĵ�����ַ
    uint16_t WriteAddr;
    //����״̬
    uint8_t Err_Status;
    //�Ĵ�������
    uint16_t DataBuf;
    static uint8_t CalibFlag = CALIB_CLEAR;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //дԤ����
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, NULL, &Err_Status);    
    //��ַ��Ч
    if(Err_Status != OP_SUCCESS 
        || WriteAddr < SINGLE_COIL_REGION1_BEGIN 
        || WriteAddr > SINGLE_COIL_REGION1_END)
    {
        WriteAddr = 0xFFFF;
    }

    //�������ݸ��ֽ�
    DataBuf = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
    DataBuf <<= 8;
    //�������ݵ��ֽ�
    DataBuf |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];

    //����������Ч
    if((0x0000 != DataBuf)&&(0xFF00 != DataBuf))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
     
    switch(WriteAddr)
    {
        case 0x0050:
            if(ModBusBaseParam->InRomWrEn == IN_MEMORY_WR_ENABLE)
            {
                //���ݱ궨
                if(0xFF00 == DataBuf)       //�궨������
                {
                    Calib_CapMax = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
                    CalibFlag |= CALIB_CAPMAX_FLAG;
                }
                else                        //�궨���
                {
                    Calib_CapMin = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
                    CalibFlag |= CALIB_CAPMIN_FLAG;
                }
                //���ݱ궨��־λ��Ч��д��궨����  
                if(CALIB_CAPEOC_FLAG == CalibFlag)
                {
                    MB_Cap_Calibration(arg);
                    CalibFlag = CALIB_CLEAR;
                }
            }
            else
            {
                goto __rom_error;
            }
        break;

        case 0x0051:
            if(ModBusBaseParam->InRomWrEn == IN_MEMORY_WR_ENABLE)
            {
                //�ָ���������
                if(0xFF00 == DataBuf)
                {
                    if(Factory_Data_Reset() != OP_SUCCESS)
                    {
                        goto __rom_error;
                    }
                    //�ָ����ݺ�λ����
                    ModBusBaseParam->ModBus_CallBack = MB_System_Reset; 
                }
            }
            else
            {
                goto __rom_error;
            }
        break;

        case 0x0052:
            //�豸�����ⶳ
            if(0xFF00 == DataBuf)
            {
                ModBusBaseParam->ModBus_CallBack = MB_Freeze;
            }
            else    
            {
                ModBusBaseParam->Freeze = FREEZE_DISABLE;
            }
        break;

        case 0x0053:
            //�ڲ�Flashʹ�ܻ��ֹ
            if(0xFF00 == DataBuf)
            {
                ModBusBaseParam->InRomWrEn = IN_MEMORY_WR_ENABLE;
            }
            else
            {
                ModBusBaseParam->InRomWrEn = IN_MEMORY_WR_DISABLE;
            }
        break;
                
        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
            ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
            return;
    }	
    return;
__rom_error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;    
}

/**@brief       Modbus 10��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc10(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //��������
    uint16_t Index;
    //�Ĵ�����ַ
    uint16_t WriteAddr;
    //�Ĵ�������
    uint16_t RegNum;
    //����״̬
    uint8_t Err_Status;
    //���ݳ���
    uint16_t DataLen;
    //16λ�����ݴ�
    uint16_t u16temp;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    //���ݳ���
    DataLen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6];
    //дԤ����
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);    
    //�Ĵ�����ַ��Ч
    if( Err_Status != OP_SUCCESS 
        || WriteAddr < MUL_REG_REGION1_BEGIN 
        || (WriteAddr + RegNum) > (MUL_REG_REGION1_END + 1)
        || RegNum == 0)
    {
        WriteAddr = 0xFFFF;
    }

    //�ڲ�ROM���ʽ�ֹ
    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    Index = 0;
    for(Nr = 0; Nr < RegNum; Nr++, WriteAddr++)
    {
        u16temp = (uint16_t)(ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7 + Index] << 8) 
            | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[8 + Index];

        switch(WriteAddr)
        {
            case 0x0030:
                //�豸��ַ
                if(0 < u16temp && 0xF8 > u16temp)
                {
                    ModBusBaseParam->Device_Addr = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Device_Addr),
                                                (uint8_t *)&ModBusBaseParam->Device_Addr, 
                                                sizeof(ModBusBaseParam->Device_Addr));
                }
                else
                {
                    goto __error;
                }
            break;
                    
            case 0x0031:
                //������
                if(USART_BAUDRATE_115200_CODE >= u16temp)
                {
                    ModBusBaseParam->BaudRate = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->BaudRate), 
                                                (uint8_t *)&ModBusBaseParam->BaudRate, 
                                                sizeof(ModBusBaseParam->BaudRate));
                    ModBusBaseParam->ModBus_CallBack = MB_USART_ReInit;
                }
                else
                {
                    goto __error;
                }
            break;
                    
            case 0x0032:
                //��żУ��
                if(u16temp == USART_PARITY_NONE_CODE || u16temp == USART_PARITY_ODD_CODE 
                    || u16temp == USART_PARITY_EVEN_CODE)
                {
                    ModBusBaseParam->Parity = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Parity), 
                                                (uint8_t *)&ModBusBaseParam->Parity, 
                                                sizeof(ModBusBaseParam->Parity));
                    ModBusBaseParam->ModBus_CallBack = MB_USART_ReInit;
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0033:
                //��
            break;
              
            case 0x0034:
                //����ʹ��
                Device_Param->PCap_DataConvert->CompenEn = u16temp;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CompenEn), 
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CompenEn, 
                                            sizeof(Device_Param->PCap_DataConvert->CompenEn));
            break;
              
            case 0x0035:
                //�˲�ϵ��
                if(0x0A > u16temp)
                {
                    if(u16temp == 1)        //ʵʱ
                    {
                        Device_Param->DataFilter->FilterFactor = 0;
                    }
                    else if(u16temp == 2)   //ƽ��
                    {
                        Device_Param->DataFilter->FilterFactor = 4;
                    }
                    else if(u16temp == 3)   //ƽ��
                    {
                        Device_Param->DataFilter->FilterFactor = 9;
                    }
                    else                    //Ĭ��ƽ��
                    {
                        Device_Param->DataFilter->FilterFactor = 4;
                    }
                    InMemory_Write2T_MultiBytes((uint32_t)&(((DataFilterParam *)FILTERFACTOR_BASE_ADDRESS)->FilterFactor), 
                                                (uint8_t *)&Device_Param->DataFilter->FilterFactor, 
                                                sizeof(Device_Param->DataFilter->FilterFactor));
                    SwitchCurFilter(Device_Param->DataFilter->FilterFactor, Device_Param->DataFilter);
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0036:
                //�Զ��ϴ�����
                if(u16temp >= 1 && u16temp <= 4)
                {                                    
                    ModBusBaseParam->AutoUpload = (u16temp - 1) * 10;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->AutoUpload), 
                                                (uint8_t *)&ModBusBaseParam->AutoUpload, 
                                                sizeof(ModBusBaseParam->AutoUpload));
                }
            break;
              
            case 0x0037:
                //��������ϵ��K
                if(0 < u16temp)
                {
                    Device_Param->PCap_DataConvert->Correct_K = (float)u16temp / 100.0;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Correct_K), 
                                                (uint8_t *)&Device_Param->PCap_DataConvert->Correct_K, 
                                                sizeof(Device_Param->PCap_DataConvert->Correct_K));
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0038:
                //��������ϵ��B
                Device_Param->PCap_DataConvert->Correct_B = (float)u16temp - 100.0;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Correct_B), 
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Correct_B, 
                                            sizeof(Device_Param->PCap_DataConvert->Correct_B));
            break;              
                
            case 0x0039:
                //λ�ñ�����ֵ1 ����
            break;
            
            case 0x003A:
                //�ز�1 ����
            break;
            
            case 0x003B:
                //λ�ñ�����ֵ2 ����
            break;
            
            case 0x003C:
                //�ز�2 ����
            break;
            
            case 0x003D:
                //ת����λ ����
            break;
            
            case 0x003E:
                //Pcap����ʱ�� ����
            break;
              
            case 0x003F:
                //����
                if(u16temp > 0)
                {
                    Device_Param->PCap_DataConvert->HeightRange = u16temp / 10;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->HeightRange), 
                                                (uint8_t *)&Device_Param->PCap_DataConvert->HeightRange, 
                                                sizeof(Device_Param->PCap_DataConvert->HeightRange));
                }
                else
                {
                    goto __error;
                }
            break;
              
//            case 0x0040:
//                //��
//            break;              
//            
//            case 0x0041:
//                //ȼ��ѡ�� ����
//            break;
//            
//            case 0x0042:
//                //����������״ ����
//            break;
//            
//            case 0x0047:
//                //����ʱ�䷧ֵ ����
//            break;
//            
//            case 0x0048:
//                //��������ֵ ����
//            break;
//            
//            case 0x0049:
//                //©��ʱ�䷧ֵ ����
//            break;
//            
//            case 0x004A:
//                //©������ֵ ����
//            break;
            
            case 0x0060:
                //�����ʽ
                if(2 > u16temp)
                {
                    ModBusBaseParam->Output_Mode = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Output_Mode), 
                                                (uint8_t *)&ModBusBaseParam->Output_Mode, 
                                                sizeof(ModBusBaseParam->Output_Mode));
                }
                else
                {
                    goto __error;
                }
            break;
              
            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }     
            
        Index += DataLen;        
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

/**@brief       Modbus 25��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc25(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //�Ĵ�����ַ
    uint16_t WriteAddr;
    //����״̬
    uint8_t Err_Status;
    //��������
    uint16_t DataBuf;
    //�豸����
    ModBus_Device_Param *Device_Param;    
#ifdef USING_ADC_TEMPER_SENSOR
    static uint8_t TempDA_ClibFlag = CALIB_CLEAR;
    static uint8_t TempDA_ClibEn = CLIB_DISABLE;
#endif // USING_ADC_TEMPER_SENSOR
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    //дԤ����
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, NULL, &Err_Status);
    
    if(ModBusBaseParam->InRomWrEn != IN_MEMORY_WR_ENABLE)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
    
    //�Ĵ�����ַ��Ч
    if( Err_Status != OP_SUCCESS 
        || (WriteAddr > _25_FNUC_REG_REGION1_END))
    {
        WriteAddr = 0xFFFF;
    }

    //�Ĵ������ݸ��ֽ�
    DataBuf = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
    DataBuf <<= 8;
    //�Ĵ������ݵ��ֽ�
    DataBuf |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];

    //�Ĵ�������Ч
    if((0x0000 != DataBuf)&&(0xFF00 != DataBuf))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    switch(WriteAddr)
    {
        case 0x0000:
        //��
        break;
#ifdef USING_DAC
        case 0x0001:
            //����DA�궨���¿̶���
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapDA_ClibFlag |= CALIB_CAPDAHIH_FLAG;
                }
                else
                {
                    CapDA_ClibFlag |= CALIB_CAPDALOW_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;

        case 0x0002:
            //����DA�궨ʹ��
            if(0xFF00 == DataBuf)
            {
                CapDA_ClibFlag = CALIB_CLEAR;
                Device_Param->PCap_DataConvert->CapDA_ClibEn = CLIB_ENABLE;
            }
            else
            {
                Device_Param->PCap_DataConvert->CapDA_ClibEn = CLIB_DISABLE;
            }

            if(CLIB_DISABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn 
                && CapDA_ClibFlag != CALIB_CLEAR)
            {
                MB_CapDAOut_Calibration(arg);
                CapDA_ClibFlag = CALIB_CLEAR;
            }
        break;

        case 0x0003:
            //����DA�궨���������
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapDA_ClibFlag |= CALIB_CAPDAMAX_FLAG;
                }
                else
                {
                    CapDA_ClibFlag |= CALIB_CAPDAMIN_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_DAC
#ifdef USING_ADC_TEMPER_SENSOR
        case 0x0004:
            //�¶ȱ궨ʹ��
            if(0xFF00 == DataBuf)
            {
                TempDA_ClibFlag = CALIB_CLEAR;
                TempDA_ClibEn = CLIB_ENABLE;
            }
            else
            {
                TempDA_ClibEn = CLIB_DISABLE;
            }

            if((CLIB_DISABLE == TempDA_ClibEn) && (CALIB_TEMPDAEOC_FLAG == TempDA_ClibFlag))
            {
                MB_TempDAOut_Calibration(arg);
                TempDA_ClibFlag = CALIB_CLEAR;
            }
        break;

        case 0x0005:
            //�¶ȱ궨
            if(CLIB_ENABLE == TempDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    TempDA_ClibFlag |= CALIB_TEMPDAMAX_FLAG;
                    Calib_TempDAMax = Device_Param->ADC_TemperOut->TemperInAirAD;
                }
                else
                {
                    TempDA_ClibFlag |= CALIB_TEMPDAMIN_FLAG;
                    Calib_TempDAMin = Device_Param->ADC_TemperOut->TemperInAirAD;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_PWM
        case 0x0006:
            //����PWM�궨ʹ��
            if(0xFF00 == DataBuf)
            {
                CapPWM_ClibFlag = CALIB_CLEAR;
                Device_Param->PCap_DataConvert->CapPWM_ClibEn = CLIB_ENABLE;
            }
            else
            {
                Device_Param->PCap_DataConvert->CapPWM_ClibEn = CLIB_DISABLE;
            }

            if((CLIB_DISABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn) && (CALIB_CAPPWMEOC_FLAG == CapPWM_ClibFlag))
            {
                MB_CapPWMOut_Calibration(arg);
                CapPWM_ClibFlag = CALIB_CLEAR;
            }
        break;
            
        case 0x0007:
            //����PWM�궨 0��� 1��
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM1_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM0_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
            
        case 0x0008:
            //����PWM�궨 2��� 3��
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM3_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM2_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
            
        case 0x0009:
            //����PWM�궨 4��� 5��
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM5_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM4_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_PWM
            
        case 0x000A:
            //��������ʹ��
            if(0xFF00 == DataBuf)
            {
                if(BankUp_Data() != OP_SUCCESS)
                {
                    goto __error;
                }
            }
        break;
            
        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
            ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
            return;
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

/**@brief       Modbus 26��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc26(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint8_t i;
    //��������
    float fBuf;
    uint16_t Nr;
    //�Ĵ�����ַ
    uint16_t ReadAddr;
    //�Ĵ�������
    uint16_t RegNum;
    //����״̬
    uint8_t Err_Status;
    //�Ĵ�����������
    uint32_t DataBuf;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //��Ԥ����
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //�Ĵ������ʵ�ַ��Ч
    if( Err_Status != OP_SUCCESS 
        || ReadAddr < DEF_MUL_REG_REGION1_BEGIN 
        || (ReadAddr + RegNum) > (DEF_MUL_REG_REGION1_END + 2)
        || (RegNum & 0x01) != 0 || (ReadAddr & 0x01) != 0
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr += 2, ReadAddr += 2)
    {
        DataBuf = 0;
        switch(ReadAddr)
        {        
            case 0x0080:                                                        //��������
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMax        
                        - (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMin;        //�������
                DataBuf = *(uint32_t *)&fBuf;
            break;
                
            case 0x0082:
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMin;           //�������
                DataBuf = *(uint32_t *)&fBuf;
            break;
            
            case 0x0084:
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMax;           //����������
                DataBuf = *(uint32_t *)&fBuf;
            break;
#ifdef USING_DAC
            case 0x0088:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin;         //DA���
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x008A:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow;         //DA�¿̶�
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x008C:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh;        //DA�Ͽ̶�
                DataBuf = *(uint32_t *)&fBuf;
            break;
            
            case 0x008E:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax;         //DA����
                DataBuf = *(uint32_t *)&fBuf;
            break;
                                                     
#endif // USING_DAC   
#ifdef USING_ADC_TEMPER_SENSOR            
            case 0x0090:                                                                
                fBuf = Device_Param->ADC_TemperParam->Temper_K1 * 100;          //�����¶�����ϵ��K1   
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0092:
                fBuf = Device_Param->ADC_TemperParam->Temper_B1 + 100;          //�����¶�����ϵ��B1
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0094:
                fBuf = Device_Param->ADC_TemperParam->Temper_K2 * 100;          //Һ���¶�����ϵ��K2
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0096:
                fBuf = Device_Param->ADC_TemperParam->Temper_B2 + 100;          //Һ���¶�����ϵ��B2
                DataBuf = *(uint32_t *)&fBuf;
            break;
#endif // USING_ADC_TEMPER_SENSOR
            case 0x0098:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin;         //AD���
                DataBuf = *(uint32_t *)&fBuf;
            break;    
            
            case 0x009A:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow;         //AD�¿̶�
                DataBuf = *(uint32_t *)&fBuf;
            break;  
            
            case 0x009C:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh;        //AD�Ͽ̶�
                DataBuf = *(uint32_t *)&fBuf;
            break;    
            
            case 0x009E: 
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax;         //AD����
                DataBuf = *(uint32_t *)&fBuf;
            break;                         
#ifdef USING_PWM
            case 0x00A0:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0;          //PWM 0��
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00A2:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1;          //PWM 1��
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00A4:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2;          //PWM 2��
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00A6:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3;          //PWM 3��
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x00A8:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4;          //PWM 4��
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00AA:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5;          //PWM 5��
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00AC:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0;     //PWM 0��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00AE:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1;     //PWM 1��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x00B0:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2;     //PWM 2��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00B2:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3;     //PWM 3��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00B4:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4;     //PWM 4��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00B6:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5;     //PWM 5��߶�
                DataBuf = *(uint32_t *)&fBuf;
            break; 
#endif // USING_PWM
            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }

        for(i = 4; i > 0; i--)
        {
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
                = (uint8_t)(DataBuf >> ((i - 1)*8));
        }
    }
}

/**@brief       Modbus 27��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusFunc27(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //��������
    float fbuf;
    uint16_t Nr;
    //����
    uint16_t index;
    //�Ĵ�����ַ
    uint16_t WriteAddr;
    //�Ĵ�������
    uint16_t RegNum;
    //����״̬
    uint8_t Err_Status;
    //���ݳ���
    uint16_t DataLen;
    //�豸����
    ModBus_Device_Param *Device_Param;
          
    Device_Param = (ModBus_Device_Param *)arg;    
    //дԤ����
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);  
    //�ڲ�ROMδʹ�ܷ���ʧ��
    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    //���ݳ���
    DataLen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6];
    //�Ĵ������ʵ�ַ��Ч
    if( Err_Status != OP_SUCCESS 
        || WriteAddr < DEF_MUL_REG_REGION1_BEGIN 
        || (WriteAddr + RegNum) > (DEF_MUL_REG_REGION1_END + 2)
        || (RegNum & 0x01) != 0 || (WriteAddr & 0x01) != 0
        || RegNum == 0)
    {
        WriteAddr = 0xFFFF;
    }

    index = 0;
    for(Nr = 0; Nr < RegNum; Nr += 2, WriteAddr += 2)
    {
        fbuf = HexToFloat(&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7 + index]);
        switch(WriteAddr)
        {
//            case 0x0080:                                        //�������� ����
//                
//            break;
                
            case 0x0082:            
                Device_Param->PCap_DataConvert->Cap_Calib.CapMin = (uint32_t)fbuf;
                Device_Param->DataFilter->InputRangeMin = (uint32_t)fbuf;            
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMin),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib.CapMin, 
                                            sizeof(Device_Param->PCap_DataConvert->Cap_Calib.CapMin));
            break;
              
            case 0x0084:
                Device_Param->PCap_DataConvert->Cap_Calib.CapMax = (uint32_t)fbuf;
                Device_Param->DataFilter->InputRangeMax = (uint32_t)fbuf;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMax),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib.CapMax, 
                                            sizeof(Device_Param->PCap_DataConvert->Cap_Calib.CapMax));
            break;
#ifdef USING_DAC
            case 0x0088:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAMin = (uint16_t)fbuf;     
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }     
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAMin),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin));
                }
                else
                {
                    goto __error;
                }
            break;
                   
            case 0x008A:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DALow = (uint16_t)fbuf;    
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;                 
                }     
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDALow),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow));
                }                
                else
                {
                    goto __error;
                }
            break;
              
            case 0x008C:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAHigh = (uint16_t)fbuf;        
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAHigh),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh));
                }
                else
                {
                    goto __error;
                }
            break;
                  
            case 0x008E:                
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAMax = (uint16_t)fbuf;        
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }          
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAMax),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax));
                }
                else
                {
                    goto __error;
                }
            break; 
#endif // USING_DAC
#ifdef USING_ADC_TEMPER_SENSOR
            case 0x0090:                                                                
                Device_Param->ADC_TemperParam->Temper_K1 = fbuf / 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_K1),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_K1, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_K1));
            break;
              
            case 0x0092:
                Device_Param->ADC_TemperParam->Temper_B1 = fbuf - 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_B1),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_B1, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_B1));
            break;
              
            case 0x0094:
                Device_Param->ADC_TemperParam->Temper_K2 = fbuf / 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_K2),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_K2, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_K2));
            break;
              
            case 0x0096:
                Device_Param->ADC_TemperParam->Temper_B2 = fbuf - 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_B2),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_B2, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_B2));
            break;
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_DAC
            case 0x0098:       
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADMin),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin));
                }   
                else
                {
                    goto __error;
                }                
            break; 
                   
            case 0x009A:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow = (uint16_t)fbuf;                 
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADLow),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow));
                }
                else
                {
                    goto __error;
                }
            break; 
              
            case 0x009C:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADHigh),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh));
                }      
                else
                {
                    goto __error;
                }
            break; 
                   
            case 0x009E:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax = (uint16_t)fbuf;      
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADMax),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax));
                }         
                else
                {
                    goto __error;
                }  
            break;       
#endif // USING_DAC

#ifdef USING_PWM
            case 0x00A0:                                                            //PWM 0��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM0 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM0),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0));
                }   
                else
                {
                    goto __error;
                }                                     
            break;
              
            case 0x00A2:                                                            //PWM 1��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM1 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM1),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00A4:                                                            //PWM 2��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM2 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM2),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00A6:                                                            //PWM 3��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM3 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM3),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3));
                }   
                else
                {
                    goto __error;
                }         
            break; 
            
            case 0x00A8:                                                            //PWM 4��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM4 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM4),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00AA:                                                            //PWM 5��
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM5 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM5),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00AC:                                                            //PWM 0��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High0),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00AE:                                                            //PWM 1��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High1),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1));
                }   
                else
                {
                    goto __error;
                }         
            break; 
            
            case 0x00B0:                                                            //PWM 2��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High2),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00B2:                                                            //PWM 3��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High3),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00B4:                                                            //PWM 4��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4 = (uint16_t)fbuf;         
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High4),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00B6:                                                            //PWM 5��߶�
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High5),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5));
                }   
                else
                {
                    goto __error;
                }         
            break; 

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }
        index += DataLen;
#endif // USING_PWM
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

///**@brief       Modbus 2A��������Ϣ֡����
//* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
//* @return       ����ִ�н��
//* - None
//*/
//void ModbusFunc2A(ModBusBaseParam_TypeDef *ModBusBaseParam)
//{
//    uint8_t i;
//    uint8_t j;
//    uint8_t objlen;
//    uint16_t RegNum;
//    //����״̬
//    uint8_t Err_Status;
//    uint16_t WriteAddr;

//    //дԤ����
//    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);    
//    //�ڲ�ROMδʹ�ܷ���ʧ��
//    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
//    {
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
//        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//        return;
//    }
//    if( Err_Status != OP_SUCCESS 
//        || WriteAddr < MUL_VERSION_INF_BEGIN 
//        || (WriteAddr + RegNum) > (MUL_VERSION_INF_END + 1)
//        || RegNum == 0)
//    {
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
//        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//        return;
//    }

//    j = 6;
//    for(i = 0; i < RegNum; i++, WriteAddr++)
//    {
//        objlen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j];
//        switch(WriteAddr)
//        {
//            //��������
//            case 0x00E0:            
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(ORGANIZATION, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                    
//                j += objlen;
//            break;
//            //��Ʒ����    
//            case 0x00E1:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(PRODUCTION, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                               
//                j += objlen;
//            break;
//            //Ӳ���汾    
//            case 0x00E2:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(HARDWAREVER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                  
//                j += objlen;
//            break;
//            //����汾    
//            case 0x00E3:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(SOFTWAREVER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                  
//                j += objlen;
//            break;
//            //�豸ID    
//            case 0x00E4:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(DEVICENUM, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                     
//                j += objlen;
//            break;
//            //�ͻ�����    
//            case 0x00E5:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(CUSTOMER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                               
//                j += objlen;
//            break;
//                
//            default:
//                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
//                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//                return;
//        }        
//    }
//    return;
//__error:
//    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
//    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
//}

/**@brief       Modbus 2B��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @return       ����ִ�н��
* - None
*/
void ModbusFunc2B(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    uint8_t i;
    uint8_t objlen;
    uint16_t RegNum;
    uint16_t ReadAddr;
    //����״̬
    uint8_t Err_Status;

    //��Ԥ����
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);    
    
    if( Err_Status != OP_SUCCESS 
        || ReadAddr < MUL_VERSION_INF_BEGIN
        || (ReadAddr + RegNum) > (MUL_VERSION_INF_END + 1)
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }

    memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[2], &ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4], 2);
    memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[4], &ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2], 2);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 6;
    for(i = 0; i < RegNum; i++, ReadAddr++)
    {
        switch(ReadAddr)
        {
            //��������  
            case 0x00E0:
//                objlen = InMemory_Read_OneByte(ORGANIZATION);                         
                objlen = Company_Name[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((ORGANIZATION + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Company_Name[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //��Ʒ����  
            case 0x00E1:
//                objlen = InMemory_Read_OneByte(PRODUCTION);                 
                objlen = Product_Code[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((PRODUCTION + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Product_Code[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //Ӳ���汾  
            case 0x00E2:
//                objlen = InMemory_Read_OneByte(HARDWAREVER);  
                objlen = Hardware_Version[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((HARDWAREVER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Hardware_Version[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //����汾 
            case 0x00E3:                        
//                objlen = InMemory_Read_OneByte(SOFTWAREVER);
                objlen = Software_Version[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((SOFTWAREVER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Software_Version[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //�豸ID  
            case 0x00E4:
//                objlen = InMemory_Read_OneByte(DEVICENUM);   
                objlen = Device_ID[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((DEVICENUM + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Device_ID[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //�ͻ�����  
            case 0x00E5:
//                objlen = InMemory_Read_OneByte(CUSTOMER);       
                objlen = Customer_Code[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((CUSTOMER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Customer_Code[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }        
    }
}
#endif // BOOT_PROGRAM

/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM
#include "common.h"
#include "flash_if.h"
#endif // BOOT_PROGRAM

/**@brief       Modbus 41��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @return       ����ִ�н��
* - None
*/
void ModbusFunc41(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM
    
    uint16_t WriteAddr;                     //�Ĵ�����ַ    
    uint16_t DataLen;                       //���ݳ���
    /* �����������ڲ�EEPROM�Ĳ�����־, ����STM32L0ϵ�������оƬ���ʾ�����������ڲ�FLASH�Ĳ�����־ */
    uint16_t Erase_EEPROM_Flag = 0;         
    uint16_t packetnum;                     //�ܰ���
    uint16_t packetcnt;                     //�����
    uint16_t prt;                           //����
    uint32_t tpcksum;                       //��У���
    uint32_t *ramdata;                      //����ָ��
    static uint16_t PacketCnt;              // �����
    static uint16_t PacketNum;              // �ܰ���
    static uint32_t Flashadrdst;            // FLASH��ַ
    static uint32_t FileCheckSum;           // �����ļ�У���
    static uint32_t FileRunCheckSum;        // �����ļ�ʵʱУ���
    
    WriteAddr = (ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2] << 8)
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
    DataLen = (ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4] << 8)
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    
    //�Ĵ�����ַ��Ч
    if( 
#if defined(SUBCODE_IS_DEVADDR)
        (WriteAddr >> 8) != ModBusBaseParam->Device_Addr ||
#endif // defined(SUBCODE_IS_DEVADDR)
        (WriteAddr & 0xFF) < 1 
        || (WriteAddr & 0xFF) > 4
        || (DataLen != (ModBusBaseParam->ModBus_TX_RX.Receive_Len - 6)))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
    
    WriteAddr &= 0x00FF;
    
    //�洢��������Ϣ
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            4);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 4;

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0x00;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0x01;

    switch(WriteAddr)
    {
        case 0x0001:                                                                        //��ʼ����      
            ModBusBaseParam->UpgradeWaitTime = -1;        
            if((0 != DataLen) && (2 != DataLen))
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                break;
            }
            else 
            {
                if(DataLen == 2)
                {
                    Erase_EEPROM_Flag = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6] * 256 
                                        + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7];    //��ȡ������־
                }
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            }  
            PacketNum = 0;
            PacketCnt = 0;

            if(Erase_EEPROM_Flag)
            {         
            #if defined(STM32F0)                
                InFlash_Erase_Page(0, 4);
            #elif defined(STM32L0)
                InEEPROM_Erase_MultiWord(0, (IN_EEPROM_END - IN_EEPROM_START + 1) / 4);
            #endif // defined(STM32F0) or defined(STM32L0)
            }
            break;
          
        case 0x0002:                                                                                //���Դ����
            if(0 != DataLen)
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                break;
            }          
            FLASH_If_Init();                                                                          //FLASH����
            if(FLASH_If_GetWriteProtectionStatus() != FLASHIF_PROTECTION_NONE)
            {
                if(FLASH_If_WriteProtectionConfig(FLASHIF_WRP_DISABLE) == FLASHIF_OK)
                {
                    HAL_FLASH_OB_Launch();
                }
            }
            FLASH_If_Erase(APPLICATION_ADDRESS);
            Flashadrdst = APPLICATION_ADDRESS;
            ModBusBaseParam->ProgErase = ERASE_FLAG;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            InMemory_Write_OneByte(ADDR_ERASEFLAG, ERASE_FLAG);
            break;

        case 0x0003:                //���������ļ�
            packetnum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6] * 256 + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7];    //��ȡ�ܰ���
            packetcnt = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[8] * 256 + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[9];
            if((0 == PacketNum) && (1 < packetnum) && (0 == packetcnt))
            {
                FileCheckSum = 0;
                FileRunCheckSum = 0;
                PacketNum = packetnum;
                PacketCnt = packetcnt;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;

                for(prt = 0; prt < 4; prt++)
                {
                    FileCheckSum <<= 8;
                    FileCheckSum += ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }
            }
            else if((PacketNum == packetnum) && (1 < packetnum) 
                && (PacketCnt == (packetcnt - 1)) && (PacketNum > packetcnt))
            {
                tpcksum = 0;
                DataLen = DataLen - 4;

                for(prt = 0; prt < DataLen; prt++)
                {
                    tpcksum += ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }

                Decoding(&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10], DataLen);

                for(prt = 0; prt < DataLen; prt++)
                {
                    ModBusBaseParam->ModBus_TX_RX.Receive_Buf[prt] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }
                ramdata = (uint32_t*)ModBusBaseParam->ModBus_TX_RX.Receive_Buf;

                if(FLASH_If_Write(Flashadrdst, ramdata, DataLen/4)  == 0)
                {
                    PacketCnt++;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                    FileRunCheckSum += tpcksum;
                    Flashadrdst += DataLen;
                }
                else
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                }
            }
            else if((PacketNum == packetnum) && (1 < packetnum) 
                && (PacketCnt == packetcnt) && (PacketNum > packetcnt))
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            }
            else
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
            }
            break;

        case 0x0004:                                                                                  //ִ��Ӧ�ó���
            if((((FileRunCheckSum == FileCheckSum) && ((PacketCnt + 1) == PacketNum)) 
                || (0 == PacketNum))&&(0 == DataLen))
            {
                if(0 != PacketNum)
                {
                    ModBusBaseParam->UpgradeWaitTime = 0;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                }
                else if(ModBusBaseParam->ProgErase == ERASE_FLAG)
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                }
                else
                {
                    ModBusBaseParam->UpgradeWaitTime = 0;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                }
            }
            else
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_LRC_ERR;
            }
            break;

        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
            break;
    }
    
#else
    uint16_t temp[2];    
    
    temp[0] = ((uint16_t)ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2] << 8) 
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
    temp[1] = ((uint16_t)ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4] << 8) 
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    if(temp[0] != 0x0001 || temp[1] != 0x0000)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[0] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0];
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[1] 
                                                    | MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
    }
    else
    {
        InMemory_Write_OneByte(ADDR_UPGRADEFLAG, 0xFF);
        memcpy(ModBusBaseParam->ModBus_TX_RX.Send_Buf, ModBusBaseParam->ModBus_TX_RX.Receive_Buf, 5);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[5] = 0x01;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[6] = 0x00;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 7;        
        ModBusBaseParam->ModBus_CallBack = MB_System_Reset;
    }
#endif // BOOT_PROGRAM
}

/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifndef BOOT_PROGRAM

/**@brief       Modbus ��Ϣ֡�Զ��ϴ�����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void ModbusAutoUpload(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    float fbuf;
    uint8_t Nr;
    uint32_t DataBuf;
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[0] = ModBusBaseParam->Device_Addr;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] = 0x04;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = 0x08;

    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
    fbuf = (float)Device_Param->PCap_DataConvert_Out->LiquidHeightAD;
    DataBuf = *(uint32_t *)&fbuf;
    for(Nr = 4; Nr > 0; Nr--)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> ((Nr - 1)*8));
    }

    DataBuf = *(uint32_t *)&Device_Param->PCap_DataConvert_Out->PCap_Temper_Value;
    for(Nr = 4; Nr > 0; Nr--)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> ((Nr - 1)*8));
    }
#if defined(USING_MODBUS_RTU)
    MODBUS_RTU_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#elif defined(USING_MODBUS_ASCII)
    MODBUS_ASCII_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)
}

/**@brief       ���ݱ궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_Cap_Calibration(void *arg)
{
    //�豸����
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    if(Calib_CapMin < Calib_CapMax)
    {
        //�������
        Device_Param->PCap_DataConvert->Cap_Calib.CapMin = Calib_CapMin;
        //��������
        Device_Param->PCap_DataConvert->Cap_Calib.CapMax = Calib_CapMax;
        
        Device_Param->DataFilter->InputRangeMin = Calib_CapMin;
        Device_Param->DataFilter->InputRangeMax = Calib_CapMax;
        //���ݱ궨����д���ڲ�Flash
        InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib),
                                    (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib, 
                                    sizeof(Device_Param->PCap_DataConvert->Cap_Calib));
    }
}

#ifdef USING_DAC
/**@brief       ����DAֵ�궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_CapDAOut_Calibration(void *arg)
{
    //�豸����
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    if(CapDA_ClibFlag == (CALIB_CAPDAMAX_FLAG | CALIB_CAPDAMIN_FLAG))
    {
        if(Calib_DAMax <= Calib_DAMin)
        {
            return;
        }
        Calib_DAHigh = 0;
        Calib_DALow = 0;        
    }
    else
    {
        if((Calib_DAMax <= Calib_DAHigh) 
            || (Calib_DAHigh <= Calib_DALow) 
            || (Calib_DALow <= Calib_DAMin))
        {
            return;
        }
    }    
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin = Calib_DAMin;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow = Calib_DALow;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh = Calib_DAHigh;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax = Calib_DAMax;                
    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib),
                                (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib, 
                                sizeof(Device_Param->PCap_DataConvert->CapDA_Calib));            
}
#endif // USING_DAC

#ifdef USING_ADC_TEMPER_SENSOR
/**@brief       �¶�DAֵ�궨����
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_TempDAOut_Calibration(void *arg)
{
    //�豸����
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    //����¶�DA�궨ֵ��Ч��д���ڲ�Flash
    if((Calib_TempDAMax > Calib_TempDAMin) && (Calib_TempDAMax <= DAC_VALUE_MAX))
    {
        Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMin = Calib_TempDAMin;
        Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMax = Calib_TempDAMax;
        Device_Param->ADC_TemperParam->TempDARange = Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMax 
                                                - Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMin;
        InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->ADC_Temper_Calib),
                                    (uint8_t *)&Device_Param->ADC_TemperParam->ADC_Temper_Calib, 
                                    sizeof(Device_Param->ADC_TemperParam->ADC_Temper_Calib));               
    }
}
#endif // USING_ADC_TEMPER_SENSOR

#ifdef USING_PWM
/**@brief       ����PWMֵ�궨
* @param[in]    arg : �û��Զ���Ĳ���,����Ϊ�豸����
* @return       ����ִ�н��
* - None
*/
void MB_CapPWMOut_Calibration(void *arg)
{
    //�豸����
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    if(CapPWM_ClibFlag == CALIB_CAPPWMEOC_FLAG)
    {
        if(Calib_PWM0 <= Calib_PWM1 || Calib_PWM1 <= Calib_PWM2
             || Calib_PWM2 <= Calib_PWM3 || Calib_PWM3 <= Calib_PWM4
             || Calib_PWM4 <= Calib_PWM5)
        {
            return;
        }     
    }
    else
    {
        return;
    }                    
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0 = Calib_PWM0;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1 = Calib_PWM1;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2 = Calib_PWM2;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3 = Calib_PWM3;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4 = Calib_PWM4;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5 = Calib_PWM5;
    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib),
                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib, 
                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib));      
}

#endif // USING_PWM

/**@brief       Modbus �������³�ʼ��
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - int����ֵ(OP_SUCCESS)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static int MB_USART_ReInit(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{        
/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
    rt_sem_take(ModBusBaseParam->TX_Lock, RT_WAITING_FOREVER);    //��ȡ�ź���
#else
    while(Sensor_USART_Get_TX_Cplt_Flag() == 0);    //�ȴ����ڷ������
#endif // USING_RT_THREAD_OS
    
    UNUSED(arg);
    Sensor_USART_Init(  ModBusBaseParam->BaudRate, 
                        ModBusBaseParam->Parity);
    
/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
    rt_sem_release(ModBusBaseParam->TX_Lock);    //�ͷ��ź���
#endif  // USING_RT_THREAD_OS
    return OP_SUCCESS;
}

/**@brief       Modbus �豸����
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - int����ֵ(OP_SUCCESS)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static int MB_Freeze(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{           
    UNUSED(arg);
    ModBusBaseParam->Freeze = FREEZE_ENABLE;
    
    return OP_SUCCESS;
}

/**@brief       ϵͳ��λ
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - int����ֵ(OP_SUCCESS)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static int MB_System_Reset(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
    rt_sem_take(ModBusBaseParam->TX_Lock, RT_WAITING_FOREVER);    //��ȡ�ź���
#else
    while(Sensor_USART_Get_TX_Cplt_Flag() == 0);    //�ȴ����ڷ������
#endif // USING_RT_THREAD_OS
    
    UNUSED(arg);
    HAL_NVIC_SystemReset();
    
    return OP_SUCCESS;
}

/**@brief       ����ModBus�����Ҳ�����豸��ַ
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - int����ֵ(OP_SUCCESS)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static int MB_SendData_NoCheck(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //����Modbus RTU
#if defined(USING_MODBUS_RTU)
    MODBUS_RTU_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#elif defined(USING_MODBUS_ASCII)
    UNUSED(arg);
    MODBUS_ASCII_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)
    
    return OP_SUCCESS;
}

uint8_t System_Param_Buf[SYSTEM_PARAM_LEN];

/**@brief       ����ǰ���в���������
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static uint8_t BankUp_Data(void)
{
    InMemory_Read_MultiBytes(SYSTEM_PARAM_BASE_ADDRESS, System_Param_Buf, SYSTEM_PARAM_LEN);
    
    return InMemory_Write_MultiBytes(SYSTEM_PARAM_BAK2_BASE_ADDRESS,
                                    System_Param_Buf,
                                    SYSTEM_PARAM_LEN);
}

/**@brief       �ָ���������(�����ݲ�����ȡ�������ǵ�ǰϵͳ����)
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��;  
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
* @note         ����������ModBusBaseParam->ModBus_CallBack��,����ModBus��Ӧ��λ�������
*/
static uint8_t Factory_Data_Reset(void)
{
    InMemory_Read_MultiBytes(SYSTEM_PARAM_BAK2_BASE_ADDRESS, System_Param_Buf, SYSTEM_PARAM_LEN);
    
    return InMemory_Write2T_MultiBytes(SYSTEM_PARAM_BASE_ADDRESS,
                                        System_Param_Buf,
                                        SYSTEM_PARAM_LEN);
}


#endif // BOOT_PROGRAM

#endif // __PICOCAP_APP_H
