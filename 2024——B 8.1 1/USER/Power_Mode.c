#include "ti_msp_dl_config.h"
#include "FFT.h"
#include "arm_math.h"
#include "arm_const_structs.h"



void Power_Count(uint16_t* Voltage_Value,uint16_t* Current_Value,uint8_t Power_Flag)
{
    float FFT_Voltage[512];
    float FFT_Current[512];
    uint16_t Voltage;//电压相关幅值
    uint16_t Current;//电流相关幅值
    float32_t Voltage_Per;//电压相关相位
    float32_t Current_Per;//电流相关相位
    if(Power_Flag)
    {
        //Voltage_Per = FFT_Count(Voltage_Value, FFT_Voltage);//对电压数组做FFT，并返回相位值
        //Current_Per = FFT_Count(Current_Value,FFT_Current);//对电流数组进行FFT，并返回相位值
        
    }
}