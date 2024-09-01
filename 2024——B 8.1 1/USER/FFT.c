#include "FFT.h"
#include "ti_msp_dl_config.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/***********************************************
函数功能：使用DSP库进行1024点的FFT运算
参数：1、数据输入数组；2、FFT结果输出数组
返回值：测量出的相位值
**********************************************/
float FFT_Input[2048];

float FFT_Count(float* DATA_IN,float* FFT_Val,float* Per)
{
    volatile uint16_t i = 0;//循环变量

    float FFT_Output[1024];
    for(i = 0;i < 1024;i++)
    {
        FFT_Input[2 * i] = (float)DATA_IN[i];
        FFT_Input[2 * i + 1] = 0;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024,FFT_Input,0,1);
    arm_cmplx_mag_f32(FFT_Input,FFT_Output,1024);
    for(i = 0;i < 512;i++)
    {
        if(i == 0)
        {
            FFT_Val[i] = FFT_Output[i] / 1024;
        }
        else 
        {
            FFT_Val[i] = FFT_Output[i] / 512;
        }
    }
    Per[0] = FFT_Input[10];
    Per[1] = FFT_Input[11];
    return atan2(FFT_Input[11],FFT_Input[10]) * 180 / PI;
}