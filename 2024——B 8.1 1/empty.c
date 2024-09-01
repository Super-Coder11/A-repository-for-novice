/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "FFT.h"
#include "stdio.h"
#include "string.h"
#include "UART.h"
#include "Power_Mode.h"
#include "math.h"

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter

//2. 以FFT_Voltage定义卡尔曼结构体并初始化参数
KFP KFP_FFT_Wave={0.02,0,0,0,0.001,0.543};
//3.以FFT_Current定义卡尔曼结构体并初始化参数
KFP KFP_Current_Wave = {0.02,0,0,0,0.001,0.543};
//3.以相位差定义卡尔曼结构体并初始化参数
KFP KFP_Per_Wave = {0.02,0,0,0,0.001,0.543};
//4.以谐波定义卡尔曼结构体并初始化参数
KFP Wave_2 = {0.02,0,0,0,0.001,0.543};

KFP Wave_3 = {0.02,0,0,0,0.001,0.543};

KFP Wave_4 = {0.02,0,0,0,0.001,0.543};

KFP Wave_5 = {0.02,0,0,0,0.001,0.543};

KFP Wave_6 = {0.02,0,0,0,0.001,0.543};

KFP Wave_7 = {0.02,0,0,0,0.001,0.543};

KFP Wave_8 = {0.02,0,0,0,0.001,0.543};

KFP Wave_9 = {0.02,0,0,0,0.001,0.543};

KFP Wave_10 = {0.02,0,0,0,0.001,0.543};
////5.以功率定义卡尔曼结构体并初始化参数
KFP KFP_Power = {0.02,0,0,0,0.001,0.543};
//6.以功率因数定义卡尔曼结构体并初始化参数
KFP KFP_PowerPer = {0.02,0,0,0,0.001,0.543};

uint16_t Voltage_DMA[1024];//电压互感器DMA采集值
uint16_t Current_DMA[1024];//电流互感器DMA采集值

uint16_t Voltage[1024];//电压互感器测量原始值
uint16_t Current[1024];//电流互感器电压测量原始值

float32_t Current_Wave[10];//电流相关谐波分量计算值
float32_t Current_Wave_Inf_Middle;//THD计算分量中间值
float32_t Current_Wave_Inf;//THD计算结果

float32_t Voltage_Per[2];
float32_t Current_per[2];

float32_t Voltage_Val;//Voltage有效值
float32_t Current_Val;//current有效值

float32_t Voltage_Middle;//VOltage中间值
float32_t Current_Middle;//Current中间值

float32_t Voltage_Avr;//电压平均值
float32_t Current_Avr;//电流平均值

float32_t Voltage_Set[1024];//电压修正值存放
float32_t Current_Set[1024];//电流修正值存放

float32_t Voltage_PerCount;//电压相位
float32_t Current_PerCount;//电流相位

//float32_t Current_Get;//电流计算值
//float32_t Voltage_Get;//电压计算值

float32_t Per_Gap;//相位差

float32_t FFT_Voltage[512];
float32_t FFT_Current[512];

float32_t Power;//视在功率计算
float32_t Power_Eff;//有功功率计算

float32_t Power_Per;//功率因数

//相关标志位
uint8_t Mode_Set;//模式选择标志位
uint8_t ADC_Flag;//ADCDMA搬运完成标志位 等于2时标志完成


//卡尔曼滤波器1
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }



int main(void)
{
    SYSCFG_DL_init();
    //初始化部分

    //ADC0DMA初始化
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,(uint32_t) DL_ADC12_getFIFOAddress(ADC12_0_INST));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &Voltage_DMA[0]);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);//只使能ADC0的中断

    //ADC1DMA初始化
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID,(uint32_t) DL_ADC12_getFIFOAddress(ADC12_1_INST));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) &Current_DMA[0]);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);


    //定时器开始计时
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) 
    {
       if(ADC_Flag == 1)
       {
            uint16_t i = 0;
            DL_TimerG_stopCounter(TIMER_0_INST);
            //将修正弥补量引入
            /*for(i=0;i<1024;i++)
            {
                Voltage[i] = Voltage[i] * 0.43 - 1.09;//V
                Current[i] = Current[i] * 2.57 - 0.66;//mA
            }*/


            
            for(i=0;i<1024;i++)
            {
                Voltage_Avr += Voltage[i];
                Current_Avr += Current[i];
            }
            Voltage_Avr /= 1024;
            Current_Avr /= 1024;
            for(i = 0;i < 1024;i++)
            {
                Voltage_Set[i] = Voltage[i] - Voltage_Avr;
                Current_Set[i] = Current[i] - Current_Avr;
            }

            //开始计算相关参数

            for(i=0;i<1024;i++)
            {
                //计算电压有效值
                Voltage_Middle += Voltage_Set[i] * Voltage_Set[i];
                //计算电流有效值
                Current_Middle += Current_Set[i] * Current_Set[i];
            }
            //计算有效值
            Voltage_Middle /= 1024;
            arm_sqrt_f32(Voltage_Middle,&Voltage_Val);
            Current_Middle /= 1024;
            arm_sqrt_f32(Current_Middle,&Current_Val);
            

            //计算有功功率
            for(i = 0;i < 1024;i++)
            {
                Power_Eff += (Voltage_Set[i] * Current_Set[i]);
            }
            if(Power_Eff < 0)
                Power_Eff = -Power_Eff;
            Power_Eff /= 1024;

            //计算视在功率
            Power = Voltage_Val * Current_Val;

            //计算功率因数
            Power_Per = Power_Eff / Power * 100;
            Power_Per = Power_Per + 200 / Power_Per;


            //Voltage_PerCount = FFT_Count(Voltage, FFT_Voltage, Voltage_Per);
            Current_PerCount = FFT_Count(Current_Set, FFT_Current, Current_per);
            //Per_Gap = Voltage_PerCount - Current_PerCount;
            for(i = 1;i <= 10;i++)
            {
                Current_Wave[i - 1] = FFT_Current[5 * i];
            }
            for(i = 1;i <= 9;i++)
            {
                Current_Wave_Inf_Middle += Current_Wave[i] * Current_Wave[i];
            }
            arm_sqrt_f32(Current_Wave_Inf_Middle, &Current_Wave_Inf);
            Current_Wave_Inf /= (Current_Wave[0] + Current_Wave_Inf);
           
         

            //对相位差进行卡尔曼滤波
            /*if(Per_Gap < 0)
                Per_Gap += 360;
            Per_Gap = kalmanFilter(&KFP_Per_Wave,Per_Gap);
            Per_Gap += 30;*/


            
            //对电压进行卡尔曼滤波
            Voltage_Val = kalmanFilter(&KFP_FFT_Wave,Voltage_Val);
            Voltage_Val=Voltage_Val;

            //对电流进行卡尔曼滤波
            Current_Val = kalmanFilter(&KFP_Current_Wave,Current_Val);
            Current_Val=Current_Val;

            //对功率滤波
            Power_Eff = kalmanFilter(&KFP_Power,Power_Eff);
            Power_Eff *= 1.023;

            //对功率因数滤波
            Power_Per = kalmanFilter(&KFP_PowerPer,Power_Per);
            if(Power_Per > 100)
                Power_Per = 100;

            //对谐波进行卡尔曼滤波
            Current_Wave[1] = kalmanFilter(&Wave_2,Current_Wave[1]);
            Current_Wave[2] = kalmanFilter(&Wave_3,Current_Wave[2]);
            Current_Wave[3] = kalmanFilter(&Wave_4,Current_Wave[3]);
            Current_Wave[4] = kalmanFilter(&Wave_5,Current_Wave[4]);
            Current_Wave[5] = kalmanFilter(&Wave_6,Current_Wave[5]);
            Current_Wave[6] = kalmanFilter(&Wave_7,Current_Wave[6]);
            Current_Wave[7] = kalmanFilter(&Wave_8,Current_Wave[7]);
            Current_Wave[8] = kalmanFilter(&Wave_9,Current_Wave[8]);
            Current_Wave[9] = kalmanFilter(&Wave_10,Current_Wave[9]);
        
            //计算功率因数
            //将数据输出 
            printf("x1.val=%d\xff\xff\xff",(int)(Voltage_Val * 100));
            printf("x0.val=%d\xff\xff\xff",(int)(Current_Val * 100));
            printf("x4.val=%d\xff\xff\xff",(int)(Current_Wave_Inf * 100));
            printf("x2.val=%d\xff\xff\xff",(int)Power_Eff);
            printf("x3.val=%d\xff\xff\xff",(int)(Power_Per));
            printf("x5.val=%d\xff\xff\xff",(int)(Current_Wave[1]));
            printf("x6.val=%d\xff\xff\xff",(int)(Current_Wave[2]));
            printf("x7.val=%d\xff\xff\xff",(int)(Current_Wave[3]));
            printf("x8.val=%d\xff\xff\xff",(int)(Current_Wave[4]));
            printf("x9.val=%d\xff\xff\xff",(int)(Current_Wave[5]));
            printf("x10.val=%d\xff\xff\xff",(int)(Current_Wave[6]));
            printf("x11.val=%d\xff\xff\xff",(int)(Current_Wave[7]));
            printf("x12.val=%d\xff\xff\xff",(int)(Current_Wave[8]));
            printf("x13.val=%d\xff\xff\xff",(int)(Current_Wave[9]));
            printf("");
            printf("");
            printf("");
            printf("");
            printf("");
            printf("");
            printf("");
           // printf("");

            Current_Wave_Inf = 0;
            Current_Wave_Inf_Middle = 0;
            Voltage_Avr = Current_Avr = 0;
            Power_Eff = 0;


               //将标志位置0之后，使能定时器开始工作
            ADC_Flag = 0;
            DL_TimerG_startCounter(TIMER_0_INST);
       }
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) 
    {
        //关注RX接收数据事件
        case DL_UART_MAIN_IIDX_RX:
            Mode_Set = DL_UART_Main_receiveData(UART_0_INST);
         break;
        default:
        break;
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    uint16_t i = 0;
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_DMA_DONE:
            for(i = 0;i < 1024;i++)
            {
                Voltage[i] = ((Voltage_DMA[i] * 3300 / 4096) * 0.365 );
                Current[i] = ((Current_DMA[i] * 3300 / 4096) * 3.570);
            }
            ADC_Flag = 1;
            break;
        default:
            break;
    }
}
