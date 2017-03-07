//
//  kalman.c
//  kalman_test
//
//  Created by Dongcheng on 2017/1/6.
//  Copyright © 2017年 Dongcheng. All rights reserved.
//


#include "kalman.h"



void Kalman_Filter_Init(KalmanCountData * Kalman_Struct)
{
    Kalman_Struct -> Angle_err                 = 0;
    Kalman_Struct -> Q_bias                         = 0;
    Kalman_Struct -> PCt_0                         = 0;
    Kalman_Struct -> PCt_1                         = 0;
    Kalman_Struct -> E                                 = 0;
    Kalman_Struct -> K_0                         = 0;
    Kalman_Struct -> K_1                         = 0;
    Kalman_Struct -> t_0                         = 0;
    Kalman_Struct -> t_1                         = 0;
    Kalman_Struct -> Pdot[0]                 = 0;
    Kalman_Struct -> Pdot[1]                 = 0;
    Kalman_Struct -> Pdot[2]                 = 0;
    Kalman_Struct -> Pdot[3]                 = 0;
    Kalman_Struct -> PP[0][0]                 = 1;
    Kalman_Struct -> PP[0][1]                 = 0;
    Kalman_Struct -> PP[1][0]                 = 0;
    Kalman_Struct -> PP[1][1]                 = 1;
    Kalman_Struct -> Angle_Final         = 0;
    Kalman_Struct -> Gyro_Final                 = 0;
    
}


/**
 ******************************************************************************
 * @file    void Kalman_Filter(float Accel,        float Gyro ,KalmanCountData * Kalman_Struct)
 * @author  willieon
 * @version V0.1
 * @date    January-2015
 * @brief   卡尔曼滤波计算
 *
 *
 ******************************************************************************
 * @attention
 *                Accel:加速度计数据处理后进来的角度值
 *                Gyro :陀螺仪数据处理后进来的角速度值
 *                Kalman_Struct:递推运算所需要的中间变量，由用户定义为全局结构体变量
 *                Kalman_Struct -> Angle_Final  为滤波后角度最优值
 *                Kalman_Struct -> Gyro_Final   为后验角度值
 ******************************************************************************
 */

void Kalman_Filter(float Accel,        float Gyro ,KalmanCountData * Kalman_Struct)
{
    //陀螺仪积分角度（先验估计）
    Kalman_Struct -> Angle_Final += (Gyro - Kalman_Struct -> Q_bias) * dt;
    
    //先验估计误差协方差的微分
    Kalman_Struct -> Pdot[0] = Q_angle - Kalman_Struct -> PP[0][1] - Kalman_Struct -> PP[1][0];
    Kalman_Struct -> Pdot[1] = - Kalman_Struct -> PP[1][1];
    Kalman_Struct -> Pdot[2] = - Kalman_Struct -> PP[1][1];
    Kalman_Struct -> Pdot[3] = Q_gyro;
    
    //先验估计误差协方差的积分
    Kalman_Struct -> PP[0][0] += Kalman_Struct -> Pdot[0] * dt;
    Kalman_Struct -> PP[0][1] += Kalman_Struct -> Pdot[1] * dt;
    Kalman_Struct -> PP[1][0] += Kalman_Struct -> Pdot[2] * dt;
    Kalman_Struct -> PP[1][1] += Kalman_Struct -> Pdot[3] * dt;
    
    //计算角度偏差
    Kalman_Struct -> Angle_err = Accel - Kalman_Struct -> Angle_Final;
    
    //卡尔曼增益计算
    Kalman_Struct -> PCt_0 = C_0 * Kalman_Struct -> PP[0][0];
    Kalman_Struct -> PCt_1 = C_0 * Kalman_Struct -> PP[1][0];
    
    Kalman_Struct -> E = R_angle + C_0 * Kalman_Struct -> PCt_0;
    
    Kalman_Struct -> K_0 = Kalman_Struct -> PCt_0 / Kalman_Struct -> E;
    Kalman_Struct -> K_1 = Kalman_Struct -> PCt_1 / Kalman_Struct -> E;
    
    //后验估计误差协方差计算
    Kalman_Struct -> t_0 = Kalman_Struct -> PCt_0;
    Kalman_Struct -> t_1 = C_0 * Kalman_Struct -> PP[0][1];
    
    Kalman_Struct -> PP[0][0] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_0;
    Kalman_Struct -> PP[0][1] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_1;
    Kalman_Struct -> PP[1][0] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_0;
    Kalman_Struct -> PP[1][1] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_1;
    
    Kalman_Struct -> Angle_Final += Kalman_Struct -> K_0 * Kalman_Struct -> Angle_err;         //后验估计最优角度值
    Kalman_Struct -> Q_bias        += Kalman_Struct -> K_1 * Kalman_Struct -> Angle_err;                 //更新最优估计值的偏差
    Kalman_Struct -> Gyro_Final   = Gyro - Kalman_Struct -> Q_bias;                                                 //更新最优角速度值
    
}
