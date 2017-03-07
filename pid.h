//
//  pid.h
//  kalman_test
//
//  Created by Dongcheng on 2017/1/6.
//  Copyright © 2017年 Dongcheng. All rights reserved.
//

#ifndef pid_h
#define pid_h

#include <stdio.h>


#define IF_THE_INTEGRAL_SEPARATION  0
//#define IF_THE_INTEGRAL_SEPARATION  1   //是否积分分离  0-不分离，1 -分离

typedef struct
{
    double SetPoint; // 设定目标 Desired Value
    double Proportion; // 比例常数 Proportional Const
    double Integral; // 积分常数 Integral Const
    double Derivative; // 微分常数 Derivative Const
    double LastError; // Error[-1]
    double PrevError; // Error[-2]
    double SumError; // Sums of Errors  
}PID;

#if IF_THE_INTEGRAL_SEPARATION            //是否积分分离预编译开始

double PIDCalc(double NextPoint ,double SepLimit, PID *pp);   //带积分分离的PID运算

#else

double PIDCalc( double NextPoint, PID *pp);     //不带积分分离的PID运算

#endif        //是否积分分离预编译结束

void PIDInit (double SetPoint, double Proportion, double Integral, double Derivative, PID *pp);

#endif








