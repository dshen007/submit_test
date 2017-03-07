//
//  pid.c
//  kalman_test
//
//  Created by Dongcheng on 2017/1/6.
//  Copyright © 2017年 Dongcheng. All rights reserved.
//

#include "pid.h"
#include "math.h"
#if IF_THE_INTEGRAL_SEPARATION

double PIDCalc(double NextPoint ,double SepLimit, PID *pp)
{
    double dError, Error,Flag;
    Error = pp->SetPoint - NextPoint;         // 偏差
    if(abs(Error) > SepLimit)        //当偏差大于分离上限积分分离
    {
        Flag = 0;
    }
    else       //当偏差小于分离上限，积分项不分离
    {
        Flag = 1;
        pp->SumError += Error;         // 积分
    }
    dError = pp->LastError - pp->PrevError;         // 当前微分
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
    return (
            pp->Proportion                *                Error                 // 比例项
            + Flag * pp->Integral        *                pp->SumError         // 积分项
            + pp->Derivative                *                dError                 // 微分项
            );
}

#else

double PIDCalc( double NextPoint, PID *pp)
{
    double dError, Error;
    Error = pp->SetPoint - NextPoint;                         // 偏差
    pp->SumError += Error;                                        // 积分
    dError = pp->LastError - pp->PrevError;                // 当前微分
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
    return (pp->Proportion        *        Error                // 比例项
            + pp->Integral                *        pp->SumError         // 积分项
            + pp->Derivative        *        dError         // 微分项
            );
}

#endif





void PIDInit (double SetPoint, double Proportion, double Integral, double Derivative, PID *pp)
{
    pp -> SetPoint = SetPoint; // 设定目标 Desired Value
    pp -> Proportion = Proportion; // 比例常数 Proportional Const
    pp -> Integral = Integral; // 积分常数 Integral Const
    pp -> Derivative = Derivative; // 微分常数 Derivative Const
    pp -> LastError = 0; // Error[-1]
    pp -> PrevError = 0; // Error[-2]
    pp -> SumError = 0; // Sums of Errors
    
    //memset ( pp,0,sizeof(struct PID));   //need include "string.h"
}


