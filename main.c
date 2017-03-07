
#include "kalman.h"
#include "stdio.h"
#include "stdlib.h"
#include "pid.h"

int main(void)

{
    KalmanCountData k;
    
    PID PIDControlStruct;
    
    Kalman_Filter_Init(&k); //初始化卡尔曼滤波
    
    PIDInit(50, 1, 0.04, 0.2, &PIDControlStruct);
   
    int m,n;
    
    double pwm, out;
    
    for(int a = 0;a<80;a++)
    {
        m = 1+ rand() %100;
        n = 1+ rand() %100;
        Kalman_Filter((float)m,(float)n,&k);
        out = PIDCalc(k.Angle_Final, &PIDControlStruct);
        
        pwm = out;
        
        if(pwm>=0)
        {
            printf("角速度：%3d and 偏转量%3d 卡尔曼滤波得到角度： %6f -正向pwm- %6f\r\n",m,n,k.Angle_Final,pwm);
        }
        else
        {
            printf("角速度：%3d and 偏转量%3d 卡尔曼滤波得到角度： %6f -反方向pwm- %6f\r\n",m,n,k.Angle_Final,pwm);
        }
        
        
    }
 
    return 0;
    
}
