#include "pid.h"

float PIDcal(float setpoint,float actual_position,float Kp,float Ki,float Kd,float epsilon,float MAX,float MIN)
{
	static float pre_error = 0;         // 定义前一次误差
	static float integral = 0;          // 定义积分项累加值
	float error;                        // 定义最新误差
	float derivative;                   // 定义微分项
	float output;                       // 定义输出
 
	// 计算误差
	error = setpoint - actual_position;
 
	// 死区设置，在误差极小的情况下不做积分处理，保证稳定性
	if(fabs(error) > epsilon)
	{
	   integral = integral + error*dt;   // 积分项,做完壁障之后，积分清零
	}
 
	derivative = (error - pre_error)/dt; // 微分项
 
	output = Kp*error + Ki*integral + Kd*derivative; // 各项乘以系数得到PID输出
    float sign;
    if(output != 0)
    {
        sign=output/fabs(output);//求出输出的正负号
    }
	// 输出限幅，确保输出不会太夸张
	if(fabs(output) > MAX)
	{
		output = sign*MAX;
	}
	else if(fabs(output) < MIN)
	{
		output = 0;
	}
 
        // 更新误差
        pre_error = error;
 
        return output;
}
