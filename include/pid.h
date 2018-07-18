# pragma once  //防止多次包含
 
#include <iostream>
#include <cmath>
using namespace std;

#define dt 0.05

/*
 *PID参数计算
 *
 */
float PIDcal(float setpoint,float actual_position,float Kp,float Ki,float Kd,float epsilon,float MAX,float MIN); //PID函数