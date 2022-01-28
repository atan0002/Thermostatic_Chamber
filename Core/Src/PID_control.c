/*
 * PID_control.c
 *
 *  Created on: Jan 5, 2022
 *      Author: Natalia Wiśniewska
 */

#include "PID_control.h"
float calculate_discrete_pid(pid_d *pid, float setpoint, float measured){


	error = setpoint-measured;

	//proportional part
	P = pid->p.Kp * error;

	//integral part
	integral = pid->previous_integral + (error+pid->previous_error) ;
	pid->previous_integral = integral;
	I = pid->p.Ki*integral*(pid->p.dt/2.0);

	//derivative part
	derivative = (error - pid->previous_error)/pid->p.dt;
	pid->previous_error = error;
	D = pid->p.Kd*derivative;

	//sum of all parts
	u = P  + I + D;

	return u;
}

//here is function which calculate width
int CalculateWidth(float u_control){

if(u_control<0){
	width=0;
}
else{
	float temp;
	temp=(u_control*u_control*MaxWidth)/(230*230);
	temp=round(temp);
	width=(int)temp;
	//here is saturation
		if(width<CriticLowerThreshold){
			width=0;
		}
		else if(width>CriticHigherThreshold){
				width=MaxWidth;
		}
}
return width;
}
