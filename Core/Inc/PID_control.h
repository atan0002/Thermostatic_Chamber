/*
 * PID_control.h
 *
 *  Created on: Jan 5, 2022
 *      Author: Natalia Wiśniewska
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// parameters for 1 s of heater work
// lower and higher threshold are physical limitations of SSR
#define CriticLowerThreshold 100
#define CriticHigherThreshold 9900
#define MaxWidth 10000 // value of ARR(Counter Period)
#define PWM_DUTY_CYCLE 1000 //1000 ms


typedef float float32_t;

typedef struct{
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;
    float32_t dt;
}pid_parameters_t;

typedef struct{
    pid_parameters_t p;
    float32_t previous_error, previous_integral;
} pid_d;


int width;

float start_u;
float u, P, I, D, error, integral, derivative;
float diff;


float calculate_discrete_pid(pid_d *pid, float setpoint, float measured);
int CalculateWidth(float u_control);
#endif /* INC_PID_CONTROL_H_ */
