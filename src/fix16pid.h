#ifndef FIXED_PID_CONTROLLER_H
#define FIXED_PID_CONTROLLER_H

#include "fix16.h"

#define PID_KP 	F16(0.2)
#define PID_KI	F16(0.01)
#define PID_KD	F16(0.001)
#define PID_TAU	F16C(0,0)
#define PID_LIM_MIN F16C(0,0) 
#define PID_LIM_MAX F16C(329,0)
#define PID_LIM_MIN_INT F16C(-160,0)
#define PID_LIM_MAX_INT	F16C(160,0)	
#define SAMPLE_TIME_S F16C(1,0) //ADC timer period


typedef struct {

	// Controller gains 
	fix16_t Kp;
	fix16_t Ki;
	fix16_t Kd;

	// Derivative low-pass filter time constant
	fix16_t tau;

	// Output limits
	fix16_t limMin;
	fix16_t limMax;
	
	// Integrator limits
	fix16_t limMinInt;
	fix16_t limMaxInt;

	// Sample time (in seconds)
	fix16_t interval;
    fix16_t setpoint;
    fix16_t measurement;

	// Controller "memory"
    fix16_t error;
    fix16_t proportional;
	fix16_t integrator;
	fix16_t differentiator;
	fix16_t prevError;              // Required for integrator   
	fix16_t prevMeasurement;		// Required for differentiator 

	// Controller output 
	fix16_t out;

} PIDController;

void  PIDController_Init(PIDController *pid);
fix16_t PIDController_Update(PIDController *pid, fix16_t setpoint, fix16_t measurement, fix16_t interval);

#endif
