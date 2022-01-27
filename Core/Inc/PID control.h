/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
 * https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45
 * https://www.inflearn.com/course/stm32cubelde-stm32f4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif


typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;
	
	double reference;
	double meas_value;
	double meas_value_prev;
	double meas_rate;
	double meas_rate_prev;

	float error;
	float error_prev;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;
	
	float error_sum_max;
	float error_sum_min;

	float p_result;
	float i_result;
	float d_result;
	
	float pid_result;
	float integral_anti_windup;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;

extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDDouble altitude;
extern PIDDouble lat;
extern PIDDouble lon;
extern PIDDouble yaw_heading;
extern PIDDouble yaw_rate;

void Double_PID_Calculation(PIDDouble* axis, float set_point, float value, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter, int is_yaw);
void Double_PID_Calculation_Rate(PIDDouble* axis, float set_point, float value, float rate, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter, int is_yaw);
void Single_PID_Calculation(PIDDouble* axis, float set_point, float value, float out_error_sum_max, int out_filter, int is_yaw);
void Single_PID_Calculation_Rate(PIDDouble* axis, float set_point, float angle, float rate, float out_error_sum_max, int out_filter, int is_yaw);
void Double_GPS_PID_Calculation(PIDDouble* axis, double set_point, double value, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter);

void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
