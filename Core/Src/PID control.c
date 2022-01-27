//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

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

#include "PID control.h"

PIDDouble roll;
PIDDouble pitch;

PIDDouble yaw_heading;
PIDDouble yaw_rate;

PIDDouble altitude;

PIDDouble lat;
PIDDouble lon;

#define DT 0.002f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

void Double_PID_Calculation(PIDDouble* axis, float set_point, float value, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter, int is_yaw)
{
	/*********** Double PID Outer Begin *************/
	axis->out.reference = set_point;	//Set point of outer PID control
	axis->out.meas_value = value;			//Current Value
	//error + P output
	axis->out.error = axis->out.reference - axis->out.meas_value;
	if(is_yaw == 1)
	{
		if(axis->out.error > 180.f) axis->out.error -= 360.f;
		else if(axis->out.error < -180.f) axis->out.error += 360.f;
	}

	axis->out.p_result = axis->out.error * axis->out.kp;
	axis->out.error_sum += axis->out.error * DT;
	// i output
	axis->out.error_sum_max = out_error_sum_max;
	axis->out.error_sum_min = -out_error_sum_max;
	if(axis->out.error_sum > axis->out.error_sum_max) axis->out.error_sum = axis->out.error_sum_max;
	else if(axis->out.error_sum < axis->out.error_sum_min) axis->out.error_sum = axis->out.error_sum_min;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;
	//d output
	axis->out.error_deriv = -(axis->out.meas_value - axis->out.meas_value_prev) / DT;
	axis->out.meas_value_prev = axis->out.meas_value;
	if(out_filter == 0) axis->out.d_result = axis->out.error_deriv * axis->out.kd;
	else
	{
		axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.3f + axis->out.error_deriv * 0.7f;
		axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;
	}
	//result
	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;
	/*********** Double PID Outer End *************/

	/*********** Double PID Inner Begin *************/
	axis->in.reference = axis->out.pid_result;
	axis->in.meas_value = -(axis->out.error_deriv);

	axis->in.error = axis->in.reference - axis->in.meas_value;
	axis->in.p_result = axis->in.error * axis->in.kp;
	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;

	axis->in.error_sum_max = in_error_sum_max;
	axis->in.error_sum_min = -in_error_sum_max;
	if(axis->in.error_sum > axis->in.error_sum_max) axis->in.error_sum = axis->in.error_sum_max;
	else if(axis->in.error_sum < axis->in.error_sum_min) axis->in.error_sum = axis->in.error_sum_min;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;
	axis->in.meas_value_prev = axis->in.meas_value;

	if(in_filter == 0) axis->in.d_result = axis->in.error_deriv * axis->in.kd;
	else
	{
		axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;
		axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;
	}

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result;
	/*********** Double PID Inner End *************/
}

void Double_PID_Calculation_Rate(PIDDouble* axis, float set_point, float value, float rate, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter, int is_yaw)
{
	/*********** Double PID Outer Begin *************/
	axis->out.reference = set_point;
	axis->out.meas_value = value;

	axis->out.error = axis->out.reference - axis->out.meas_value;
	axis->out.p_result = axis->out.error * axis->out.kp;
	axis->out.error_sum += axis->out.error * DT;

	axis->out.error_sum_max = out_error_sum_max;
	axis->out.error_sum_min = -out_error_sum_max;
	if(axis->out.error_sum > axis->out.error_sum_max) axis->out.error_sum = axis->out.error_sum_max;
	else if(axis->out.error_sum < axis->out.error_sum_min) axis->out.error_sum = axis->out.error_sum_min;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;

	axis->out.error_deriv = -rate;
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;
	/*********** Double PID Outer End *************/

	/*********** Double PID Inner Begin *************/
	axis->in.reference = axis->out.pid_result;
	axis->in.meas_value = rate;

	axis->in.error = axis->in.reference - axis->in.meas_value;
	axis->in.p_result = axis->in.error * axis->in.kp;
	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;

	axis->in.error_sum_max = in_error_sum_max;
	axis->in.error_sum_min = -in_error_sum_max;
	if(axis->in.error_sum > axis->in.error_sum_max) axis->in.error_sum = axis->in.error_sum_max;
	else if(axis->in.error_sum < axis->in.error_sum_min) axis->in.error_sum = axis->in.error_sum_min;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;
	axis->in.meas_value_prev = axis->in.meas_value;

	if(in_filter == 0) axis->in.d_result = axis->in.error_deriv * axis->in.kd;
	else
	{
		axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;
		axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;
	}

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result;
	/*********** Double PID Inner End *************/
}

void Single_PID_Calculation(PIDDouble* axis, float set_point, float value, float out_error_sum_max, int out_filter, int is_yaw)
{
	/*********** Double PID Outer Begin *************/
	axis->out.reference = set_point;	//Set point of outer PID control
	axis->out.meas_value = value;			//Current Value
	//error + P output
	axis->out.error = axis->out.reference - axis->out.meas_value;
	if(is_yaw == 1)
	{
		if(axis->out.error > 180.f) axis->out.error -= 360.f;
		else if(axis->out.error < -180.f) axis->out.error += 360.f;
	}
	axis->out.p_result = axis->out.error * axis->out.kp;
	axis->out.error_sum += axis->out.error * DT;
	// i output
	axis->out.error_sum_max = out_error_sum_max;
	axis->out.error_sum_min = -out_error_sum_max;
	if(axis->out.error_sum > axis->out.error_sum_max) axis->out.error_sum = axis->out.error_sum_max;
	else if(axis->out.error_sum < axis->out.error_sum_min) axis->out.error_sum = axis->out.error_sum_min;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;
	//d output
	axis->out.error_deriv = -(axis->out.meas_value - axis->out.meas_value_prev) / DT;
	axis->out.meas_value_prev = axis->out.meas_value;
	if(out_filter == 0) axis->out.d_result = axis->out.error_deriv * axis->out.kd;
	else
	{
		axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.3f + axis->out.error_deriv * 0.7f;
		axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;
	}
	//result
	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;
	/*********** Double PID Outer End *************/
}

void Single_PID_Calculation_Rate(PIDDouble* axis, float set_point, float value, float rate, float out_error_sum_max, int out_filter, int is_yaw)
{
	/*********** Double PID Outer Begin *************/
		axis->out.reference = set_point;
		axis->out.meas_value = value;

		axis->out.error = axis->out.reference - axis->out.meas_value;
		axis->out.p_result = axis->out.error * axis->out.kp;
		axis->out.error_sum += axis->out.error * DT;

		axis->out.error_sum_max = out_error_sum_max;
		axis->out.error_sum_min = -out_error_sum_max;
		if(axis->out.error_sum > axis->out.error_sum_max) axis->out.error_sum = axis->out.error_sum_max;
		else if(axis->out.error_sum < axis->out.error_sum_min) axis->out.error_sum = axis->out.error_sum_min;
		axis->out.i_result = axis->out.error_sum * axis->out.ki;

		axis->out.error_deriv = -rate;
		axis->out.d_result = axis->out.error_deriv * axis->out.kd;

		axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;
		/*********** Double PID Outer End *************/
}

void Double_GPS_PID_Calculation(PIDDouble* axis, double set_point, double value, float out_error_sum_max, float in_error_sum_max, int out_filter, int in_filter)
{
	/*********** Double PID Outer Begin *************/
	axis->out.reference = set_point;	//Set point of outer PID control
	axis->out.meas_value = value;			//Current Value
	//error + P output
	axis->out.error = axis->out.reference - axis->out.meas_value;
	axis->out.p_result = axis->out.error * axis->out.kp;
	axis->out.error_sum += axis->out.error * DT;
	// i output
	axis->out.error_sum_max = out_error_sum_max;
	axis->out.error_sum_min = -out_error_sum_max;
	if(axis->out.error_sum > axis->out.error_sum_max) axis->out.error_sum = axis->out.error_sum_max;
	else if(axis->out.error_sum < axis->out.error_sum_min) axis->out.error_sum = axis->out.error_sum_min;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;
	//d output
	axis->out.error_deriv = -(axis->out.meas_value - axis->out.meas_value_prev) / DT;
	axis->out.meas_value_prev = axis->out.meas_value;
	if(out_filter == 0) axis->out.d_result = axis->out.error_deriv * axis->out.kd;
	else
	{
		axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.3f + axis->out.error_deriv * 0.7f;
		axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;
	}
	//result
	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;
	/*********** Double PID Outer End *************/

	/*********** Double PID Inner Begin *************/
	axis->in.reference = axis->out.pid_result;
	axis->in.meas_value = -(axis->out.error_deriv);

	axis->in.error = axis->in.reference - axis->in.meas_value;
	axis->in.p_result = axis->in.error * axis->in.kp;
	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;

	axis->in.error_sum_max = in_error_sum_max;
	axis->in.error_sum_min = -in_error_sum_max;
	if(axis->in.error_sum > axis->in.error_sum_max) axis->in.error_sum = axis->in.error_sum_max;
	else if(axis->in.error_sum < axis->in.error_sum_min) axis->in.error_sum = axis->in.error_sum_min;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;
	axis->in.meas_value_prev = axis->in.meas_value;

	if(in_filter == 0) axis->in.d_result = axis->in.error_deriv * axis->in.kd;
	else
	{
		axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;
		axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;
	}

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result;
	/*********** Double PID Inner End *************/
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading.in);
	Reset_PID_Integrator(&yaw_heading.out);
	Reset_PID_Integrator(&yaw_rate.out);

	Reset_PID_Integrator(&altitude.in);
	Reset_PID_Integrator(&altitude.out);

	Reset_PID_Integrator(&lat.in);
	Reset_PID_Integrator(&lat.out);

	Reset_PID_Integrator(&lon.in);
	Reset_PID_Integrator(&lon.out);
}



