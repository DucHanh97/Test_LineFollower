#include "stm32f10x.h"
#include "SysClock.h"
#include "gpio_driver.h"
#include "timer_driver.h"
#include "motor.h"

/*----- Define gains of PID controler -----*/
#define Kp					25
#define Ki					1
#define Kd					0.5
#define BASE_SPEED	30
/*-----------------------------------------*/

//typedef struct
//{
//	Motor_TypeDef *left_motor;
//	Motor_TypeDef *right_motor;
//}CAR_TypeDef;

/*------------ Prototype Functions ------------*/
int8_t read_sensor_error(void);
int16_t PID_value_calculate(int8_t error);
void car_following_line_handle(void);
/*---------------------------------------------*/

void TIM_IRQ_Callback(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM1){}
}

/*----------------- global arguments -----------------*/
//static CAR_TypeDef car; 
static Motor_TypeDef left_motor;
static Motor_TypeDef right_motor;

static int8_t		P = 0;
static int16_t	I = 0;
static int8_t		D = 0;
static int8_t		Pre_error = 0;
/*----------------------------------------------------*/

/*------------ */
int8_t read_sensor_error(void)
{
	/* 5 sensors with following states
		1 1 1 1 1 = 31: error = 5
		0 0 0 0 1	= 1: error = 4
		0 0 0 1 1 = 3: error = 3
		0 0 0 1 0 = 2: error = 2
		0 0 1 1 0 = 6: error = 1			- right
		0 0 1 0 0 = 4: error = 0			- center
		0 1 1 0 0 = 12: error = -1		- left
		0 1 0 0 0 = 8: error = -2
		1 1 0 0 0 = 24: error = -3
		1 0 0 0 0 = 16: error = -4
		0 0 0 0 0 = 0: error = -5
	*/
	int8_t error = 0;
	uint8_t sensor[5];

	sensor[0] = GPIO_Read_Pin(GPIOA, PIN_2);
	sensor[1] = GPIO_Read_Pin(GPIOA, PIN_3);
	sensor[2] = GPIO_Read_Pin(GPIOA, PIN_4);
	sensor[3] = GPIO_Read_Pin(GPIOA, PIN_5);
	sensor[4] = GPIO_Read_Pin(GPIOA, PIN_6);
	
	if((sensor[0] == 1) & (sensor[1] == 1) & (sensor[2] == 1) & (sensor[3] == 1) & (sensor[4] == 1))
	{
		error = 5;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 0) & (sensor[3] == 0) & (sensor[4] == 1))
	{
		error = 4;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 0) & (sensor[3] == 1) & (sensor[4] == 1))
	{
		error = 3;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 0) & (sensor[3] == 1) & (sensor[4] == 0))
	{
		error = 2;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 1) & (sensor[3] == 1) & (sensor[4] == 0))
	{
		error = 1;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 1) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = 0;
	}
	else if((sensor[0] == 0) & (sensor[1] == 1) & (sensor[2] == 1) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = -1;
	}
	else if((sensor[0] == 0) & (sensor[1] == 1) & (sensor[2] == 0) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = -2;
	}
	else if((sensor[0] == 1) & (sensor[1] == 1) & (sensor[2] == 0) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = -3;
	}
	else if((sensor[0] == 1) & (sensor[1] == 0) & (sensor[2] == 0) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = -4;
	}
	else if((sensor[0] == 0) & (sensor[1] == 0) & (sensor[2] == 0) & (sensor[3] == 0) & (sensor[4] == 0))
	{
		error = -5;
	}
	
	return error;
}
/*----------------------------------------------------------------------------------------*/

/*------------------------------------- PID Calculate ------------------------------------*/
int16_t PID_value_calculate(int8_t error)
{
	P = error;
	I += error;
	D = error - Pre_error;
	Pre_error = error;
	int16_t PID_value = (int16_t)(Kp * P + Ki * I + Kd * D);
	return PID_value;
}

/*----------------------------------------------------------------------------------------*/

/*---------------------------- Following line handle -------------------------------------*/
void car_following_line_handle(void)
{
	int8_t delta_pid_speed = (int8_t)(PID_value_calculate(read_sensor_error()) / 10);
	uint8_t right_speed = (uint8_t)(BASE_SPEED - delta_pid_speed);
	uint8_t left_speed = (uint8_t)(BASE_SPEED + delta_pid_speed);
	motor_Control(&left_motor, MOTOR_CW, left_speed);
	motor_Control(&right_motor, MOTOR_CW, right_speed);
}
/*----------------------------------------------------------------------------------------*/

/*-------------------------------------- CAR INIT-----------------------------------------*/
/*----------------------------------------------------------------------------------------*/

int8_t error;
//uint8_t sensor[5];

int main(void)
{
	SysClockConfig();
	
	GPIO_Config(GPIOA, PIN_2, INPUT, IN_PP);
	GPIO_Config(GPIOA, PIN_3, INPUT, IN_PP);
	GPIO_Config(GPIOA, PIN_4, INPUT, IN_PP);
	GPIO_Config(GPIOA, PIN_5, INPUT, IN_PP);
	GPIO_Config(GPIOA, PIN_6, INPUT, IN_PP);
	
	motor_Init(&right_motor, GPIOB, PIN_14, TIM1, CH1);
	motor_Init(&left_motor, GPIOB, PIN_15, TIM1, CH2);
	TIM_PWM_Config(right_motor.timer, right_motor.Tim_Channel, 72, 1000);
	TIM_PWM_Config(left_motor.timer, left_motor.Tim_Channel, 72, 1000);
	GPIO_Config(GPIOA, PIN_8, OUT50, O_AF_PP);
	GPIO_Config(GPIOB, PIN_14, OUT50, O_GP_PP);
	GPIO_Config(GPIOA, PIN_9, OUT50, O_AF_PP);
	GPIO_Config(GPIOB, PIN_15, OUT50, O_GP_PP);
	
	motor_Control(&left_motor, MOTOR_STOP, 22);
	motor_Control(&right_motor, MOTOR_STOP, 38);	
	
	while(1)
	{
		error = read_sensor_error();
		if((error == 5) | (error == -5))
		{
			motor_Control(&left_motor, MOTOR_STOP, 0);
			motor_Control(&right_motor, MOTOR_STOP, 0);
		}
		else
		{
		car_following_line_handle();
		}

//		sensor[0] = GPIO_Read_Pin(GPIOA, PIN_2);
//		sensor[1] = GPIO_Read_Pin(GPIOA, PIN_3);
//		sensor[2] = GPIO_Read_Pin(GPIOA, PIN_4);
//		sensor[3] = GPIO_Read_Pin(GPIOA, PIN_5);
//		sensor[4] = GPIO_Read_Pin(GPIOA, PIN_6);
	}
}

