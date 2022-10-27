#include "stm32f10x.h"
#include "SysClock.h"
#include "gpio_driver.h"
#include "timer_driver.h"
#include "motor.h"
#include "follow_line.h"

//typedef struct
//{
//	Motor_TypeDef *left_motor;
//	Motor_TypeDef *right_motor;
//}CAR_TypeDef;

Motor_TypeDef left_motor;
Motor_TypeDef right_motor;

void TIM_IRQ_Callback(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM1){}
}

static int8_t error;
uint8_t	miss_way = 0;
/*-------------------------------------- CAR INIT-----------------------------------------*/
/*----------------------------------------------------------------------------------------*/


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
		if(error == 5)
		{
			motor_Control(&left_motor, MOTOR_STOP, 0);
			motor_Control(&right_motor, MOTOR_STOP, 0);
		}
		else if(error == -5)
		{
			if(miss_way == 0)
			{
				motor_Control(&left_motor, MOTOR_STOP, 0);
				motor_Control(&right_motor, MOTOR_STOP, 0);
			}
			if(miss_way == 1)
			{
				motor_Control(&left_motor, MOTOR_CW, BASE_SPEED);
				motor_Control(&right_motor, MOTOR_CW, 0);
			}
			if(miss_way == 2)
			{
				motor_Control(&left_motor, MOTOR_CW, 0);
				motor_Control(&right_motor, MOTOR_CW, BASE_SPEED);
			}
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

