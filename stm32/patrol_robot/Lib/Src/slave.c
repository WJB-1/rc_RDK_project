//
// Created by lml on 2026/3/26.
//

#include "slave.h"

slave slave_0 = {0,0,0,0,0,0};
slave slave_1 = {0,0,0,0,0,0};

void slave_Init()
{
	//打开PWM通道
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	//归中
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
}

//舵机参数复位归零
void reset_slave(slave *Slave)
{
	Slave->PWM = 0;
	Slave->status = 0;
	Slave->work_ccr = 0;
	Slave->work_times = 0;
	Slave->work = 0;
	Slave->crr_value = 0;
}

//控制单个舵机，无阻塞，输入时间单位为毫秒
void cloud_control_one(int ID,float angle,float use_time)
{
	switch (ID)
	{
	case 0:
		if (slave_0.status == 0)
		{
			slave_0.work_times = (int)roundf(use_time / 10.0f);
			slave_0.crr_value = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
			slave_0.PWM = (10.0f * angle / 270.0f + 2.5f) / 100.0f * 10000.0f;
			slave_0.work_ccr = (int)roundf((slave_0.PWM - (float)slave_0.crr_value) / (float)slave_0.work_times);
			if (slave_0.work_ccr)
			{
				slave_0.status = 1;
			}
		}
		break;
	case 1:
		if (slave_1.status == 0)
		{
			slave_1.work_times = (int)roundf(use_time / 10.0f);
			slave_1.crr_value = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
			slave_1.PWM = (10.0f * angle / 180.0f + 2.5f) / 100.0f * 10000.0f;
			slave_1.work_ccr = (int)roundf((slave_1.PWM - (float)slave_1.crr_value) / (float)slave_1.work_times);
			if (slave_1.work_ccr)
			{
				slave_1.status = 1;
			}
		}
		break;
	}
	if (slave_0.status || slave_1.status)
	{
		if ((htim3.Instance->DIER & TIM_IT_UPDATE) == 0) {
			//开启定时器中断
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
}
