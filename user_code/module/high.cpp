#include "high.h"
#include "tim.h"
#include "remote_control.h"
#include "Communicate.h"
#include "Motor.h"
#include "Can_receive.h"
//还要接受上板自动模式的数据 或者直接在这里计算
TOP top;


void TOP::init()
{
    //获取遥控器指针
    top_RC = remote_control.get_remote_control_point();
    last_top_RC = remote_control.get_last_remote_control_point();

    top_last_key_v = 0;
    fp32 lift_left_pid_parm[5] = {LIFT_LEFT_KP, LIFT_LEFT_KI, LIFT_LEFT_KD, LIFT_LEFT_MAX_IOUT, LIFT_LEFT_MAX_OUT};
    fp32 lift_right_pid_parm[5] = {LIFT_RIGHT_KP, LIFT_RIGHT_KI, LIFT_RIGHT_KD, LIFT_RIGHT_MAX_IOUT, LIFT_RIGHT_MAX_OUT};
    chassis_high_motor[0].speed_pid.init(PID_SPEED, lift_left_pid_parm, &chassis_high_motor[0].speed, &chassis_high_motor[0].speed_set, NULL);
    chassis_high_motor[1].speed_pid.init(PID_SPEED, lift_right_pid_parm, &chassis_high_motor[1].speed, &chassis_high_motor[1].speed_set, NULL);

    //初始化抬升电机
    for (uint8_t i = 0; i < 2; ++i)
    {
       //抬升电机数据更新
        chassis_high_motor[i].init(can_receive.get_chassis_high_motor_measure_point(i));//在Can_receive里面加上返回电机数据的函数
        //初始化pid
        
        chassis_high_motor[i].speed_pid.pid_clear();
    }

    // const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    // const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜波函数生成  ?
    // chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    // chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    top.lift_state = stop;
    top.save_data = stop;
    lift_lenth = 0.0f;
    //更新一下数据
    feedback_update();
}



void TOP::feedback_update()
{
    //记录上一次遥控器值
    // last_chassis_RC->key.v = chassis_RC->key.v;
    top_last_key_v = top_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 2; ++i)
    {
        //更新抬升电机速度
        chassis_high_motor[i].speed = CHASSIS_HIGH_RPM_TO_VECTOR_SEN * chassis_high_motor[i].motor_measure->speed_rpm;
        // chassis_high_motor[i].accel = *chassis_motive_motor[i].speed_pid.data.error_delta * CHASSIS_CONTROL_FREQUENCE;
    }
    lift_lenth = 1.0*(chassis_high_motor[0].motor_measure->round*360)/19+1.0*(chassis_high_motor[0].motor_measure->ecd*360)/19/8192; 
    //需要在Can_revice里面加上记圈，Motor结构体里面加上round变量

}

//--------------------------------救援-----------------------------------------------


void TOP::save_task(){
			if (left_switch_is_down)
			{
				if (right_rocker_down)
				{
          top.save_data = 0;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 970);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2030);
					//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);
				}
				else if(right_rocker_up)
				{
          top.save_data = 1;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
					//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_SET);
				}
			}
			//else if(top_RC->mouse.z > 0&&top_RC->key.v == KEY_PRESSED_OFFSET_F)
			//{
				//save_data = 1;
			//}else if(top_RC->mouse.z < 0&&top_RC->key.v == KEY_PRESSED_OFFSET_F)
			//{
				//save_data = 0;
			//}
			
			
			// if(top.save_data == 0)
			// {
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
			// }else
			// {
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1030);//1450
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1980);//2300
			// }
}

//-----------------------------------------------------------------------------------
//--------------------------------抬升-----------------------------------------------
// int16_t *lift_Reset_key;

// 键盘模式值
// int8_t lift_keyboard = 0;

/************函数开始*************/
// 这里使用了define判断的方式让条件更加易懂，其中switch为遥控器的左右三档开关，rocker为左右的拨杆
void TOP::lift_set_mode()
{

        //遥控器控制
        if(right_switch_is_down)//右拨杆向下     
        {
            // if (left_rocker_up)
            // {
            //     top.lift_state = up;
            // }
            // else if(left_rocker_down)
            // {
            //     top.lift_state = down;
            // }
            // else
            // {
            //     top.lift_state = stop;
            // }
            top.lift_state = hand;
        }else if(KEY_TOP_Z){ //键盘控制
            if (top_RC->mouse.y < 0)
            {
                top.lift_state = up;
            }
            if(top_RC->mouse.y > 0)
            {
                top.lift_state = down;
            }
            if(top_RC->mouse.y == 0)
            {
                top.lift_state = stop;
            }
        }else {
            top.lift_state = stop;
        }

        //电控限位
        if(top.lift_lenth > lift_down && state_is_down) //电控限位
        {
            top.lift_state = stop;
        }   
        if(top.lift_lenth < lift_up && state_is_up)
        {
            top.lift_state = stop;
        }

}

void TOP::lift_control()
{   

    fp32 vx_set = 0;
    chassis_high_motor[0].speed_set = 0;
    chassis_high_motor[1].speed_set = 0;
    //改变chassis_high_motor[i].speed_set的值
    // if (state_is_stop)
    // {
    //     // strt.can.lift.left_target   =   -13 * 19;
    //     // strt.can.lift.right_target  =   13 * 19;
    //     chassis_high_motor[0].speed_set = 0;
    //     chassis_high_motor[1].speed_set = 0;
    // }
    // if (state_is_up)
    // {
    //     // strt.can.lift.left_target   =   -60 * 19;
    //     // strt.can.lift.right_target  =   60 * 19;
    //     chassis_high_motor[0].speed_set = -100;
    //     chassis_high_motor[1].speed_set = 100;
    // }

    // if (state_is_down)
    // {
    //     // strt.can.lift.left_target   =   10*19;//40 * 19;
    //     // strt.can.lift.right_target  =   -10*19;//-40 * 19;
    //     chassis_high_motor[0].speed_set = 100;
    //     chassis_high_motor[1].speed_set = -100;
    // }
    if(top.lift_state == hand){
        vx_set = top_RC->rc.ch[3] / 10;
        chassis_high_motor[0].speed_set = vx_set;
        chassis_high_motor[1].speed_set = -1*vx_set;
    }

    for (int i = 0; i < 2; i++)
    {
        //计算抬升电机的输出电流
        chassis_high_motor[i].current_set = chassis_high_motor[i].speed_pid.pid_calc();
    }
}




void TOP::output()
{
    //赋值电流值
    for (int i = 0; i < 2; i++)
    {
        chassis_high_motor[i].current_give = (int16_t)(chassis_high_motor[i].current_set);
    }

    //电流输出控制,通过调整宏定义控制
    can_receive.can_cmd_chassis_high_motor(chassis_high_motor[0].current_give, chassis_high_motor[1].current_give);
}
//-----------------------------------------------------------------------------------