#include "high.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif


High high;

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

void High::init()
{
    high_RC = remote_control.get_remote_control_point();
    last_high_RC = remote_control.get_last_remote_control_point();

    for (uint8_t i = 0; i < 4; ++i)
    {

        //动力电机数据
        high_motive_motor[i].init(can_receive.get_chassis_high_motor_measure_point(i));
        //初始化pid
        fp32 high_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        high_motive_motor[i].speed_pid.init(PID_SPEED, high_speed_pid_parm, &high_motive_motor[i].speed, &high_motive_motor[i].speed_set, NULL);
        high_motive_motor[i].speed_pid.pid_clear();

        fp32 high_angle_pid_parm[5] = {MOTIVE_MOTOR_ANGLE_PID_KP, MOTIVE_MOTOR_ANGLE_PID_KI, MOTIVE_MOTOR_ANGLE_PID_KD, MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT, MOTIVE_MOTOR_ANGLE_PID_MAX_OUT};
        high_motive_motor[i].angle_pid.init(PID_ANGLE, high_angle_pid_parm, &high_motive_motor[i].total_angle , &high_motive_motor[i].angle_set, 0);
        high_motive_motor[i].angle_pid.pid_clear();
        high_motive_motor[i].angle_set = 0;
        high_motive_motor[i].speed_set= 0;
    }

    for (uint8_t i = 0; i < 4; ++i)
    {
        //设置初始值
        stretch_moto_start_angle[i] = high_motive_motor[i].total_angle;
        high_motive_motor[i].angle_error = high_motive_motor[i].total_angle - high_motive_motor[i].angle_set;
        motor_status[i] = WAIT;
    }

    // 电机软件限位，需要测试后开启
    //更新一下数据
    feedback_update();
}

/**
 * @brief          状态更新函数
 * @param[in]
 * @retval         none
 */
void High::feedback_update(){
    //记录上一次遥控器值
    high_last_key_v = high_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度，加速度是速度的PID微分
        high_motive_motor[i].speed = HIGH_MOTOR_RPM_TO_VECTOR_SEN * high_motive_motor[i].motor_measure->speed_rpm;
        high_motive_motor[i].total_angle = high_motive_motor[i].motor_measure->total_angle;
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (high_motive_motor[i].angle_error < ANGLE_ERR_TOLERANT && high_motive_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
            motor_status[i] = READY;
        else
            motor_status[i] = WAIT;
    }
    // 这两个变量暂时没有用到，目的是为了伸出一半还能收回
    
}

/**
 * @brief          行为切换设置
 * @param[in]
 * @retval         none
 */
void High::set_mode(){
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"high_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void High::behaviour_mode_set()
{
    last_high_behaviour_mode = high_behaviour_mode;
    last_high_mode = high_mode;

    //遥控器设置模式
    if (switch_is_up(high_RC->rc.s[HIGH_MODE_CHANNEL])) //右拨杆上
    {
        high_behaviour_mode = HIGH_ZERO_FORCE;
    }
    else if (switch_is_mid(high_RC->rc.s[HIGH_MODE_CHANNEL])) //右拨杆中
    {
        high_behaviour_mode = HIGH_OPEN;
    }
    else if (switch_is_down(high_RC->rc.s[HIGH_MODE_CHANNEL])) //右拨杆下
    {
        high_behaviour_mode = HIGH_CLOSE;
    }

    //根据行为模式选择一个控制模式
    if (high_behaviour_mode == HIGH_ZERO_FORCE || high_behaviour_mode == HIGH_OPEN)
    {
        high_mode = HIGH_HAND;
    }
    else if(high_behaviour_mode == HIGH_CLOSE)
    {
        high_mode = HIGH_AUTO;
    }
}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void High::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制抬升电机和伸爪电机
    //vhigh_set控制抬升电机速度，vstretch_set控制伸爪电机速度
    fp32 vhigh_set = 0.0f, vstretch_set = 0.0f;
    fp32 angle_set = 0;

    //获取控制设置值
    behaviour_control_set(&vhigh_set, &vstretch_set);

    if (high_mode == HIGH_HAND)
    {
        high_motive_motor[HIGH_LIFT_LEFT_ID].angle_set += vhigh_set;
        high_motive_motor[HIGH_LIFT_RIGHT_ID].angle_set += -vhigh_set;
        high_motive_motor[HIGH_STRETCH_L_ID].angle_set += vstretch_set;
        high_motive_motor[HIGH_STRETCH_R_ID].angle_set += vstretch_set;
        for (int i = 0;i < 4;i++)
        {
            // motor_angle_limit(&high_motive_motor[i]);
        }
    }
    
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vhigh_set, 通常控制纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void High::behaviour_control_set(fp32 *vhigh_set, fp32 *vstretch_set)
{

    if (vhigh_set == NULL || vstretch_set == NULL)
    {
        return;
    }
    //无力
    if (high_behaviour_mode == HIGH_ZERO_FORCE)
    {
        *vhigh_set = 0.0f;
        *vstretch_set = 0.0f;
    }
    else if (high_behaviour_mode == HIGH_OPEN)
    {
        high_open_set_control(vhigh_set, vstretch_set);
    }

    last_high_RC->key.v = high_RC->key.v;
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void High::high_open_set_control(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    static int16_t high_channel = 0, stretch_channel = 0;

    rc_deadband_limit(high_RC->rc.ch[HIGH_X_CHANNEL], high_channel, RC_DEADBAND);
    rc_deadband_limit(high_RC->rc.ch[HIGH_Y_CHANNEL], stretch_channel, RC_DEADBAND);

    *vx_set = high_RC->rc.ch[HIGH_X_CHANNEL] / HIGH_OPEN_RC_SCALE;
    *vy_set = -high_RC->rc.ch[HIGH_Y_CHANNEL] / HIGH_OPEN_RC_SCALE;

}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void High::solve()
{

    if (high_behaviour_mode == HIGH_OPEN)
    {

        for (int i = 0; i < 4; i++)
        {
            high_motive_motor[i].speed_set = high_motive_motor[i].angle_pid.pid_calc();
            high_motive_motor[i].current_give = high_motive_motor[i].speed_pid.pid_calc();
            // motor_set_control(&high_motive_motor[i]);
            
        }
        // raw控制直接返回
        return;
    }
    else if (high_behaviour_mode == HIGH_CLOSE)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_set_control(&high_motive_motor[i]);
        }
        //TODO:其实这里需要把两个2006只用速度环，伸出电机继续使用双环
    }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void High::output()
{
    if (high_behaviour_mode == HIGH_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            high_motive_motor[i].current_give = 0.0f;
        }
    }
    can_receive.can_cmd_chassis_high_motor(high_motive_motor[HIGH_LIFT_LEFT_ID].current_give, high_motive_motor[HIGH_LIFT_RIGHT_ID].current_give,
                                          high_motive_motor[HIGH_STRETCH_L_ID].current_give, high_motive_motor[HIGH_STRETCH_R_ID].current_give);
}

/**
 * @brief          双环pid计算
 * @param[out]     
 * @retval         none
 */

void High::motor_set_control(M3508_motor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed_set = motor->angle_pid.pid_calc();
    motor->current_give = motor->speed_pid.pid_calc();
    
}
void High::motor_angle_limit(M3508_motor *motor)
{
    if (motor->total_angle < motor->min_angle)
    {
        motor->total_angle = motor->min_angle;
    }
    else if (motor->total_angle > motor->max_angle)
    {
        motor->total_angle = motor->max_angle;
    }
}