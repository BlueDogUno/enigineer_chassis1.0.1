#ifndef HIGH_H
#define HIGH_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "tim.h"

#define HIGH_DEBUG_FLAG 0

//拨矿电机方向
#define HIGH_UPLOAD_MOTOR_TURN 1
//伸爪电机方向
#define HIGH_STRETCH_MOTOR_TURN 1

//任务控制间隔 2ms
#define HIGH_CONTROL_TIME_MS 2

//前后的遥控器通道号码
#define HIGH_X_CHANNEL 3

#define HIGH_OPEN_RC_SCALE 0.1 // 遥控器乘以该比例发送到can上

//选择取矿机构状态 开关通道号
#define HIGH_MODE_CHANNEL 1
//选择取矿机构状态 开关通道号
#define STRETCH_MODE_CHANNEL 0

#define ANGLE_ERR_TOLERANT 2000


//拨矿电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 1000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 0.1f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//拨矿电机角度环PID
#define MOTIVE_MOTOR_ANGLE_PID_KP 1000.0f 
#define MOTIVE_MOTOR_ANGLE_PID_KI 0.0f
#define MOTIVE_MOTOR_ANGLE_PID_KD 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_OUT 6000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_motor_RPM_TO_VECTOR 0.000415809748903494517209f
#define HIGH_MOTOR_RPM_TO_VECTOR_SEN M3508_motor_RPM_TO_VECTOR
#define HIGH_MOTOR_RPM_TO_VECTOR_SEN M3508_motor_RPM_TO_VECTOR

// 各电机角度限幅
#define LIFT_LIMIT_ANGLE 30000.0f

#define MOTOR_SPEED_TO_HIGH_SPEED 0.25f

//拨矿过程最大速度
#define NORMAL_MAX_HIGH_SPEED 1.0f //2.0
//伸爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 4.0f //2.0

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0

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

// 用于电机ID
typedef enum
{
    CAN_LIFT_L_MOTOR = 0,
    CAN_LIFT_R_MOTOR,
};                    

// 用于自动模式下的电机控制
typedef enum
{
    SPIN_MOTOR = 0,
    YAW_MOTOR,
    SUCTION_MOTOR,
    MOTOR_NUM,
};  

typedef enum
{
    HIGH_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    HIGH_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

    HIGH_CLOSE,                       //全自动，操作手没有权限控制

} high_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    HIGH_AUTO,

    HIGH_HAND,      //用了自动模式的都说好

} high_mode_e;      //控制模式

typedef enum
{
    READY,

    WAIT,    

} motor_status;

class High 
{
public:
    const RC_ctrl_t *high_RC; //抓取机构使用的遥控器指针
    RC_ctrl_t *last_high_RC; //抓取机构使用的遥控器指针

    uint16_t high_last_key_v;  //遥控器上次按键

    high_behaviour_e high_behaviour_mode; //抓取机构行为状态机
    high_behaviour_e last_high_behaviour_mode; //抓取机构上次行为状态机

    high_mode_e high_mode; //抓取机构控制状态机
    high_mode_e last_high_mode; //抓取机构上次控制状态机

    M3508_motor high_motive_motor[2];
    int32_t moto_start_angle[2];
    bool_t lift_state;

    uint8_t motor_status[2];
    
    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();

    //void auto_control(auto_mode_e *auto_mode);

    // void motor_angle_limit(M3508_motor *motor);
    //行为模式
    void behaviour_control_set(fp32 *vhigh_set);

    void high_open_set_control(fp32 *vx_set);

    void motor_set_control(M3508_motor *motor);

    void solve();

    void output();

    int8_t save_state;

    void save_init();

    void save_rc_control();

    void save_control_send();

};


extern High high;


typedef enum{
    ZROE_FORCE=0,
    ONE,
    TWO,
    THREE,
}save_state;


#define ONE_POSITION        (high.save_state == ONE)
#define TWO_POSITION        (high.save_state == TWO)
#define THREE_POSITION          (high.save_state == THREE)

#endif