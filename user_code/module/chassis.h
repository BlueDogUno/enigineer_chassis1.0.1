#ifndef CHASSIS_H
#define CHASSIS_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Super_cap.h"

#include "Config.h"

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

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 0

//初始yaw轴角度
#define INIT_YAW_SET  0.0f

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 1

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.46f //英雄为0.22+0.24=0.46  官方步兵为0.2

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

/*----------------按键-------------------------*/

//底盘前后左右控制按键
#define KEY_CHASSIS_FRONT           if_key_pessed(chassis_RC, KEY_PRESSED_CHASSIS_FRONT)
#define KEY_CHASSIS_BACK            if_key_pessed(chassis_RC, KEY_PRESSED_CHASSIS_BACK)
#define KEY_CHASSIS_LEFT            if_key_pessed(chassis_RC, KEY_PRESSED_CHASSIS_LEFT)
#define KEY_CHASSIS_RIGHT           if_key_pessed(chassis_RC, KEY_PRESSED_CHASSIS_RIGHT)

#define KEY_CHASSIS_TOP             if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_TOP)
#define KEY_CHASSIS_SWING           if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_SWING)
#define KEY_CHASSIS_PISA            if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_PISA)
#define KEY_CHASSIS_SUPER_CAP       if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_SUPER_CAP)
#define KEY_UI_UPDATE               if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_UI_UPDATE)

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f //4
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f //2.0
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //1.5
//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 8.0f

//原地旋转小陀螺下Z轴转速
#define TOP_WZ_ANGLE_STAND 2.5f
//移动状态下小陀螺转速
#define TOP_WZ_ANGLE_MOVE 1.5f


#define CHASSIS_WZ_SET_SCALE 0.1f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f //0.7
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f


//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

#define MISS_CLOSE 0
#define MISS_BEGIN 1
#define MISS_OVER 2



#define PISA_DELAY_TIME 500
#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上



//chassis motor speed PID
//底盘电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 6000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f



//功率控制参数
#define POWER_DEFAULT_LIMIT 50.0f  //默认功率限制
#define WARNING_POWER_DISTANCE 10.0f //距离超过率的距离
#define WARNING_POWER_BUFF 30.0f   
 //警告能量缓冲  通过计算超级电容 电压低于12v得到的值

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 48000.0f   // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f     //16000
#define POWER_TOTAL_CURRENT_LIMIT 12000.0f      //20000

typedef enum
{
    CHASSIS_ZERO_FORCE,                  //底盘表现为无力,底盘电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    CHASSIS_NO_MOVE,                     //底盘表现为不动，但推动底盘存在抵抗力,底盘电机速度控制值为0,应用于遥控器开关处下位，需要底盘停止运动的场合

    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常底盘跟随云台,底盘移动速度由遥控器和键盘按键一起决定，同时会控制底盘跟随云台，从而计算旋转速度,应用于遥控器开关处于上位

    CHASSIS_NO_FOLLOW_YAW,               //底盘移动速度和旋转速度均由遥控器决定,应用于只需要底盘控制的场合

    CHASSIS_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

                                         
} chassis_behaviour_e;                   //行为模式

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  //底盘跟随云台,底盘移动速度由遥控器和键盘决定,旋转速度由云台角度差计算出CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW 选择的控制模式

    CHASSIS_VECTOR_NO_FOLLOW_YAW,      //底盘不跟随云台,底盘移动速度和旋转速度由遥控器决定，无角度环控制CHASSIS_NO_FOLLOW_YAW 和 CHASSIS_NO_MOVE 选择的控制模式*/

    CHASSIS_VECTOR_RAW,                //底盘不跟随云台.底盘电机电流控制值是直接由遥控器通道值计算出来的，将直接发送到 CAN 总线上CHASSIS_OPEN 和 CHASSIS_ZERO_FORCE 选择的控制模式*/


} chassis_mode_e;                      //控制模式


struct speed_t
{
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;
    fp32 min_speed;
};



class Chassis {
public:
    const RC_ctrl_t *chassis_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_chassis_RC; //底盘使用的遥控器指针

    uint16_t chassis_last_key_v;  //遥控器上次按键

    chassis_behaviour_e chassis_behaviour_mode; //底盘行为状态机
    chassis_behaviour_e last_chassis_behaviour_mode; //底盘上次行为状态机

    chassis_mode_e chassis_mode; //底盘控制状态机
    chassis_mode_e last_chassis_mode; //底盘上次控制状态机

    M3508_motor chassis_motive_motor[4]; //底盘动力电机数据

    First_order_filter chassis_cmd_slow_set_vx;        //使用一阶低通滤波减缓设定值
    First_order_filter chassis_cmd_slow_set_vy;        //使用一阶低通滤波减缓设定值

    Pid chassis_wz_angle_pid;        //底盘角度pid

    speed_t x;
    speed_t y;
    speed_t z;

    fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
    fp32 chassis_relative_angle_set; //设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 chassis_yaw;   //底盘的yaw角度
    fp32 chassis_pitch; //底盘的pitch角度
    fp32 chassis_roll;  //底盘的roll角度


    //任务流程
    void init();

    void set_mode();

    void feedback_update();

    void set_contorl();

    void solve();

    void power_ctrl();

    void output();

    //行为控制

    void chassis_behaviour_mode_set();

    void chassis_behaviour_control_set(fp32 *vx_set_, fp32 *vy_set_, fp32 *angle_set);

    void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set);

    void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);

    void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);

    void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    //功能性函数
    void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set);

    void chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4]);

    fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
};


extern Chassis chassis;


//超电模块
extern Super_Cap cap;

#endif