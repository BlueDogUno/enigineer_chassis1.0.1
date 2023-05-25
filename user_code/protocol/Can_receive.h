#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan1
#define BOARD_COM_CAN hcan2

//底盘动力电机编号
enum motive_chassis_motor_id_e
{
  //底盘动力电机接收
  MOTIVE_FR_MOTOR = 0,
  MOTIVE_FL_MOTOR,
  MOTIVE_BL_MOTOR,
  MOTIVE_BR_MOTOR,
  //抬升电机数据接收
  HIGH_LEFT_MOTOR = 0,
  HIGH_RIGHT_MOTOR = 1,
};

/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //抬升电机ID CAN2
  CAN_HIGH_LEFT_MOTOR_ID = 0x205,
  CAN_HIGH_RIGHT_MOTOR_ID = 0x206,
  
  CAN_CHASSIS_HIGH_ALL_ID = 0x1FF,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x301,
  CAN_GIMBAL_BOARD_COM_ID = 0x302,
  CAN_COOLING_BOARM_COM_ID = 0x303,
  CAN_42MM_SPEED_BOARD_COM_ID = 0x304,
  CAN_UI_COM_ID = 0x305,

  //超级电容接收ID
  CAN_SUPER_CAP_ID = 0x211

} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int16_t round;
} motor_measure_t;

// TODO 超电还未对接
//  //rm motor data
//  typedef struct
//  {
//    fp32 input_vot;
//    fp32 supercap_vot;
//    fp32 input_current;
//    fp32 target_power;
//  } super_cap_measure_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_0;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32 gimbal_yaw_angle;

  // UI状态
  fp32 gimbal_pitch_angle;
  bool_t auto_state;
  bool_t aim_state;
  bool_t fric_state;
} chassis_receive_t;

//底盘发送数据结构体
typedef struct
{
  //测试热量及ID
  uint16_t id1_42mm_cooling_limit; // 42mm测速热量上限
  uint16_t id1_42mm_cooling_rate;  // 42mm测速热量冷却
  uint16_t id1_42mm_cooling_heat;  // 42mm测速实时热量
  uint8_t color;                   //判断红蓝方
  uint8_t robot_id;                //机器人编号

  //测速速度及底盘模式
  uint16_t id1_42mm_speed_limi; // 42mm测速射速上限
  uint16_t bullet_speed;        // 42mm测速实时射速

  uint8_t chassis_behaviour;

  uint8_t game_progress;

} chassis_send_t;

typedef struct
{
  float input_vot;     //输入电压
  float cap_vot;       //超级电容电压
  float input_current; //输入电流
  float target_power;  //目标功率
} cap_receive_t;

class Can_receive
{
public:
  //动力电机反馈数据结构体
  motor_measure_t chassis_motive_motor[4];
  motor_measure_t chassis_high_motor[2];

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  //板间通信
  //底盘接收信息
  chassis_receive_t chassis_receive;

  chassis_send_t chassis_send;

  //超电数据
  cap_receive_t cap_receive;

  void init();

  //电机数据接收
  void get_motive_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //动力电机数据

  void can_cmd_chassis_motive_motor_reset_ID();

  const motor_measure_t *get_chassis_motive_motor_measure_point(uint8_t i);

  //板间通信函数
  void receive_rc_board_com(uint8_t data[8]);

  void receive_gimbal_board_com(uint8_t data[8]);

  void receive_ui_board_com(uint8_t data[8]);

  // 发送枪口热量及ID
  void send_cooling_and_id_board_com(uint16_t id1_42mm_cooling_limit, uint16_t id1_42mm_cooling_rate, uint16_t id1_42mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口速度及底盘模式
  void send_42mm_speed_and_mode_board_com(uint16_t id1_42mm_speed_limi, uint16_t bullet_speed, uint8_t chassis_behaviour, uint8_t temp_game_progress);

  //发送超级电容设定功率
  void can_cmd_super_cap_power(uint16_t set_power);

  // 获取超电输入电压、电容电压、输入电流、设定功率
  void get_super_cap_data(uint8_t data[8]);

  const motor_measure_t *get_chassis_high_motor_measure_point(uint8_t i);

  void can_cmd_chassis_high_motor(int16_t motor1, int16_t motor2);

  void get_high_motor_measure(uint8_t num, uint8_t data[8]);
};

#endif
