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
  LIFT_LEFT_MOTOR = 0,
  LIFT_RIGHT_MOTOR,
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
  CAN_SS_BOARD_COM_ID = 0x302,
  CAN_SEND_LIFT_AUTO_COM_ID = 0x303,
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

  fp32 total_angle;
  fp32 offset_angle;
  fp32 last_total_angle;
  fp32 angle_err;

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
  uint8_t s0;
  uint8_t s1;

  // UI状态
  bool_t stretch_state;
  bool_t yaw_state;
  bool_t roll_state;
  bool_t flip_state; 

  //自动抬升模式
  uint8_t lift_auto_mode;

} chassis_receive_t;

//底盘发送数据结构体
typedef struct
{
  uint8_t game_progress;
}chassis_send_t;

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
  motor_measure_t chassis_high_motor[4];

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

  void receive_ui_board_com(uint8_t data[8]);

  void receive_ss_board_com(uint8_t data[8]);

  void send_lift_auto_state(bool_t lift_state);


  // 获取超电输入电压、电容电压、输入电流、设定功率
  void get_super_cap_data(uint8_t data[8]);

  const motor_measure_t *get_chassis_high_motor_measure_point(uint8_t i);

  void can_cmd_chassis_high_motor(int16_t motor1, int16_t motor2);

  void get_high_motor_measure(uint8_t num, uint8_t data[8]);
};

#endif
