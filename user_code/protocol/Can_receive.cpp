#include "can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_motive_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_motive_motor[num].last_ecd = chassis_motive_motor[num].ecd;
    chassis_motive_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_motive_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_motive_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_motive_motor[num].temperate = data[6];
}

void Can_receive::get_high_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_high_motor[num].last_ecd = chassis_high_motor[num].ecd;
    chassis_high_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_high_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_high_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_high_motor[num].temperate = data[6];
    if(chassis_high_motor[num].ecd - chassis_high_motor[num].last_ecd > 5000)
    {
        chassis_high_motor[num].round--;
    }
    if(chassis_high_motor[num].ecd - chassis_high_motor[num].last_ecd < -5000)
    {
        chassis_high_motor[num].round++;
    }
    chassis_high_motor[num].total_angle = chassis_high_motor[num].round * 8192 + chassis_high_motor[num].ecd - chassis_high_motor[num].offset_angle;
    chassis_high_motor[num].angle_err = chassis_high_motor[num].last_total_angle - chassis_high_motor[num].total_angle;

}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_MOTIVE_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::can_cmd_chassis_high_motor(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_HIGH_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void Can_receive::can_cmd_chassis_motive_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          返回底盘动力电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_chassis_motive_motor_measure_point(uint8_t i)
{
    return &chassis_motive_motor[i];
}

const motor_measure_t *Can_receive::get_chassis_high_motor_measure_point(uint8_t i)
{
    return &chassis_high_motor[i];
}

void Can_receive::receive_rc_board_com(uint8_t data[8])
{
    chassis_receive.ch_0 = (int16_t)(data[0] << 8 | data[1]);
    chassis_receive.ch_2 = (int16_t)(data[2] << 8 | data[3]);
    chassis_receive.ch_3 = (int16_t)(data[4] << 8 | data[5]);
    chassis_receive.v = (uint16_t)(data[6] << 8 | data[7]);
}

void Can_receive::receive_ss_board_com(uint8_t data[8])
{
    chassis_receive.s0 = data[0];
    chassis_receive.s1 = data[1];
}

void Can_receive::receive_ui_board_com(uint8_t data[8])
{
    chassis_receive.stretch_state = data[0];
    chassis_receive.yaw_state = data[1];
    chassis_receive.roll_state = data[2];
    chassis_receive.flip_state = data[3];
    chassis_receive.v = (uint16_t)(data[5] << 8 | data[6]);
}

void Can_receive::send_lift_auto_state(bool_t lift_state){
    
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_SEND_LIFT_AUTO_COM_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = lift_state;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}