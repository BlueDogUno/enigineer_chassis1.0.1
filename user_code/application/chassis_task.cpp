#include "chassis_task.h"

#include "system_config.h" 

#include "chassis.h"



uint8_t chassis_flag = 0;

/**
  * @brief          chassis_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void chassis_task(void *pvParameters) {

    //空闲一段时间
    //vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis.init();
    //超级电容初始化
  

    // for (uint8_t i=0; i<10; i++){
    //     vTaskDelay(2);
       cap.init();
    // }

    while(true) {

      chassis_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

      //设置模式
      chassis.set_mode();
      //反馈数据
      chassis.feedback_update();
      //设置控制量
      chassis.set_contorl();
      //解算
      chassis.solve();
      //功率控制
      chassis.power_ctrl();
      //电流输出
      chassis.output();


      //系统延时
      vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
