#include "high_task.h"

#include "system_config.h"

#include "high.h"



/**
  * @brief          chassis_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void high_task(void *pvParameters) 
{



  //空闲一段时间
  //vTaskDelay(HIGH_TASK_INIT_TIME);
  top.init();

  while (true)
  {

    //反馈数据
    top.feedback_update();

    //救援电机控制
    top.save_task();

    //抬升模式改变
    top.lift_set_mode();

    //抬升赋值
    top.lift_control();

    //电流输出
    top.output();

    //系统延时
    vTaskDelay(HIGH_CONTROL_TIME_MS);
  }
  
}
