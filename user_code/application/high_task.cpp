#include "high_task.h"

#include "system_config.h" 

#include "high.h"
#include "Communicate.h"

/**
  * @brief          high_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void high_task(void *pvParameters) 
{
    //空闲一段时间
    vTaskDelay(HIGH_TASK_INIT_TIME);
    high.init();


    while(true) 
    { 
        //设置模式
        high.set_mode();

        //反馈数据
        high.feedback_update();

        //设置控制量
        high.set_control();

        //解算
        high.solve();

        //电流输出
        high.output();
        //系统延时
        vTaskDelay(HIGH_TASK_INIT_TIME);
    }
}

