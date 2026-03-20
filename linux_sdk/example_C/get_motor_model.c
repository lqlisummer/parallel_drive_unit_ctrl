/*
 * get_motor_model.c
 * 功能：获取电机型号，设置电机为转速模式，使能电机，使用无感（开环）模式开启强拖，设置转速为 2
 * @author  xufuliang
 * @date    2025-09-18
 */

#include <stdio.h>
#include "RobotControl.h"
#include "CTypes.h"
int main(int argc, char* argv[]) {
    //生成主板对象
    RobotCtx* ctx = robot_create(0xFD);
    if (!ctx) return -1;
    //连接主板
    int value = robot_config_net(ctx, "192.168.3.245", 15021, 14999, "192.168.3.11");
    if (value)
    {
        printf("连接主板成功\n");
    }
    else
    {
        printf("连接主板失败\n");
    }
    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 123, 2);


    char buf[128] = { '\0' };
   
    //设置电机转速模式
   value = robot_motor_get_motor_model(motor, buf);

   if (value)
   {
       printf("set_control_mode success \n");
           
   }
   else
   {
       printf("set_control_mode failed \n");
   }
  
    printf("model:%s\n", buf);
  
    //释放电机对象
    robot_destroy_motor(ctx, motor);
    //释放主板对象
    robot_destroy(ctx);
    return 0;


}