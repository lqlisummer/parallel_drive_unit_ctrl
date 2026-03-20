/*
 * get_strong_dragging_model.c
 * 功能：获取电机当前强拖模式
 * @author  xufuliang
 * @date    2025-08-27
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
    RobotMotor* motor = robot_create_motor(ctx, 12, 1);

    unsigned int model = -1;
    //获取强拖模式
    if (robot_motor_get_StrongDragging(motor,  &model))
    {
    
        printf("get_StrongDragging %d\n", model);
    }
    else 
    {
        printf("get_StrongDragging %d\n", model);
    }

    //释放电机对象
    robot_destroy_motor(ctx, motor);
    //释放主板对象
    robot_destroy(ctx);
    return 0;


}