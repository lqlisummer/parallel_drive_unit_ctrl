/*
 * set_control_world.c
 * 功能： 设置电机校验
 * @author  xufuliang
 * @date    2025-08-27
 */
#include <stdio.h>
#include "RobotControl.h"
#include "CTypes.h"
#ifdef _WIN32
#include <windows.h>
#endif 
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
 /*   int id = robot_motor_get_motor_id(ctx);
    printf("获取电机ID:%d\n", id);*/
    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 13, 2);
    //设置电机执行器校准
    int return_value = robot_motor_set_control_world(motor, CTRL_CALIBRATE_ENCODER);
    if (return_value)
    {
        printf("执行器校准 成功\n");
    }
    else
    {

        printf("执行器校准 失败\n");
    }

  
    //释放电机对象
    robot_destroy_motor(ctx,motor);
#ifdef _WIN32
    Sleep(30000);
#else
    sleep(30);
#endif

    motor = robot_create_motor(ctx, 13, 2);
    //设置电机执行器校准
    return_value = robot_motor_set_control_world(motor, CTRL_CALIBRATE_ENCODER);
    if (return_value)
    {
        printf("执行器校准 成功\n");
    }
    else
    {

        printf("执行器校准 失败\n");
    }


    //释放电机对象
    robot_destroy_motor(ctx,motor);
    //释放主板对象
    robot_destroy(ctx);

    return 0;

}