#ifndef CTYPES_H
#define CTYPES_H




typedef enum {

    CTRL_NONE = 0x0000,

    CTRL_SERVO_OFF = 0xF001,    // 失能
    CTRL_SERVO_ON = 0xF002,     // 使能
    CTRL_CLEAR_FAULT = 0xF086,  // 清除错误

    // 传感器相关
    CTRL_CALIBRATE_ENCODER = 0x101F,  // 编码器校准
    CTRL_RETURN_ZERO = 0x102F,        // 执行器自动回零
    CTRL_POSITION_SET_ZERO = 0x104F,  // 执行器位置置零

    // 控制相关
    CTRL_FRICTION_IDENTIFY = 0x201F,  // 摩擦校准
    CTRL_COGGING_IDENTIFY = 0x202F,   // 齿槽转矩校准

    // 功能开关 0x3xxxF
    CTRL_FRICTION_COMP_OFF = 0x300F,       // 失能摩擦补偿
    CTRL_FRICTION_COMP_ON = 0x301F,        // 使能摩擦补偿
    CTRL_COGGING_COMP_OFF = 0x302F,        // 失能齿槽转矩补偿
    CTRL_COGGING_COMP_ON = 0x303F,         // 使能齿槽转矩补偿
    CTRL_TLOAD_COMP_OFF = 0x304F,          // 失能负载补偿
    CTRL_TLOAD_COMP_ON = 0x305F,           // 使能负载补偿
    CTRL_SOFT_POS_LIMIT_OFF = 0x306F,      // 失能软限位
    CTRL_SOFT_POS_LIMIT_ON = 0x307F,       // 使能软限位
    CTRL_OVER_TEMP_PROTECT_OFF = 0x308F,   // 失能过温保护
    CTRL_OVER_TEMP_PROTECT_ON = 0x309F,    // 使能过温保护
    CTRL_HEART_BEAT_PROTECT_OFF = 0x30AF,  // 失能心跳保护
    CTRL_HEART_BEAT_PROTECT_ON = 0x30BF,   // 使能心跳保护

} ControlWord_e;

// 电机控制模式
typedef enum {

    MOTOR_CTRL_MODE_NONE = 0,         // 无控制模式
    MOTOR_CTRL_MODE_CURRENT = 1,      // 电流控制模式
    MOTOR_CTRL_MODE_VELOCITY = 2,     // 速度控制模式
    MOTOR_CTRL_MODE_POSITION = 3,     // 位置控制模式
    MOTOR_CTRL_MODE_PD = 4,           // 转矩位置控制模式
    MOTOR_CTRL_MODE_BRAKE = 5,        // 制动模式
    MOTOR_CTRL_MODE_OPENLOOP = 0xff,  // 开环控制模式

} MotorCtrlMode_e;

// 电机状态码
typedef enum {

    IDLE_WM = 0,                  // 空闲状态
    Init_WM = 1,                  // 初始化状态
    Normal_WM = 2,                // 正常控制状态
    Fault_WM = 3,                 // 异常状态
    ENCODER_CAIL_WM = 4,          // 特殊状态 电机编码器校准
    LINER_HALL_CAIL_WM = 5,       // 特殊状态 电机编码器校准
    OUTPUT_ENCODER_CAIL_WM = 6,   // 特殊状态 出轴编码器校准
    ACTUATOR_RETURN_ZERO_WM = 7,  // 特殊状态 执行器回零模式
    FRICTION_IDENTIFY_WM = 8,     // 特殊状态 摩擦辨识模式

}UserWorkMode_e;

//通信控制指令
typedef enum{
  
    ConfigNetWork = 0x600,        //网络通信参数设置
    SetId,                        //设置电机ID
    ControlMode,                  //控制模式
    ControlWord,                  //控制字
    WorkMode,                     //工作模式
    FastMode,                     //快速上报模式
    Reboot,                       //软件重启                     
    ControlSetPos,                //指令位置
    ControlSetVel,                //指令速度
    ControlSetTor,                //指令力矩
    ControlSetCur,                //指令电流
    ControlSetPID,                //指令PID
    ControlSetPD,                 //指令PD
    GetMotorPVCT,                 //反馈PVCT
    ErrCode,                      //反馈错误码
    Inverter,                     //反馈逆变器状态

    SetDirction,                  //指令方向
    SetOutputShaftRatio,          //指令减速比
    SetPosThreshold,              //指令位置限制
    SetVCTMax,                    //指令最大VCT
    SetOutputOffset,              //指令出轴传感器位置偏置
    SetOutEnable,                 //指令输出轴编码器使能
    SetTorToCur,                  //指令转矩到电流多项式系数

    AcBaseParams,                 //执行器基础参数集合
    SendCalibrationData,	      //发送校准数据到SDK
    SetFocThetaSorce,			  //设置角度源
    SetM1Voltage				  //设置强拖电压
        
}Command_Types;

/*角度获取模式*/
typedef enum {
    e_ForceTheta = 0,    // 强制角度标志
    e_SensorLess = 1,    // 无感运行标志
    e_Sensor = 2,        // 有感运行标志
    e_SensorFusion = 3,  // 传感器融合角度
    e_ThetaSorceAll,
} ThetaSorce_e;
#endif // TYPES_H