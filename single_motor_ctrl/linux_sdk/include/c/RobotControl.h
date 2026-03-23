#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#ifdef _WIN32
#  define EXPORT_API  __declspec(dllexport)
#else
#  define EXPORT_API  __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif


    EXPORT_API typedef struct RobotCtx RobotCtx;
    EXPORT_API typedef struct RobotMotor RobotMotor;

    /**
    * @brief 一键固件升级进度回调函数
    * @param num 进度值
    * @param error 错误码 1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return void
    */
    typedef void (*progress_callback)(int num, int error);

    /**
    * @brief 初始化主板对象
    * @param dev_id  主板ID
    * @return 主板对象，失败返回nullptr
    */
    EXPORT_API  RobotCtx* robot_create(unsigned short dev_id);
    
    /**
    * @brief 释放主板对象
    * @param ctx  主板对象
    * @return void
    */
    EXPORT_API void  robot_destroy(RobotCtx* ctx);

    /**
    * @brief 连接主板
    * @param ctx  主板对象
    * @param local_ip  本地ip
    * @param local_port 本地端口 
    * @param remote_port  主板端口
    * @param remote_ip  主板ip
    * @return 连接是否成功，1 为成功，0为失败 ,-1 主板对象为nullptr
    */
    EXPORT_API int robot_config_net(RobotCtx* ctx,
        const char* local_ip,
        int         local_port,
        int         remote_port,
        const char* remote_ip);



    /**
   * @brief 设置主板快速模式参数
   * @param ctx  主板对象
   * @param enable  快速模式是否开启 1 开启 0关闭
   * @param period_ms  快速模式上报频率
   * @return  1 设置成功 0 设置失败 ,-1 主板对象为nullptr
   */
    EXPORT_API  int robot_set_fast_mode(RobotCtx* ctx, int enable, int period_ms);

   
    /**
    * @brief 创建单电机对象
    * @param ctx           主板对象
    * @param canId         canId
    * @param motor         返回一个电机对象
    * @param canLineId     can线Id
    * @return RobotMotor 类型指针
    */

    EXPORT_API RobotMotor* robot_create_motor(RobotCtx* ctx, unsigned short canId, unsigned short canLineId);

 
    /**
    * @brief 设置电机控制模式
    * @param motor 电机对象
    * @param idx 控制模式索引值 详见Types.h 文件 枚举 MotorCtrlMode_e
    * @return 1 设置成功 0 设置失败
    */
    EXPORT_API int robot_motor_set_control_mode(RobotMotor* motor, unsigned int idx);


    /**
    * @brief 设置电机控制字
    * @param motor 电机对象
    * @param idx 模式索引值 详见Types.h 文件 枚举 UserWorkMode_e
    * @return 1 设置成功 0 设置失败
    */
    EXPORT_API int robot_motor_set_control_world(RobotMotor* motor, unsigned int idx);


    /**
     * @brief 设置位置、速度和电流控制参数
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    EXPORT_API void robot_motor_set_pos(RobotMotor* motor, float Pos, float Vel, float Cur);


    /**
    * @brief 设置位置控制参数
    * @param Pos 位置设定值 单位:rad
    * @return void
    */
    EXPORT_API void  robot_motor_set_pose(RobotMotor* motor, float Pos);

    /**
     * @brief  根据电机列表组大包发送
     * @param ctx 主板对象
     * @return void
     */

    EXPORT_API void robot_motor_set_big_pose(RobotCtx* ctx );


    /**
    * @brief 获取PVCTFAST 位置、速度和电流控制参数
    * @param Pos 位置设定值 单位:rad
    * @param Vel 速度设定值 单位:rad/s
    * @param Cur 电流设定值 单位:A
    * @return void
    */

    EXPORT_API void robot_motor_get_PVCTFast(RobotMotor* motor, float* Pos, float* Vel, float* Cur);



    /**
     *@brief 获取PVCT 位置、速度和电流控制参数
     *@param Pos 存储位置值的引用  单位:rad
     *@param Vel 存储速度值的引用  单位:rad/s
     *@param Cur 存储电流值的引用  单位:A
     *@param Tor_l 存储负载转矩值的引用  单位: N.m
     *@param Tor_e 存储电磁转矩值的引用  单位: N.m
     *@return void
     */

    EXPORT_API void robot_motor_get_PVCT(RobotMotor* motor, float* Pos, float* Vel, float* Cur, float* Tor_l, float* Tor_e);


    /**
     *@brief 获取编码器值
     *@param motor        电机对象
     *@param EncoderValue 编码器值
     *@return void
    */

    EXPORT_API void robot_motor_get_EncoderValue(RobotMotor* motor, int* EncoderValue);


   


    /**
   * @brief 设置强拖
   * @param motor       电机对象
   * @param Anglesource 设置强拖模式，见CTypes.h 枚举ThetaSorce_e
   * @param Voltage     电压
   * @return  1 返回成功，0 返回失败
   */

    EXPORT_API int robot_motor_set_StrongDragging(RobotMotor* motor, unsigned int Anglesource,float Voltage);



    /**
    * @brief 获取电机强拖模式
    * @param motor       电机对象
    * @param Anglesource 获取强拖模式，见CTypes.h 枚举ThetaSorce_e
    * @return  1 返回成功，0 返回失败
    */

    EXPORT_API int robot_motor_get_StrongDragging(RobotMotor* motor, unsigned int* Anglesource);




    /**
    * @brief  设置电机ID
    * @param ctx 主板对象
    * @param Id  电机ID
    * @return  1 返回成功，0 返回失败， -1 主板对象为nullptr
    */

    EXPORT_API int  robot_motor_set_motor_id(RobotCtx* ctx,unsigned short Id);



    /**
    * @brief  获取电机ID
    * @param ctx 主板对象
    * @param canid can id
    * @param lineid line id
    * @return  1 返回成功，0 返回失败
    */

    EXPORT_API int  robot_motor_get_motor_id(RobotCtx* ctx, unsigned short* canid, unsigned short* lineid);

    /**
    * @brief 一键ota主板升级
    * @param ctx 主板对象
    * @param filename 文件的绝对路径
    * @param func 返回进度及错误码，1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return  1 返回成功
    */

    EXPORT_API int  robot_mother_board_one_click_ota_upgradeing(RobotCtx* ctx, const char* filename, progress_callback func);

   /**
    * @brief 查看主板固件版本号
    * @param ctx   主板对象
    * @param version   固件版本号
    * @return 返回 1 为成功,0为失败
   */

    EXPORT_API int  robot_motor_get_mother_board_firmware_version(RobotCtx* ctx,char* version);

    /**
    * @brief 一键ota电机升级
    * @param motor    电机对象
    * @param filename 文件的绝对路径
    * @param func 返回进度及错误码，1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return   1 返回成功
   */

    EXPORT_API int  robot_motor_one_click_ota_upgradeing(RobotMotor* motor, const char* filename, progress_callback func);

    /**
    * @brief 查看电机固件版本号
    * @param motor   电机对象
    * @param version 固件版本号
    * @return   1 返回成功 0 返回失败
    */

    EXPORT_API int  robot_motor_get_motor_firmware_version(RobotMotor* motor, char* version);

    /**
    * @brief 查看电机型号
    * @param motor 电机对象
    * @param model 电机型号
    * @return   1 返回成功 0 返回失败
    */

    EXPORT_API int  robot_motor_get_motor_model(RobotMotor* motor, char* model);



    /**
    * @brief 设置电机速度和电流控制参数
    * @param Vel 速度设定值 单位:rad/s
    * @return void
    */

    EXPORT_API void robot_motor_set_vel(RobotMotor* motor, float Vel);



    /**
     * @brief 设置电机电流控制参数
     * @param Cur 电流设定值 单位:A
     * @return void
     */
    EXPORT_API void robot_motor_set_cur(RobotMotor* motor, float Cur);

    /**
    * @brief 设置位置PD控制参数
    * @param PDKp 位置环比例系数
    * @param PDKd 位置环微分系数
    * @return void
    */

    EXPORT_API void robot_motor_set_pd(RobotMotor* motor, float PDKp, float PDKd);


    /**
      * @brief 设置位置、速度PID控制参数
      * @param PosKp 位置环比例系数
      * @param VelKp 速度环比例系数
      * @param VelKi 速度环积分系数
      * @return void
     */

    EXPORT_API void robot_motor_set_pid(RobotMotor* motor, float PosKp, float VelKp,float VelKi);


    /**
    * @brief 释放单电机结构体
    * @param  ctx           主板对象
    * @param  motor         电机指针
    * @return void
    */

    EXPORT_API void robot_destroy_motor(RobotCtx* ctx, RobotMotor* motor);

    /**
   * @brief 获取sdk版本
   * @return char*  sdk版本号
   */

    EXPORT_API char* get_sdk_version();

#ifdef __cplusplus
}
#endif



#endif