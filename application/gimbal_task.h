#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"



#define UART_PRINT_GIMBAL_YAW_SPEED 0

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        7000                     // 5000  
#define PITCH_SPEED_PID_KI        0     //60.0f     60       300
#define PITCH_SPEED_PID_KD        0          
#define PITCH_SPEED_PID_MAX_OUT   30000.0f      //30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f      //10000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        5500.0f         //3600.0f
#define YAW_SPEED_PID_KI        10.0f            //20.0f
#define YAW_SPEED_PID_KD        0.1f            //0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f        //30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f        //5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15     //12
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f         //0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.1         //0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f   //10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f   //0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f   //26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f    //0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f    //0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f   //10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f    //0.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f      //15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f      //0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f       //0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f //10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f //0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        18.0f //8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f  //0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.1f  //0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   13.0f //10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f  //0.0f


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 301
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//掉头180 按键 现在没用
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

//视觉自瞄跟随的灵敏度，（还没与视觉对接
#define VISION_YAW_SEN   0.2f     //0.025f  0.2f
#define VISION_PITCH_SEN 0.5f     //0.025f  0.3f

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1



#define PITCH_TURN   1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191





//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_absolute_angle_pid;
    pid_type_def gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;

typedef struct
{
    fp32 zero_yaw;
    fp32 max_pitch;
    fp32 min_pitch;             //全向云台，仅记录零点   
    uint16_t zero_yaw_ecd;      //全向云台，仅记录零点  
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
   // vision_control_t* vision_ctrl;      
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;


/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);


/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);


/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters);




/**
  * @brief          使用UART1发送视觉计算机所需的反馈数据
  * @param[in]      shoot_control_t *
  * @retval         void
  */
extern void uart_send_vision_feedback_data(gimbal_control_t* gimbal_ctrl);

/**
  * @brief          获取云台YAW的相对角度（自定义UI拿来绘制）
  * @param[in]      none
  * @retval         fp32 [-PI, PI]
  */
extern fp32 get_gimbal_yaw_relative_angle(void);




static void gimbal_init(gimbal_control_t *init);
static void gimbal_set_mode(gimbal_control_t *gimbal_mode_set);
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static fp32 gimbal_PID_calc(pid_type_def *pid, fp32 get, fp32 set, fp32 error_delta);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_absolute_angle_ctrl(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_set_control(gimbal_control_t *gimbal_control_set);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
#endif