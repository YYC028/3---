
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 200

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//旋转的遥控器通道号码    
#define CHASSIS_Z_CHANNEL 4

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f    //0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f    //0.005f
//遥控器旋转摇杆（max 660）转化成车体旋转速度（rad/s）的比例
#define CHASSIS_WZ_RC_SEN 0.006f    //0.006f
//底盘跟随云台模式下，食指拨轮控制底盘扭腰的动作幅度，凭感觉调吧
#define CHASSIS_WRIGGLE_SEN 0.4f    //0.4f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
//#define CHASSIS_WZ_RC_SEN 0.01f

//底盘小陀螺速度（rad/s）    
#define CHASSIS_GYROSCOPE_SPEED 2.6f * PI

#define CHASSIS_ACCEL_X_NUM 0.001f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f           //0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_DASH_KEY  KEY_PRESSED_OFFSET_SHIFT
//按下按键，底盘弹舱转到后，并开启弹仓盖。再按一次关闭。
#define CHASSIS_TURN_ASS_KEY KEY_PRESSED_OFFSET_Z

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f

//底盘运动过程平稳前进速度（只按方向键）
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程平稳平移速度（只按方向键）
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
//底盘运动过程最大前进速度（按住SHIFT）
#define DASH_MAX_CHASSIS_SPEED_X   3.9f
//底盘运动过程最大平移速度（按住SHIFT）
#define DASH_MAX_CHASSIS_SPEED_Y   3.9f

#define CHASSIS_WZ_SET_SCALE -0.00625f       //0.1f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 8500
#define M3505_MOTOR_SPEED_PID_KI 24
#define M3505_MOTOR_SPEED_PID_KD 0.5f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 16.0f      //40.0f  before 16.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.4f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 9.0f  //6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘会跟随云台相对角度
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘有底盘角度控制闭环
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘有旋转速度控制
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.

} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  const gimbal_motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  const gimbal_motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  pid_type_def motor_speed_pid[4];           //底盘电机速度pid
  pid_type_def chassis_angle_pid;            //底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_wz;  //使用一阶低通滤波减缓设定值

  fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_relative_angle;      //底盘与云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  //设置相对云台控制角度
  fp32 chassis_yaw_set;             

  fp32 vx_max_normal_speed;  //前方向最大速度 单位m/s
  fp32 vx_min_normal_speed;  //后方向最大速度 单位m/s
  fp32 vy_max_normal_speed;  //左方向最大速度 单位m/s
  fp32 vy_min_normal_speed;  //右方向最大速度 单位m/s

  fp32 vx_max_dash_speed;    //前方向最大冲刺速度 单位m/s
  fp32 vx_min_dash_speed;    //后方向最大冲刺速度 单位m/s
  fp32 vy_max_dash_speed;    //左方向最大冲刺速度 单位m/s
  fp32 vy_min_dash_speed;    //右方向最大冲刺速度 单位m/s

  fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度

  bool_t is_whipping;

} chassis_move_t;


/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     wz_set: 旋转速度指针     
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          获取底盘的运行状态。   0：无异常   1：底盘电机离线   2：准备工作   3：发生异常
  * @param[out]     none
  * @retval         uint8_t
  */
extern uint8_t get_chassis_state(void);

/**
  * @brief          获取底盘是否正处于小陀螺状态。   0：未启动   1：已启动
  * @param[out]     none
  * @retval         uint8_t
  */
extern uint8_t get_chassis_is_whipping_flag(void);


static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
#endif