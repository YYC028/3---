
#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


  //射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              1500    //2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f      //2*pi/8191/36
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               7.0f		//7.0f
#define CONTINUE_TRIGGER_SPEED      10.0f		//10.0f
#define READY_TRIGGER_SPEED         5.0f		//5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         2.0f    //1.0f
#define BLOCK_TIME                  500     //700
#define REVERSE_TIME                500     //500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f


//拨弹轮有多少个弹孔
#define TRIGGER_BULLET_HOLE					7.0f


//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        1000.0f     //800.0f   700-1000
#define TRIGGER_ANGLE_PID_KI        0.5f       //《50   0.1-1
#define TRIGGER_ANGLE_PID_KD        0.5f       //《200    0.1-1



#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f	//1000.0f
#define TRIGGER_READY_PID_MAX_IOUT  300.0f		 //2000.0f

#define SHOOT_SPEED_MAX             30.0f
#define SHOOT_SPEED_HIGH            30.0f     //14.6f
#define SHOOT_SPEED_LOW             30.0f
#define FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR 0.1884955592148f

#define SHOOT_SPEED_ADD_VALUE 8.0f

#define SHOOT_HEAT_REMAIN_VALUE     80

//任务开始空闲一段时间
#define SHOOT_TASK_INIT_TIME 357
//云台YAW任务控制间隔 2ms
#define SHOOT_CONTROL_TIME_MS 1
//云台YAW任务控制间隔 0.002s
#define SHOOT_CONTROL_TIME 0.001f
//云台YAW任务控制频率，尚未使用这个宏
#define SHOOT_CONTROL_FREQUENCE 1000.0f
//3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//摩擦轮3508速度闭环PID
#define M3508_MOTOR_SPEED_PID_KP 10.0f		//40.0f
#define M3508_MOTOR_SPEED_PID_KI 0.0f		//10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f   //100.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT 
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 5000.0f   //5000.0f

#define FRIC_TURN       1   //摩擦轮换向，当摩擦轮转向异常时，调转此宏
#define TRIGGER_TURN    0   //拨弹轮换向，当拨弹轮转向异常时，调转此宏

//自动设置摩擦轮速度，按照挡位手动分配合适的比例因子
//提高速度则改小，降低速度则改大
#define FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_ONE   0.700000000f     //0.1460952f    //15(m/s)
#define FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_TWO   0.76666111f     //0.103666111f     //18(m/s)
#define FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_THERR 0.793500000f     //0.113500000f     //22(m/s)
#define FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_FOUR  0.744000f        //0.334000f    //30(m/s)

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t* motor_measure;
    fp32 speed;                     //摩擦轮实时转速（rpm）
    fp32 speed_set;                 //摩擦轮期望转速（rpm）
    int16_t give_current;           //摩擦轮控制电流
    pid_type_def pid;               //PID数据
    ramp_function_source_t ramp;    //斜波数据
} fric_motor_t;

typedef struct
{
    const motor_measure_t* motor_measure;
    pid_type_def pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
} trigger_motor_t;

typedef struct
{
    const RC_ctrl_t* shoot_rc;
    shoot_mode_e shoot_mode;

    fric_motor_t fric_motor[2];
    trigger_motor_t trigger_motor;

    fp32 shoot_speed_limit;     //利用裁判系统的读数，设置的摩擦轮速度
    fp32 shoot_speed_set;       //期望弹丸发射速度（m/s）
    //fp32 shoot_freq_set;      //期望弹丸发射频率（fps）
    fp32 shoot_limit;           //从裁判系统获取的射速上限

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;


//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义

extern fp32 my_abs(fp32 val);
extern uint32_t my_pow(uint32_t j, uint32_t m);
extern uint8_t judge_len(int32_t val);


/**
  * @brief          发射机构任务，间隔 SHOOT_CONTROL_TIME_MS 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void shoot_task(void const* pvParameters);

/**
  * @brief          获取拨弹轮电机控制电流
  * @param[in]      int16_t
  * @retval         void
  */
extern int16_t get_trigger_motor_given_current(void);

/**
  * @brief          获取发射机构运行状态   0：正在运行   1：有设备离线   2：准备就绪   3：发生异常/正在启动
  * @param[in]      none
  * @retval         uint8_t
  */
extern uint8_t get_shoot_state(void);

#endif
