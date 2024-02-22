#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "pid.h"
#include "usart.h"


/**
  * @brief          初始化"shoot_control"变量，包括pid初始化， 遥控器指针初始化，3508电机指针初始化，电机初始化
  * @param[out]     shoot_control_init:"shoot_control_t"变量指针.
  * @retval         none
  */
static void shoot_init(shoot_control_t *shoot_control_init);

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      shoot_control_t*
  * @retval         void
  */
static void shoot_set_mode(shoot_control_t *shoot_ctrl);

/**
  * @brief          发射机构测量数据更新，包括电机速度，各开关状态等
  * @param[out]     shoot_control_update:"shoot_control_t"变量指针.
  * @retval         none
  */
static void shoot_feedback_update(shoot_control_t *shoot_control_update);

/**
  * @brief          设置发射机构控制设置值
  * @param[out]     shoot_control_t:"shoot_ctrl"变量指针.
  * @retval         none
  */
static void shoot_set_contorl(shoot_control_t *shoot_ctrl);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      shoot_control_t *
  * @retval         void
  */
static void shoot_bullet_control(shoot_control_t *shoot_ctrl);

/**
  * @brief          拨弹轮堵转保护
  * @param[in]      shoot_control_t *
  * @retval         void
  */
static void trigger_motor_turn_back(shoot_control_t *shoot_ctrl);


/**
  * @brief          简易绝对值函数（fp32）
  * @param[in]      fp32：待处理的数值
  * @retval         fp32：绝对值运算的结果
  */
fp32 my_abs(fp32 val);

/**
 * @brief           从裁判系统中获取弹丸射击速度上限，并设置到摩擦轮速度
 * @author          LZX
 * @param[in]       shoot_control_t*
 * @retval          none
 */
static void shoot_speed_to_fric_speed(shoot_control_t* fric_shoot_to_speed);


shoot_control_t shoot_control;          //射击数据

extern ext_game_robot_state_t robot_state;
/**
  * @brief          发射机构任务，间隔 SHOOT_CONTROL_TIME_MS 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{	
    //底盘初始化，根据新的硬件架构更改了部分代码   
    shoot_init(&shoot_control);
  

    while (1)
    {
        //设置发射机构工作模式
        shoot_set_mode(&shoot_control);
        //更新数据
        shoot_feedback_update(&shoot_control); 
        //设置控制量
        shoot_set_contorl(&shoot_control);
        //发送控制电流   
       CAN_cmd_fric(shoot_control.fric_motor[0].give_current, shoot_control.fric_motor[1].give_current, get_trigger_motor_given_current());         

        //系统延时
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
			
		}

#if INCLUDE_uxTaskGetStackHighWaterMark
        //chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif 
}



/**
  * @brief          初始化"shoot_control"变量，包括pid初始化， 遥控器指针初始化，3508电机指针初始化，电机初始化
  * @param[out]     shoot_control_init:"shoot_control_t"变量指针.
  * @retval         none
  */
static void shoot_init(shoot_control_t *shoot_control_init)
{
    if (shoot_control_init == NULL)
    {
        return;
    }

    //摩擦轮速度环pid值
    const static fp32 fric_speed_pid[3] = { M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD };
    //拨弹轮PID
    static const fp32 trigger_speed_pid[3] = { TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD };

    //发射机构开机状态为停止
    shoot_control_init->shoot_mode = SHOOT_STOP;
    //获取遥控器指针
    shoot_control_init->shoot_rc = get_remote_control_point();

    //获取电机数据指针，初始化PID   
    shoot_control_init->fric_motor[0].motor_measure = get_fric_motor_measure_point(0);
    shoot_control_init->fric_motor[1].motor_measure = get_fric_motor_measure_point(1);
    shoot_control_init->trigger_motor.motor_measure = get_trigger_motor_measure_point();
    PID_init(&shoot_control_init->fric_motor[0].pid, PID_POSITION, fric_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control_init->fric_motor[1].pid, PID_POSITION, fric_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control_init->trigger_motor.pid, PID_POSITION, trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);

    //更新数据
    shoot_feedback_update(shoot_control_init);

    ramp_init(&shoot_control_init->fric_motor[0].ramp, SHOOT_CONTROL_TIME * 0.001f, SHOOT_SPEED_LOW, 0.0f);
    ramp_init(&shoot_control_init->fric_motor[1].ramp, SHOOT_CONTROL_TIME * 0.001f, SHOOT_SPEED_LOW, 0.0f);
    shoot_control_init->fric_motor[0].give_current = 0;
    shoot_control_init->fric_motor[1].give_current = 0;
    shoot_control_init->trigger_motor.ecd_count = 0;
    shoot_control_init->trigger_motor.angle = shoot_control_init->trigger_motor.motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control_init->trigger_motor.given_current = 0;
    shoot_control_init->move_flag = 0;
    shoot_control_init->trigger_motor.set_angle = shoot_control_init->trigger_motor.angle;
    shoot_control_init->trigger_motor.speed_set = 0.0f;
    shoot_control_init->key_time = 0;

    shoot_control_init->shoot_speed_limit = SHOOT_SPEED_HIGH / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR * 60.0f;
}


/**
  * @brief          发射机构测量数据更新，包括电机速度，各开关状态等
  * @param[out]     shoot_control_update:"shoot_control_t"变量指针.
  * @retval         none
  */
static void shoot_feedback_update(shoot_control_t *shoot_control_update)
{
    if (shoot_control_update == NULL)
    {
        return;
    }

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = { 1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f };
    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor.motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control_update->trigger_motor.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control_update->trigger_motor.motor_measure->ecd - shoot_control_update->trigger_motor.motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control_update->trigger_motor.ecd_count--;
    }
    else if (shoot_control_update->trigger_motor.motor_measure->ecd - shoot_control_update->trigger_motor.motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control_update->trigger_motor.ecd_count++;
    }

    shoot_control_update->trigger_motor.angle = rad_format((shoot_control_update->trigger_motor.ecd_count * ECD_RANGE + shoot_control_update->trigger_motor.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE);


    //鼠标按键
    shoot_control_update->last_press_l = shoot_control_update->press_l;
    shoot_control_update->last_press_r = shoot_control_update->press_r;
    shoot_control_update->press_l = shoot_control_update->shoot_rc->mouse.press_l;
    shoot_control_update->press_r = shoot_control_update->shoot_rc->mouse.press_r;
    //左键长按计时
    if (shoot_control_update->press_l)
    {
        if (shoot_control_update->press_l_time < PRESS_LONG_TIME)
            shoot_control_update->press_l_time += SHOOT_CONTROL_TIME_MS;
    }
    else
        shoot_control_update->press_l_time = 0;
    //右键长按计时
    if (shoot_control_update->press_r)
    {
        if (shoot_control_update->press_r_time < PRESS_LONG_TIME)
            shoot_control_update->press_r_time += SHOOT_CONTROL_TIME_MS;
    }
    else
        shoot_control_update->press_r_time = 0;

    //射击开关下档时间计时
    if (shoot_control_update->shoot_mode != SHOOT_STOP && switch_is_down(shoot_control_update->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_control_update->rc_s_time < RC_S_LONG_TIME)
            shoot_control_update->rc_s_time += SHOOT_CONTROL_TIME_MS;
    }
    else
        shoot_control_update->rc_s_time = 0;

    //更新摩擦轮电机速度，加速度是速度的PID微分。    
    shoot_control_update->fric_motor[0].speed = shoot_control_update->fric_motor[0].motor_measure->speed_rpm;
    shoot_control_update->fric_motor[1].speed = shoot_control_update->fric_motor[1].motor_measure->speed_rpm;

    //从裁判系统中设置射速上限
   // shoot_control_update->shoot_limit = (fp32)(robot_state.shooter_id1_17mm_speed_limit);
		shoot_control_update->shoot_limit = 30;
    //shoot_control_update->shoot_speed_set = shoot_control.shoot_limit;
    if (shoot_control_update->shoot_mode == SHOOT_STOP)
    {
        shoot_control_update->shoot_speed_set = 0.0f;
    }
    shoot_speed_to_fric_speed(shoot_control_update);
}


/**
* @brief          射击状态机设置，上挡摩擦轮关闭，中档摩擦轮开启，下档转动拨盘发射子弹
  * @param[in]      shoot_control_t*
  * @retval         void
  */
static void shoot_set_mode(shoot_control_t *shoot_ctrl)
{
    static int8_t last_s = RC_SW_UP;

    //中档摩擦轮开启
    if (switch_is_mid(shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])  && shoot_ctrl->shoot_mode == SHOOT_STOP)
    {
        shoot_ctrl->shoot_mode = SHOOT_READY_FRIC;      //摩擦轮提速阶段
    }
		//上档摩擦轮关闭
    else if ((switch_is_up(shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_ctrl->shoot_mode != SHOOT_STOP))
    {
        shoot_ctrl->shoot_mode = SHOOT_STOP;            //关闭
    }

    //处于上档， 可以使用键盘开启摩擦轮
    if (switch_is_up(shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_ctrl->shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_ctrl->shoot_mode == SHOOT_STOP)
    {
        shoot_ctrl->shoot_mode = SHOOT_READY_FRIC;
    }
    //处于上档， 可以使用键盘关闭摩擦轮
    else if (switch_is_up(shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_ctrl->shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_ctrl->shoot_mode != SHOOT_STOP)
    {
        shoot_ctrl->shoot_mode = SHOOT_STOP;
    }

    //当处于摩擦轮提速阶段，摩擦轮速度提到期望值，进入预供弹阶段
    if ((shoot_ctrl->shoot_mode == SHOOT_READY_FRIC) && (my_abs(shoot_ctrl->fric_motor[0].speed) >= my_abs(shoot_ctrl->shoot_speed_limit - 10.0f)) && (my_abs(shoot_ctrl->fric_motor[1].speed) >= my_abs(shoot_ctrl->shoot_speed_limit - 10.0f)))
    {
        shoot_ctrl->shoot_mode = SHOOT_READY;             //直接进入预备射击阶段，取消预供弹阶段    
    }
    //当意外进入预供弹阶段，立刻进入预备射击阶段  
    else if (shoot_ctrl->shoot_mode == SHOOT_READY_BULLET)
    {
        shoot_ctrl->shoot_mode = SHOOT_READY;       //预备射击阶段
    }
		
    else if (shoot_ctrl->shoot_mode == SHOOT_READY)
    {
        if ((switch_is_down(shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_ctrl->press_l && shoot_ctrl->last_press_l == 0))
        {
            shoot_ctrl->shoot_mode = SHOOT_BULLET;
        }
    }
 //当处于射击结束阶段时：
//    else if (shoot_ctrl->shoot_mode == SHOOT_DONE)
//    {
//        //根据限位开关，进入预备射击阶段
//        if (shoot_ctrl->key == SWITCH_TRIGGER_OFF)
//        {
//            shoot_ctrl->key_time++;
//            if (shoot_ctrl->key_time > SHOOT_DONE_KEY_OFF_TIME)
//            {
//                shoot_ctrl->key_time = 0;
//                shoot_ctrl->shoot_mode = SHOOT_READY_BULLET;
//                shoot_ctrl->shoot_mode = SHOOT_READY;
//            }
//        }
//        else
//        {
//            shoot_ctrl->key_time = 0;
//            shoot_ctrl->shoot_mode = SHOOT_BULLET;
//        }
//    }


    if (shoot_ctrl->shoot_mode > SHOOT_READY_FRIC)
    {
        if ((shoot_ctrl->press_l_time == PRESS_LONG_TIME) || (shoot_ctrl->rc_s_time == RC_S_LONG_TIME))
        {
            shoot_ctrl->shoot_mode = SHOOT_CONTINUE_BULLET;
				}
        else if (shoot_ctrl->shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_ctrl->shoot_mode = SHOOT_READY_BULLET;
        }
    }


    last_s = shoot_ctrl->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}


/**
  * @brief          设置发射机构控制设置值
  * @param[out]     shoot_control_t:"shoot_ctrl"变量指针.
  * @retval         none
  */
static void shoot_set_contorl(shoot_control_t *shoot_ctrl)
{
	
	    //以下为拨盘速度赋值
    if (shoot_ctrl->shoot_mode == SHOOT_STOP)
    {
        shoot_ctrl->trigger_motor.speed_set = 0.0f;
        shoot_ctrl->trigger_motor.trigger_speed_set = 0.0f;
//        shoot_ctrl->shoot_speed_set = 0.0f;
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_READY_FRIC)
    {
        shoot_ctrl->trigger_motor.speed_set = 0.0f;
        shoot_ctrl->trigger_motor.trigger_speed_set = 0.0f;
//        shoot_ctrl->shoot_speed_set = SHOOT_SPEED_LOW;
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_READY_BULLET)
    {
        if (shoot_ctrl->key == SWITCH_TRIGGER_OFF)
        {
            shoot_ctrl->trigger_motor.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back(shoot_ctrl);
        }
        else
        {
//            shoot_ctrl->trigger_motor.speed_set = 0.0f;
//            if (shoot_ctrl->press_r)
//                shoot_ctrl->shoot_speed_set = SHOOT_SPEED_HIGH;
//            else
//                shoot_ctrl->shoot_speed_set = SHOOT_SPEED_LOW;
        }
        shoot_ctrl->trigger_motor.pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_ctrl->trigger_motor.pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_READY)
    {
        shoot_ctrl->trigger_motor.speed_set = 0.0f;
        shoot_ctrl->trigger_motor.trigger_speed_set = 0.0f;
//        if (shoot_ctrl->press_r)
//            shoot_ctrl->shoot_speed_set = SHOOT_SPEED_HIGH;
//        else
//            shoot_ctrl->shoot_speed_set = SHOOT_SPEED_LOW;
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_BULLET)
    {
        shoot_ctrl->trigger_motor.pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_ctrl->trigger_motor.pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control(shoot_ctrl);
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        shoot_ctrl->trigger_motor.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back(shoot_ctrl);
    }
    else if (shoot_ctrl->shoot_mode == SHOOT_DONE)
    {
      //  shoot_ctrl->shoot_speed_set = 0.0f;
        shoot_ctrl->trigger_motor.speed_set = 0.0f;
    }


		
		
		
		
		//以下赋值摩擦轮以及拨弹轮电流
    if (shoot_ctrl->shoot_mode == SHOOT_STOP)
    {
        PID_clear(&shoot_ctrl->trigger_motor.pid);
        shoot_ctrl->trigger_motor.given_current = 0;
        
        if (shoot_ctrl->fric_motor[0].speed_set > 5.0f)
        {
            shoot_ctrl->fric_motor[0].speed_set -= SHOOT_SPEED_ADD_VALUE;
            PID_calc(&shoot_ctrl->fric_motor[0].pid, shoot_ctrl->fric_motor[0].speed, shoot_ctrl->fric_motor[0].speed_set);
            shoot_ctrl->fric_motor[0].give_current = (int16_t)(shoot_ctrl->fric_motor[0].pid.out);
        }
        else
        {
            shoot_ctrl->fric_motor[0].speed_set = 0.0f;
            PID_clear(&shoot_ctrl->fric_motor[0].pid);
            shoot_ctrl->fric_motor[0].give_current = 0;
        }

        if (shoot_ctrl->fric_motor[1].speed_set < -5.0f)
        {
            shoot_ctrl->fric_motor[1].speed_set += SHOOT_SPEED_ADD_VALUE;
            PID_calc(&shoot_ctrl->fric_motor[1].pid, shoot_ctrl->fric_motor[1].speed, shoot_ctrl->fric_motor[1].speed_set);
            shoot_ctrl->fric_motor[1].give_current = (int16_t)(shoot_ctrl->fric_motor[1].pid.out);
        }
        else
        {
            shoot_ctrl->fric_motor[1].speed_set = 0.0f;
            PID_clear(&shoot_ctrl->fric_motor[1].pid);
            shoot_ctrl->fric_motor[1].give_current = 0;
        }
    }
    else
    {
        //拨弹轮电机PID计算
        PID_calc(&shoot_ctrl->trigger_motor.pid, shoot_ctrl->trigger_motor.speed, shoot_ctrl->trigger_motor.speed_set);
        shoot_ctrl->trigger_motor.given_current = (int16_t)(shoot_ctrl->trigger_motor.pid.out);
        if (shoot_ctrl->shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_ctrl->trigger_motor.given_current = 0;
        }

        shoot_ctrl->fric_motor[0].speed_set += SHOOT_SPEED_ADD_VALUE;
        if (shoot_ctrl->fric_motor[0].speed_set > shoot_ctrl->shoot_speed_limit)
            shoot_ctrl->fric_motor[0].speed_set = shoot_ctrl->shoot_speed_limit;
        shoot_ctrl->fric_motor[1].speed_set -= SHOOT_SPEED_ADD_VALUE;
        if (shoot_ctrl->fric_motor[1].speed_set < -shoot_ctrl->shoot_speed_limit)
            shoot_ctrl->fric_motor[1].speed_set = -shoot_ctrl->shoot_speed_limit;

        PID_calc(&shoot_ctrl->fric_motor[0].pid, shoot_ctrl->fric_motor[0].speed, shoot_ctrl->fric_motor[0].speed_set);
        PID_calc(&shoot_ctrl->fric_motor[1].pid, shoot_ctrl->fric_motor[1].speed, shoot_ctrl->fric_motor[1].speed_set);
        shoot_ctrl->fric_motor[0].give_current = (int16_t)(shoot_ctrl->fric_motor[0].pid.out);
        shoot_ctrl->fric_motor[1].give_current = (int16_t)(shoot_ctrl->fric_motor[1].pid.out);
    }
}


/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      shoot_control_t *
  * @retval         void
  */
static void shoot_bullet_control(shoot_control_t *shoot_ctrl)
{
    //每次拨动 一定角度
    if (shoot_ctrl->move_flag == 0)
    {
        shoot_ctrl->trigger_motor.set_angle = rad_format(shoot_ctrl->trigger_motor.angle + (PI * 2.0f / TRIGGER_BULLET_HOLE));
        shoot_ctrl->move_flag = 1;
    }

    if (rad_format(shoot_ctrl->trigger_motor.set_angle - shoot_ctrl->trigger_motor.angle) > 0.05f)
    {
        shoot_ctrl->trigger_motor.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back(shoot_ctrl);
    }
    else
    {
        shoot_ctrl->move_flag = 0;
        shoot_ctrl->trigger_motor.trigger_speed_set = 0.0f;
        shoot_ctrl->shoot_mode = SHOOT_READY;     //转过指定角度，回到预备射击阶段 
    }
}


/**
  * @brief          获取拨弹轮电机控制电流
  * @param[in]      int16_t
  * @retval         void
  */
int16_t get_trigger_motor_given_current(void)
{
    return shoot_control.trigger_motor.given_current;
}


fp32 my_abs(fp32 val)
{
    if (val > 0.0f)
        return val;
    else
        return -val;
}


uint32_t my_pow(uint32_t j, uint32_t m)
{
    fp32 result = 1;
    for (; m > 0; m--)
        result *= j;
    return result;
}


uint8_t judge_len(int32_t val)
{
    uint8_t num = 1;

    if (val < 0)
        val = -val;

    while (1)
    {
        val /= 10;
        if (val > 0)
            num++;
        else
            break;
    }
    return num;
}


/**
  * @brief          拨弹轮堵转保护
  * @param[in]      shoot_control_t *
  * @retval         void
  */
static void trigger_motor_turn_back(shoot_control_t *shoot_ctrl)
{
    if (shoot_ctrl->block_time < BLOCK_TIME)
    {
          shoot_ctrl->trigger_motor.speed_set = shoot_ctrl->trigger_motor.trigger_speed_set;
    }
    else
    {
        shoot_ctrl->trigger_motor.speed_set = -shoot_ctrl->trigger_motor.trigger_speed_set;
    }

    if (fabs(shoot_ctrl->trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_ctrl->block_time < REVERSE_TIME)
    {
        shoot_ctrl->block_time += SHOOT_CONTROL_TIME_MS;
        shoot_ctrl->reverse_time = 0;
    }
    else if (shoot_ctrl->block_time >= BLOCK_TIME && shoot_ctrl->reverse_time < REVERSE_TIME)
    {
        shoot_ctrl->reverse_time += SHOOT_CONTROL_TIME_MS;
    }
    else
    {
        shoot_ctrl->block_time = 0;
    }
}





/**
 * @brief           从裁判系统中获取弹丸射击速度上限，并设置到摩擦轮速度
 * @author          LZX
 * @param[in]       shoot_control_t*
 * @retval          none
 */
static void shoot_speed_to_fric_speed(shoot_control_t* fric_shoot_to_speed)
{
    if (shoot_control.shoot_limit == 15)
    {
        fric_shoot_to_speed->shoot_speed_limit = 15.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_ONE * 65.0f * PI;
    }
    else if (shoot_control.shoot_limit == 18)
    {
        fric_shoot_to_speed->shoot_speed_limit = 18.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_TWO * 60.0f * PI;
    }
    else if (shoot_control.shoot_limit == 22)
    {
        fric_shoot_to_speed->shoot_speed_limit = 22.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_THERR * 60.0f * PI;
    }
    else if (shoot_control.shoot_limit == 30)
    {
        fric_shoot_to_speed->shoot_speed_limit = 30.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_FOUR * 60.0f * PI;
    }
    else
        fric_shoot_to_speed->shoot_speed_limit = 15.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_ONE * 60.0f * PI;
		
		//fric_shoot_to_speed->shoot_speed_limit = 23.0f / FRIC_MOTOR_TO_SHOOT_SPEED_VECTOR_ONE * 60.0f * PI;
}



