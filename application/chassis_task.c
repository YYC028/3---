#include "chassis_task.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "INS_task.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
//底盘运动数据
chassis_move_t chassis_move;


void chassis_task(void const *pvParameters)
{

	
		        //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
     chassis_init(&chassis_move);

	
    while(1)
    {
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

			   //发送电流
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                              chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);

			
				 vTaskDelay(2); //500HZ
    }



}



/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
		
	const static fp32 chassis_x_order_filter[1] = { CHASSIS_ACCEL_X_NUM };
    const static fp32 chassis_y_order_filter[1] = { CHASSIS_ACCEL_Y_NUM };
    const static fp32 chassis_z_order_filter[1] = { CHASSIS_ACCEL_Z_NUM };
		
		
		
    uint8_t i;

    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
		
		    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_z_order_filter);

		
		
		
	//最大 最小速度
    chassis_move_init->vx_max_normal_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_normal_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_normal_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_normal_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vx_max_dash_speed = DASH_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_dash_speed = -DASH_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_dash_speed = DASH_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_dash_speed = -DASH_MAX_CHASSIS_SPEED_Y;
		
		
		
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}



/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}



static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

     if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {

         chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;         //此模式方便底盘掉电重新上电自动回中
    }

    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
       chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;               //云台无力模式，底盘也没有角度控制
    }
     else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
          chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;        //还是底盘跟随云台模式，也可以小陀螺
    }
}


static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    
     chassis_rc_to_control_vector(&vx_set, &vy_set, &angle_set,chassis_move_control);
   // chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //计算旋转PID角速度

        if (chassis_move_control->chassis_RC->key.v & SWING_KEY)
        {
            chassis_move_control->is_whipping = 1;
            chassis_move_control->wz_set = CHASSIS_GYROSCOPE_SPEED;    //键鼠控制时按住ctrl变为小陀螺
        }
        else
        {
            chassis_move_control->is_whipping = 0;
          // chassis_move_control->wz_set = CHASSIS_GYROSCOPE_SPEED;   //为了方便展示小陀螺，现在是遥感拨到上方直接开转，正常要用下面的这行（底盘跟随云台
					chassis_move_control->wz_set = -PID_calc_swing_wz(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);//底盘跟随云台,
        }

        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //计算旋转的角速度
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, delat_angle, 0);
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) 
    {
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = angle_set; //加了个负号
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}


/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     wz_set: 旋转速度指针 
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel, wz_channel;
    fp32 vx_set_channel, vy_set_channel, wz_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Z_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;

    //键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vx_set_channel = chassis_move_rc_to_vector->vx_max_dash_speed;
        else
            vx_set_channel = chassis_move_rc_to_vector->vx_max_normal_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vx_set_channel = chassis_move_rc_to_vector->vx_min_dash_speed;
        else
            vx_set_channel = chassis_move_rc_to_vector->vx_min_normal_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vy_set_channel = chassis_move_rc_to_vector->vy_max_dash_speed;
        else
            vy_set_channel = chassis_move_rc_to_vector->vy_max_normal_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vy_set_channel = chassis_move_rc_to_vector->vy_min_dash_speed;
        else
            vy_set_channel = chassis_move_rc_to_vector->vy_min_normal_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, wz_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    if (wz_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && wz_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
    *wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw控制直接返回
        return;
    }

    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //功率控制
   // chassis_power_control(chassis_move_control_loop);

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}



static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] =  vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =  vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}