#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "user_lib.h"


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
    
//云台控制所有相关数据
gimbal_control_t gimbal_control;
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;



/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void gimbal_task(void const *pvParameters)
{
     vTaskDelay(GIMBAL_TASK_INIT_TIME);
      //云台初始化,等待陀螺仪收敛
    gimbal_init(&gimbal_control);
    while(1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_feedback_update(&gimbal_control); 
        gimbal_set_control(&gimbal_control);     
        gimbal_control_loop(&gimbal_control);  

        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;

        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current);   //拨弹轮电机的电流在另一任务中算得     
		    vTaskDelay(1);
    }

}



static void gimbal_init(gimbal_control_t *init)
{
    static const fp32 Yaw_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP,YAW_GYRO_ABSOLUTE_PID_KI,YAW_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_speed_pid[3] = { YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD };    

    static const fp32 Pitch_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP,PITCH_GYRO_ABSOLUTE_PID_KI,PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Pitch_speed_pid[3] = { PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD };

    
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
   
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
   
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, Yaw_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
  
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,PID_POSITION,Pitch_pid,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
	//初始化电机零偏编码值offset
	init->gimbal_yaw_motor.offset_ecd = 5740;
	init->gimbal_pitch_motor.offset_ecd = 7623;
    //初始化pitch最大最小机械角度，防止电机期望角度超过死角产生抖动或过热
	init->gimbal_pitch_motor.max_relative_angle = 0.80; 
	init->gimbal_pitch_motor.min_relative_angle = -0.50; 
			
			
    gimbal_feedback_update(init);
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}


/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{

    //云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);


    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
        feedback_update->gimbal_pitch_motor.offset_ecd);


    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);




    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);



    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
        feedback_update->gimbal_yaw_motor.offset_ecd);

    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
				
				
}

static void gimbal_set_mode(gimbal_control_t *gimbal_mode_set)
{

     //开关控制 云台状态   
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
         gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;  //不走pid
			   gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW; 
    }

    //其实拨杆在上面或中间都是陀螺仪控制而不是用编码值，所以其实可以合并
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])) 
    {
        
        if ((switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[1]) || gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r) )
        {
             gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
             gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        }

        else
         {
             gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
             gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        }
				

    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))     
    {
        if (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r )
        {
             gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
             gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        }
        else
       {
             gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
             gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        }
    }


}


static void gimbal_set_control(gimbal_control_t *gimbal_control_set)
{


    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    static int16_t yaw_channel = 0, pitch_channel = 0;  

    rc_deadband_limit(   gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(  gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    add_yaw_angle = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    add_pitch_angle= - (pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);




    //1. yaw电机模式控制
    if (gimbal_control_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_control_set->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle*1000000;
    }
    else if (gimbal_control_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_yaw_absolute_angle_ctrl(&gimbal_control_set->gimbal_yaw_motor, add_yaw_angle);
    }


    //2. pitch电机模式控制
    if (gimbal_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_control_set->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle*1500000;
    }
    else if (gimbal_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
          //gyro模式下，陀螺仪角度控制
          gimbal_absolute_angle_limit(&gimbal_control_set->gimbal_pitch_motor, add_pitch_angle);
    }

}
    

/**
  * @brief          云台yaw控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制 
  * @param[out]     gimbal_motor:yaw电机
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_ctrl(gimbal_motor_t *gimbal_motor, fp32 add)
{
    //static fp32 bias_angle;
    static fp32 angle_set;

    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}



/**
  * @brief          云台pitch控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;

    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //计算出一个最大的添加角度，
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}



static void gimbal_control_loop(gimbal_control_t *control_loop)
{

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
       control_loop->gimbal_yaw_motor.current_set =  control_loop->gimbal_yaw_motor.raw_cmd_current;
        control_loop->gimbal_yaw_motor.given_current =  control_loop->gimbal_yaw_motor.current_set ;
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }


    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
         control_loop->gimbal_pitch_motor.current_set =  control_loop->gimbal_pitch_motor.raw_cmd_current;
        control_loop->gimbal_pitch_motor.given_current =  control_loop->gimbal_pitch_motor.current_set ;
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }

}


static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}










/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}




static fp32 gimbal_PID_calc(pid_type_def *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->fdb = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->Kp * pid->err;
    pid->Iout += pid->Ki * pid->err;
    pid->Dout = pid->Kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}




const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

