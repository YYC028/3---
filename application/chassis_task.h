
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 200

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//��ת��ң����ͨ������    
#define CHASSIS_Z_CHANNEL 4

//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f    //0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f    //0.005f
//ң������תҡ�ˣ�max 660��ת���ɳ�����ת�ٶȣ�rad/s���ı���
#define CHASSIS_WZ_RC_SEN 0.006f    //0.006f
//���̸�����̨ģʽ�£�ʳָ���ֿ��Ƶ���Ť���Ķ������ȣ�ƾ�о�����
#define CHASSIS_WRIGGLE_SEN 0.4f    //0.4f

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
//#define CHASSIS_WZ_RC_SEN 0.01f

//����С�����ٶȣ�rad/s��    
#define CHASSIS_GYROSCOPE_SPEED 2.6f * PI

#define CHASSIS_ACCEL_X_NUM 0.001f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f

//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f           //0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_DASH_KEY  KEY_PRESSED_OFFSET_SHIFT
//���°��������̵���ת���󣬲��������ָǡ��ٰ�һ�ιرա�
#define CHASSIS_TURN_ASS_KEY KEY_PRESSED_OFFSET_Z

//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f

//�����˶�����ƽ��ǰ���ٶȣ�ֻ���������
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//�����˶�����ƽ��ƽ���ٶȣ�ֻ���������
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
//�����˶��������ǰ���ٶȣ���סSHIFT��
#define DASH_MAX_CHASSIS_SPEED_X   3.9f
//�����˶��������ƽ���ٶȣ���סSHIFT��
#define DASH_MAX_CHASSIS_SPEED_Y   3.9f

#define CHASSIS_WZ_SET_SCALE -0.00625f       //0.1f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 8500
#define M3505_MOTOR_SPEED_PID_KI 24
#define M3505_MOTOR_SPEED_PID_KD 0.5f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 16.0f      //40.0f  before 16.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.4f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 9.0f  //6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̻������̨��ԽǶ�
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //�����е��̽Ƕȿ��Ʊջ�
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //��������ת�ٶȿ���
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
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  const gimbal_motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //���̵������
  pid_type_def motor_speed_pid[4];           //���̵���ٶ�pid
  pid_type_def chassis_angle_pid;            //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_wz;  //ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_normal_speed;  //ǰ��������ٶ� ��λm/s
  fp32 vx_min_normal_speed;  //��������ٶ� ��λm/s
  fp32 vy_max_normal_speed;  //��������ٶ� ��λm/s
  fp32 vy_min_normal_speed;  //�ҷ�������ٶ� ��λm/s

  fp32 vx_max_dash_speed;    //ǰ����������ٶ� ��λm/s
  fp32 vx_min_dash_speed;    //����������ٶ� ��λm/s
  fp32 vy_max_dash_speed;    //����������ٶ� ��λm/s
  fp32 vy_min_dash_speed;    //�ҷ���������ٶ� ��λm/s

  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�

  bool_t is_whipping;

} chassis_move_t;


/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     wz_set: ��ת�ٶ�ָ��     
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          ��ȡ���̵�����״̬��   0�����쳣   1�����̵������   2��׼������   3�������쳣
  * @param[out]     none
  * @retval         uint8_t
  */
extern uint8_t get_chassis_state(void);

/**
  * @brief          ��ȡ�����Ƿ�������С����״̬��   0��δ����   1��������
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