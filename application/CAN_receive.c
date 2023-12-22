#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "string.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/*
电机数据,:
    0：底盘电机1 3508电机,  
    1：底盘电机2 3508电机,
    2：底盘电机3 3508电机,
    3：底盘电机4 3508电机;
    4：yaw云台电机 6020电机; 
    5：pitch云台电机 6020电机; 
		
		
    6：拨弹电机 2006电机
    7：左摩擦轮电机，M3508    
    8：右摩擦轮电机，M3508    
*/
 motor_measure_t motor_chassis[9];


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];
static CAN_TxHeaderTypeDef  CP_tx_message;
static uint8_t              CP_can_send_data[8];


/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &CHASSIS_CAN)
    {
        switch (rx_header.StdId)
        {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i = 0;
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);

            break;
        }


		case CAN_YAW_MOTOR_ID:
		case CAN_PIT_MOTOR_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_M1_ID;
			get_motor_measure(&motor_chassis[i], rx_data);
			break;
		}

        default:
        {
            break;
        }
        }
    }
    else if (hcan == &FRIC_CAN)
    {
        switch (rx_header.StdId)
        {
		case CAN_FRIC_MOTOR1_ID:
		case CAN_FRIC_MOTOR2_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_FRIC_MOTOR1_ID + 7;
			get_motor_measure(&motor_chassis[i], rx_data);
			break;
		}

        default:
        {
           	static uint8_t i = 0;
	
			   get_motor_measure(&motor_chassis[6], rx_data);
            break;
        }
        }
    }

}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = 0 ;
    gimbal_can_send_data[5] = 0 ;
    gimbal_can_send_data[6] = 0 ;
    gimbal_can_send_data[7] = 0 ;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}




/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor_left:  (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor_right: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_fric(int16_t motor_left, int16_t motor_right,int16_t shoot)
{
    uint32_t send_mail_box;
    fric_tx_message.StdId = CAN_FRIC_ALL_ID;
    fric_tx_message.IDE = CAN_ID_STD;
    fric_tx_message.RTR = CAN_RTR_DATA;
    fric_tx_message.DLC = 0x08;
    fric_can_send_data[0] = motor_left >> 8;
    fric_can_send_data[1] = motor_left;
    fric_can_send_data[2] = motor_right >> 8;
    fric_can_send_data[3] = motor_right;
    fric_can_send_data[4] = shoot >>8 ;
    fric_can_send_data[5] = shoot;
    fric_can_send_data[6] = 0;
    fric_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&FRIC_CAN, &fric_tx_message, fric_can_send_data, &send_mail_box);
}


/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[CAN_YAW_MOTOR_ID - 0x201];
}


/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[CAN_PIT_MOTOR_ID - 0x201];
}


/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[CAN_TRIGGER_MOTOR_ID - 0x201];
}


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


/**
  * @brief          返回发射机构电机 3508和2006电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i + 7];
}