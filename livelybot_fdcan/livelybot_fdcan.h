#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"
#include "convert.h"


/* 各个数据类型的无限制 */
#define  NAN_FLOAT  NAN
#define  NAN_INT32  0x80000000
#define  NAN_INT16  0x8000
#define  NAN_INT8   0x80


/* dq 电压模式 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float volt);
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t volt);
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t volt);

/* dq 电流模式 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float current);
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t current);
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t current);

/* 力矩控制 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque);
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque);
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque);

/* 位置、速度和力矩控制 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque);
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque);
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque);

/* 位置 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos);
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos);
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos);

/* 速度 */
void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel);
void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel);
void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel);

/* 超时时间 */
void set_out_time_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t);

/* 位置、速度、力矩、PD控制（运控模式） */
void set_pos_vel_tqe_kp_kd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
void set_pos_vel_tqe_kp_kd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
void set_pos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

/* 位置、速度、力矩、PD控制（真运控模式） */
void set_pos_vel_tqe_kp_kd_float_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
void set_pos_vel_tqe_kp_kd_int32_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
void set_pos_vel_tqe_kp_kd_int16_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

/* 停止位置、速度、力矩、PD控制 */
void set_stoppos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t stop_pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

/* 速度、速度限制 */
void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max);

/* 位置、速度、加速度限制（梯形控制） */
void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc);
void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc);
void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc);

/* 速度、加速度控制 */
void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc);
void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc);
void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc);

/* vfoc固定模式 */
void set_vfoc_lock(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vol);

/* 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样) */
void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms);

/* 重设零点 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 保存设置 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 重启电机 */
void set_motor_reset_int8(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 电机停止 */
void set_motor_stop_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_stop_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_stop_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 电机刹车 */
void set_motor_brake_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_brake_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_brake_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 读取电机状态 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 查询电机固件版本 */
void read_motor_version_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);


#endif
