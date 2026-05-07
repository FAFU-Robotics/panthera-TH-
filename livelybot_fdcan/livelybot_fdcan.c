#include "livelybot_fdcan.h"
#include "my_fdcan.h"



//static void print_data(uint8_t *data, uint16_t len)
//{
//    for (int i = 0; i < len; i++)
//    {
//        printf("%d\r\n", &data[i]);
//    }
//    printf("\r\n\r\n");
//}


/**
 * @brief 电压控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电压，例：0.3 -> 0.3v
 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float volt)
{
    //                          dq电压模式    2个float      d                        q
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0E, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
                            //  查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            //  占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[9], &volt, sizeof(volt));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电压，单位：0.001V
 */
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t volt)
{
    //                   		 dq电压模式   2个32位      d                        q
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0A, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            //  占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[9], &volt, sizeof(volt));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电压，单位：0.1V
 */
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t volt)
{
    //                   		dq电压模式   2个16位       d           q
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x06, 0x1a, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[7], &volt, sizeof(volt));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电流，例：0.3 -> 0.3A
 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float current)
{
    //                   		dq电流模式   2个32位       q电流       			  d电流
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[5], &current, sizeof(current));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电流，单位：0.001A
 */
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t current)
{
    //                   		 dq电流模式   2个32位      q电流       			   d电流
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[5], &current, sizeof(current));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param volt 电流，单位：0.1A
 */
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t current)
{
    //                   		dq电流模式   2个16位      q电流       d电流       占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x06, 0x1C, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[5], &current, sizeof(current));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque)
{
    //                     		  位置模式    float  6个        位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00,
                            // 			速度                 	力矩
                            0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc,
                            //			kp                	    kd
                            0xcc, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //			最大力矩
                            0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[14], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque)
{
    //                     位置模式    		 int32  6个    	    位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x08, 0x06, 0x20, 0x00, 0x00,
                            //          速度                    力矩
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          kp  					kd
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          最大力矩
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[14], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque)
{
    //                     		位置模式     int16   6个        位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80,
                            //速度      力矩        kp          kd          最大力矩
                            0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50,
                           };

    my_memcpy(&cmd[10], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，float型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque)
{
    //                           位置模式     int32        位置                    速度                    			  力矩                    停止位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0xcd, 0xcc, 0xcc, 0x3d, 0x0e, 0x25, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[9], &vel, sizeof(float));
    my_memcpy(&cmd[15], &torque, sizeof(float));
    my_memcpy(&cmd[19], &pos, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int32型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque)
{
    //                           位置模式     int32       位置                    速度                    			  力矩                    停止位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0a, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[9], &vel, sizeof(int32_t));
    my_memcpy(&cmd[15], &torque, sizeof(int32_t));
    my_memcpy(&cmd[19], &pos, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int16型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque)
{
    //                            位置模式   2个int16      位置        速度		 2个int16	    力矩
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x06, 0x25, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[7], &vel, sizeof(int16_t));
    my_memcpy(&cmd[11], &torque, sizeof(int16_t));
    my_memcpy(&cmd[13], &pos, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos)
{
    //                           位置模式   1个float      位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[5], &pos, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 */
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos)
{
    //                           位置模式   1个int32      位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50,
                           };

    my_memcpy(&cmd[5], &pos, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 */
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos)
{
    //                          位置模式     1个int16     位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[5], &pos, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 1 转/秒，如 vel = 0.1 -> 0.1 转/秒
 */
void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel)
{
    //							  位置模式     2个float 	位置					速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50,
                           };

    my_memcpy(&cmd[9], &vel, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 */
void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel)
{
    //							  位置模式     2个int32 	位置					速度					占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50,
                           };

    my_memcpy(&cmd[9], &vel, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel)
{
    //							位置模式     2个int16 	  位置		  速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[7], &vel, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 运控模式 float (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param tqe 前馈力矩：（单位见文档）
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x0A,
        0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0e, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x1C, 0x04, 0x00, 0x11, 0x0f,
    };

    my_memcpy(&cmd[5], &pos, sizeof(float));
    my_memcpy(&cmd[9], &vel, sizeof(float));
    my_memcpy(&cmd[13], &tqe, sizeof(float));
    my_memcpy(&cmd[19], &kp, sizeof(float));
    my_memcpy(&cmd[23], &kd, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 运控模式 int32 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.001 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.001 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x0A,
        0x0B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0A, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x18, 0x04, 0x00, 0x11, 0x0f,
    };

    my_memcpy(&cmd[5], &pos, sizeof(int32_t));
    my_memcpy(&cmd[9], &vel, sizeof(int32_t));
    my_memcpy(&cmd[13], &tqe, sizeof(int32_t));
    my_memcpy(&cmd[19], &kp, sizeof(int32_t));
    my_memcpy(&cmd[23], &kd, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 运控模式 int16 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x0A,
        0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x06, 0x2b, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x14, 0x04, 0x00, 0x11, 0x0f,
        // 占位（fdcan）
        0x50, 0x50,
    };

    my_memcpy(&cmd[5], &pos, sizeof(int16_t));
    my_memcpy(&cmd[7], &vel, sizeof(int16_t));
    my_memcpy(&cmd[9], &tqe, sizeof(int16_t));
    my_memcpy(&cmd[13], &kp, sizeof(int16_t));
    my_memcpy(&cmd[15], &kd, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 float (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param tqe 前馈力矩：（单位见文档）
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_float_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x15,
        0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0e, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x1C, 0x04, 0x00, 0x11, 0x0f,
    };

    my_memcpy(&cmd[5], &pos, sizeof(float));
    my_memcpy(&cmd[9], &vel, sizeof(float));
    my_memcpy(&cmd[13], &tqe, sizeof(float));
    my_memcpy(&cmd[19], &kp, sizeof(float));
    my_memcpy(&cmd[23], &kd, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 int32 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.001 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.001 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int32_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x15,
        0x0B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0A, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x18, 0x04, 0x00, 0x11, 0x0f,
    };

    my_memcpy(&cmd[5], &pos, sizeof(int32_t));
    my_memcpy(&cmd[9], &vel, sizeof(int32_t));
    my_memcpy(&cmd[13], &tqe, sizeof(int32_t));
    my_memcpy(&cmd[19], &kp, sizeof(int32_t));
    my_memcpy(&cmd[23], &kd, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 int16 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int16_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
{
    static uint8_t cmd[] =
    {
        0x01, 0x00, 0x15,
        0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x06, 0x2b, 0x00, 0x00, 0x00, 0x00,
        // 查询指令
        0x14, 0x04, 0x00, 0x11, 0x0f,
        // 占位（fdcan）
        0x50, 0x50,
    };

    my_memcpy(&cmd[5], &pos, sizeof(int16_t));
    my_memcpy(&cmd[7], &vel, sizeof(int16_t));
    my_memcpy(&cmd[9], &tqe, sizeof(int16_t));
    my_memcpy(&cmd[13], &kp, sizeof(int16_t));
    my_memcpy(&cmd[15], &kd, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、速度限幅控制（如果 vel > vel_max，则用 vel_max） int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param vel_max 速度限幅：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max)
{
    //							位置模式				  位置        速度		  			  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x28, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[11], &vel_max, sizeof(int16_t));
    my_memcpy(&cmd[7], &vel, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel_max 速度限制，单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param acc 加速度，单位：1 转/秒^2
 */
void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc)
{
    // 							位置模式				  位置		  						  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0E, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[5], &pos, sizeof(float));
    my_memcpy(&cmd[11], &vel_max, sizeof(float));
    my_memcpy(&cmd[15], &acc, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel_max 速度限制：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.00001 转/秒^2，如 acc = 50000 表示 0.5 转/秒^2
 */
void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc)
{
    // 							位置模式				  位置		  						  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0A, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[5], &pos, sizeof(int32_t));
    my_memcpy(&cmd[11], &vel_max, sizeof(int32_t));
    my_memcpy(&cmd[15], &acc, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param vel_max 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 acc = 100 表示 0.1 转/秒^2
 */
void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc)
{
    // 							位置模式				  位置		  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x80, 0x06, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[5], &pos, sizeof(int16_t));
    my_memcpy(&cmd[9], &vel_max, sizeof(int16_t));
    my_memcpy(&cmd[11], &acc, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度，单位： 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param acc 加速度，单位：1 转/秒^2
 */
void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc)
{
    //							位置模式				  位置					  速度								  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0D, 0x29, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[9], &vel, sizeof(float));
    my_memcpy(&cmd[15], &acc, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 vel = 500 表示 0.5 转/秒^2
 */
void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc)
{
    //							位置模式				  位置					  速度								  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x09, 0x29, 0x00, 0x00, 0x00, 0x00,
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f,
                           };

    my_memcpy(&cmd[9], &vel, sizeof(int32_t));
    my_memcpy(&cmd[15], &acc, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
 */
void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc)
{
    //							位置模式				  位置		  速度					  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x29, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50
                           };

    my_memcpy(&cmd[7], &vel, sizeof(int16_t));
    my_memcpy(&cmd[11], &acc, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 使用电压模式将电机固定（电机不会动，用于减少电机停止的声音，但是电流会增大）
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vol d相电压
 */
void set_vfoc_lock_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vol)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x12, 0x05, 0x19, 0x00, 0x00,
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f,
                            // 占位（fdcan）
                            0x50, 0x50, 0x50, 0x50
                           };

    my_memcpy(&cmd[5], &vol, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 设置电机超时时间，电机超过超时时间没接受到新指令，电机进入刹车模式
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param t 电机超时时间，单位：1ms
 */
void set_out_time_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t)
{
    static uint8_t cmd[] = {0x05, 0x1f, 0x00, 0x00};

    my_memcpy(&cmd[2], &t, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
 *          1. 周期返回电机位置、速度、力矩数据。
 *          2. 返回数据格式和使用  0x17，0x01  指令获取的格式一样
 *          3. 周期单位为 ms。
 *          4. 最小周期为 1ms，最大周期 32767ms。
 *          5. 如需停止周期返回数据，将周期给 0 即可，或者给电机断电。
 * @param id 电机ID
 * @param t 返回周期（单位：ms）
 */
void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms)
{
    static uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    fdcan_send(fdcanHandle, 0x8000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 重设电机零位
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 保存电机设置
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机软重启
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_reset_int8(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x08, 0x64, 0x20, 0x72, 0x65, 0x73, 0x65, 0x74, 0x0A, 0x50};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00, 
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00, 
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00, 
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f, 
                            // 查询指令
                            0x1C, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f, 
                            // 查询指令
                            0x18, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f, 
                            // 查询指令
                            0x14, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 float，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x1C, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int32，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x18, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int16，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x14, 0x04, 0x00, 0x11, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机固件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_version_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x15, 0xB5, 0x02};

    fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}
