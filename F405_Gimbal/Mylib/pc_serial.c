/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "arm_atan2_f32.h"
#include "debug.h"
#include "main.h"

PCRecvData pc_recv_data;
PCSendData pc_send_data;
float pc_pitch,pc_yaw;
uint8_t PC_Shoot_flag;
uint8_t last_shoot_flag;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    if (PCbuffer[0] == '!' && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        //数据解码
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        /*射击标志位*/
        PC_Shoot_flag = pc_recv_data.shoot_flag - last_shoot_flag;
        last_shoot_flag = pc_recv_data.shoot_flag;
        /*识别标志位*/
        if(pc_recv_data.enemy_id != 0)
        {
            pc_pitch = pc_recv_data.pitch/100.0f;
            pc_yaw = pc_recv_data.yaw;
        }
        PCSolve();
    }
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
extern F105_Typedef F105;
extern Gimbal_Typedef Gimbal;
extern Status_t Status;
void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.start_flag = '!';
    pc_send_data.robot_color = F105.JudgeReceive_info.RobotRed;
    pc_send_data.shoot_level = F105.JudgeReceive_info.BulletSpeedLevel;
    pc_send_data.mode = Status.GimbalMode;
    //pc_send_data.which_balance = F105.whichbalance - 2; //只有两位，ID-2再发
    //pc_send_data.change_priority_flag = 0;            //在actiontask中修改
    pc_send_data.frame_id++;
    pc_send_data.pitch = (short)(Gimbal.Pitch.Gyro * 100.0f);
    pc_send_data.yaw = Gimbal.Yaw.Gyro;

    Append_CRC16_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    DMA_Cmd(PC_UART_SEND_DMAx_Streamx, ENABLE);
}
