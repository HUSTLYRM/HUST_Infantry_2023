/**********************************************************************************************************
 * @文件     SDCardTask.c
 * @说明     读写SD卡，记录日志
 * @版本  	 V1.0
 * @作者     段康晟
 * @日期     2022.7
 **********************************************************************************************************/
 /**********************************************************************************************************
 * @文件     SDCardTask.c
 * @说明     SD卡两种操作，	1.SD_READ_MODE定义时将会不断读取sd卡内的文件发送出去，我们会用串口接收并保存到电脑，这样不需要拔掉SD卡也可以读数据，读取的数据用python脚本处理分析得到最终的情况
 *							2.非SD_READ_MODE模式将会不断写入发送NUMBER_ERROR种错误情况的次数
 *							tips:注意SD_READ_MODE模式读取时最好进入debug模式，接着打开电脑串口调试助手等待接收，最后将调试助手接收到的内容保存成csv文件即可读取完毕，读取完成后可以将delete_flag1，delete_flag2，delete_flag3在debug种都设置成1，这样会删除文件，否则一直将存在
 * @版本  	 V2.0
 * @作者     李蔚明,郭嘉豪
 * @日期     2023.7
 **********************************************************************************************************/
#include "main.h"
#include "stdlib.h"
const char csv_head[350] = "game_time,chassisMode,total_number_10hz,total_number_1000hz,Shooted,Module_Offline,OverFirespeed,OverHeat,OverPowerLimit,Hit,Drop_gyros,Drop_ChassisMotor1,Drop_ChassisMotor2,Drop_ChassisMotor3,Drop_ChassisMotor4,Drop_SteerMotor1,Drop_SteerMotor2,Drop_SteerMotor3,Drop_SteerMotor4,Drop_Judge,Drop_SuperPower,Drop_F405,100*actual_vol\n";
const char printstr[(NUMBER_ERROR+4)*3+2] = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n";
char WriteBuffer[350] = "";                        			/* 写缓冲区，*/  
char ReadBuffer[100] = "";							/* 读缓冲区，受到无线设备读取文件波特率限制，我们常用无限调试器读取，无限调试器波特率115200*/
float sd_read_var[VAR_NUMBER+1];

//删除文件的结构体变量 3个变量防止误操作
uint8_t delete_flag1,delete_flag2,delete_flag3;
TCHAR DataFile[] = "0:Infantry.csv"; //文件名
SDStatus sd_status;
SD_Saver_t SD_saver={0};
extern uint8_t Anomalies_tag;
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern INA260 INA260_1;
extern TaskHandle_t User_Tasks[TASK_NUM];
extern roboDisconnect Robot_Disconnect;
extern SuperPower superpower;
float temp1 = 2.0, temp2 = 3.0;
int count = 0;
int Init_file_size=0;
/**********************************************************************************************************
*函 数 名: SDLOG
*功能说明: 写入文件
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
extern float start_time_SD;
extern char SD_init_flag;
void Init_SDcard(){
    // SD卡初始化
	SD_init_flag = 0;
	start_time_SD = 0;
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)	
	{
		IWDG_Feed();//喂狗	
		if( start_time_SD > 5.0)
		{
			vTaskDelete(User_Tasks[SDCARD_TASK]);
			return;
		}
        sd_status.SD_init_result = 0; // printf("SD卡初始化失败，请确保SD卡已正确接入开发板，或换一张SD卡测试！\n");
    }
	if(sd_status.Status == SD_RESPONSE_NO_ERROR)
		sd_status.SD_init_result = 1;     // printf("SD卡初始化成功！\n");

    //挂载文件系统
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //挂载失败
    else
        sd_status.SD_FS_Mount_result = 1; //挂载成功
	SD_init_flag = 1;
}

/**********************************************************************************************************
*函 数 名: SDLOG
*功能说明: 写入文件
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //清空缓存区
    if (SD_START == write_type)
    {
		sprintf(WriteBuffer, csv_head);
    }
    else if (SD_WARNING == write_type)
    {
        sprintf(WriteBuffer, "[WARNING]:  ");
        sprintf(WriteBuffer + 13, str);
    }
    else if (SD_INFO == write_type)
    {
        sprintf(WriteBuffer + 10, str);
    }
	else if (SD_CSV == write_type)
	{
		count++;
        sprintf(WriteBuffer, printstr , JudgeReceive.game_progress, F405.Chassis_Flag,SD_saver.total_number_10hz, SD_saver.total_number_1000hz
                                                    ,SD_saver.number_error[Shooted]
													,SD_saver.number_error[Module_Offline]
													,SD_saver.number_error[OverFirespeed]
													,SD_saver.number_error[OverHeat]
													,SD_saver.number_error[OverPowerLimit]
		                                            ,SD_saver.number_error[Hit]
		                                            ,SD_saver.number_error[Drop_gyros]
		                                            ,SD_saver.number_error[Drop_ChassisMotor1]
													,SD_saver.number_error[Drop_ChassisMotor2]
		                                            ,SD_saver.number_error[Drop_ChassisMotor3]
		                                            ,SD_saver.number_error[Drop_ChassisMotor4]
		                                            ,SD_saver.number_error[Drop_SteerMotor1]
		                                            ,SD_saver.number_error[Drop_SteerMotor2]
		                                            ,SD_saver.number_error[Drop_SteerMotor3]
		                                            ,SD_saver.number_error[Drop_SteerMotor4]
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[0])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[1])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[2])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[3])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[4])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[5])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[6])
//													,(int)(10000*Robot_Disconnect.motor_rec_timer[7])
		                                            ,SD_saver.number_error[Drop_Judge]
													,SD_saver.number_error[Drop_SuperPower]
		                                            ,SD_saver.number_error[Drop_F405]
		                                            //,SD_saver.number_error[Low_vol]
													,(int)(superpower.actual_vol*100)
		);
	}
    else
    {
        sprintf(WriteBuffer, "[ERROR]:  ");
        sprintf(WriteBuffer + 11, str);
    }

    if (sd_status.SD_FS_Open_result) //确保已经打开再写
    {
        sd_status.res_sd = f_write(&sd_status.fnew, WriteBuffer, strlen(WriteBuffer), &sd_status.fnum);
        if (sd_status.res_sd == FR_OK)
            sd_status.SD_FS_Write_result = 1; //写入成功
        else
            sd_status.SD_FS_Write_result = 0;
    }
}
uint32_t OpenSDCard()
{
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE | FA_READ); //打开文件，如果文件不存在则创建它
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //获得文件已经写入的长度
        sd_status.SD_FS_Open_result = 1;     //打开文件成功

#ifndef SD_READ_MODE //写入模式
        if (file_size > 0)                   //判断文件非空（大于零的数），并寻找添加头（即文件尾）
            sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
#endif
    }
    else
        sd_status.SD_FS_Open_result = 0;
	
	return file_size;
}

void CloseSDCard()
{
	sd_status.res_sd = f_sync(&sd_status.fnew);
	if(sd_status.res_sd != FR_OK)
	{
		f_close(&sd_status.fnew);
		sd_status.SD_FS_Open_result = 0;

	}
}

/**********************************************************************************************************
*函 数 名: ReadSDCard
*功能说明: VOFA读取sd卡表格数据
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
uint32_t bytesRead;
void ReadSDCard()
{
		static int i = 0;
		// 读取文件中的数据
		f_read(&sd_status.fnew,ReadBuffer,100,&bytesRead);
		if(bytesRead)
		{
			VOFA_Send();
			memset(ReadBuffer, 0, sizeof(ReadBuffer));
		}
}
double now_time = 0;

void SDCard_task(void *pvParameters)
{
//    int random = 0;
//    random = rand();
//    sprintf(DataFile,"0:infantry_%d.csv",random);
	Init_SDcard();
    Init_file_size = OpenSDCard();
    if (sd_status.SDCard_task_init == 0)
    {
        //任务初始化
        sd_status.SDCard_task_init = 1;
			
#ifndef SD_READ_MODE
        SDLOG(SD_START, "");
#endif
    }
	
    while (1)
    {
		IWDG_Feed();//喂狗		
		temp1++;
        sd_status.count++;
#ifndef SD_READ_MODE

        if (sd_status.count % 10 == 0) // 1hz
        {
			if(sd_status.SD_FS_Open_result == 0)
				OpenSDCard(); 
        }
//        if (sd_status.count % 2 == 0) // 500hz
//        {

//        }

//        if (sd_status.count % 5 == 0) // 200hz
//        {
//        }

        if (sd_status.count % 1 == 0) // 10HZ
        {
            SDLOG(SD_CSV, "TEST");
        }
        if (sd_status.count % 10 == 9) // 1hz
        {
            CloseSDCard(); //每一秒将所有数据保存下来
        }


#else

				if(delete_flag1&delete_flag2&delete_flag3)
				{
					sd_status.SD_FS_DELETE_result	 = f_unlink(DataFile);
				}
				else
				{
					if (sd_status.count % 1 == 0) // 100HZ
					{
							ReadSDCard();
					}
				}
					
#endif
	
#ifndef SD_READ_MODE
		vTaskDelay(100);
#else		
		vTaskDelay(10);
#endif				
    }

    //关闭
    // f_mount(NULL, "0:", 1); /* 不再使用文件系统，取消挂载文件系统 */
}
