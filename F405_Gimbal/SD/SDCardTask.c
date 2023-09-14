/**********************************************************************************************************
 * @文件     SDCardTask.c
 * @说明     读写SD卡，记录日志
 * @版本  	 V1.0
 * @作者     段康晟
 * @日期     2022.7
 **********************************************************************************************************/
#include "main.h"
#include "stdlib.h"

char WriteBuffer[128] = "";              /* 写缓冲区*/
TCHAR DataFile[] = "0:infantry.csv"; //文件名
SDStatus sd_status;

extern Disconnect Robot_Disconnect;
uint8_t Missing[MissNum];

void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //清空缓存区
    if (SD_START == write_type)         //初始化写表头
    {
        sprintf(WriteBuffer, "Yaw,Pitch,Gyro,Chassis,Pluck,Friction1,Friction2,PC,RC\n");
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
	else if (SD_CSV == write_type)        //数据写入表格
	{
        sprintf(WriteBuffer,"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",Missing[Yaw]
                                                          ,Missing[Pitch]
                                                          ,Missing[GYRO]
                                                          ,Missing[Chassis]
                                                          ,Missing[Pluck]
                                                          ,Missing[Friction0]
                                                          ,Missing[Friction1]
                                                          ,Missing[PC]
                                                          ,Missing[RC]);
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
void OpenSDCard()
{
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE); //打开文件，如果文件不存在则创建它
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //获得文件已经写入的长度
        sd_status.SD_FS_Open_result = 1;     //打开文件成功
        if (file_size > 0)                   //判断文件非空（大于零的数），并寻找添加头（即文件尾）
            sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
    }
    else
        sd_status.SD_FS_Open_result = 0;
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

void MissingUpdate(void)
{
    if(Robot_Disconnect.F105_DisConect >= 30)           {Missing[Chassis] = OFFLINE;}
    else                                                {Missing[Chassis] = ONLINE;}
    if(Robot_Disconnect.Gyro_DisConnect >= 30)          {Missing[GYRO] = OFFLINE;}
    else                                                {Missing[GYRO] = ONLINE;}
    if(Robot_Disconnect.PitchMotor_DisConnect >= 30)    {Missing[Pitch] = OFFLINE;}
    else                                                {Missing[Pitch] = ONLINE;}
    if(Robot_Disconnect.YawMotor_DisConnect >= 30)      {Missing[Yaw] = OFFLINE;}
    else                                                {Missing[Yaw] = ONLINE;}
    if(Robot_Disconnect.Pluck_DisConnect >= 30)         {Missing[Pluck] = OFFLINE;}
    else                                                {Missing[Pluck] = ONLINE;}
    if(Robot_Disconnect.Friction_DisConnect[0] >= 30)   {Missing[Friction0] = OFFLINE;}
    else                                                {Missing[Friction0] = ONLINE;}
    if(Robot_Disconnect.Friction_DisConnect[1] >= 30)   {Missing[Friction1] = OFFLINE;}
    else                                                {Missing[Friction1] = ONLINE;}
    if(Robot_Disconnect.PC_DisConnect >= 30)            {Missing[PC] = OFFLINE;}
    else                                                {Missing[PC] = ONLINE;}
    if(Robot_Disconnect.RC_DisConnect >= 30)            {Missing[RC] = OFFLINE;}
    else                                                {Missing[RC] = ONLINE;}

}

void SDCard_task(void *pvParameters)
{
//    int random = 0;
//    random = rand();
//    sprintf(DataFile,"0:infantry_%d.csv",random);
    // SD卡初始化
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)
        sd_status.SD_init_result = 0; // printf("SD卡初始化失败，请确保SD卡已正确接入开发板，或换一张SD卡测试！\n");
    sd_status.SD_init_result = 1;     // printf("SD卡初始化成功！\n");

    //挂载文件系统
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //挂载失败
    else
        sd_status.SD_FS_Mount_result = 1; //挂载成功

    OpenSDCard();
    if (sd_status.SDCard_task_init == 0)
    {
        //任务初始化
        sd_status.SDCard_task_init = 1;
        SDLOG(SD_START, "");
    }

    while (1)
    {
        sd_status.count++;
        if (sd_status.count % 1000 == 0) // 1hz
        {
			if(sd_status.SD_FS_Open_result == 0)
				OpenSDCard(); 
        }
        if (sd_status.count % 2 == 0) // 500hz
        {
            MissingUpdate();
        }

        if (sd_status.count % 5 == 0) // 200hz
        {
        }

        if (sd_status.count % 100 == 0) // 10HZ
        {
            SDLOG(SD_CSV, "TEST");
        }
        if (sd_status.count % 1000 == 999) // 1hz
        {
            CloseSDCard(); //每一秒将所有数据保存下来
        }



		vTaskDelay(1);
		
    }

    //关闭
    // f_mount(NULL, "0:", 1); /* 不再使用文件系统，取消挂载文件系统 */
}
