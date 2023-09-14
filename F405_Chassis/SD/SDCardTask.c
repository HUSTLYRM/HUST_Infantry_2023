/**********************************************************************************************************
 * @�ļ�     SDCardTask.c
 * @˵��     ��дSD������¼��־
 * @�汾  	 V1.0
 * @����     �ο���
 * @����     2022.7
 **********************************************************************************************************/
 /**********************************************************************************************************
 * @�ļ�     SDCardTask.c
 * @˵��     SD�����ֲ�����	1.SD_READ_MODE����ʱ���᲻�϶�ȡsd���ڵ��ļ����ͳ�ȥ�����ǻ��ô��ڽ��ղ����浽���ԣ���������Ҫ�ε�SD��Ҳ���Զ����ݣ���ȡ��������python�ű���������õ����յ����
 *							2.��SD_READ_MODEģʽ���᲻��д�뷢��NUMBER_ERROR�ִ�������Ĵ���
 *							tips:ע��SD_READ_MODEģʽ��ȡʱ��ý���debugģʽ�����Ŵ򿪵��Դ��ڵ������ֵȴ����գ���󽫵������ֽ��յ������ݱ����csv�ļ����ɶ�ȡ��ϣ���ȡ��ɺ���Խ�delete_flag1��delete_flag2��delete_flag3��debug�ֶ����ó�1��������ɾ���ļ�������һֱ������
 * @�汾  	 V2.0
 * @����     ��ε��,���κ�
 * @����     2023.7
 **********************************************************************************************************/
#include "main.h"
#include "stdlib.h"
const char csv_head[350] = "game_time,chassisMode,total_number_10hz,total_number_1000hz,Shooted,Module_Offline,OverFirespeed,OverHeat,OverPowerLimit,Hit,Drop_gyros,Drop_ChassisMotor1,Drop_ChassisMotor2,Drop_ChassisMotor3,Drop_ChassisMotor4,Drop_SteerMotor1,Drop_SteerMotor2,Drop_SteerMotor3,Drop_SteerMotor4,Drop_Judge,Drop_SuperPower,Drop_F405,100*actual_vol\n";
const char printstr[(NUMBER_ERROR+4)*3+2] = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n";
char WriteBuffer[350] = "";                        			/* д��������*/  
char ReadBuffer[100] = "";							/* �����������ܵ������豸��ȡ�ļ����������ƣ����ǳ������޵�������ȡ�����޵�����������115200*/
float sd_read_var[VAR_NUMBER+1];

//ɾ���ļ��Ľṹ����� 3��������ֹ�����
uint8_t delete_flag1,delete_flag2,delete_flag3;
TCHAR DataFile[] = "0:Infantry.csv"; //�ļ���
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
*�� �� ��: SDLOG
*����˵��: д���ļ�
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
extern float start_time_SD;
extern char SD_init_flag;
void Init_SDcard(){
    // SD����ʼ��
	SD_init_flag = 0;
	start_time_SD = 0;
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)	
	{
		IWDG_Feed();//ι��	
		if( start_time_SD > 5.0)
		{
			vTaskDelete(User_Tasks[SDCARD_TASK]);
			return;
		}
        sd_status.SD_init_result = 0; // printf("SD����ʼ��ʧ�ܣ���ȷ��SD������ȷ���뿪���壬��һ��SD�����ԣ�\n");
    }
	if(sd_status.Status == SD_RESPONSE_NO_ERROR)
		sd_status.SD_init_result = 1;     // printf("SD����ʼ���ɹ���\n");

    //�����ļ�ϵͳ
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //���ⲿSPI Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ���SPI�豸��ʼ��

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //����ʧ��
    else
        sd_status.SD_FS_Mount_result = 1; //���سɹ�
	SD_init_flag = 1;
}

/**********************************************************************************************************
*�� �� ��: SDLOG
*����˵��: д���ļ�
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //��ջ�����
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

    if (sd_status.SD_FS_Open_result) //ȷ���Ѿ�����д
    {
        sd_status.res_sd = f_write(&sd_status.fnew, WriteBuffer, strlen(WriteBuffer), &sd_status.fnum);
        if (sd_status.res_sd == FR_OK)
            sd_status.SD_FS_Write_result = 1; //д��ɹ�
        else
            sd_status.SD_FS_Write_result = 0;
    }
}
uint32_t OpenSDCard()
{
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE | FA_READ); //���ļ�������ļ��������򴴽���
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //����ļ��Ѿ�д��ĳ���
        sd_status.SD_FS_Open_result = 1;     //���ļ��ɹ�

#ifndef SD_READ_MODE //д��ģʽ
        if (file_size > 0)                   //�ж��ļ��ǿգ����������������Ѱ�����ͷ�����ļ�β��
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
*�� �� ��: ReadSDCard
*����˵��: VOFA��ȡsd���������
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t bytesRead;
void ReadSDCard()
{
		static int i = 0;
		// ��ȡ�ļ��е�����
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
        //�����ʼ��
        sd_status.SDCard_task_init = 1;
			
#ifndef SD_READ_MODE
        SDLOG(SD_START, "");
#endif
    }
	
    while (1)
    {
		IWDG_Feed();//ι��		
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
            CloseSDCard(); //ÿһ�뽫�������ݱ�������
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

    //�ر�
    // f_mount(NULL, "0:", 1); /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
}
