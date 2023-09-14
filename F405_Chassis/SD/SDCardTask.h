#ifndef __SDCARDTASK_H
#define __SDCARDTASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "bsp_spi_sdcard.h"

#include "counter.h"

//#define SD_READ_MODE


#define VAR_NUMBER 5
// #define SDLOG()
enum SDWRITE
{
    SD_START,
    SD_WARNING,
    SD_INFO,
    SD_ERROR,
    SD_CSV
};

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE 512      /* Block Size in Bytes */
#define NUMBER_OF_BLOCKS 10 /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE (BLOCK_SIZE * NUMBER_OF_BLOCKS)

typedef struct SDStatus
{
    uint8_t SD_init_result;     // SD卡初始化
    uint8_t SD_FS_Write_result; // 写结果
    uint8_t SD_FS_Read_result;  // 读结果
    uint8_t SD_FS_Open_result;  // 打开文件结果
    uint8_t SD_FS_Mount_result; // 挂载文件系统结果
    uint8_t SDCard_task_init;   // SD卡任务成功初始化结果
		uint8_t SD_FS_DELETE_result;// 删除文件结过

    SD_Error Status;
    FATFS fs;
    FIL fnew;
    FRESULT res_sd; /* 文件操作结果 */
    UINT fnum;      /* 文件成功读写数量 */

    int32_t count;

    uint32_t last_cnt;
    float delta_t;
} SDStatus;

#define NUMBER_ERROR 19 //下面enum的数量，修改这个要对应修改csv_head[]表头名字
enum airforceStatus_Typedef
{
	//对应裁判系统接收ox206的扣血信息
	Shooted=0,			//被攻击扣血
	Module_Offline,		//模块离线扣血
	OverFirespeed,		//超射速扣血
	OverHeat,			//超热量扣血
	OverPowerLimit,		//超功率
	Hit,				//被撞击扣血
	
	//自定义错误情况
	Drop_gyros,			//陀螺仪掉线
	Drop_ChassisMotor1,	//7 电机掉线
	Drop_ChassisMotor2,
	Drop_ChassisMotor3,
	Drop_ChassisMotor4,
	Drop_SteerMotor1,
	Drop_SteerMotor2,
	Drop_SteerMotor3,
	Drop_SteerMotor4,
	Drop_Judge,			//裁判系统掉线
	Drop_SuperPower,	//超级电容
	Drop_F405,			//F405掉线
	
	Low_vol,			//半电容低电压
};	

typedef struct{
	int number_error[NUMBER_ERROR];//多少种错误记录类型
	int total_number_10hz;//10hz数据记录总数 上面除以总数就是发送概率，首先读取文件载入上次的次数
	int total_number_1000hz;//1000hz数据记录总数 上面除以总数就是发送概率 ， 首先读取文件载入上次的次数
}SD_Saver_t;

void Init_SDcard();
void SDCard_task(void *pvParameters);
void SDLOG(enum SDWRITE write_type, const char *str);
void ReadSDCard();

#endif

