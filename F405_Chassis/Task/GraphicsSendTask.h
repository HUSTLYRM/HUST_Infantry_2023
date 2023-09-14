#ifndef __GRAPHICS_SEND_TASK_H
#define __GRAPHICS_SEND_TASK_H

#include "stdint.h"


/*屏幕宽度*/
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920			//屏幕分辨率

/* 裁判系统数据包参数 */
#define SEND_MAX_SIZE    128    //上传数据最大的长度
#define HEADER_LEN  5       //帧头长度
#define CMD_LEN          2       //命令码长度
#define CRC_LEN          2       //尾部CRC16校验
#define DRAWING_PACK     15      //画1个图数据包长度

/* 裁判系统数据段内容ID */
#define Drawing_Delete_ID				0x0100
#define Drawing_Graphic1_ID     		0x0101
#define Drawing_Graphic2_ID 			0x0102
#define Drawing_Graphic5_ID     		0x0103
#define Drawing_Graphic7_ID     		0x0104
#define Drawing_Char_ID     			0x0110


/*操作类型*/
#define Op_None 0
#define Op_Add 1
#define Op_Change 2
#define Op_Delete 3
#define Op_Init		1		//初始化，也就是增加图层
/* 图层清除操作	*/
#define CLEAR_ONE_LAYER 1U
#define CLEAR_ALL 2U




/*颜色*/
#define Red_Blue 0
#define Yellow   1
#define Green    2
#define Orange   3
#define Purple	 4
#define Pink     5
#define Cyan 	 6
#define Black    7
#define White    8


//图形数据结构体
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;	

/* 图形绘制类型 */
typedef enum {
	TYPE_LINE = 0U,
	TYPE_RECTANGLE = 1U,
	TYPE_CIRCLE = 2U,
	TYPE_OVAL = 3U,
	TYPE_ARC = 4U,
	TYPE_FLOAT = 5U,
	TYPE_INT = 6U,
	TYPE_CHAR = 7U,
} graphic_tpye;



typedef __packed struct
{
	uint8_t operate_tpye;		//0空操作  1删除单个图层  2删除所有图层
	uint8_t layer;					//图层号  0~9
}client_custom_graphic_delete_t;//客户端删除图形

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;//客户端绘制一个图形

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;//客户端绘制两个图形

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;//客户端绘制五个图形

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;//客户端绘制七个图形

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	char data[30];
}ext_client_custom_character_t;//客户端绘制字符


/*裁判系统发送信息库*详情见裁判系统串口协议*/
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
}student_interactive_header_data_t;//交互数据


typedef __packed struct         //帧头帧尾9B
{
	uint16_t data_cmd_id;	//数据段内容ID  :2B
	uint16_t sender_ID;	//发送者ID        :2B
	uint16_t receiver_ID;	//接受者ID      :2B
	ext_client_custom_graphic_seven_t graphic_custom;//自定义图形数据: 客户端绘制七个图形  ：105B
}ext_student_interactive_header_data_t;	

typedef __packed struct
{
	uint16_t data_cmd_id;	//数据段内容ID                      :2B
	uint16_t sender_ID;	//发送者ID														:2B
	uint16_t receiver_ID;	//接受者ID													:2B
	ext_client_custom_character_t char_custom;//自定义字符串数据   :45B
}ext_student_interactive_char_header_data_t;



void JudgementDataSend(void);
void JudgementCustomizeGraphics(int Op_type);
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void referee_data_load_Graphic(int Op_type);

void referee_data_load_shootUI(uint8_t operate_type,uint8_t robot_level);
void referee_data_load_NumberUI(void);
void GraphicSendtask(void *pvParameters);

void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID,uint16_t receiverID, uint8_t* data, uint16_t pack_len);
void Send_toReferee(uint16_t cmd_id, uint16_t data_len);

graphic_data_struct_t* Line_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[]);
graphic_data_struct_t* Rectangle_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[]);
graphic_data_struct_t* FloatData_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, float data_f, uint8_t size ,uint8_t valid_bit, uint16_t line_width, int color,uint8_t name[]);
graphic_data_struct_t* CharGraphic_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color,uint8_t name[]);



void Lanelines_Init(void);
void Char_Init(void);
void CarPosture_Change(short Yaw_100 ,uint8_t Init_Cnt);
void PitchUI_Change(float Pitch ,uint8_t Init_Cnt);

enum{
    Gimbal = 0,
    Friction,
    Armor,
    Fire,
    ChangeNum
};

enum{
    LastState = 0,
    NowState
};

#endif
