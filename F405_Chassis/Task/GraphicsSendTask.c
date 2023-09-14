/**********************************************************************************************************
 * @文件     Graphics_Send.c
 * @说明     裁判系统图形发送
 * @版本  	 V2.0
 * @作者     王宁
 * @日期     2023.5.1
**********************************************************************************************************/
#include "main.h"



#define CAP_GRAPHIC_NUM 9			//超级电容电量图形显示细分格数
extern unsigned char JudgeSend[SEND_MAX_SIZE];
extern JudgeReceive_t JudgeReceive;
extern SuperPower superpower;
extern F405_typedef F405;

int pitch_change_flag;
int cap_percent_change_flag;
int BigFrictSpeed_change_flag;
int Pitch_change_flag;
int vol_change_array[CAP_GRAPHIC_NUM];
float last_cap_vol;
short lastBigFrictSpeed;



/**********************************************************************************************************
 * @文件     Graphics_Send.c
 * @日期     2023.4


参考：Robomaster 串口协议附录v1.4 		



裁判系统通信协议

	帧首部					命令id(绘制UI是0x0301)		数据段（首部+内容）			尾部2字节校验位 CRC16
*********************		*********************		*********************		*********************
*					*		*					*		*					*		*					*
*	frame_header	*		*	cmd_id			*		*	data			*		*	frame_tail		*
*	(5 bytes)		*	+	*	(2 bytes)		*	+	*	(n bytes)		*	+	*	(2 bytes)		*
*					*		*					*		*					*		*	  				*
*********************		*********************		*********************		*********************



**********************************************************************************************************/


/*			变量定义				*/
uint8_t Transmit_Pack[128];//裁判系统数据帧
uint8_t data_pack[DRAWING_PACK*7] = {0};//数据段部分
uint8_t DMAsendflag;
/**********************************************************************************************************
*函 数 名: Send_UIPack
*功能说明: 发送整个UI数据包（数据段首部和内容）
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID,uint16_t receiverID, uint8_t* data, uint16_t pack_len)
{
	student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = data_cmd_id;
	custom_interactive_header.send_ID = SendID;
	custom_interactive_header.receiver_ID = receiverID;

	uint8_t header_len = sizeof(custom_interactive_header);//数据段首部长度
	
	memcpy((void*)(Transmit_Pack + 7), &custom_interactive_header, header_len);	//将数据段的数据段进行封装（封装段首）
	memcpy((void*)(Transmit_Pack + 7 + header_len), data, pack_len);			//将数据帧的数据段进行封装（封装数据）

	Send_toReferee(0x0301,pack_len + header_len);//打包并发送整帧数据
}


/**********************************************************************************************************
*函 数 名: Send_toReferee
*功能说明: 打包数据帧并发送给裁判系统
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Send_toReferee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len; 
 
	//帧首部封装
	{
		Transmit_Pack[0] = 0xA5;
		memcpy(&Transmit_Pack[1],(uint8_t*)&data_len, sizeof(data_len));//数据段中data的长度
		Transmit_Pack[3] = seq ++ ;
		Append_CRC8_Check_Sum(Transmit_Pack,HEADER_LEN);  //帧头校验CRC8
	}
	
	//命令ID
	memcpy(&Transmit_Pack[HEADER_LEN],(uint8_t*)&cmd_id, CMD_LEN);
	
	//尾部数据校验CRC16
	Append_CRC16_Check_Sum(Transmit_Pack,Frame_Length);  

	uint8_t send_cnt = 1;//发送次数连续发3次
	while (	send_cnt )
	{
		send_cnt --;
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//通过UART+DMA的方式发送
		DMA1_Stream4->NDTR = Frame_Length;
		DMA_Cmd(DMA1_Stream4, ENABLE);	  

		DMAsendflag=1;//DMA传输完成标志，在中断中置0

		while (DMAsendflag == 1)
		{
		}
        //vTaskDelay(1);

	}

}

/**********************************************************************************************************
*函 数 名: Deleta_Layer
*功能说明: 清除图层
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Deleta_Layer(uint8_t layer , uint8_t deleteType)
{
	static client_custom_graphic_delete_t Delete_Graphic;//必须为静态变量，否则函数结束时会销毁该变量内存
	Delete_Graphic.layer = layer;
	Delete_Graphic.operate_tpye = deleteType;
	Send_UIPack(Drawing_Delete_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, (uint8_t *)&Delete_Graphic, sizeof(Delete_Graphic)); // 画字符

}


/**********************************************************************************************************
*函 数 名: CharGraphic_Draw
*功能说明: 得到字符图形数据结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
graphic_data_struct_t* CharGraphic_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color,uint8_t name[])
{


	static graphic_data_struct_t drawing;//必须为静态变量，否则函数结束时会销毁该变量内存
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CHAR;//7为字符数据
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;

	drawing.start_angle = size;//字体大小
	drawing.end_angle = len;//字符长度
	drawing.width = line_width;

	for(uint8_t i = DRAWING_PACK;i < DRAWING_PACK + 30;i++)
		data_pack[i] = 0;
	return &drawing;

}

/**********************************************************************************************************
*函 数 名: Char_Draw
*功能说明: 绘制字符
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Char_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color,uint8_t name[],uint8_t * str_data)
{
	graphic_data_struct_t * P_graphic_data;
	P_graphic_data = CharGraphic_Draw(0,Op_Type, startx, starty, size, len ,line_width , color , name);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	memset(&data_pack[DRAWING_PACK], 0, 30);
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)str_data ,len);
	Send_UIPack(Drawing_Char_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK + 30); // 绘制字符

}


/**********************************************************************************************************
*函 数 名: FloatData_Draw
*功能说明: 得到绘制浮点图形结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
graphic_data_struct_t* FloatData_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, float data_f, uint8_t size ,uint8_t valid_bit, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//必须为静态变量，否则函数结束时会销毁该变量内存
	static int32_t Data1000;
	Data1000 = (int32_t)(data_f * 1000 );
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_FLOAT;//5为浮点数据
	drawing.width = line_width;//线宽
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;//字体大小
	drawing.end_angle = valid_bit;//有效位数

	drawing.radius = Data1000 & 0x03ff;
	drawing.end_x  = (Data1000 >> 10) & 0x07ff;
	drawing.end_y  = (Data1000 >> 21) & 0x07ff;	
	return &drawing;

}


/**********************************************************************************************************
*函 数 名: Line_Draw
*功能说明: 直线图形数据结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
graphic_data_struct_t* Line_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//必须为静态变量，否则函数结束时会销毁该变量内存
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_LINE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	return &drawing;

}

/**********************************************************************************************************
*函 数 名: Rectangle_Draw
*功能说明: 矩形图形数据结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
graphic_data_struct_t* Rectangle_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//必须为静态变量，否则函数结束时会销毁该变量内存
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_RECTANGLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	return &drawing;

}

/**********************************************************************************************************
*函 数 名: Circle_Draw
*功能说明: 圆形图形数据结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
graphic_data_struct_t* Circle_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint32_t radius, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//必须为静态变量，否则函数结束时会销毁该变量内存
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CIRCLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;
    drawing.radius = radius;
	return &drawing;

}

/**********************************************************************************************************
*函 数 名: Lanelines_Init
*功能说明: 车道线初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	graphic_data_struct_t * P_graphic_data;
	//第一条车道线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.49, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.36 , 0 ,4 , Orange , LaneLineName1);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	//第二条车道线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.54, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.68 , 0 ,4 , Orange , LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	Send_UIPack(Drawing_Graphic2_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 画两个图形
}

/**********************************************************************************************************
*函 数 名: Shootlines_Init
*功能说明: 枪口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ShootLines_Init(void)
{
	static uint8_t ShootLineName1[] = "SL1";
	static uint8_t ShootLineName2[] = "SL2";
    static uint8_t ShootLineName3[] = "SL3";
	static uint8_t ShootLineName4[] = "SL4";
    static uint8_t ShootLineName5[] = "SL5";
	static uint8_t ShootLineName6[] = "SL6";
    static uint8_t ShootLineName7[] = "SL7";
	graphic_data_struct_t * P_graphic_data;

#if Robot_ID == 44
	//四米射击点
	P_graphic_data = Circle_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-19, SCREEN_WIDTH * 0.5-12 , 2 , 4 , Green , ShootLineName1);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
//    //四米射击圈
//	P_graphic_data = Circle_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-14, SCREEN_WIDTH * 0.5-10 , 40 , 3 , Green , ShootLineName2);
//	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
	//4m横线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-41, SCREEN_WIDTH * 0.5-12, SCREEN_LENGTH * 0.5-27 , SCREEN_WIDTH * 0.5-12 ,1 , Green , ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-11, SCREEN_WIDTH * 0.5-12, SCREEN_LENGTH * 0.5+3 , SCREEN_WIDTH * 0.5-12 ,1 , Green , ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK*3] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    //4m竖线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-19, SCREEN_WIDTH * 0.5-4, SCREEN_LENGTH * 0.5-19 , SCREEN_WIDTH * 0.5+10 ,1 , Green , ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK*4] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-19, SCREEN_WIDTH * 0.5-20, SCREEN_LENGTH * 0.5-19 , SCREEN_WIDTH * 0.5-34 ,1 , Green , ShootLineName6);
	memcpy(&data_pack[DRAWING_PACK*5] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    //凑数
    P_graphic_data = Line_Draw(1,Op_None,0, 0, SCREEN_LENGTH * 0.5-14 , SCREEN_WIDTH * 0.5-24 ,1 , Green , ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK*6] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
	Send_UIPack(Drawing_Graphic7_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK*7); // 画两个图形
#elif Robot_ID == 45
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-40, SCREEN_WIDTH * 0.5-72, SCREEN_LENGTH * 0.5+40 , SCREEN_WIDTH * 0.5-72 ,1 , Green , ShootLineName3);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5, SCREEN_WIDTH * 0.5-40, SCREEN_LENGTH * 0.5, SCREEN_WIDTH * 0.5-112 ,1 , Green , ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK]  , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-30, SCREEN_WIDTH * 0.5-92, SCREEN_LENGTH * 0.5+30 , SCREEN_WIDTH * 0.5-92 ,1 , Green , ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-20, SCREEN_WIDTH * 0.5-112, SCREEN_LENGTH * 0.5+20 , SCREEN_WIDTH * 0.5-112 ,1 , Green , ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK*3] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    Send_UIPack(Drawing_Graphic5_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK*5); // 画两个图形
#elif Robot_ID == 46
	float x_bias = -7;
	float y_bias = -2;
	//射击横线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-40+x_bias, SCREEN_WIDTH * 0.5-72+y_bias, SCREEN_LENGTH * 0.5+40+x_bias, SCREEN_WIDTH * 0.5-72 +y_bias,1 , Green , ShootLineName3);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-30+x_bias, SCREEN_WIDTH * 0.5-92+y_bias, SCREEN_LENGTH * 0.5+30+x_bias , SCREEN_WIDTH * 0.5-92+y_bias ,1 , Green , ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-20+x_bias, SCREEN_WIDTH * 0.5-112+y_bias, SCREEN_LENGTH * 0.5+20+x_bias , SCREEN_WIDTH * 0.5-112+y_bias ,1 , Green , ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK*3] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
	//射击竖线
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5+x_bias, SCREEN_WIDTH * 0.5-40+y_bias, SCREEN_LENGTH * 0.5+x_bias, SCREEN_WIDTH * 0.5-112+y_bias ,1 , Green , ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK]  , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	
    Send_UIPack(Drawing_Graphic5_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK*5); // 画两个图形
#elif Robot_ID == 47
	float x_bias = +14;
	float y_bias = -30;
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-40+x_bias, SCREEN_WIDTH * 0.5-72+y_bias, SCREEN_LENGTH * 0.5+40+x_bias , SCREEN_WIDTH * 0.5-72+y_bias ,1 , Green , ShootLineName3);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5+x_bias, SCREEN_WIDTH * 0.5-40+y_bias, SCREEN_LENGTH * 0.5+x_bias, SCREEN_WIDTH * 0.5-117+y_bias ,1 , Green , ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK]  , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
//    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-30+x_bias, SCREEN_WIDTH * 0.5-102+y_bias, SCREEN_LENGTH * 0.5+30+x_bias , SCREEN_WIDTH * 0.5-102+y_bias ,1 , Green , ShootLineName1);
//	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-20+x_bias, SCREEN_WIDTH * 0.5-117+y_bias, SCREEN_LENGTH * 0.5+20+x_bias , SCREEN_WIDTH * 0.5-117+y_bias ,1 , Green , ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK*3] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-35+x_bias, SCREEN_WIDTH * 0.5-87+y_bias, SCREEN_LENGTH * 0.5+35+x_bias , SCREEN_WIDTH * 0.5-87+y_bias ,1 , Green , ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK*4] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
    
    Send_UIPack(Drawing_Graphic5_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK*5); // 画两个图形
#endif
}

/**********************************************************************************************************
*函 数 名: CarPosture_Change
*功能说明: 车身姿态绘制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint16_t RectCenterX = SCREEN_LENGTH * 0.4;
uint16_t RectCenterY = SCREEN_WIDTH * 0.7;
uint16_t startX, startY, endX, endY;
float angle;
float angle1;
void CarPosture_Change(short Yaw_100 ,uint8_t Init_Cnt)
{
	static uint8_t LaneLineName1[] = "po1";
	static uint8_t LaneLineName2[] = "po2";
	static uint8_t LaneLineName3[] = "po3";
	static uint8_t LaneLineName4[] = "po4";
	static uint8_t LaneLineName5[] = "po5";
	static uint8_t LaneLineName6[] = "po6";
	static uint8_t LaneLineName7[] = "po7";
	graphic_data_struct_t * P_graphic_data;

	static uint16_t len = 50;
	static uint16_t centerx = 200 , centery = 700;
	angle = (Yaw_100 / 100.0f)*PI/180.0f + PI;
	angle1 = (Yaw_100 / 100.0f);

	if(angle1 <  0)
		angle1 +=360;
	uint8_t optype = Init_Cnt == 0?Op_Change:Op_Add;

	P_graphic_data = Line_Draw(0,optype,centerx,
										centery,
										centerx + len * (- arm_sin_f32(angle)),
										centery + len * (+ arm_cos_f32(angle)),4 , Orange , LaneLineName2);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	P_graphic_data = Line_Draw(0,optype,200,700,200,800  ,8 , Pink , LaneLineName6);//枪口标识线
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 画7个图形

}

/*超级电容电量绘制*/
float Length;
void CapDraw(float CapVolt,uint8_t Init_Flag)
{
    static uint8_t CapName1[] = "Out";
	static uint8_t CapName2[] = "In";
    graphic_data_struct_t * P_graphic_data;
    if(Init_Flag)
    {
        P_graphic_data = Rectangle_Draw(0,Op_Add,0.25*SCREEN_LENGTH,0.1*SCREEN_WIDTH,0.75*SCREEN_LENGTH,0.15*SCREEN_WIDTH,6,Cyan,CapName1);
        memcpy(data_pack,(uint8_t *)P_graphic_data,DRAWING_PACK);
        
        P_graphic_data = Line_Draw(0,Op_Add,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.75*SCREEN_LENGTH,0.125*SCREEN_WIDTH,27,Green,CapName2);
        memcpy(&data_pack[DRAWING_PACK],(uint8_t*)P_graphic_data,DRAWING_PACK);
        
        Send_UIPack(Drawing_Graphic2_ID,JudgeReceive.robot_id,JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 2);
    }
    else
    {
        if (CapVolt > 20.0f)
        {
            Length = 0.5*SCREEN_LENGTH;
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Green,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <= 20.0f && CapVolt >= 15.0f)
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Green,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <15.0f && CapVolt >= 9.0f)
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Yellow,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <9.0f )
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Orange,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
    }
}
    
/*字符变化发送*/
void CharChange(uint8_t Init_Flag)
{
    uint8_t GimbalNormal[]      = "NORMAL";
    uint8_t GimbalAuto[]        = "AUTO";
    uint8_t GimbalPowerdown[]   = "POWERDOWN";
    uint8_t GimbalBigBuf[]      = "BIGBUF";
    uint8_t GimbalSmlBuf[]      = "SMLBUF";
    
    uint8_t FrictionOff[]       = "OFF";
    uint8_t FrictionOn[]        = "ON";
    
    uint8_t ArmorLost[]         = "LOST";
    uint8_t ArmorID_1[]         = "ID_1";
    uint8_t ArmorID_2[]         = "ID_2";
    uint8_t ArmorID_3[]         = "ID_3";
    uint8_t ArmorID_4[]         = "ID_4";
    uint8_t ArmorID_5[]         = "ID_5";
    uint8_t ArmorID_Sentry[]    = "Sentry";
    
    uint8_t FireAuto[]          = "AUTO";
    uint8_t FireManual[]        = "MANUAL";

    /*云台状态改变*/
    static uint8_t GimbalChangeName[] = "bal";
    if(Init_Flag)
    {
        Char_Draw(0,Op_Add,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Green,GimbalChangeName,GimbalNormal);
    }
    else
    {
        switch(F405.Gimbal_Flag)
        {
            case Gimbal_Act_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Green,GimbalChangeName,GimbalNormal);
            break;
            
            case Gimbal_Powerdown_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalPowerdown),2,Pink,GimbalChangeName,GimbalPowerdown);
            break;
            
            case Gimbal_Armor_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalAuto),2,Cyan,GimbalChangeName,GimbalAuto);
            break;
            
            case Gimbal_BigBuf_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalBigBuf),2,Orange,GimbalChangeName,GimbalBigBuf);
            break;
            
            case Gimbal_SmlBuf_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.55*SCREEN_WIDTH,20,sizeof(GimbalSmlBuf),2,Orange,GimbalChangeName,GimbalSmlBuf);
            break;
        }
    }
    
    /*摩擦轮状态改变*/
    static uint8_t FrictionChangeName[] = "mcl";
    if(Init_Flag)
    {
        Char_Draw(0,Op_Add,0.9*SCREEN_LENGTH,0.50*SCREEN_WIDTH,20,sizeof(FrictionOff),2,Pink,FrictionChangeName,FrictionOff);
    }
    else
    {
        switch(F405.Fric_Flag)
        {
            case 1:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.50*SCREEN_WIDTH,20,sizeof(FrictionOn),2,Green,FrictionChangeName,FrictionOn);
            break;
            
            case 0:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.50*SCREEN_WIDTH,20,sizeof(FrictionOff),2,Pink,FrictionChangeName,FrictionOff);
            break;
            
            default:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.50*SCREEN_WIDTH,20,sizeof(FrictionOff),2,Pink,FrictionChangeName,FrictionOff);
            break;
        }
    }
    
    /*辅瞄目标状态改变*/
    static uint8_t ArmorChangeName[] = "arm";
    if(Init_Flag)
    {
        Char_Draw(0,Op_Add,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorLost),2,Pink,ArmorChangeName,ArmorLost);
    }
    else
    {
        switch(F405.Enemy_ID)
        {
            case ARMOR_ID_1:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_1),2,Green,ArmorChangeName,ArmorID_1);
            break;
            
            case ARMOR_ID_2:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_2),2,Green,ArmorChangeName,ArmorID_2);
            break;
            
            case ARMOR_ID_3:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_3),2,Green,ArmorChangeName,ArmorID_3);
            break;
                        
            case ARMOR_ID_4:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_4),2,Green,ArmorChangeName,ArmorID_4);
            break;
                                    
            case ARMOR_ID_5:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_5),2,Green,ArmorChangeName,ArmorID_5);
            break;
            
            case ARMOR_ID_Sentry:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorID_Sentry),2,Green,ArmorChangeName,ArmorID_Sentry);
            break;
            
            default:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.45*SCREEN_WIDTH,20,sizeof(ArmorLost),2,Pink,ArmorChangeName,ArmorLost);
            break;
        }
    }
    
    /*切换开火模式*/
    static uint8_t FireChangeName[] = "fcn";
    if(Init_Flag)
    {
        Char_Draw(0,Op_Add,0.9*SCREEN_LENGTH,0.40*SCREEN_WIDTH,20,sizeof(FireManual),2,Green,FireChangeName,FireManual);
    }
    else
    {
        switch(F405.AutoFire_Flag)
        {
            case 1:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.40*SCREEN_WIDTH,20,sizeof(FireAuto),2,Cyan,FireChangeName,FireAuto);
            break;
            
            case 0:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.40*SCREEN_WIDTH,20,sizeof(FireManual),2,Green,FireChangeName,FireManual);
            break;
            
            default:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.40*SCREEN_WIDTH,20,sizeof(FireManual),2,Green,FireChangeName,FireManual);
            break;
        }
    }
}

/**********************************************************************************************************
*函 数 名: Char_Init
*功能说明: 字符数据初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Char_Init(void)
{
	static uint8_t PitchName[] = "pit";
    static uint8_t GimbalName[] = "gim";
    static uint8_t FrictionName[] = "fri";
    static uint8_t ArmorName[] = "amr";
    static uint8_t CapStaticName[] = "cpt";
    static uint8_t FireName[] = "frm";
	/*				PITCH字符			*/
	uint8_t pitch_char[] = "PITCH :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, 20, sizeof(pitch_char) ,2 , Yellow , PitchName,pitch_char);
    
    /*              GIMBAL字符*/
	uint8_t gimbal_char[] = "GIMBAL :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(gimbal_char) ,2 , Yellow , GimbalName,gimbal_char);
    
    /*              FRICTION字符*/
	uint8_t friction_char[] = "FRICTION :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(friction_char) ,2 , Yellow , FrictionName,friction_char);
    
    /*              ARMOR字符*/
	uint8_t armor_char[] = "ARMOR :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(armor_char) ,2 , Yellow , ArmorName,armor_char);
    
    /*              FIREMODE字符*/
	uint8_t fire_char[] = "FIREMODE :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(fire_char) ,2 , Yellow , FireName,fire_char);
    
    /*              CAP字符*/
	uint8_t cap_char[] = "CAP :       V";
	Char_Draw(0,Op_Add, 0.40 * SCREEN_LENGTH, 0.1 * SCREEN_WIDTH, 30, sizeof(cap_char) ,2 , Yellow , CapStaticName,cap_char);

}


/**********************************************************************************************************
*函 数 名: PitchUI_Change
*功能说明: Pitch角度绘制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PitchUI_Change(float Pitch ,uint8_t Init_Cnt)
{
	static uint8_t PitchName[] = "Pit";
	static uint8_t optype;

	optype = (Init_Cnt == 0)?Op_Change:Op_Add;
	 
	graphic_data_struct_t * P_graphic_data;
    
	P_graphic_data = FloatData_Draw(0,optype,0.90 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH , Pitch, 20 ,4 , 2, Green, PitchName);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK); // 画字符

}

/**********************************************************************************************************
*函 数 名: CapUI_Change
*功能说明: 电容电量绘制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CapUI_Change(float CapVolt ,uint8_t Init_Cnt)
{
	static uint8_t CapName[] = "cpv";
	static uint8_t optype;

	optype = (Init_Cnt == 0)?Op_Change:Op_Add;
	 
	graphic_data_struct_t * P_graphic_data;
	P_graphic_data = FloatData_Draw(0,optype,0.42 * SCREEN_LENGTH + 100, 0.1 * SCREEN_WIDTH , CapVolt, 30 ,4 , 2, Orange, CapName);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK); // 画字符

}

/**********************************************************************************************************
*函 数 名: GraphicSendtask
*功能说明: 图形发送任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
extern ext_student_interactive_char_header_data_t custom_char_draw;  //自定义字符绘制
uint32_t UITask_RunTime = 0UL;
uint32_t UISendTask_high_water;
uint8_t InitFlag = 1;//F405.Graphic_Init_Flag
float Pitch;
void GraphicSendtask(void *pvParameters)
{
	// static u8 Init_Cnt = 20;//图形初始化次数，由于丢包可以多初始化几次
	static int Op_Type = Op_None;
   	while (1) 
	{
		UITask_RunTime++;
        if(F405.Graphic_Init_Flag)
        {
             InitFlag = 10;
        }

        /*静止UI绘制*/

        
        /*动态UI绘制*/
		CarPosture_Change(F405.Yaw_100, InitFlag); //车辆姿态绘制
        
		Pitch = (float)(F405.Pitch_100/100.0f);    //pitch角度
        if(UITask_RunTime%3 == 1)
		PitchUI_Change(Pitch , InitFlag);
        
//      CapDraw(AD_actual_value , InitFlag);       //超级电容电量
        if(UITask_RunTime%3 == 2)
        CapUI_Change(superpower.actual_vol, InitFlag);    //超级电容电压
        
        if(UITask_RunTime%5 == 1)
        CharChange(InitFlag);                      //字符UI变化
         
        if(InitFlag > 0)
        {
            InitFlag --;
            Char_Init();              //字符
    //		Lanelines_Init();         //车道线
            ShootLines_Init();        //枪口线
        }
        
//		
		IWDG_Feed();//喂狗
		

#if INCLUDE_uxTaskGetStackHighWaterMark
		UISendTask_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
//		vTaskDelay(100); 	 
    }
}
