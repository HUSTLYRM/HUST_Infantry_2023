#ifndef __QUEUE_H
#define __QUEUE_H

//==============================================================================
//  ��������
//==============================================================================
#define QUEUE_DATA_T  double         //�����������Ͷ���

//==============================================================================
//  ���Ͷ���
//==============================================================================
typedef struct QUEUE_HandleTypeDef{
    unsigned int head;                      //����ͷָ��
    unsigned int tail;                      //����βָ��
    unsigned int buffer_length;             //���л��泤�ȣ���ʼ��ʱ��ֵ��
    QUEUE_DATA_T * buffer;		              //���л������飨��ʼ��ʱ��ֵ��
}QUEUE_HandleTypeDef;

typedef enum
{
    QUEUE_OK       = 0x00U,                 //����OK
    QUEUE_ERROR    = 0x01U,                 //���д���
    QUEUE_BUSY     = 0x02U,                 //����æ
    QUEUE_TIMEOUT  = 0x03U,                 //���г�ʱ
    QUEUE_OVERLOAD = 0x04U,                 //��������
    QUEUE_VOID     = 0x05U                  //�����ѿ�
} QUEUE_StatusTypeDef;


//==============================================================================
//  ������������
//==============================================================================
extern void Queue_Init(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * buffer, unsigned int len);
extern void Queue_Clear(QUEUE_HandleTypeDef * hqueue);
extern unsigned int Queue_Count(QUEUE_HandleTypeDef * hqueue);
extern QUEUE_StatusTypeDef Queue_Push(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T data);
extern unsigned int Queue_Push_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len);
extern QUEUE_StatusTypeDef Queue_Pop(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata);
extern unsigned int Queue_Pop_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len); 
extern QUEUE_StatusTypeDef Queue_Peek(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata);
extern unsigned int Queue_Peek_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len);

#endif

