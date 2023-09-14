#include "main.h"

//==============================================================================
//  ��������
//==============================================================================
void Queue_Init(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * buffer, unsigned int len);
void Queue_Clear(QUEUE_HandleTypeDef * hqueue);
unsigned int Queue_Count(QUEUE_HandleTypeDef * hqueue);
QUEUE_StatusTypeDef Queue_Push(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T data);
unsigned int Queue_Push_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len);
QUEUE_StatusTypeDef Queue_Pop(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata);
unsigned int Queue_Pop_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len);
QUEUE_StatusTypeDef Queue_Peek(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata);
unsigned int Queue_Peek_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len);



//==============================================================================
//  �������ƣ�Queue_Init
//  �������ܣ���ʼ�������������У�ÿ�����б�����ִ�иú�������ʹ�á�
//  ����������hqueue           ���б���ָ��
//  ����������buffer           ���л�������ַ
//  ����������len              ���л���������
//  �������أ�void
//==============================================================================
void Queue_Init(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * buffer, unsigned int len)
{
    hqueue->buffer = buffer;
    hqueue->buffer_length = len;
    Queue_Clear(hqueue);
}

//==============================================================================
//  �������ƣ�Queue_Clear
//  �������ܣ���ն���
//  ����������hqueue           ���б���ָ��
//  �������أ�void
//==============================================================================
void Queue_Clear(QUEUE_HandleTypeDef * hqueue)
{
    hqueue->head = 0;
    hqueue->tail = 0;
}

//==============================================================================
//  �������ƣ�Queue_Count
//  �������ܣ���ȡ���������ݵĸ���
//  ����������hqueue           ���б���ָ��
//  �������أ����������ݵĸ���
//==============================================================================
unsigned int Queue_Count(QUEUE_HandleTypeDef * hqueue)
{
    if(hqueue->head <= hqueue->tail)
    {
        return (unsigned int)(hqueue->tail - hqueue->head);
    }
    else
    {
        return (unsigned int)(hqueue->buffer_length + hqueue->tail - hqueue->head);
    }
}

//==============================================================================
//  �������ƣ�Queue_Push
//  �������ܣ�ѹ�����ݵ�������
//  ����������hqueue           ���б���ָ��
//  ����������data             ��ѹ����е�����
//  �������أ�����״̬
//==============================================================================
QUEUE_StatusTypeDef Queue_Push(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T data)
{
    unsigned int tmp = (hqueue->tail + 1) % hqueue->buffer_length;

    if(tmp == hqueue->head)
    {
        return QUEUE_OVERLOAD;
    }
    else
    {
        hqueue->buffer[hqueue->tail] = data;
        hqueue->tail = tmp;
        return QUEUE_OK;
    }
}

//==============================================================================
//  �������ƣ�Queue_Push_Array
//  �������ܣ�ѹ��һ�����ݵ�������
//  ����������hqueue           ���б���ָ��
//  ����������pdatas           ��ѹ����е������ַ
//  ����������len              ��ѹ����е����鳤��
//  �������أ��ɹ�ѹ��������ݵ�����
//==============================================================================
unsigned int Queue_Push_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len)
{
    unsigned int i;
    for(i=0; i<len; i++)
    {
        if(Queue_Push(hqueue,pdatas[i]) == QUEUE_OVERLOAD)
        {
            break;
        }
    }
    return i;
}


//==============================================================================
//  �������ƣ�Queue_Pop
//  �������ܣ��Ӷ����е�������
//  ����������hqueue           ���б���ָ��
//  ����������pdata            ���������е����ݻ����ַ
//  �������أ�����״̬
//==============================================================================
QUEUE_StatusTypeDef Queue_Pop(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata)
{
    if(hqueue->head == hqueue->tail)
    {
        return QUEUE_VOID;
    }
    else
    {
        *pdata = hqueue->buffer[hqueue->head];
        hqueue->head = (hqueue->head + 1) % hqueue->buffer_length;
        return QUEUE_OK;
    }
}

//==============================================================================
//  �������ƣ�Queue_Pop_Array
//  �������ܣ��Ӷ����е���һ������
//  ����������hqueue           ���б���ָ��
//  ����������pdatas           ���������е����ݻ����ַ
//  ����������len              ���������е����ݵ���󳤶�
//  �������أ�ʵ�ʵ������ݵ�����
//==============================================================================
unsigned int Queue_Pop_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len)
{
    unsigned int i;
    for(i=0; i<len; i++)
    {
        if(Queue_Pop(hqueue, &pdatas[i]) == QUEUE_VOID)
        {
            break;
        }
    }
    return i;
}


//==============================================================================
//  �������ƣ�Queue_Peek
//  �������ܣ��Ӷ���ͷ���������ݣ���ɾ�������е����ݣ�
//  ����������hqueue           ���б���ָ��
//  ����������pdata            �����ض��е����ݻ����ַ
//  �������أ�����״̬
//==============================================================================
QUEUE_StatusTypeDef Queue_Peek(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdata)
{
    if(hqueue->head == hqueue->tail)
    {
        return QUEUE_VOID;
    }
    else
    {
        *pdata = hqueue->buffer[hqueue->head];
        return QUEUE_OK;
    }
}

//==============================================================================
//  �������ƣ�Queue_Peek_Array
//  �������ܣ��Ӷ����з���һ�����ݣ���ɾ�������е����ݣ�
//  ����������hqueue           ���б���ָ��
//  ����������pdatas           �����ض��е����ݻ����ַ
//  ����������len              �����ض��е����ݵ���󳤶�
//  �������أ�ʵ�ʷ������ݵ�����
//==============================================================================
unsigned int Queue_Peek_Array(QUEUE_HandleTypeDef * hqueue, QUEUE_DATA_T * pdatas, unsigned int len)
{
    unsigned int i;
    if(hqueue->head == hqueue->tail)
    {
        return 0;
    }
    if(Queue_Count(hqueue) < len)
    {
        len = Queue_Count(hqueue);
    }
    for(i=0; i<len; i++)
    {
        pdatas[i] = hqueue->buffer[(hqueue->head + i) % hqueue->buffer_length];
    }
    return len;
}




