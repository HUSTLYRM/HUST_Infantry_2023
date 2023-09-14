#include "debug.h"

GlobalDebugger global_debugger;

// JSCOPE RTT MODE
#ifdef JSCOPE_RTT_MODE

char JS_RTT_UpBuffer[512]; // J-Scope RTT Buffer
int JS_RTT_Channel = 1;    // J-Scope RTT Channel
const char jscope_symbol[] = "JScope_f4f4";
acValBuffer tempBuffer;

void JscopeRTTInit()
{
    SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, jscope_symbol, &JS_RTT_UpBuffer[0], sizeof(JS_RTT_UpBuffer), SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
}

void JscopeWrite(float setPoint, float samplePoint)
{
    tempBuffer.set_point = setPoint;
    tempBuffer.raw_point = samplePoint;
    SEGGER_RTT_Write(JS_RTT_Channel, &tempBuffer, sizeof(tempBuffer));
}

#endif // JSCOPE_RTT_MODE

float Ozone[8]; //使用Ozone显示的变量

int8_t checkIMUOn()
{
    for (int8_t i = 0; i < 2; i++)
    {
        //接收数据小于一定数认为是未开启
        if (global_debugger.imu_debugger[i].recv_msgs_num < 50)
        {
            return FALSE;
        }
    }
    return TRUE;
}
