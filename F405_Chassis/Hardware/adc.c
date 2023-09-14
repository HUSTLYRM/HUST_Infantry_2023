#include "main.h"

uint16_t ADC_ConvertedValue[ADC_NUM * ADC_SAMPLE_NUM];

/**
 * @brief 超级电容ADC配置
 * @param[in] void
 */
void PowerAdc_Configuration()
{
    GPIO_InitTypeDef gpio;

    POWER_ADC_AHBxPeriphClockCmd(POWER_ADC_RCC_AHBxPeriph_GPIOx, ENABLE);
    gpio.GPIO_Pin = POWER_ADC_GPIO_Pin_x;
    gpio.GPIO_Mode = GPIO_Mode_AIN; //模拟输入
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(POWER_ADC_GPIOx, &gpio);

    // DMA设置
    DMA_InitTypeDef dma;
    POWER_ADC_DMA_AHBxPeriphClockCmd(POWER_ADC_RCC_AHBxPeriph_DMAx, ENABLE);
    DMA_DeInit(POWER_ADC_DMAx_Streamx);
    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = POWER_ADC_ADCx_DR_Address;  //数据传输的外设首地址
    dma.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue; //自己定义待发送数组的首地址，要强制转换为32位
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = ADC_NUM * ADC_SAMPLE_NUM;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    DMA_Init(POWER_ADC_DMAx_Streamx, &dma);
    DMA_Cmd(POWER_ADC_DMAx_Streamx, ENABLE);

    //通用ADC设置
    // ADC_DeInit();
    // ADC_CommonInitTypeDef adc_comm;
    // adc_comm.ADC_Mode = ADC_Mode_Independent;                     //只用了一个ADC
    // adc_comm.ADC_Prescaler = ADC_Prescaler_Div6;                  //时钟6分频
    // adc_comm.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             // DMA使能，用于12位精度
    // adc_comm.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //两个采样阶段之间的延迟5个时钟

    // ADC1配置
    ADC_InitTypeDef adc;
    POWER_ADC_ADC_RCC_APBxPeriphClockCmd(POWER_ADC_RCC_APBxPeriph_ADCx, ENABLE);
    ADC_Init(POWER_ADC_ADCx, &adc);
    adc.ADC_Resolution = ADC_Resolution_12b;                  //可选的分辨率有12位、10位、8位和6位。分辨率越高，AD转换数据精度越高，转换时间也越长；分辨率越低，AD转换数据精度越低，转换时间也越短。
    adc.ADC_ScanConvMode = ENABLE;                            //各通道顺序扫描,若采集多个通道，必须开启此模式
    adc.ADC_ContinuousConvMode = ENABLE;                      //配置是启动自动连续转换还是单次转换。
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; //禁止触发,使用软件触发
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = ADC_NUM; // 3个通道
    ADC_Init(POWER_ADC_ADCx, &adc);
    ADC_TempSensorVrefintCmd(ENABLE); //开启温度通道

    // 配置 ADC 通道转换顺序和采样时间周期
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);  // u_in
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_9, 2, ADC_SampleTime_480Cycles);  // u_in
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_16, 3, ADC_SampleTime_480Cycles); // u_in  温度通道
    ADC_DMACmd(POWER_ADC_ADCx, ENABLE);

    // 使能DMA请求 after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(POWER_ADC_ADCx, ENABLE); // 使能DMA请求 after last transfer (Single-ADC mode)
    ADC_DMACmd(POWER_ADC_ADCx, ENABLE);                         // 使能ADC DMA
    ADC_Cmd(POWER_ADC_ADCx, ENABLE);
    ADC_SoftwareStartConv(POWER_ADC_ADCx); //开始adc转换，软件触发
}

/**
 * @brief 超级电容DAC配置
 * @param[in] void
 */
void PowerDac_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* GPIOA Periph clock enable */
    POWER_DAC_RCC_AHBxPeriphClockCmd(POWER_DAC_RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_InitStructure.GPIO_Pin = POWER_DAC_GPIO_Pin_x1 | POWER_DAC_GPIO_Pin_x1; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                //模拟输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POWER_DAC_GPIOx, &GPIO_InitStructure);
    GPIO_SetBits(POWER_DAC_GPIOx, POWER_DAC_GPIO_Pin_x1 | POWER_DAC_GPIO_Pin_x1); // PA.4 输出高

    DAC_InitTypeDef DAC_InitType;
    POWER_DAC_DAC_RCC_APBxPeriphClockCmd(POWER_DAC_DAC_RCC_APBxPeriph_DAC, ENABLE); //使能DAC通道时钟
    DAC_InitType.DAC_Trigger = DAC_Trigger_None;                                    //不使用触发功能 TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;                      //不使用波形发生
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;            //屏蔽、幅值设置
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable;                       // DAC1输出缓存关闭 BOFF1=1
    DAC_Init(POWER_DAC_Channel_x1, &DAC_InitType);                                  //初始化DAC通道1
    DAC_Init(POWER_DAC_Channel_x2, &DAC_InitType);                                  //初始化DAC通道1
    DAC_Cmd(POWER_DAC_Channel_x1, ENABLE);                                          //使能DAC1
    DAC_Cmd(POWER_DAC_Channel_x2, ENABLE);                                          //使能DAC2
    POWER_DAC_SetChannelx1Data(DAC_Align_12b_R, 0);                                 // 12位右对齐数据格式设置DAC值
    POWER_DAC_SetChannelx2Data(DAC_Align_12b_R, 0);                                 // 12位右对齐数据格式设置DAC值
}

/**
 * @brief 超级电容充电IO配置
 * @param[in] void
 */
void PowerChargeIO_Configuration()
{
    GPIO_InitTypeDef gpio_init;
    POWER_CHARGE_RCC_AHBxPeriphClockCmd(POWER_CHARGE_RCC_AHBxPeriph_GPIOx | POWER_UNCHARGE_RCC_AHBxPeriph_GPIOx, ENABLE);

    // 充电IO口
    gpio_init.GPIO_Pin = POWER_CHARGE_GPIO_Pin_x1 | POWER_CHARGE_GPIO_Pin_x2;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(POWER_CHARGE_GPIOx, &gpio_init);

    Charge_Off;                                                 //初始化时不充电
    GPIO_SetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x2); //拉高Diode Mode

    // 放电IO口
    gpio_init.GPIO_Pin = POWER_UNCHARGE_GPIO_Pin_x1 | POWER_UNCHARGE_GPIO_Pin_x2;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(POWER_UNCHARGE_GPIOx, &gpio_init);
}

void SuperPower_Configuration()
{
    PowerAdc_Configuration();

    PowerDac_Configuration();

    PowerChargeIO_Configuration();

    Bat_on;
    CAP_off;
}
