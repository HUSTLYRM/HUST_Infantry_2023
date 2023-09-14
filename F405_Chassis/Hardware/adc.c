#include "main.h"

uint16_t ADC_ConvertedValue[ADC_NUM * ADC_SAMPLE_NUM];

/**
 * @brief ��������ADC����
 * @param[in] void
 */
void PowerAdc_Configuration()
{
    GPIO_InitTypeDef gpio;

    POWER_ADC_AHBxPeriphClockCmd(POWER_ADC_RCC_AHBxPeriph_GPIOx, ENABLE);
    gpio.GPIO_Pin = POWER_ADC_GPIO_Pin_x;
    gpio.GPIO_Mode = GPIO_Mode_AIN; //ģ������
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(POWER_ADC_GPIOx, &gpio);

    // DMA����
    DMA_InitTypeDef dma;
    POWER_ADC_DMA_AHBxPeriphClockCmd(POWER_ADC_RCC_AHBxPeriph_DMAx, ENABLE);
    DMA_DeInit(POWER_ADC_DMAx_Streamx);
    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = POWER_ADC_ADCx_DR_Address;  //���ݴ���������׵�ַ
    dma.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue; //�Լ����������������׵�ַ��Ҫǿ��ת��Ϊ32λ
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

    //ͨ��ADC����
    // ADC_DeInit();
    // ADC_CommonInitTypeDef adc_comm;
    // adc_comm.ADC_Mode = ADC_Mode_Independent;                     //ֻ����һ��ADC
    // adc_comm.ADC_Prescaler = ADC_Prescaler_Div6;                  //ʱ��6��Ƶ
    // adc_comm.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             // DMAʹ�ܣ�����12λ����
    // adc_comm.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //���������׶�֮����ӳ�5��ʱ��

    // ADC1����
    ADC_InitTypeDef adc;
    POWER_ADC_ADC_RCC_APBxPeriphClockCmd(POWER_ADC_RCC_APBxPeriph_ADCx, ENABLE);
    ADC_Init(POWER_ADC_ADCx, &adc);
    adc.ADC_Resolution = ADC_Resolution_12b;                  //��ѡ�ķֱ�����12λ��10λ��8λ��6λ���ֱ���Խ�ߣ�ADת�����ݾ���Խ�ߣ�ת��ʱ��ҲԽ�����ֱ���Խ�ͣ�ADת�����ݾ���Խ�ͣ�ת��ʱ��ҲԽ�̡�
    adc.ADC_ScanConvMode = ENABLE;                            //��ͨ��˳��ɨ��,���ɼ����ͨ�������뿪����ģʽ
    adc.ADC_ContinuousConvMode = ENABLE;                      //�����������Զ�����ת�����ǵ���ת����
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; //��ֹ����,ʹ���������
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = ADC_NUM; // 3��ͨ��
    ADC_Init(POWER_ADC_ADCx, &adc);
    ADC_TempSensorVrefintCmd(ENABLE); //�����¶�ͨ��

    // ���� ADC ͨ��ת��˳��Ͳ���ʱ������
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);  // u_in
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_9, 2, ADC_SampleTime_480Cycles);  // u_in
    ADC_RegularChannelConfig(POWER_ADC_ADCx, ADC_Channel_16, 3, ADC_SampleTime_480Cycles); // u_in  �¶�ͨ��
    ADC_DMACmd(POWER_ADC_ADCx, ENABLE);

    // ʹ��DMA���� after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(POWER_ADC_ADCx, ENABLE); // ʹ��DMA���� after last transfer (Single-ADC mode)
    ADC_DMACmd(POWER_ADC_ADCx, ENABLE);                         // ʹ��ADC DMA
    ADC_Cmd(POWER_ADC_ADCx, ENABLE);
    ADC_SoftwareStartConv(POWER_ADC_ADCx); //��ʼadcת�����������
}

/**
 * @brief ��������DAC����
 * @param[in] void
 */
void PowerDac_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* GPIOA Periph clock enable */
    POWER_DAC_RCC_AHBxPeriphClockCmd(POWER_DAC_RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_InitStructure.GPIO_Pin = POWER_DAC_GPIO_Pin_x1 | POWER_DAC_GPIO_Pin_x1; // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                //ģ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POWER_DAC_GPIOx, &GPIO_InitStructure);
    GPIO_SetBits(POWER_DAC_GPIOx, POWER_DAC_GPIO_Pin_x1 | POWER_DAC_GPIO_Pin_x1); // PA.4 �����

    DAC_InitTypeDef DAC_InitType;
    POWER_DAC_DAC_RCC_APBxPeriphClockCmd(POWER_DAC_DAC_RCC_APBxPeriph_DAC, ENABLE); //ʹ��DACͨ��ʱ��
    DAC_InitType.DAC_Trigger = DAC_Trigger_None;                                    //��ʹ�ô������� TEN1=0
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;                      //��ʹ�ò��η���
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;            //���Ρ���ֵ����
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable;                       // DAC1�������ر� BOFF1=1
    DAC_Init(POWER_DAC_Channel_x1, &DAC_InitType);                                  //��ʼ��DACͨ��1
    DAC_Init(POWER_DAC_Channel_x2, &DAC_InitType);                                  //��ʼ��DACͨ��1
    DAC_Cmd(POWER_DAC_Channel_x1, ENABLE);                                          //ʹ��DAC1
    DAC_Cmd(POWER_DAC_Channel_x2, ENABLE);                                          //ʹ��DAC2
    POWER_DAC_SetChannelx1Data(DAC_Align_12b_R, 0);                                 // 12λ�Ҷ������ݸ�ʽ����DACֵ
    POWER_DAC_SetChannelx2Data(DAC_Align_12b_R, 0);                                 // 12λ�Ҷ������ݸ�ʽ����DACֵ
}

/**
 * @brief �������ݳ��IO����
 * @param[in] void
 */
void PowerChargeIO_Configuration()
{
    GPIO_InitTypeDef gpio_init;
    POWER_CHARGE_RCC_AHBxPeriphClockCmd(POWER_CHARGE_RCC_AHBxPeriph_GPIOx | POWER_UNCHARGE_RCC_AHBxPeriph_GPIOx, ENABLE);

    // ���IO��
    gpio_init.GPIO_Pin = POWER_CHARGE_GPIO_Pin_x1 | POWER_CHARGE_GPIO_Pin_x2;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(POWER_CHARGE_GPIOx, &gpio_init);

    Charge_Off;                                                 //��ʼ��ʱ�����
    GPIO_SetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x2); //����Diode Mode

    // �ŵ�IO��
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
