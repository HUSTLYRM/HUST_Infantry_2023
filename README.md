# Infantry_HUST_2023

华中科技大学狼牙战队2023赛季步兵电控代码

## 简介

1. 硬件配置

硬件使用STM32F405RGT6作为主控芯片，主控板为自研通用云台板和超级电容板。

陀螺仪采用ICM20602

电机型号为GM6020、M3508、M2006，使用C610和C620电调。

使用Jlink在SWD模式下调试。

2. 功能

- 舵轮+麦轮运动解算控制
- 功率预测和限制                    
- 被动电容控制
- 客户端UI界面的绘制
- 卡尔曼滤波陀螺仪解算        
- 摩擦轮掉速检测热量限制
- 双轴云台控制

3. 部署需要调节的参数有

- 陀螺仪控制模式和电机角控制模式下的云台pid参数
- 底盘四轮驱动pid参数
- 轮组型号宏定义
- 车辆ID宏定义
- 云台初始角度电机编码器参数
- 软件限位角度
- 功率限制阈值参数
- 超级电容的充放电电流限制

## 环境配置

- Keil μVision V5.30（整车主体工程部分）
- CubeMX V6.6.1（陀螺仪硬件驱动部分）
- Jscope（可视化调试工具）

## 文件结构

```
│  功率控制.pdf
│  步兵电路框图.pdf
│  步兵软件框图.pdf
│
├─F405_Chassis						   //底盘工程文件夹
│  ├─Algorithm							//算法层
│  │      algorithmOfCRC.c
│  │      algorithmOfCRC.h
│  │      DataScope_DP.C
│  │      DataScope_DP.h
│  │      FIR.c
│  │      FIR.h
│  │      pid.c
│  │      pid.h
│  │
│  ├─FreeRTOS							//FreeRTOS库
│  │
│  ├─Hardware							//外设驱动层
│  │      adc.c
│  │      adc.h
│  │      can1.c
│  │      can1.h
│  │      can2.c
│  │      can2.h
│  │      counter.c
│  │      counter.h
│  │      dac.c
│  │      dac.h
│  │      i2c.c
│  │      i2c.h
│  │      ina260.c
│  │      ina260.h
│  │      iwdg.c
│  │      iwdg.h
│  │      tim2.c
│  │      tim2.h
│  │      tim4.c
│  │      tim4.h
│  │      uart4.c
│  │      uart4.h
│  │      uart5.c
│  │      uart5.h
│  │      uart6.c
│  │      uart6.h
│  │      usart1.c
│  │      usart1.h
│  │      usart2.c
│  │      usart2.h
│  │      usart2_2.c
│  │      usart3.c
│  │      usart3.h
│  │      WTBI_MOTOR_CONTROL.c
│  │      WTBI_MOTOR_CONTROL.h
│  │
│  ├─Library							//STM32F4标准库文件
│  │
│  ├─RTOS_Test					//用于测试RTOS运行的模块
│  │      CPU_Task.c
│  │      CPU_Task.h
│  │      debug.c
│  │      debug.h
│  │      os_tick.c
│  │      os_tick.h
│  │      tools.c
│  │      tools.h
│  │
│  ├─SD								//SD卡模块
│  │      bsp_spi_sdcard.c
│  │      bsp_spi_sdcard.h
│  │      diskio.c
│  │      diskio.h
│  │      ff.c
│  │      ff.h
│  │      ffconf.h
│  │      integer.h
│  │      SDCardTask.c
│  │      SDCardTask.h
│  │
│  ├─SEGGER_RTT_V784	//可视化调试SEGGER库文件
│  │
│  ├─Task							//任务层
│  │      CharSendTask.c
│  │      CharSendTask.h
│  │      ChassisTask.c
│  │      ChassisTask.h
│  │      DataReceiveTask.c
│  │      DataReceiveTask.h
│  │      DataSendTask.c
│  │      DataSendTask.h
│  │      GraphicsSendTask.c
│  │      GraphicsSendTask.h
│  │      JumpCal_Task.c
│  │      JumpCal_Task.h
│  │      PowerControlTask.c
│  │      PowerControlTask.h
│  │      StartTask.c
│  │      StartTask.h
│  │      ZeroCheckTask.c
│  │      ZeroCheckTask.h
│  │
│  └─User
│      │  FreeRTOSConfig.h
│      │  main.c
│      │  main.h
│      │  startup_stm32f40_41xxx.lst
│      │  stm32f4xx.h
│      │  stm32f4xx_conf.h
│      │  stm32f4xx_it.c
│      │  stm32f4xx_it.h
│      │  system_stm32f4xx.c
│      │  system_stm32f4xx.h
│      │



├─F405_Gimbal									//云台工程文件夹
│  ├─Algorithm									//算法层
│  │      algorithmOfCRC.c
│  │      algorithmOfCRC.h
│  │      pid.c
│  │      pid.h
│  │      queueData.c
│  │      queueData.h
│  │      TD.c
│  │      TD.h
│  │      user_lib.c
│  │      user_lib.h
│  │
│  ├─FreeRTOS								//FreeRTOS库文件
│  │
│  ├─INS											//陀螺仪解算模块
│  │      debug.c
│  │      debug.h
│  │      GimbalEstimateTask.c
│  │      GimbalEstimateTask.h
│  │      icm20602.c
│  │      icm20602.h
│  │      ins.c
│  │      ins.h
│  │      kalman_filter.c
│  │      kalman_filter.h
│  │      QuaternionEKF.c
│  │      QuaternionEKF.h
│  │      RtosTaskCheck.c
│  │      RtosTaskCheck.h
│  │
│  ├─Library								//STM32F4标准库文件
│  │
│  ├─Mylib								   //外设驱动层和硬件驱动
│  │      adc.c
│  │      adc.h
│  │      can1.c
│  │      can1.h
│  │      can2.c
│  │      can2.h
│  │      counter.c
│  │      counter.h
│  │      FrictionWheel.c
│  │      FrictionWheel.h
│  │      gpio.c
│  │      gpio.h
│  │      gyro_uart.c
│  │      gyro_uart.h
│  │      iwdg.c
│  │      iwdg.h
│  │      laser.c
│  │      laser.h
│  │      led.c
│  │      led.h
│  │      MicroSw.c
│  │      MicroSw.h
│  │      MySensors.c
│  │      mySensors.h
│  │      pc_serial.c
│  │      pc_serial.h
│  │      pc_uart.c
│  │      pc_uart.h
│  │      SteeringEngine.c
│  │      SteeringEngine.h
│  │      tim4.c
│  │      tim4.h
│  │      tim7.c
│  │      tim7.h
│  │      uart4.c
│  │      uart4.h
│  │      uart6.c
│  │      uart6.h
│  │      usart1.c
│  │      usart1.h
│  │      usart2.c
│  │      usart2.h
│  │      usart3.c
│  │      usart3.h
│  │
│  ├─RTOS_Test					//RTOS运行测试工具
│  │      CPU_Task.c
│  │      CPU_Task.h
│  │      os_tick.c
│  │      os_tick.h
│  │      tools.c
│  │      tools.h
│  │
│  ├─SD								//SD卡模块
│  │      bsp_spi_sdcard.c
│  │      bsp_spi_sdcard.h
│  │      diskio.c
│  │      diskio.h
│  │      ff.c
│  │      ff.h
│  │      ffconf.h
│  │      integer.h
│  │      SDCardTask.c
│  │      SDCardTask.h
│  │
│  ├─SEGGER_RTT_V784		//可视化调试SEGGER库文件
│  │
│  ├─Task								//任务层
│  │      ActionTask.c
│  │      ActionTask.h
│  │      ChassisTask.c
│  │      ChassisTask.h
│  │      ControlTask.c
│  │      ControlTask.h
│  │      DataReceiveTask.c
│  │      DataReceiveTask.h
│  │      DataSendTask.c
│  │      DataSendTask.h
│  │      GimbalTask.c
│  │      GimbalTask.h
│  │      ShootTask.c
│  │      ShootTask.h
│  │      Start_Task.c
│  │      Start_Task.h
│  │      usart4_gryo.c
│  │      usart4_gryo.h
│  │      ZeroCheckTask.c
│  │      ZeroCheckTask.h
│  │
│  └─User
│          arm_cortexM4lf_math.lib
│          FreeRTOSConfig.h
│          main.c
│          main.h
│          startup_stm32f40_41xxx.s
│          stm32f4xx.h
│          stm32f4xx_conf.h
│          stm32f4xx_it.c
│          stm32f4xx_it.h
│          system_stm32f4xx.c
│          system_stm32f4xx.h
│
└─IMU													 //IMU工程文件夹
    │  .gitattributes
    │  .mxproject
    │  IMU_2022.ioc
    │
    ├─Core												//自定义外设配置层
    │  ├─Inc
    │  │      can.h
    │  │      dma.h
    │  │      gpio.h
    │  │      main.h
    │  │      spi.h
    │  │      stm32f0xx_hal_conf.h
    │  │      stm32f0xx_it.h
    │  │      tim.h
    │  │
    │  └─Src
    │          can.c
    │          dma.c
    │          gpio.c
    │          main.c
    │          spi.c
    │          stm32f0xx_hal_msp.c
    │          stm32f0xx_it.c
    │          system_stm32f0xx.c
    │          tim.c
    │
    ├─Drivers							//HAL库文件
    │  │
    │  └─STM32F0xx_HAL_Driver
    │
    ├─Fusion						//融合滤波算法
    │      Fusion.h
    │      FusionAhrs.c
    │      FusionAhrs.h
    │      FusionBias.c
    │      FusionBias.h
    │      FusionCalibration.h
    │      FusionCompass.c
    │      FusionCompass.h
    │      FusionTypes.h
    │
    └─MDK-ARM				//keil工程文件及用户自定义文件
        │  AHRS.c
        │  AHRS.h
        │  icm.c
        │  icm.h
        │  iic.c
        │  iic.h
        │  Kalman_Filter.c
        │  Kalman_Filter.h
        │  qmc5883l.c
        │  qmc5883l.h
        │  startup_stm32f042x6.lst
        │  startup_stm32f042x6.s
        │  struct_typedef.h
        │  TempControl.c
        │  TempControl.h
        │  tim3.c
```



## 编程规范

### 1. 文件规范

- 命名：大驼峰型(ChassisTask.c, AlgorithmOfCRC.c)
- 所有源文件在文件开头添加注释，注释规范如下：

```
/**
	*@brief 简述
	*@author 作者
	*@date 2021-09-18
*/
```

- 除内联函数外，所有函数不得在头文件实例化
- 头文件应使用 ifndef 进行保护

```
<sample.h>
#ifndef __SAMPLE_H
#define __SAMPLE_H
#endif
```

### 2. 函数规范

#### 1. 注释模板

```
/**
	*@brief 简述
	*@param1 参数1
	*@param2 参数2（有几个参数就写几个）
	*@return 返回值
*/
```

#### 2. 命名规范

小驼峰(myName) 

动词+名词(getLength, setParam) or 名词+动词(usartInit, dataRst)

#### 3. 作用域规范

- 只在当前文件内部使用的函数：声明为static
- 需要外部调用的函数：使用接口函数+函数指针 详见4. C语言实现类

### 3. 变量规范

#### 1. 命名规范

| 类型           | 规范                  | 示例                             |
| -------------- | --------------------- | -------------------------------- |
| 结构体类型定义 | 大驼峰，最后加_t      | typedef struct{ }MotorData_t;    |
| 结构体变量     | 大驼峰                | MotorData_t YawMotor;            |
| 一般变量       | 全小写，下划线分隔    | present_value                    |
| 指针变量       | 以p_开头              | int *p_value                     |
| 全局变量       | 以g_开头              | g_judge                          |
| 宏定义，常量   | 全部大写              | REMOTE_RX_BUFFER_MAX             |
| 枚举类型       | 全部大写，最后加_ENUM | typedef enum{ }MOTOR_MODEL_ENUM; |

#### 2. 使用规范

- 变量名称基本原则：
  - 禁止无意义的命名
  - 禁止出现拼音。请使用简介明了的英文或缩写
  - 好的代码是不解自明的：通过变量名即可明白该变量的作用。必要时添加注释辅助说明。
- 同一模块的变量，使用结构体封装管理 如Remote_Data_t
- 特定模块的变量，封装在函数内部，定义为函数内部变量(static)
- 文件内部变量均标为static
- 必需时才声明为全局变量，用g_标注

## 如何使用

本工程中，步兵的各项功能均模块化，可以直接移植相关.c和.h文件到其他工程，简单修改API和硬件配置即可完成编译要求。

Keil编译完成无错误之后，通过Jlink的SWD工作模式下载到主控板，即可运行程序。

## 硬件和软件框图

见步兵软件框图.pdf 和 步兵电路框图.pdf

## 优化方向

1. 优化程序架构

   提高模块化程度，做到各个功能模块去耦合，更换硬件或者修改拓扑时不需要对软件程序进行大规模修改。

2. 增加异常检测和应对机制

   比如在某个电机掉线的时候可以通过应急方式让步兵尽量正常的工作，不至于直接瘫痪失去战斗能力。

3. 增加稳定的LOG系统

   目前SD卡的读写不是很稳定，时常出现无法正常初始化和无法正常读取数据的情况。并且读取数据采用的方式速度太慢，一场比赛的数据要读三十到四十分钟，对于临场排查故障非常不利。

4. 更稳定、易调试的控制系统设计

   pid控制已经满足上场对抗的基本要求，但是为了追求辅瞄以及能量机关击打更好的效果，我们希望能够探索更有效快速的控制系统，与此同时不能失去调试成本和效果之间的平衡。



## 特别鸣谢

陀螺仪解算及云台控制器：哈尔滨工程大学 创梦之翼战队 王洪玺 https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example

功率控制策略：浙江大学 HelloWorld战队 电控技术知识库 [Hello World 技术知识库 (zju-helloworld.github.io)](https://zju-helloworld.github.io/Wiki/)