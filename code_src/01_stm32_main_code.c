// 241014 同步到git仓库
// Title：基于机器视觉的智能循迹避障小车
#include "stm32f10x.h"// Device header
#include "Delay.h"	//延时函数模块
#include "OLED.h"//OLED显示模块
#include "Motor.h"//电机模块
#include "Car_Action.h"//小车运动模块
#include "HCSR04.h"//超声波模块
#include "Beep.h"//蜂鸣器模块
#include "UART.h"//串口通信模块
#include "string.h"//字符串头文件
//状态标志位
u8 Car_State	=0;//蓝牙控制状态标志位
u8 HCSR_State	=0;//超声波避障状态标志位
u8 OpenMV_State=0;//OpenMV状态标志位
u8 Distate=0;//距离状态标志位
//变量初始化
float Dis      = 0;//超声波测得距离值
int   SRDis_TH = 30;//超声波避障距离阈值
int   PWML     = 0;//左电机的PWM值
int   PWMR     = 0;//右电机的PWM值

int main(void)
{
	OLED_Init();//OLED初始化
	UART1_Init();//串口1初始化
	UART2_Init();//串口二初始化
	MotorAll_Init();//电机初始化
	Beep_Init();//蜂鸣器初始化
	//OLED显示超声波距离字符串
	OLED_ShowString(1, 1, "SRDis:");
	OLED_ShowString(1, 10, ".");
	OLED_ShowString(1, 13, "cm");
	while(1)
	{
		PWML=PWM_RxPacket[0];//PI控制左电机的PWM值
		PWMR=PWM_RxPacket[1];//PI控制右电机PWM
		Show_SRDis();//超声波获取当前距离
		Dis=Get_DisVal();//获取当前超声波测得距离

		if(Dis > SRDis_TH)//距离大于阈值 
		{
			Distate = 0; //状态标志位置0
			Beep_Off();//不报警
		}
		if(Dis<SRDis_TH)//距离小于阈值 
		{
			Distate=1; //状态标志位置1
			Beep_Warning();//报警
		}
		//模式判断 是否超声波避障模式
		//超声波避障模式且距离大于阈值
		if (HCSR_State==1&&Distate==0 )
		{
				Car_Up();//小车前进	
		}
		//超声波避障模式且距离小于阈值
		if(HCSR_State==1&&Distate==1)
			{
					Car_Stop();//停车
					Delay_ms(200);
					Car_TurnLeft();//左转
					Delay_ms(500);
					Car_Stop();//停车
					Delay_ms(500);
		}
		//判断OpenMV是否回传了PWM值
		if(OpenMV_State==1)//当OpenMV回传PWM值
		{
			if(PWML<=100)//0到100为正转
			{
				MotorL_SetSpeed(PWML);
			}
			else //大于100为反转
			{
				MotorL_SetSpeed(-(PWML-100));
			}
			if(PWMR<=100)
			{
				MotorR_SetSpeed(PWMR);
			}
			else 
			{
				MotorR_SetSpeed(-(PWMR-100));
			}
		}
/************串口USATR2接收信息*************/
		//串口USART2收到上位机命令
		if (UART2_RxFlag == 1)
		{
			OLED_ShowString(4, 1, "                ");//清屏OLED第四行
			OLED_ShowString(4, 1, "RXCmd:");//显示RXCmd字符串
			OLED_ShowString(4, 7, UART2_RxPacket);//显示收到的命令			
			//TRAK 循迹模式
			if (strcmp(UART2_RxPacket, "TRAK") == 0  )
			{
				UART1_SendString("1");//用USART向OpenMV发送1选择循迹程序
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Tracking Mode");
				OpenMV_State=1;//OpenMV状态置1
			}
			//TROB循迹避障模式
			else if (strcmp(UART2_RxPacket, "TROB") == 0 )//选择循迹避障模式
			{
				int counter=0;
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "TRA&OBS Mode");
				OpenMV_State=1;//OpenMV状态为1
				if (Dis<SRDis_TH)//当距离小于阈值
				{
					Car_Stop();//停车
					Beep_Warning();//报警
					UART1_SendString("0");//向OpenMV模块发送0 选择默认模式
				}
				if(Dis>SRDis_TH)//如果距离大于阈值
				{
					counter+=1;
					if(counter>10)//计数滤波 判断10次以为真
					{
							Beep_Off();
							UART1_SendString("3");//向OpenMV发送3 进入循迹避障模式
					}
				}
			}
			//OBST避障模式
			else if (strcmp(UART2_RxPacket, "OBST") == 0)
			{
				UART1_SendString("2");//OpenMV通信2
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Obstacle Mode");
				OpenMV_State=2;//OpenMV状态置2
			}
			//HSLT水平舵机左转
			else if (strcmp(UART2_RxPacket, "HSLT") == 0)
			{
				UART1_SendString("4");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Hor_Servo_Left++");
			}
			//HSRT水平舵机右转
			else if (strcmp(UART2_RxPacket, "HSRT") == 0)
			{
				UART1_SendString("5");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Hor_Servo_Left--");
			}
			//VSUP垂直舵机向上
			else if (strcmp(UART2_RxPacket, "VSUP") == 0)//
			{
				UART1_SendString("6");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Ver_Servo_Up++");
			}
			//VSDW垂直舵机向下
			else if (strcmp(UART2_RxPacket, "VSDW") == 0)
			{
				UART1_SendString("7");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Ver_Servo_Down--");
			}
			//ATHP增大循迹角度阈值
			else if (strcmp(UART2_RxPacket, "ATHP") == 0)
			{
				UART1_SendString("8");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Angle_TH_Up");
			}
			//ATHD减小角度阈值
			else if (strcmp(UART2_RxPacket, "ATHD") == 0)
			{
				UART1_SendString("9");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Angle_TH_Down");
			}
			//DTHP 增加距离阈值
			else if (strcmp(UART2_RxPacket, "DTHP") == 0)
			{
				UART1_SendString("10");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Dis_TH_Up");
			}
			//DTHD减小距离阈值
			else if (strcmp(UART2_RxPacket, "DTHD") == 0)
			{
				UART1_SendString("11");
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Dis_TH_Down");
			}		
			//OFFSOpenMV回到默认状态0且停车
			else if(strcmp(UART2_RxPacket, "OFFS") == 0)
			{
				UART1_SendString("0");
				OpenMV_State=0;
				Car_Stop();//停车
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "OpenMV No Mode");
			}
			//BTCO选择遥控控制模式
			else if(strcmp(UART2_RxPacket, "BTCO") == 0)
			{
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "BlueTooth Mode");
				OLED_ShowString(3, 1, "Car State:");
				Car_State=1;//小车状态置1 蓝牙遥控模式
				OLED_ShowNum(3, 11, Car_State, 1);
			}
			//OFFB关闭蓝牙遥控模式
			else if(strcmp(UART2_RxPacket, "OFFB") == 0)
			{
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "BlueTooth  OFF");
				OLED_ShowString(3, 1, "Car State:");
				Car_State=0;//小车状态置0
				OLED_ShowNum(3, 11, Car_State, 1);
			}
			//蓝牙模式下 UPUP小车前进
			else if(strcmp(UART2_RxPacket, "UPUP") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 1, "BTCmd:");
				OLED_ShowString(3, 1, "Car State:");
				OLED_ShowNum(3, 11, Car_State, 1);
				OLED_ShowString(2, 7, "UpUp");
				Car_Up();
			}
			//蓝牙模式下 DOWN小车后退
			else if(strcmp(UART2_RxPacket, "DOWN") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 7, "Down");
				Car_Down();
			}
			//蓝牙模式下 RIGH小车右转
			else if(strcmp(UART2_RxPacket, "RIGH") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 7, "Trun Right");
				Car_TurnRight();
			}
			//蓝牙模式下 LEFT小车左转
			else if(strcmp(UART2_RxPacket, "LEFT") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 7, "Trun Left");
				Car_TurnLeft();
			}
			//蓝牙模式下 SPIN小车旋转
			else if(strcmp(UART2_RxPacket, "SPIN") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 7, "Spin");
				Car_Spin();
			}
			//蓝牙模式下 STOP停车
			else if(strcmp(UART2_RxPacket, "STOP") == 0 && (Car_State==1))
			{
				OLED_ShowString(2, 7, "Stop");
				Car_Stop();
			}

			//SROB 进入超声波避障模式
			else if(strcmp(UART2_RxPacket, "SROB") == 0 )
			{
				HCSR_State=1;
				OLED_ShowString(2, 7, "SR OBS");
			}
			//OFFR停止退出超声波避障
			else if(strcmp(UART2_RxPacket, "OFFR") == 0 )
			{
				OLED_ShowString(2, 7, "Stop SR");
				HCSR_State=0;//避障标志位清0
				Car_Stop();
			}	
			//DWPV减小PID控制的P值
			else if(strcmp(UART2_RxPacket, "DWPV") == 0 )
			{
				UART1_SendString("12");
			}
			//UPPV增加PID控制P值
			else if(strcmp(UART2_RxPacket, "UPPV") == 0 )
			{
				UART1_SendString("13");
			}	
			//UPIV增加PID控制I值	
			else if(strcmp(UART2_RxPacket, "UPIV") == 0 )
			{
				UART1_SendString("14");
			}		
			//DWIV减小PID控制的I值
			else if(strcmp(UART2_RxPacket, "DWIV") == 0 )
			{
				UART1_SendString("15");
			}		
			else //没有收到软件指令 等待命令
			{
				OLED_ShowString(2, 1, "                ");
				OLED_ShowString(2, 1, "Wait Right Cmd");
			}
			UART2_RxFlag = 0;//串口二标志位更新
		}
/************串口USATR1接收信息*************/
		if (UART1_RxFlag == 1)//如果收到OpenMv发来的命令
		{
			//OMV1 小车直行
			if (strcmp(UART1_RxPacket, "OMV1") == 0  )
			{
					Car_Up();
			}
			//OMV2小车左转
			else if (strcmp(UART1_RxPacket, "OMV2") == 0  )
			{
					Car_TurnLeft();
			}  
			//OMV3小车右转                               
			else if (strcmp(UART1_RxPacket, "OMV3") == 0  )
			{
					Car_TurnRight();
			}
			//OMV4停车        
			else if (strcmp(UART1_RxPacket, "OMV4") == 0  )
			{
					Car_Stop();
			}
			UART1_RxFlag = 0;//串口USART1标志位更新
		}
	}
}

/*************************Delay延时模块*******************************
模块功能：产生X (us ms s)延时效果
*********************************************************************/
#include "stm32f10x.h"
//0到233015微妙延时
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;			//设置定时器重装值ARR=72x
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;	//设置时钟源为HCLK 启动定时器
	while(!(SysTick->CTRL & 0x00010000));//等待计数到0
	SysTick->CTRL = 0x00000004;		//关闭定时器
}

//函数功能：0~4294967295毫秒延时
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}

 //0~4294967295秒级延时
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
/********************Timer定时器模块*******************************
用于定时器TIM2 TIM3初始化 以及PWM值的设定
TIM2用于超声波测距 TIM用于PWM电机控制
*********************************************************************/
#include "stm32f10x.h"//头文件

//初始化定时器TIM3 并配置PWM初始化参数
void PWM12_Init(void)
{	//开启TIM3和GPIOA的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//初始化GPIO口 PA6 PA7 用于产生PWM信号
	GPIO_InitTypeDef GPIO_InitStructure;
	//复用推挽输出模式因为复用了TIM3外设
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO A6 A7 
	//使用内部时钟
	TIM_InternalClockConfig(TIM3);
	//配置时基单元参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//定时器不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;//ARR自动重装值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;//PSC预分频值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//到达ARR触发一次中断 停止计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);//初始化单元
	//输出比较结构体配置
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//补全结构体中未配置参数
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出比较极性选择
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出使能

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);//初始化 TIM3 OC1
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能CCR1自动重装
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);//初始化 TIM3 OC2	
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能CCR2自动重装
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);//开启预装载
	TIM_Cmd(TIM3, ENABLE);//开启定时器3
	TIM3->CCR1 = 0;//设置输出比较值
	TIM3->CCR2 = 0;
}

//设置PWM1比较值 为Compare 即输出比较值
void PWM12_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM3, Compare);
}

//设置PWM2比较值 为Compare
void PWM12_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM3, Compare);
}

//超声波测距定时器TIM2 初始化
void Timer_Init(void)
{	//开启定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//使用内部时钟
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65535;//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//预分频值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//CNT到达ARR中断一次
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	TIM_SetCounter(TIM2,0);//CNT值清0
}

//开启TIM2定时器
void Timer_ON(void)
{
	TIM_Cmd(TIM2,ENABLE);
}

//关闭TIM2定时器
void Timer_OFF(void)
{
	TIM_Cmd(TIM2,DISABLE);
}

/******************************Motor电机模块**************************
电机初始化设置 以及电机PWM设置
*********************************************************************/
#include "stm32f10x.h"                  // Device header
#include "Timer.h"

void MotorAll_Init(void)
{
	//开启电机驱动口的四个GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	PWM12_Init();//开启定时器
}

//设置右路电机速度 PWM
void MotorR_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)//Speed值为正
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);//电机正转
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		PWM12_SetCompare1(Speed);//设置Speed转速
	}
	else//Speed值为负
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);//电机反转
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		PWM12_SetCompare1(-Speed);//设为-Speed转速
	}
}

//设置左路电机PWM 速度
void MotorL_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		PWM12_SetCompare2(Speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		PWM12_SetCompare2(-Speed);
	}
}
/******************************模块简介*******************************
模块功能：小车运动 模块函数封装 
*********************************************************************/
#include "stm32f10x.h"                  // Device header
#include "Motor.h" 
#include "Delay.h"  
void Car_Stop()//小车停止
{
	MotorR_SetSpeed(0);
	MotorL_SetSpeed(0);
}

void Car_Up()//小车前进
{
	MotorR_SetSpeed(40);
	MotorL_SetSpeed(40);	
}

void Car_Down()//小车后退
{
	MotorR_SetSpeed(-40);
	MotorL_SetSpeed(-40);
}

void Car_TurnRight()//小车右转
{
	MotorR_SetSpeed(-45);
	MotorL_SetSpeed(35);
}

void Car_TurnLeft()//小车左转
{
	MotorR_SetSpeed(35);
	MotorL_SetSpeed(-45);
}

void Car_Spin()//小车旋转
{
	MotorR_SetSpeed(40);
	MotorL_SetSpeed(-50);
}
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.H"
#include "OLED.h"
//参数初始化
float Num;
int i=0,Counter,Sum,Dec,CNT_Mean,CN1,CN2;
//超声波模块触发函数
void SR04_Start(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_0);//A0置1大于10us
	Delay_us(15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}

void SR04_Init(void)
{
	Timer_Init();//开启定时器3
	GPIO_InitTypeDef GPIO_InitStructure;//定义初始化结构体
	EXTI_InitTypeDef EXTI_InitStructure;//定义外部中断结构体
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//开GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开AFIO时钟
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);
	//脉冲触发端口 Trig=PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//回波接收端口 Echo=PA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	SR04_Start();
}
void EXTI1_IRQHandler(void)
{
	Delay_us(50);
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{	
		Timer_ON();//开始计时
		CN1 = TIM_GetCounter(TIM2);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1));//A1为高电平时一直循环下去
		CN2 = TIM_GetCounter(TIM2);
		Counter = CN2 - CN1;//获得计数差
		EXTI_ClearITPendingBit(EXTI_Line1);//清除标志位
	}
}
                         
void Show_SRDis(void)
{
	for(i=0;i<5;i++)//均值滤波
	{
		SR04_Init();
		Sum+=Counter;
	}
	CNT_Mean=Sum/5;
	Num=CNT_Mean*0.017;//b/5 * 17/1000
	Dec=(17*Sum/5)%100;//100取余得到小数点后值
	OLED_ShowNum(1,7,Num,3);
	OLED_ShowNum(1,11,Dec,2);
	Delay_ms(100);
	Sum=0;//更新Sum
}

//返回超声波测距值
float Get_DisVal(void)
{
	return Num;
}
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void Beep_Init(void)
{	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB10 GPIO结构体初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
	
void Beep_On(void)//开启蜂鸣器
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
}

void Beep_Off(void)//关闭蜂鸣器
{
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
}

void Beep_Warning(void)//蜂鸣器报警
{
	Beep_On();
	Delay_ms(500);
	Beep_Off();
	Delay_ms(500);	
}
/******************************UART串口通信模块***********************
串口一和二的初始化函数封装 串口一和二中断语句句柄 以及串口一发送函数
*********************************************************************/
#include "stm32f10x.h"                // Device header
//char用于接收字符 接收缓存区
char UART1_RxPacket[100];//存放小车运动命令	
char UART2_RxPacket[100];//存放APP的控制命令
char PWM_RxPacket[100];//存放PWM值
uint8_t UART1_RxFlag;//串口一是否收到数据标志位
uint8_t UART2_RxFlag;//串口二是否收到数据标志位
static int PWML=0,PWMR=0;

//串口USART1的初始化
void UART1_Init(void)
{
	//开启UART时钟和GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//将PA9配置为复用推挽输出 供USART1的TX输出信号使用
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//将PA10配置为上拉输入 供USART1的RX接收信号使用
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//配置UART参数
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;//波特率9600
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控不使用流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//既要发送又要接收
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//选择一位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//无校验为选择8位字长
	USART_Init(USART1, &USART_InitStructure);//UART1结构体初始化
	//中断配置
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//中断线
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART1, ENABLE);//开启USART1
}

//串口USART2的初始化
void UART2_Init(void)
{
	//开启UART时钟和GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//将PA2配置为复用推挽输出 供USART2的TX输出信号使用
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//将P3配置为上拉输入 供USART2的RX接收信号使用
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//配置UART参数
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;//波特率9600
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控不使用流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//只接收
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//选择一位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//无校验为选择8位字长
	USART_Init(USART2, &USART_InitStructure);//UART1结构体初始化
	//中断配置
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//中断线
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//串口二通信中断等级高
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART2, ENABLE);//开启USART2
}

//USART1中断函数执行句柄
void USART1_IRQHandler(void)
{
	static uint8_t RxState1 = 0;//状态变量S
	static uint8_t RxState3 = 0;//PWM值状态变量S
	static uint8_t pRxPacket1= 0;//指示接收到哪一个数据了
	static uint8_t pRxPacket3= 0;//PWM值接收到哪一个数据了
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)//判断RXNE标志位是否置一
	{
		uint8_t RxData1 = USART_ReceiveData(USART1);//定义RxData同时赋值 接收数据
		//接收PWM值
		if (RxState3 == 0)
		{
			if(RxData1=='P')//PWM包头一判断
			{
				RxState3 = 1;
				pRxPacket3 = 0;
			}
		}
		else if (RxState3 == 1)
		{
			if(RxData1=='W')//PWM包头二判断
			{
				RxState3 = 2;
				pRxPacket3 = 0;
			}
		}
		else if (RxState3 == 2)
		{
			if(RxData1=='M')//PWM包头三判断
			{
				RxState3 = 3;
				pRxPacket3 = 0;
			}
		}
		else if (RxState3 == 3)
		{
			if (RxData1 == '$')//判断是不是第一个包尾
			{
				RxState3 = 4;//是则跳转到状态4
			}
			else//不是包尾才接收数据
			{
				PWM_RxPacket[pRxPacket3] = RxData1;
				pRxPacket3 ++;
			}
		}
		else if (RxState3 == 4)
		{
			if (RxData1 == '#')//判断是否是第二个包尾
			{
				RxState3 = 0;
				PWM_RxPacket[pRxPacket3] = '\0';//加一个字符串结束标志位确定字符串的长度
				UART1_RxFlag = 1;//表示接收完成
			}
		}
		//接收小车运动控制命令
		if (RxState1 == 0)
		{
			if (RxData1 == '$' && UART1_RxFlag == 0)//防止发的太快还没处理完
			{
				RxState1 = 1;
				pRxPacket1 = 0;
			}
		}
		else if (RxState1 == 1)
		{
			if (RxData1 == '&')//判断是不是第一个包尾
			{
				RxState1 = 2;//是则跳转到状态2
			}
			else//不是包尾才接收数据
			{
				UART1_RxPacket[pRxPacket1] = RxData1;
				pRxPacket1 ++;
			}
		}
		else if (RxState1 == 2)
		{
			if (RxData1 == '#')//判断是否是第二个包尾
			{
				RxState1 = 0;
				UART1_RxPacket[pRxPacket1] = '\0';//加一个字符串结束标志位确定字符串的长度
				UART1_RxFlag = 1;//表示接收完成
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除标志位
	}
}

//USART2中断函数执行句柄
void USART2_IRQHandler(void)
{
	static uint8_t RxState2= 0;//状态变量S
	static uint8_t pRxPacket2 = 0;//指示接收到哪一个数据了
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)//判断RXNE标志位是否置一
	{
		uint8_t RxData2 = USART_ReceiveData(USART2);//定义RxData同时赋值
		//接收蓝牙APP指令
		if (RxState2 == 0)
		{
			if (RxData2 == '%' && UART2_RxFlag == 0)//防止发的太快还没处理完
			{
				RxState2 = 1;
				pRxPacket2 = 0;
			}
		}
		else if (RxState2 == 1)
		{
			if (RxData2 == '@')//判断是不是第一个包尾
			{
				RxState2 = 2;//是则跳转到状态2
			}
			else//不是包尾才接收数据
			{
				UART2_RxPacket[pRxPacket2] = RxData2;
				pRxPacket2 ++;
			}
		}
		else if (RxState2 == 2)
		{
			if (RxData2 == '?')//判断是否是第二个包尾
			{
				RxState2 = 0;
				UART2_RxPacket[pRxPacket2] = '\0';//加一个字符串结束标志位确定字符串的长度
				UART2_RxFlag = 1;//表示接收完成
			}
		}
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);//清除标志位
	}
}

void UART1_SendByte(uint8_t Byte)//发送一个8字节的变量
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//等待发送寄存器空标志位置1
}

void UART1_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		UART1_SendByte(String[i]);
	}
}
/*************************OLED显示模块*******************************
封装了控制OLED显示字符和数字的函数
*********************************************************************/
#include "stm32f10x.h"
#include "OLED_Font.h"//OLED字模库
//宏定义
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))
//端口初始化
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	OLED_W_SCL(1);//SCL写1
	OLED_W_SDA(1);//SDA写1
}

//I2C起始帧
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);//SDA先从1->0
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

//IIC停止帧
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);//SCL先从0->1
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

//IICf发送一个Byte
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟 不处理应答信号
	OLED_W_SCL(0);
}

//OLED写命令
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

//OLED写数据
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

//OLED光标位置 X (0-127) Y(0-7)
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

//OLED清屏
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

//OLED显示字符 4行16列
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

//OLED显示字符串 4行16列
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

//返回X的Y次方
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

//显示十进制正数 4行16列 最大10位
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

//OLED初始化
void OLED_Init(void)
{
	uint32_t i, j;

	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	OLED_I2C_Init();			//端口初始化
	OLED_WriteCommand(0xAE);	//关闭显示
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);	//设置显示开始行
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置
	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭
	OLED_WriteCommand(0xA6);	//设置正常/倒转显示
	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0xAF);	//开启显示
	OLED_Clear();				//OLED清屏
}