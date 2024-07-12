#include "gd32f10x.h"
#include "systick.h"
#include <stdio.h>
#include <float.h>
#include "led_driver.h"
#include "key_driver.h"
#include "oled_driver.h"
#include "tim_driver.h"
#include "enc_driver.h"
#include "uart_driver.h"
#include "LQ_SOFTI2C.h"
#include "LQ_MPU6050_DMP.h"
#include "LQ_IIC_Gyro.h"
#include "pwm_driver.h"
#include "pid.h"
#include "boat_controller.c"


/*!
	\brief      main function
	\param[in]  none
	\param[out] none
	\retval     none
*/
#define NO_DETECTION -1
#define NEAR 1
#define SEEN 0
float base_speed = 900;
//for low battery
float side_high = 650;
float side_low  = 620;

//for moddle battery 11.7
// float side_high = 630;
// float side_low  = 600;

//for high battery
// float side_high = 600;
// float side_low  = 580;

float duty_base = 900;

char txt[64];			//数组，用来存放屏幕显示的内容
int32_t duty = 0;	    //调整占空比的变量
int32_t duty_yaw = 0;
int32_t duty_line = 0;
uint8_t sensor_Value[4];//sensor

uint16_t Dis = 0.0; // ultra



//speed definition
float Yaw0 = 0.0; 

//control poloelectric
float diff = 0.0;

float lspeed = 590;
float rspeed = 590;
float max_turn_speed = 60;

int saw_target = 0;

int ret_num = 0;

int32_t duty_find = 0;

int turn_record = 0;


int main(void)
{
	
	systick_config();
	rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1); // AHB主频是1分频
	systick_config();						  // 系统主频72MHZ,采用外部晶振,由两个宏决定(__SYSTEM_CLOCK_72M_PLL_HXTAL与HXTAL_VALUE)
	rcu_periph_clock_enable(RCU_AF);		  // 管脚复用时钟alternate function clock使能

	led_init(); // LED
	key_init();
	OLED_Init(); // OLED

	//pwm
    Brushless_Motor_Init();	//无感无刷电机初始化
	delay_1ms(200);
	
	//iic
	SOFT_IIC_Init();              //IIC初始化
    LQ_DMP_Init();                          //DMP初始化（源码无法查看）
	OLED_CLS();                             //OLED清屏
	delay_1ms(100);

	//iic reading yaw0
	LQ_DMP_Read(); 
	Yaw0 = Yaw;

	
	
	// unsigned char receive_data;

	USART2_Init(115200);
	int ret_num = 0;

	//pid_yaw
	pid_param_t pid_yaw;
	pid_yaw.kp = 0.6;
	pid_yaw.kd = 0.1;
	pid_yaw.ki = 0.0;
	pid_yaw.imax = 50; // 设置积分限幅

	Controller* controller = Controller_init(0);
	Controller_SetState(controller);

	while (1)
	{ 
		//iic部分
        LQ_DMP_Read();                        //调用DMP读函数
		
		//获取视觉信息
		duty_find = get_res_duty();

		int ctrl_res = Controller_Control(controller);

		
		//pid
		duty =  - PIDLocCtrl(&pid_yaw, Yaw - diff -Yaw0);
		//靠近的时候前进
		if( ctrl_res == NEAR) {
			Yaw0 = Yaw;
			//pid
			
			while(Controller_Control(controller) == NEAR) {
				LQ_DMP_Read(); 
				delay_1ms(100);
				duty =  - PIDLocCtrl(&pid_yaw, Yaw - diff -Yaw0);
				Motor_BLSrun(base_speed,side_high + duty, side_high - duty);
				delay_1ms(100);
			}
			delay_1ms(400);
		}
		
	
		// Motor_BLSrun(0,0,0);

		diff += 0.06;
		if(diff >= 3) {
			Yaw0 = Yaw;
			diff = 0;
			pid_yaw.integrator = 0;
			pid_yaw.last_error = 0;
		}
		led_toggle();
		delay_1ms(150); 
	}
}
