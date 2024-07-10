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
#include "Photoelectric.h"

/*!
	\brief      main function
	\param[in]  none
	\param[out] none
	\retval     none
*/
#define NO_DETECTION -1
float base_speed = 900;
float side_high = 650;
float side_low  = 620;
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

int is_blue(int val) {
	if(val == 50 || val == 30 || val == 0 || val == -30 || val == -50) return 1;
	return 0;
}
void turn_until_on(void) {
	sensor_Value[0] = Read_sensor(sensor1);
	sensor_Value[1] = Read_sensor(sensor2);
	sensor_Value[2] = Read_sensor(sensor3);
	sensor_Value[3] = Read_sensor(sensor4);
	while(sensor_Value[0]==0 || sensor_Value[1]==0 || sensor_Value[2]==0 || sensor_Value[3]==0) {
		
		Motor_BLSrun(duty_base,500,side_high);
		sensor_Value[0] = Read_sensor(sensor1);
		sensor_Value[1] = Read_sensor(sensor2);
		sensor_Value[2] = Read_sensor(sensor3);
		sensor_Value[3] = Read_sensor(sensor4);
		delay_1ms(200);
	}
	delay_1ms(100);
}

//找黑色
void back_turn(void) {
	duty_find = get_res_duty();
	while(duty_find != 2) { //黑色跑道不在中间

		sprintf((char*)txt,"finding", duty_find);
		OLED_P6x8Str(0,5,txt);

		if(is_blue(duty_find) || duty_find == 1  ) { //有红色或者蓝色
			return;
		}

		while(black_detected(duty_find) == 0){ // 此时视野里没有黑色
			Motor_BLSrun(duty_base, 500,side_high);//右转
			delay_1ms(100);
			duty_find = get_res_duty();
			if(is_blue(duty_find) || duty_find == 1  ) { //有红色或者蓝色
				return;
			}
		}
		if(duty_find == -2) { //横着的
			//它总是歪啊
			while (duty_find == -2) {
				Motor_BLSrun(duty_base,500,side_high + 30); //旋转
				duty_find = get_res_duty();
				delay_1ms(100);
			}
			//运用光电。
			// sensor_Value[0] = Read_sensor(sensor1);
			// sensor_Value[1] = Read_sensor(sensor2);
			// sensor_Value[2] = Read_sensor(sensor3);
			// sensor_Value[3] = Read_sensor(sensor4);
			// //还没触碰到黑线
			// while(sensor_Value[0]!=1 || sensor_Value[1]!=1 || sensor_Value[2]!=1 || sensor_Value[3]!=1) {
			// 	sensor_Value[0] = Read_sensor(sensor1);
			// 	sensor_Value[1] = Read_sensor(sensor2);
			// 	sensor_Value[2] = Read_sensor(sensor3);
			// 	sensor_Value[3] = Read_sensor(sensor4);
			// 	Motor_BLSrun(duty_base,540,side_high + 20);
			// 	delay_1ms(200);
			// 	Motor_BLSrun(duty_base,side_high,side_high + 10);
			// 	delay_1ms(200);
			// }
			Motor_BLSrun(duty_base,side_high,side_high + 10);//往前
			delay_1ms(2000);
			// turn_until_on();
			break;
		}
		if(duty_find == -100 || duty_find == 100) { // 在左或者右，就先向前走一走
			Motor_BLSrun(duty_base,side_high,side_high);
			delay_1ms(200);
		}
		if(duty_find == 100) { //在视野左侧
			Motor_BLSrun(duty_base,side_high,500);

		}

		if(duty_find == -100) { // 在视野右侧
			Motor_BLSrun(duty_base,500,side_high);
		}
		delay_1ms(200);
	}
	

	
}

int black_detected(int val) {
	if(val == -2 || val == 2 || val== 100 || val  == -100) return 1;
	return 0;
}

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

	//photoelectric
	sensor_init();
	delay_1ms(100);

	
	
	// unsigned char receive_data;

	USART2_Init(115200);
	int ret_num = 0;

	//pid line
	pid_param_t pid_line;
	pid_line.kp = 1.4;
	pid_line.kd = 0.3;
	pid_line.ki = 0.1;
	pid_line.imax = 100; // 设置积分限幅
	float line_error = 0; //用于表示横向偏差

	//pid_yaw
	pid_param_t pid_yaw;
	pid_yaw.kp = 0.6;
	pid_yaw.kd = 0.1;
	pid_yaw.ki = 0.0;
	pid_yaw.imax = 50; // 设置积分限幅


	while (1)
	{ 
		//iic部分
        LQ_DMP_Read();                        //调用DMP读函数
		
		//获取视觉信息
		duty_find = get_res_duty();

		if(black_detected(duty_find) == 0) { //没有黑色
			// 更新PID控制器的状态
			Yaw0 = Yaw;
			pid_yaw.integrator = 0;
			pid_yaw.last_error = 0;
			diff = 0;
			pid_line.integrator = 0;
			pid_line.last_error = 0;
		}
		
		sprintf((char*)txt,"df:%d", duty_find);	
		OLED_P6x8Str(0,4,txt);

		if(duty_find == 1) { //看见红色，停止
			Motor_BLSrun(duty_base,500,500);
			delay_1ms(150); 
			continue;
		}
		
		if(black_detected(duty_find) == 0) { //看见蓝色
			//转弯前进
			while(is_blue(duty_find)) {
				//不知道为什么会歪
				Motor_BLSrun(duty_base, side_high + duty_find, 15 + side_high - duty_find); 
				sprintf((char*)txt,"blue:%d", duty_find);
				OLED_P6x8Str(0,5,txt);
				duty_find = get_res_duty();
				delay_1ms(100);
				//看见红色，停止
				if(duty_find == 1) { 
					Motor_BLSrun(540,500,500);
					delay_1ms(150); 
					break;
				}
			}
			//冲线
			Motor_BLSrun(duty_base,side_high,side_high);
			delay_1ms(2000); 
			//看不见了，停一下
			Motor_BLSrun(duty_base,500,500);

			back_turn(); // 找黑色

			continue;
		}
		

		//photoelectric 光电部分
		sensor_Value[0] = Read_sensor(sensor1);
		sensor_Value[1] = Read_sensor(sensor2);
		sensor_Value[2] = Read_sensor(sensor3);
		sensor_Value[3] = Read_sensor(sensor4);
		if(sensor_Value[0]!=1 || sensor_Value[1]!=1 || sensor_Value[2]!=1 || sensor_Value[3]!=1) // 没有在黑线上
			back_turn();
		
		
		if(sensor_Value[0] == 0 && sensor_Value[1] == 1  && sensor_Value[2] == 1 && sensor_Value[3] == 1) { // 车左前方有白
			line_error = -8.0;
			Yaw0 = Yaw;
			lspeed = rspeed = side_low;
		}
		else if(sensor_Value[0] == 0 && sensor_Value[1] == 0 &&  sensor_Value[3] == 1 ) { // 车左前方有白 0011 0001
			line_error = -20.0;
			Yaw0 = Yaw;
			lspeed = rspeed = side_low;
		}
		else if(sensor_Value[3] == 0 && sensor_Value[2] == 1 && sensor_Value[0] == 1 && sensor_Value[1] == 1) { //右前方有白
			line_error = 8.0;
			Yaw0 = Yaw;
			lspeed = rspeed = side_low;
		}
		else if(sensor_Value[0] == 1 &&  sensor_Value[3] == 0 && sensor_Value[2] == 0) { // 车左前方有白 1100 1000
			line_error = 20.0;
			Yaw0 = Yaw;
			lspeed = rspeed = side_low;
		}
		else {
			line_error = 0.0;
			lspeed = rspeed = side_high;
		}
		
		//pid
		duty_yaw =PIDLocCtrl(&pid_yaw, Yaw - diff -Yaw0);
		duty_line =PIDLocCtrl(&pid_line, line_error);

		duty =  1.2 * duty_line -  duty_yaw;
	
		// lspeed = rspeed = side_high;
		if( duty_base > 500 )
			Motor_BLSrun(base_speed,3 + lspeed + duty, rspeed-duty);
		else
			Motor_BLSrun(base_speed,0,0);
		
	
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
