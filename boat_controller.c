/*------------------------------------------
    Controller 类
    功能：控制小船，设置状态
-------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "uart_driver.h"
#include "pwm_driver.h"
#include "tim_driver.h"

#define NEAR 1
#define SEEN 0
#define NO_DETECTION -1
/*
state:
1:near
0:not near
-1:not seen
*/

int side_speed = 680;
int base_speed = 900;
/*------------------------------------------
    定义 Controller 结构体
-------------------------------------------*/
typedef struct {
    int state;
} Controller;


/*------------------------------------------
    初始化 Controller
-------------------------------------------*/

Controller* Controller_init(int initial_state) {
    Controller* controller = (Controller*)malloc(sizeof(Controller));
    if (controller != NULL) {
        controller->state = initial_state;
    }
    return controller;
}

/*------------------------------------------
    设置控制值
-------------------------------------------*/
void Controller_setValue(Controller* controller, int new_value) {
    if (controller != NULL) {
        controller->state = new_value;
    }
}


/*------------------------------------------
    获取控制值
-------------------------------------------*/
int Controller_getValue(Controller* controller) {
    if (controller != NULL) {
        return controller->state;
    }
    return -1; // 返回一个默认错误值
}

/*------------------------------------------
    销毁 Controller
-------------------------------------------*/
void Controller_destroy(Controller* controller) {
    if (controller != NULL) {
        free(controller);
    }
}


/*------------------------------------------
    设置控制器状态

    状态:
        1:near
        0:not near
        -1:not seen

-------------------------------------------*/
void Controller_SetState(Controller* controller) {
    if (controller == NULL)
        return;
    //实际运行，从串口获得信息
    int val = get_res_duty();
    delay_1ms(50);
    if(val == 1) { //接近
        Controller_setValue(controller, 1);
    }
    else if (val != -1) { // 视野里有
        Controller_setValue(controller, 0);
    }
    else { //失去追踪
        Controllre_setValue(controller,-1);
    }
        
}
/*------------------------------------------
    控制核心代码
    开始之前更新状态
    状态:
        1:near
        0:not near
        -1:not seen


-------------------------------------------*/
int Controller_Control(Controller* controller) {
    Controller_SetState(controller);
    int ret;
    if(controller->state == NEAR) {
        SendDashCommand();
        ret = NEAR;
    }
    else if (controller->state == SEEN) {
        SendGoCommand();
        ret = SEEN;
    }
    else if(controller->state == NO_DETECTION) {
        SendTurnCommand();
        ret = NO_DETECTION;
    }
    return ret;
}

void SendDashCommand(void) {
    
    printf("dashing\n");
    
    // Motor_BLSrun(base_speed, side_speed , side_speed);
    // delay_1ms(1000);
}

void SendGoCommand(void) {
    
    printf("goint straight\n");
    int val = get_res_duty();
    delay_1ms(50);
    Motor_BLSrun(base_speed, side_speed + val, side_speed-val);
    delay_1ms(50);
}

void SendTurnCommand(void) {
    
    printf("turning\n");
    Motor_BLSrun(base_speed, side_speed + 50, 500);
    delay_1ms(50);

}


int main() {
    // 初始化控制器并设置初始值
    Controller* controller = Controller_init(0);
    if (controller == NULL) {
        printf("初始化控制器失败\n");
        return 1;
    }

    // 获取和打印控制值
    printf("控制值: %d\n", Controller_getValue(controller));

    // 设置新的控制值
    Controller_setValue(controller, 20);
    printf("新的控制值: %d\n", Controller_getValue(controller));

    // 销毁控制器
    Controller_destroy(controller);

    return 0;
}