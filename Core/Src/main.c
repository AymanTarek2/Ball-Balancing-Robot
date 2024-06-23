#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

//Camera-Reading Section

static char rx_buffer[10]; // Buffer to store received characters
static uint8_t rx_index = 0; // Index to keep track of received character position
int ball_coordinate_x = 15;
int ball_coordinate_y = 15;


void usart_init(){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x45; //115200 baud rate
	USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
}

//Interrupt
//Receiving data from Esp32 Cam
void USART1_IRQHandler(void) {
	if (USART1->SR & USART_SR_RXNE) {
		char received_char = USART1->DR; // Read received character

		if (received_char >= '0' && received_char <= '9')
			rx_buffer[rx_index++] = received_char;

        if (received_char == ',' || received_char == '\n') {
        	// Null-terminate buffer to make it a valid string
        	rx_buffer[rx_index] = '\0';
        	int data = atoi(rx_buffer);

        	rx_index = 0;

        	if(received_char == ',')
        		ball_coordinate_x = data;
        	else if(received_char == '\n')
        		ball_coordinate_y = data;
		}
		    if(rx_index >= sizeof(rx_buffer) - 1 || received_char == 'A'){
		    	rx_buffer[rx_index] = '\0';
		    	rx_index = 0;
		    }
		}
}


//PID Control Section

#define dir_pin_x   0
#define dir_pin_y   1
#define step_pin_x  2
#define step_pin_y  3

#define PID_KP_x  0.67096f
#define PID_KI_x  0.0307f
#define PID_KD_x  3.2852f

#define PID_KP_y  0.67096f
#define PID_KI_y  0.03041f
#define PID_KD_y  3.2852f


#define PID_TAU 0.01f

#define PID_LIM_MIN -0.349f
#define PID_LIM_MAX  0.349f

#define PID_LIM_MIN_INT -0.25f
#define PID_LIM_MAX_INT  0.25f

#define SAMPLE_TIME_S 0.025f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

int steps_x = 0, steps_y = 0;
int desired_steps_x = 0, desired_steps_y = 0;
int setPoint_x = 16 , setPoint_y = 16;

typedef struct{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;		/* Required for integrator */
	float differentiator;
	float prevMeasurement;	/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

void PIDController_Init(PIDController *pid) {
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error = (setpoint - measurement)/200; //distance in metres

    float proportional = pid->Kp * error;

    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt)
        pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt)
    	pid->integrator = pid->limMinInt;


    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax)
        pid->out = pid->limMax;
    else if (pid->out < pid->limMin)
        pid->out = pid->limMin;

    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return (pid->out *16 * 180)/(M_PI *1.8); //convert from rad to degrees then to steps
}

void rotate_stepper_motor(char dir, char dir_pin, char step_pin , float delay){
	if(dir)
		GPIOA->ODR |= (1<<dir_pin);
	else
		GPIOA->ODR &= ~(1<<dir_pin);

	GPIOA->ODR |= (1<<step_pin);
	vTaskDelay(pdMS_TO_TICKS(0.02));

	GPIOA->ODR &= ~(1<<step_pin);
	vTaskDelay(pdMS_TO_TICKS(delay));
}

//Tasks

void pid_control_x(void *argument){
	PIDController pid_x = {PID_KP_x, PID_KI_x, PID_KD_x,
	PID_TAU, PID_LIM_MIN, PID_LIM_MAX,PID_LIM_MIN_INT,
	PID_LIM_MAX_INT,SAMPLE_TIME_S};
	PIDController_Init(&pid_x);

	for(;;){
		desired_steps_x = PIDController_Update(&pid_x, setPoint_x, ball_coordinate_x);
		vTaskDelay(pdMS_TO_TICKS(25));
	}
}

void pid_control_y(void *argument){
	PIDController pid_y = {PID_KP_y, PID_KI_y, PID_KD_y,
	PID_TAU, PID_LIM_MIN, PID_LIM_MAX,PID_LIM_MIN_INT,
	PID_LIM_MAX_INT,SAMPLE_TIME_S};
	PIDController_Init(&pid_y);

	for(;;){
		desired_steps_y = PIDController_Update(&pid_y, setPoint_y, ball_coordinate_y);
		vTaskDelay(pdMS_TO_TICKS(25));
	}
}

void stepper_motors_response_x(void *argument){
	float stepper_delay_x = 1.5;
	for(;;){
		if(desired_steps_x > steps_x){
			rotate_stepper_motor(0, dir_pin_x, step_pin_x, stepper_delay_x);
			steps_x++;
		}
		else if(desired_steps_x < steps_x){
			rotate_stepper_motor(1, dir_pin_x, step_pin_x, stepper_delay_x);
			steps_x--;
		}
	}
}

void stepper_motors_response_y(void *argument){
	float stepper_delay_y = 1.5;
	for(;;){
		if(desired_steps_y > steps_y){
			rotate_stepper_motor(0, dir_pin_y, step_pin_y, stepper_delay_y);
			steps_y++;
		}
		else if(desired_steps_y < steps_y){
			rotate_stepper_motor(1, dir_pin_y, step_pin_y, stepper_delay_y);
			steps_y--;
		}
	}
}

int main(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	GPIOA->CRL = 0x33333333;
	GPIOA->CRH = 0x444444B4;
	usart_init();
	xTaskCreate(pid_control_x, "PIDx", 128, NULL, 1, NULL);
	xTaskCreate(pid_control_y, "PIDy", 128, NULL, 1, NULL);
	xTaskCreate(stepper_motors_response_x, "Motor_x", 128, NULL, 1, NULL);
	xTaskCreate(stepper_motors_response_y, "Motor_y", 128, NULL, 1, NULL);
	vTaskStartScheduler();
}
