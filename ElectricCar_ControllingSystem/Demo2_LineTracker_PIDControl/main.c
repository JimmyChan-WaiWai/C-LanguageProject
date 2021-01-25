#include "stm32f10x.h" 
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


float Kp_Out = 214;								//200, 128 / 1600 (max 3200)   	//214, 132/ 2000 (max 4000)
float Kd_Out = 132;										//1,2,3,6									//1,2,3,8
int offset_Out = 2100;						


float Kp_In = 168;								//168, 96 / 1250
float Kd_In= 96;												//1,2,4,8
int offset_In = 1600;

int new_error,  
		last_error, 
		track_state,
		sysTick,
		msTick
		= 0;

char strBuffer[16] = {'\0'};

unsigned char timer_start = 0;

uint16_t receive_value;
uint8_t sensor_value;



void doNothing(){};
	
void delayMs(uint32_t ms)
{
	int i = 72000;
	int j	= ms *i;
	while(j!=0)//20ms
	{j--;}
}



// 0 is black, 1 is white
void get_SensorValue(){

		GPIO_SetBits(GPIOB, GPIO_Pin_12);						//Pull-Up
		SPI_I2S_SendData(SPI2,0xFF);
	
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);				//Waiting
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	
		
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);					//Activate
		SPI_I2S_SendData(SPI2,0xFF);
		receive_value = SPI_I2S_ReceiveData(SPI2);
		sensor_value = ((receive_value>>7)&(0xFF));
		
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);				//Waiting
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		
		GPIO_SetBits(GPIOB, GPIO_Pin_12);						//Pull-Up
}

void sensor_CheckHardware(){
		get_SensorValue();
	/********************/									//Test Sensor
			for (int i=0;i<=7;i++)
				{
					if (((sensor_value>>i)&(0x01))== 0x01) 
					{
						while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
						USART_SendData(USART2, i+'1');
					}
				}
	/*********************/
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void detect_If_ALLBLACK(){
			if ( sensor_value == 0x00){							//all black, black is 0
					delayMs(10);
					do{
						GPIO_ResetBits(GPIOB, GPIO_Pin_7);
						get_SensorValue();
					}	while (sensor_value == 0x00);
					track_state++;

					GPIO_SetBits(GPIOB, GPIO_Pin_7);
		}
}


void error_Clkwise_Outer(){

			if (((~sensor_value>>4)& 0x01) == 0x01) //11101111
					new_error = 1 ;
			else if (((~sensor_value>>3)& 0x01) == 0x01) //11110111
					new_error = -1 ;	
			else if (((~sensor_value>>7)& 0x01) == 0x01) //01111111
					new_error = 8 ;
			else if (((~sensor_value>>6)& 0x01) == 0x01) //10111111
					new_error = 3 ;
			else if (((~sensor_value>>5)& 0x01) == 0x01) //11011111
					new_error = 2 ;
			else if (((~sensor_value>>0)& 0x01) == 0x01) //11111110
					new_error = -8 ;
			else if (((~sensor_value>>1)& 0x01) == 0x01) //11111101
					new_error = -3 ;
			else if (((~sensor_value>>2)& 0x01) == 0x01) //11111011
					new_error = -2;




}

//black is 0, ~sensor_value, then black is 1

void error_Clkwise_Inner(){										

			if (((~sensor_value>>7)& 0x01) == 0x01) //01111111
					new_error = 8 ;
			else if (((~sensor_value>>6)& 0x01) == 0x01) //10111111
					new_error = 4 ;
			else if (((~sensor_value>>5)& 0x01) == 0x01) //11011111
					new_error = 2 ;
			else if (((~sensor_value>>0)& 0x01) == 0x01) //11111110
					new_error = -8 ;
			else if (((~sensor_value>>1)& 0x01) == 0x01) //11111101
					new_error = -4 ;
			else if (((~sensor_value>>2)& 0x01) == 0x01) //11111011
					new_error = -2 ;
			else if (((~sensor_value>>3)& 0x01) == 0x01) //11110111
					new_error = -1 ;
			else if (((~sensor_value>>4)& 0x01) == 0x01) //11101111
					new_error = 1 ;



}

void error_AtiClk_Outer(){
			if (((~sensor_value>>4)& 0x01) == 0x01) //11101111
					new_error = -1 ;
			else if (((~sensor_value>>3)& 0x01) == 0x01) //11110111
					new_error = 1 ;		
			else if (((~sensor_value>>0)& 0x01) == 0x01) //11111110
					new_error = -8 ;
			else if (((~sensor_value>>1)& 0x01) == 0x01) //11111101
					new_error = -3 ;
			else if (((~sensor_value>>2)& 0x01) == 0x01) //11111011
					new_error = -2 ;
			else if (((~sensor_value>>7)& 0x01) == 0x01) //01111111
					new_error = 8 ;
			else if (((~sensor_value>>6)& 0x01) == 0x01) //10111111
					new_error = 3 ;
			else if (((~sensor_value>>5)& 0x01) == 0x01) //11011111
					new_error = 2 ;

}

void error_AtiClk_Inner(){
			if (((~sensor_value>>0)& 0x01) == 0x01) //11111110
					new_error = -8 ;
			else	if (((~sensor_value>>1)& 0x01) == 0x01) //11111101
					new_error = -4 ;
			else if (((~sensor_value>>2)& 0x01) == 0x01) //11111011
					new_error = -2 ;
			else if (((~sensor_value>>7)& 0x01) == 0x01) //01111111
					new_error = 8 ;
			else if (((~sensor_value>>6)& 0x01) == 0x01) //10111111
					new_error = 4 ;
			else if (((~sensor_value>>5)& 0x01) == 0x01) //11011111
					new_error = 2 ;
			else if (((~sensor_value>>3)& 0x01) == 0x01) //11110111
					new_error = -1 ;
			else if (((~sensor_value>>4)& 0x01) == 0x01) //11101111
					new_error = 1 ;
}

void tune_MotorSpeed(int offset_Speed,float Kp,float Kd){
		int change_Speed, left_MotorSpeed, Right_MotorSpeed;
		change_Speed = Kp*new_error  + Kd * (new_error-last_error);
		left_MotorSpeed = offset_Speed + change_Speed;
		Right_MotorSpeed = offset_Speed -	change_Speed;
	
		if (left_MotorSpeed>=4000) left_MotorSpeed=4000;
		else if (left_MotorSpeed<=0) left_MotorSpeed=0;
	
		if (Right_MotorSpeed>=4000) Right_MotorSpeed=4000;
		else if (Right_MotorSpeed<=0) Right_MotorSpeed=0;
	

		
		TIM_SetCompare2(TIM3, left_MotorSpeed);
		TIM_SetCompare1(TIM3, Right_MotorSpeed);
	
		last_error = new_error;

}

void turn_MotorDirection(){
		TIM_SetCompare1(TIM3, 1); 
		TIM_SetCompare2(TIM3, 1);
		delayMs(10);
	
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);							//Left Motor Backward
		TIM_SetCompare1(TIM3, 1); 
		TIM_SetCompare2(TIM3, 1);
		delayMs(10);
		TIM_SetCompare1(TIM3, 1000); 
		TIM_SetCompare2(TIM3, 1000);
		delayMs(100);
		do {get_SensorValue();} while (((~sensor_value>>3)& 0x03) == 0x00);
		TIM_SetCompare1(TIM3, 1000); 
		TIM_SetCompare2(TIM3, 1000);
		delayMs(100);
		do {get_SensorValue();} while (((~sensor_value>>3)& 0x03) == 0x00);
		GPIO_SetBits(GPIOA, GPIO_Pin_0);						//Left Motor Forward
		TIM_SetCompare1(TIM3, 1); 
		TIM_SetCompare2(TIM3, 1);
		delayMs(10);

		track_state ++;
}

void stop_Motor(){
		TIM_SetCompare2(TIM3, 0);
		TIM_SetCompare1(TIM3, 0);
}


void SysTick_Handler(){
	if (timer_start==0)
		
		doNothing();
	else 
	{
		sysTick++;
		if (sysTick==1)
		{
			//sensor_CheckHardware();
			if (track_state!=5)
			{
				get_SensorValue();
				detect_If_ALLBLACK();
			}
			if (track_state <=5){
				if (track_state == 0||track_state == 1||track_state == 2||track_state == 3)	{
						error_Clkwise_Outer();
						tune_MotorSpeed(offset_Out,Kp_Out,Kd_Out);}
				
				else if (track_state == 4)	{
						error_Clkwise_Inner();
						tune_MotorSpeed(offset_In,Kp_In,Kd_In);}
				
				else if (track_state == 5)
						turn_MotorDirection();
			}
			else {
					if (track_state == 6||track_state == 7||track_state == 8||track_state == 9){
						error_AtiClk_Outer(); 
						tune_MotorSpeed(offset_Out,Kp_Out,Kd_Out);}
				
					else if (track_state == 10){
						error_AtiClk_Inner(); 
						tune_MotorSpeed(offset_In,Kp_In,Kd_In);}
				
					else if (track_state >= 11)
						stop_Motor();		
			}					
			sysTick=0;
		}
	}
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {		
			if (timer_start == 0){
				GPIO_SetBits(GPIOB, GPIO_Pin_7);
				timer_start = 1;
			}
			else{
				timer_start = 0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_7);
			}
			delayMs(50);
			EXTI_ClearITPendingBit(EXTI_Line8);
		}
}

void USART2_IRQHandler(){
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
      USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}



int main(void) 
{
	RCC_init();
	Button_LED_init();
	TIM3_PWM_init();
	SysTick_init();
	SPI2_init();
	USART2_init();

	//GPIO_SetBits(GPIOB, GPIO_Pin_7);			//Debugging LED
	
	while(1){	}
}