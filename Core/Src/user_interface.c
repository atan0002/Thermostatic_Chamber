/*
 * user_interface.c
 *
 *  Created on: Jan 3, 2022
 *      Author: Natalia Wiśniewska
 */

#include "user_interface.h"



void UserInterfaceInit(void){
	//addres of LCD
	disp.addr = (0x3F << 1);
	disp.bl = true;
    lcd_init(&disp);

    HAL_Delay(500);
    initpage=false;
    lastpage=false;

    InitPage();


}

void InitPage(void){
	sprintf((char *)disp.f_line,"  Thermostatic  ");
	sprintf((char *)disp.s_line,"     chamber    ");
	HAL_Delay(1000);

	lcd_display(&disp);
	initpage = true;
}

void FirstPage(int i){
	sprintf((char *)disp.f_line,PAGEONE_MODE1);
	sprintf((char *)disp.s_line,"       %d",i);
	HAL_Delay(200);
	lcd_display(&disp);


}

void DisplayResults(int target,float actual_temp){
	sprintf((char *)disp.f_line,"TEMP:%.2f",actual_temp);
	sprintf((char *)disp.s_line,"TARGET TEMP:%d",target);
	HAL_Delay(200);
	lcd_display(&disp);

}

int ButtonsUsage(void){

while(1){

if((HAL_GPIO_ReadPin(Button_Plus_GPIO_Port, Button_Plus_Pin)==GPIO_PIN_SET && initpage==true)||
		(HAL_GPIO_ReadPin(Button_Minus_GPIO_Port, Button_Minus_Pin)==GPIO_PIN_SET && initpage==true)){


	 val= 30;
	FirstPage(val);
	initpage=false;

}
if(HAL_GPIO_ReadPin(Button_Minus_GPIO_Port, Button_Minus_Pin)==GPIO_PIN_SET&& lastpage==false && initpage==false){

	val=val-10;
	if(val<30){
	val=30;

	}
	FirstPage(val);

}
if(HAL_GPIO_ReadPin(Button_Plus_GPIO_Port, Button_Plus_Pin)==GPIO_PIN_SET && lastpage==false && initpage==false){


	val=val+10;
	if(val>120){
		val=120;
	}
	FirstPage(val);
}

if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_RESET && initpage==false){
	lastpage=true;
	break;
}
}
return val;

}
