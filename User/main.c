/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
uint8_t x;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
x=5;

	// put your code here, to run repeatedly.
	while(1){
	//	IND_toggle();
		HAL_USART_Transmit&huart5, x, 3, 1000);

			//StartMilliDelay(1000);

	}
}

/*-----------------------------------------------------------*/
