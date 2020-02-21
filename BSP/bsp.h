/*
 * bsp.h
 *
 *  Created on: Feb 16, 2020
 *      Author: antiloope
 */

#ifndef BSP_H_
#define BSP_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

//Led define
#define Led_Pin 						GPIO_PIN_12
#define Led_GPIO_Port 					GPIOD

// Buttons define
#define BUTTON_NEW_MEASURE (uint8_t)1
#define BUTTON_RESET (uint8_t)2
#define BUTTON_CALCULATE (uint8_t)3
#define NO_BUTTONS (uint8_t)0

#define VolumeSensor_Trigger_Pin 		GPIO_PIN_13
#define VolumeSensor_Trigger_GPIO_Port 	GPIOD
#define VolumeSensor_Timer 				htim3

typedef uint8_t buttonsState;

QueueHandle_t ButtonsQueue;
QueueHandle_t measures;

uint32_t globalCounter;

typedef struct{
	float distance;
	uint32_t timeReference;
}measure;

// Used to display symbols in LCD
typedef enum{
	selected=0,
	volume_empty,
	volume,
	separator
}display_symbol;

void vBSPInit();
void vBSPLedOff();
void vBSPLedOn();
void vBSPDisplayClear();
void vBSPDisplaySymbol(uint8_t line,uint8_t column,display_symbol symbol);
void vBSPDisplayPrint(uint8_t offset_line1,const char* line1,uint8_t offset_line2, const char* line2);
float fBSPGetSensor();
void vBSPStartMeasure();

#endif /* BSP_H_ */
