#ifndef STM32F10X_HC_SR04_H_
#define STM32F10X_HC_SR04_H_

#include "stm32f4xx_hal.h"

// TIM_TypeDef and HandleTypeDef of a timer for uSec delay (period of 1 uSec)
#define usTIM 			TIM4

typedef struct {
	// 		Instance of a timer with capture mode enabled 	//
	TIM_HandleTypeDef 	htim;
	// 			Port and pin of capture timer 				//
	GPIO_TypeDef		*port;
	uint16_t 			pin;
	// 				Internal usage. Can't touch this 		//
	uint8_t 			captureIdx;
	uint32_t 			edge1Time;
	uint32_t 			edge2Time;
	uint8_t 			icFlag;
	float 				distance;
}HCSR04sensor;

/**
 * Return the measure in cm from specified sensor.
 */
float fHCSR04GetDistance(HCSR04sensor *sensor);

/**
 * Return an instance of a sensor structure with the passed arguments
 * IMPORTANT: All sensors must be initialized by this way
 */
HCSR04sensor xHCSR04InitSensor(TIM_HandleTypeDef htim,GPIO_TypeDef* port,uint16_t pin);

/**
 * This function must be called in capture interrupt callback of timer and the correspondent sensor must be specified.
 */
void vHCSR04CaptureCallbak(HCSR04sensor* sensor);


#endif /* STM32F10X_HC_SR04_H_ */
