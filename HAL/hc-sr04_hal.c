#include "hc-sr04_hal.h"

const float speedOfSound = 0.0343/2;

void usDelay(uint32_t uSec){
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1;
	usTIM->EGR = 1;
	usTIM->SR &= ~1;
	usTIM->CR1 |= 1;
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

float fHCSR04GetDistance(HCSR04sensor* sensor){
	HAL_GPIO_WritePin(sensor->port,sensor->pin,GPIO_PIN_RESET);
	usDelay(3);
	HAL_GPIO_WritePin(sensor->port,sensor->pin,GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(sensor->port,sensor->pin,GPIO_PIN_RESET);

	//Start IC timer
	HAL_TIM_IC_Start_IT(&sensor->htim, TIM_CHANNEL_1);
	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	do
	{
		if(sensor->icFlag) break;
//		if((HAL_GetTick() - startTick) > 300){
//			sensor->edge2Time = HAL_TIM_ReadCapturedValue(&sensor->htim, TIM_CHANNEL_1);
//			sensor->edge1Time = sensor->edge2Time - 600;
//			sensor->captureIdx = 0;
//			sensor->icFlag = 1;
//		}
	}while((HAL_GetTick() - startTick) < 500);  //500ms
	sensor->icFlag = 0;
	HAL_TIM_IC_Stop_IT(&sensor->htim, TIM_CHANNEL_1);

	//Calculate distance in cm
	if(sensor->edge2Time > sensor->edge1Time)
	{
		sensor->distance = ((sensor->edge2Time - sensor->edge1Time) + 0.0f)*speedOfSound;
	}
	else
	{
		//distance = 0.0f;
	}
	return sensor->distance;
}

HCSR04sensor xHCSR04InitSensor(TIM_HandleTypeDef htim,GPIO_TypeDef* port,uint16_t pin){
	HCSR04sensor ret;
	ret.htim = htim;
	ret.port = port;
	ret.pin = pin;
	ret.captureIdx = 0;
	ret.edge1Time = 0;
	ret.edge2Time = 0;
	ret.icFlag = 0;
	ret.distance = 0;
	return ret;
}

void vHCSR04CaptureCallbak(HCSR04sensor* sensor){
	if(sensor->captureIdx == 0) //First edge
	{
		sensor->edge1Time = HAL_TIM_ReadCapturedValue(&sensor->htim, TIM_CHANNEL_1); //__HAL_TIM_GetCounter(&htim3);//

		sensor->captureIdx = 1;
	}
	else if(sensor->captureIdx == 1) //Second edge
	{
		sensor->edge2Time = HAL_TIM_ReadCapturedValue(&sensor->htim, TIM_CHANNEL_1);
		sensor->captureIdx = 0;
		sensor->icFlag = 1;
	}
}
