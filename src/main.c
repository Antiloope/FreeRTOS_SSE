/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "main.h"

TaskHandle_t process;
			
void vTaskCalculateSpeed( void *pvParameters ){
	for( ;; )
	{
		measure lastMeasure;
		measure currentMeasure;
		measure auxMeasure; // This measure is used as a flag.
		auxMeasure.distance = 0.0;
		auxMeasure.timeReference = 0;

		if(xQueueReceive(measures,&lastMeasure, 0) == pdPASS){ // Load first measure
			xQueueSendToBack(measures,(void *)&auxMeasure,pdFALSE); // PushBack flag to separate measures to speeds calculations

			while(xQueueReceive(measures,&currentMeasure, 0) == pdPASS && currentMeasure.timeReference != 0){
				// Calculate differences on each interval and push back local speed calculation.
				auxMeasure.distance = currentMeasure.distance - lastMeasure.distance;
				auxMeasure.timeReference = currentMeasure.timeReference - lastMeasure.timeReference;
				xQueueSendToBack(measures,(void *)&auxMeasure,pdFALSE);

				lastMeasure = currentMeasure;
			}

			float speed = 0;
			int cant = 0;

			// Calculate mean speed
			while(xQueueReceive(measures,&currentMeasure, 0) == pdPASS){
				cant++;
				speed += currentMeasure.distance / currentMeasure.timeReference;
			}

			//Convert from cm/ms to m/s
			if(cant == 0){
				speed = fabsf(speed) * 10;
			}else{
				speed = fabsf(speed) * 10 / cant;
			}

			// Display result
			vBSPDisplayClear();

			char a[16] = {32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32};
			a[2] = 48 + (int)speed / 10;
			a[3] = 48 + (int)speed - (int)(speed / 10) * 10;
			a[4] = '.';
			a[5] = 48 + (int)((speed - (int)speed) * 10 );
			a[6] = 48 + (int)((speed * 10 - (int)(speed * 10)) * 10 );
			a[7] = 48 + (int)((speed * 100 - (int)(speed * 100)) * 10 );
			a[8] = 48 + (int)((speed * 1000 - (int)(speed * 1000)) * 10 );
			a[10] = 'm';
			a[11] = '/';
			a[12] = 's';
			vBSPDisplayPrint(0," Speed " ,0, a);
		}else{
			// There are no measures, so speed = 0
			vBSPDisplayClear();
			vBSPDisplayPrint(0," Speed " ,0, "   0 m/s");
		}

		vTaskDelete(process);
	}
}

void vTaskDisplay( void *pvParameters ){
	for( ;; )
	{
		buttonsState currentButtonsState;
		if(xQueueReceive(ButtonsQueue,&currentButtonsState, portMAX_DELAY) == pdPASS){
			if(currentButtonsState == BUTTON_NEW_MEASURE){
				// Get new measure
				measure tmp;
				tmp.distance = fBSPGetSensor();
				tmp.timeReference = globalCounter;

				// Display measure
				char line1[16] = {32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32};
				line1[0] = 'D';
				line1[1] = 'i';
				line1[2] = 's';
				line1[3] = 't';
				line1[4] = 'a';
				line1[5] = 'n';
				line1[6] = 'c';
				line1[7] = 'e';
				line1[10] = 48 + (int)tmp.distance / 10;
				line1[11] = 48 + (int)tmp.distance - (int)(tmp.distance / 10) * 10;
				line1[12] = '.';
				line1[13] = 48 + (int)((tmp.distance - (int)tmp.distance) * 10 );
				line1[14] = 'c';
				line1[15] = 'm';

				char line2[16] = {32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32};

				line2[0] = 'T';
				line2[1] = 'i';
				line2[2] = 'm';
				line2[3] = 'e';
				line2[4] = 's';
				line2[5] = 't';
				line2[6] = 'a';
				line2[7] = 'm';
				line2[8] = 'p';
				//line2[9] = 48 + (int)(tmp.timeReference / 1000000);
				line2[10] = 48 + (int)(tmp.timeReference / 100000) - (int)(tmp.timeReference / 1000000) * 10;
				line2[11] = 48 + (int)(tmp.timeReference / 10000) - (int)(tmp.timeReference / 100000) * 10;
				line2[12] = 48 + (int)(tmp.timeReference / 1000) - (int)(tmp.timeReference / 10000) * 10;
				line2[13] = 48 + (int)(tmp.timeReference / 100) - (int)(tmp.timeReference / 1000) * 10;
				line2[14] = 48 + (int)(tmp.timeReference / 10) - (int)(tmp.timeReference / 100) * 10;
				line2[15] = 48 + (int)(tmp.timeReference / 1) - (int)(tmp.timeReference / 10) * 10;
				vBSPDisplayClear();
				vBSPDisplayPrint(0,line1,0,line2);

				// Send measure to queue
				xQueueSendToBack(measures,(void *)&tmp,pdFALSE);
			}
			if(currentButtonsState == BUTTON_RESET){
				// Reset time stamp counter.
				globalCounter = 0;
				vBSPDisplayClear();
				vBSPDisplayPrint(0,"      Timer     " ,0,"     Reseted    ");
			}
			if(currentButtonsState == BUTTON_CALCULATE){
				// Start new task to calculate using queue measures
				vBSPDisplayClear();
				vBSPDisplayPrint(0," Calculating... " ,0,"");
				xTaskCreate( vTaskCalculateSpeed, "Obtener", 1000, NULL, 4, &process);
			}
		}
	}
}



int main(void)
{
	// Init BSP

	vBSPInit();

	// This queue store pressed buttons
	ButtonsQueue = xQueueCreate( 10, sizeof(buttonsState));
	if(ButtonsQueue == 0) return -3;

	// Store each measure
	measures = xQueueCreate( 5, sizeof(measure));
	if(measures == 0) return -3;

	xTaskCreate( vTaskDisplay,"Escribir",1000,NULL,3,NULL );

	vTaskStartScheduler();

	for(;;);
}
