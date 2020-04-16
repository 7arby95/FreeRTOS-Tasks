/*
 * Story1.c
 *
 *  Created on: Apr 16, 2020
 *      Author: Youssef
 */


/********************************************************************************
 * 								  Included Files								*
 ********************************************************************************/

/* Header file of the story */
#include "Story1.h"

/* FreeRTOS header files */
#include "../../../SERVICE/FREE_RTOS/FreeRTOS.h"
#include "../../../SERVICE/FREE_RTOS/task.h"
#include "../../../SERVICE/FREE_RTOS/semphr.h"
#include "../../../SERVICE/FREE_RTOS/queue.h"

/* Used hardware driver headers */
#include "../../../ECUAL/PUSH_BUTTON/pushButton.h"
#include "../../../ECUAL/LCD/LCD.h"

/* This library is included for the "strcpy" function */
#include <string.h>


/********************************************************************************
 * 								Preprocessor Macros								*
 ********************************************************************************/

#define PB_PRESSED_COUNT_INITIAL_VALUE       0
#define PB_PRESSED                           4
#define PB_RELEASED                          0


/********************************************************************************
 * 							  Static API's Declaration							*
 ********************************************************************************/

/* The initialization task */
static void Sprint2_Story1_InitTask(void *pvParameters);

/* The push button task, which decides if the button is pressed or released */
static void Sprint2_Story1_PushButtonState(void *pvParameters);

/* The control task, which decides the output of the LCD */
static void Sprint2_Story1_ControlTask(void *pvParameters);

/* The LCD task, which outputs whatever it receives from the control task */
static void Sprint2_Story1_LcdDisplayer(void *pvParameters);


/********************************************************************************
 * 							  Global Static Variables							*
 ********************************************************************************/

/* Init task handle */
static TaskHandle_t Sprint2_Story1_InitTaskHandle;

/* PB task handle */
static TaskHandle_t Sprint2_Story1_PBTaskHandle;

/* A handle used for checking if the mutex is used or not */
static TaskHandle_t Sprint2_Story1_MutexHandlerPB_Check;
static TaskHandle_t Sprint2_Story1_MutexHandlerLCD_Check;

/* The mutexes used in the story */
static SemaphoreHandle_t Story1_MutexHandlePB;
static SemaphoreHandle_t Story1_MutexHandleLCD;

/* The string which will be printed on the LCD initialized with the required contents */
static char string[16] = "Hello LCD !!";


/********************************************************************************
 * 								 API's Definitions								*
 ********************************************************************************/

void Sprint2_Story1(void)
{
	/* Task creations */
	xTaskCreate(Sprint2_Story1_InitTask, "S1", 100, NULL, 4, &Sprint2_Story1_InitTaskHandle);
	xTaskCreate(Sprint2_Story1_LcdDisplayer, "A", 150, NULL, 1, NULL);
	xTaskCreate(Sprint2_Story1_PushButtonState, "B", 100, NULL, 3, &Sprint2_Story1_PBTaskHandle);
	xTaskCreate(Sprint2_Story1_ControlTask, "C", 150, NULL, 2, NULL);

	/* Start Scheduler */
	vTaskStartScheduler();
}


/********************************************************************************
 * 							  Static API's Definitions							*
 ********************************************************************************/

void Sprint2_Story1_InitTask(void *pvParameters)
{


	while(1)
	{
		/* Pushbutton module initialization */
		pushButtonInit(BTN_0);

		/* LCD module initialization */
		LCD_init();

		/* Mutexes creation */
		Story1_MutexHandlePB = xSemaphoreCreateMutex();
		Story1_MutexHandleLCD = xSemaphoreCreateMutex();

		/* Suspend the task after finishing the initialization */
		vTaskSuspend(Sprint2_Story1_InitTaskHandle);
	}
}

void Sprint2_Story1_PushButtonState(void *pvParameters)
{
	uint8_t au8_PBPressedCount = PB_PRESSED_COUNT_INITIAL_VALUE;
	uint8_t au8_PB_Status = RELEASED;

	/* Reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* A variable to decide how frequent the task will be repeated */
	const TickType_t xFrequency = 15;

	/* Initialize the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Storing output of the function into the mutexCheck to decide whether the mutex is taken or not */
		Sprint2_Story1_MutexHandlerPB_Check = xSemaphoreGetMutexHolder(Story1_MutexHandlePB);

		/* Now the variable au8_PB_Status Decides whether the button is pressed or not */
		pushButtonGetStatus(BTN_0, &au8_PB_Status);

		/* In case the pushbutton is pressed.. */
		if((au8_PB_Status == PRESSED) && (au8_PBPressedCount < PB_PRESSED))
		{
			/* Increment this variable to figure out if the pushbutton is pressed for real,
			   or it's a debouncing problem */
			au8_PBPressedCount++;
		}
		/* In case the pushbutton is released for 60 ms.. */
		else if((au8_PB_Status == RELEASED) && (au8_PBPressedCount > PB_RELEASED))
		{
			/* Decrement the variable */
			au8_PBPressedCount--;
		}

		/* In case the pushbutton was firgured out to be pressed.. */
		if ((au8_PBPressedCount == PB_PRESSED) && (Sprint2_Story1_MutexHandlerPB_Check == NULL))
		{
			/* Take the mutex */
			xSemaphoreTake(Story1_MutexHandlePB, (TickType_t)15);
		}
		/* Otherwise.. */
		else if (au8_PBPressedCount == PB_RELEASED)
		{
			/* Give the mutex */
			xSemaphoreGive(Story1_MutexHandlePB);
		}
	}
}

void Sprint2_Story1_ControlTask(void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* A variable to decide how frequent the task will be repeated */
	const TickType_t xFrequency = 400;

	/* Initialize the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* In case the pushbutton is not pressed.. */
		if(Sprint2_Story1_MutexHandlerPB_Check == NULL)
		{
			/* Change the string accordingly */
			strcpy(string, "Hello LCD !!");
			/* And take the LCD mutex */
			xSemaphoreTake(Story1_MutexHandleLCD, (TickType_t)400);
		}else
		{
			/* Change the string accordingly */
			strcpy(string, "over-written !");
			/* And give the LCD mutex */
			xSemaphoreGive(Story1_MutexHandleLCD);
		}
	}
}

void Sprint2_Story1_LcdDisplayer(void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* A variable to decide how frequent the task will be repeated */
	const TickType_t xFrequency = 400;

	/* Initialize the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Storing output of the function into the mutexCheck to decide whether the mutex is taken or not */
		Sprint2_Story1_MutexHandlerLCD_Check = xSemaphoreGetMutexHolder(Story1_MutexHandleLCD);

		/* In case the LCD mutex is not taken, and therefore the button is pressed.. */
		if(Sprint2_Story1_MutexHandlerLCD_Check == NULL)
		{
			/* Print the received string in the second row */
			LCD_displayStringRowColumn(1, 0, string);
		}
		/* otherwise.. */
		else
		{
			/* Do the normal job */
			LCD_clear();
			vTaskDelay(200);
			LCD_displayStringRowColumn(0, 0, string);
		}
	}
}
