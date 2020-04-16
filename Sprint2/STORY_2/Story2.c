/*
 * Story2.c
 *
 * Created: 4/15/2020 9:19:37 AM
 *  Author: Khaled
 */


/********************************************************************************
 * 								  Included Files								*
 ********************************************************************************/

/* Header file of the story */
#include "Story2.h"

/* FreeRTOS header files */
#include "../../../SERVICE/FREE_RTOS/FreeRTOS.h"
#include "../../../SERVICE/FREE_RTOS/task.h"
#include "../../../SERVICE/FREE_RTOS/semphr.h"
#include "../../../SERVICE/FREE_RTOS/queue.h"

/* Used hardware driver headers */
#include "../../../ECUAL/PUSH_BUTTON/pushButton.h"
#include "../../../ECUAL/LCD/LCD.h"
#include "../../../ECUAL/LED/LED.h"


/********************************************************************************
 * 								Preprocessor Macros								*
 ********************************************************************************/

#define PB1_PRESSED_COUNT_INITIAL_VALUE       0
#define PB2_PRESSED_COUNT_INITIAL_VALUE       0
#define PB_PRESSED                            4
#define PB_RELEASED                           0


/********************************************************************************
 * 							  Static API's Declaration							*
 ********************************************************************************/

/* The initialization task */
static void Sprint2_Story2_InitTask (void *pvParameters);

/* The push button1 task, which decides if the button1 is pressed or released */
static void Sprint2_Story2_PushButton1 (void *pvParameters);

/* The push button2 task, which decides if the button2 is pressed or released */
static void Sprint2_Story2_PushButton2 (void *pvParameters);

/* The LED task, which Indicates if a button is pressed or not */
static void Sprint2_Story2_Led (void *pvParameters);

/* The LCD task, which outputs the suitable string */
static void Sprint2_Story2_Lcd (void *pvParameters);


/********************************************************************************
 * 							  Global Static Variables							*
 ********************************************************************************/

/* initiate init task handle*/
static TaskHandle_t Sprint2_Story2_InitTaskHandle;

/* PB1 task handle */
static TaskHandle_t Sprint2_Story2_PB1TaskHandle;

/* PB2 task handle */
static TaskHandle_t Sprint2_Story2_PB2TaskHandle;

/* A handle used for checking if the mutex is used or not */
static TaskHandle_t Sprint2_Story2_MutexTakenByPB1;
static TaskHandle_t Sprint2_Story2_MutexTakenByPB2;

/* The mutex used in the story */
static SemaphoreHandle_t LED_MutexHandle;


/********************************************************************************
 * 								 API's Definitions								*
 ********************************************************************************/

void Sprint2_Story2 (void)
{
	/* Task creations */
	xTaskCreate (Sprint2_Story2_InitTask, "S2", 100, NULL, 5,&Sprint2_Story2_InitTaskHandle);
	xTaskCreate (Sprint2_Story2_PushButton1, "A", 100, NULL, 4,&Sprint2_Story2_PB1TaskHandle);
	xTaskCreate (Sprint2_Story2_PushButton2, "B", 100, NULL, 3,&Sprint2_Story2_PB2TaskHandle);
	xTaskCreate (Sprint2_Story2_Led, "C", 100, NULL, 2,NULL);
	xTaskCreate (Sprint2_Story2_Lcd, "D", 400, NULL, 1,NULL);

	/*Start Scheduler*/
	vTaskStartScheduler();
}

void Sprint2_Story2_InitTask (void *pvParameters)
{


	while(1)
	{
		/* LCD module initialize */
		LCD_init();

		/* lED module initialize */
		Led_Init(LED_0);

		/* push bottom module initialize */
		pushButtonInit(BTN_0);

		/* push bottom module initialize */
		pushButtonInit(BTN_1);

		/* Mutex created */
		LED_MutexHandle = xSemaphoreCreateMutex();

		/* suspend the task after finishing the initialization */
		vTaskSuspend( Sprint2_Story2_InitTaskHandle );
	}
}

void Sprint2_Story2_PushButton1 (void *pvParameters)
{
	uint8_t au8_PB1PressedCount = PB1_PRESSED_COUNT_INITIAL_VALUE;
	uint8_t au8_PB1Status = RELEASED;

	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 15;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();


	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Now the variable au8_PB1_Status Decides whether the button1 is pressed or not */
		pushButtonGetStatus(BTN_0,&au8_PB1Status);

		/* Storing output of the function into the mutexCheck to decide whether the mutex is taken or not */
		Sprint2_Story2_MutexTakenByPB1 = xSemaphoreGetMutexHolder( LED_MutexHandle );

		/* In case the pushbutton is pressed.. */
		if ((PRESSED == au8_PB1Status) && (4 > au8_PB1PressedCount))
		{
			/* Increment this variable to figure out if the pushbutton is pressed for real,
			   or it's a debouncing problem */
			au8_PB1PressedCount++;
		}
		/* In case the pushbutton is released for 60 ms.. */
		else if ((RELEASED == au8_PB1Status) && (0 < au8_PB1PressedCount))
		{
			/* Decrement the variable */
			au8_PB1PressedCount--;
		}

		/* In case the pushbutton was firgured out to be pressed.. */
		if (  PB_PRESSED == au8_PB1PressedCount && NULL == Sprint2_Story2_MutexTakenByPB1)
		{
			/* Take the mutex */
			xSemaphoreTake( LED_MutexHandle, ( TickType_t ) 15 );
		}
		/* Otherwise.. */
		else if (  PB_RELEASED == au8_PB1PressedCount)
		{
			/* Give the mutex */
			xSemaphoreGive( LED_MutexHandle );
		}
	}
}

void Sprint2_Story2_PushButton2 (void *pvParameters)
{
	uint8_t au8_PB2PressedCount = PB2_PRESSED_COUNT_INITIAL_VALUE;
	uint8_t au8_PB2Status = RELEASED;

	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 15;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();


	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Now the variable au8_PB2_Status Decides whether the button2 is pressed or not */
		pushButtonGetStatus(BTN_1,&au8_PB2Status);

		/* Storing output of the function into the mutexCheck to decide whether the mutex is taken or not */
		Sprint2_Story2_MutexTakenByPB2 = xSemaphoreGetMutexHolder( LED_MutexHandle );

		/* In case the pushbutton is pressed.. */
		if ((PRESSED == au8_PB2Status) && (4 > au8_PB2PressedCount))
		{
			/* Increment this variable to figure out if the pushbutton is pressed for real,
			   or it's a debouncing problem */
			au8_PB2PressedCount++;
		}
		/* In case the pushbutton is released for 60 ms.. */
		else if ((RELEASED == au8_PB2Status) && (0 < au8_PB2PressedCount))
		{
			/* Decrement the variable */
			au8_PB2PressedCount--;
		}

		/* In case the pushbutton was firgured out to be pressed.. */
		if (  PB_PRESSED == au8_PB2PressedCount && NULL == Sprint2_Story2_MutexTakenByPB2)
		{
			/* Take the mutex */
			xSemaphoreTake( LED_MutexHandle, ( TickType_t ) 15 );
		}
		/* Otherwise.. */
		else if (  PB_RELEASED == au8_PB2PressedCount)
		{
			/* Give the mutex */
			xSemaphoreGive( LED_MutexHandle );
		}
	}
}

void Sprint2_Story2_Led (void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 60;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* In case the mutex is not taken.. */
		if (Sprint2_Story2_MutexTakenByPB2 != NULL || Sprint2_Story2_MutexTakenByPB1 != NULL)
		{
			/* turn on the led */
			Led_On(LED_0);
		}
		/* otherwise.. */
		else
		{
			/* turn off the led */
			Led_Off(LED_0);
		}
	}
}

void Sprint2_Story2_Lcd (void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 60;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* In case the button1 is the reason the led is on.. */
		if (Sprint2_Story2_MutexTakenByPB1 == Sprint2_Story2_PB1TaskHandle ||
			Sprint2_Story2_MutexTakenByPB2 == Sprint2_Story2_PB1TaskHandle)
		{
			LCD_displayStringRowColumn(0, 0, "led on ");
			LCD_displayStringRowColumn(1, 0, "PB1");
		}
		/* In case the button2 is the reason the led is on.. */
		if (Sprint2_Story2_MutexTakenByPB1 == Sprint2_Story2_PB2TaskHandle ||
			Sprint2_Story2_MutexTakenByPB2 == Sprint2_Story2_PB2TaskHandle)
		{
			LCD_displayStringRowColumn(0, 0, "led on ");
			LCD_displayStringRowColumn(1, 0, "PB2");
		}
		/* In case the led is off.. */
		else if(Sprint2_Story2_MutexTakenByPB2 == NULL && Sprint2_Story2_MutexTakenByPB2 == NULL)
		{
			LCD_displayStringRowColumn(0, 0, "led off");
			LCD_displayStringRowColumn(1, 0, "N/A");
		}
	}
}
