#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

//#define SEMI_HOSTING

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;

/*Tasks function prototypes*/
void vTask1_Function(void *funcParams);
void vTask2_Function(void *funcParams);

#ifdef SEMI_HOSTING
/*Semi-hosting*/
extern void initialise_monitor_handles();
#endif

int main(void)
{ 
  #ifdef SEMI_HOSTING
  /*Enable semi-hosting*/
  initialise_monitor_handles();
  printf("This is semi-hosting test for FreeRTOS project\n");
  #endif

  /*Reset the RCC clock configuration to the default reset state.
  Only HSI  is turned on, and uses as system clock source as HSI= 16MHz*/
  RCC_DeInit();

  /*Update SytemCoreClock variable to 16MHz*/
  SystemCoreClockUpdate();

  /*Create tasks for RTOS*/
  xTaskCreate(vTask1_Function, "Task1", configMINIMAL_STACK_SIZE, NULL, 2, &xTask1_Handler);
 
  xTaskCreate(vTask2_Function, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, &xTask2_Handler);
 
  /*Start the scheduler*/
  vTaskStartScheduler();

  /*By addig scheduler the below code will never be executed
  since above scheduler always runs, resulting in functions 
  runs where functions will never return*/
  while(1)
  { 

  }
  return 0;
}

void vTask1_Function(void *funcParams)
{
  while(1)
  {
    #ifdef SEMI_HOSTING
    printf("Task 1 function\n");
    #endif
  }
}

void vTask2_Function(void *funcParams)
{
  while(1)
  {
    #ifdef SEMI_HOSTING 
    printf("Task 2 function\n");
    #endif
  }
}
