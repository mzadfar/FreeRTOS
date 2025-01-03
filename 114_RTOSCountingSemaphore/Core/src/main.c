#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;

/*Tasks function prototypes*/
static void vHandlerTask(void* funcParams);
static void vPeriodicTask(void* funcParams);

/* Enable the software interrupt and set its priority. */
static void prvSetupSoftwareInterrupt();

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xCountingSemaphore;

/*Private setup for hardware peripherals*/
static void Peripheral_Setup(void);
static void RCC_Setup(void);
static void GPIO_Setup(void);
static void VirtualCOMPort_Setup(void);
void rtosDelayMs(uint32_t ms);

void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText);
void USART_SendNumber(USART_TypeDef* USARTx, uint32_t sendNumber);

/*Macros section*/
#define TRUE  1
#define FALSE 0

/*Global variables section*/
char userText[99];
uint8_t switchPriority = FALSE;

int main(void)
{
    /*Reset the RCC clock configuration to the default reset state.
    Only HSI  is turned on, and uses as system clock source as HSI= 16MHz*/
    RCC_DeInit();

    /*Update SytemCoreClock variable to 16MHz*/
    SystemCoreClockUpdate();

    /*Initialize peripherals*/
    Peripheral_Setup();

    BaseType_t status;

    // Enable the CYCCNT counter.
    DWT_CTRL |= (1 << 0);

    // Start Recording
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

    sprintf(userText, "Demo of usage of counting semaphore\r\n");
    USART_SendText(USART3, userText);

    /* Before a semaphore is used it must be explicitly created.  In this example
    a counting semaphore is created.  The semaphore is created to have a maximum
    count value of 10, and an initial count value of 0. */
    xCountingSemaphore = xSemaphoreCreateCounting(10, 0);

    /* Check the semaphore was created successfully. */
    if (xCountingSemaphore != NULL)
    {
        /* Enable the button interrupt and set its priority. */
        prvSetupSoftwareInterrupt();

        /* Create the 'handler' task.  This is the task that will be synchronized
        with the interrupt.  The handler task is created with a high priority to
        ensure it runs immediately after the interrupt exits.  In this case a
        priority of 3 is chosen. */
        status = xTaskCreate(vHandlerTask, "Handler", 500, NULL, 1, NULL);
        configASSERT(status == pdPASS);

        /* Create the task that will periodically generate a software interrupt.
        This is created with a priority below the handler task to ensure it will
        get preempted each time the handler task exist the Blocked state. */
        status = xTaskCreate(vPeriodicTask, "Periodic", 500, NULL, 3, NULL);
        configASSERT(status == pdPASS);

        /*Start the scheduler*/
        vTaskStartScheduler();
    }

    sprintf(userText, "Counting semaphore create failed.. \r\n");
    USART_SendText(USART3, userText);

    /*By addig scheduler the below code will never be executed
    since above scheduler always runs, resulting in functions
    runs where functions will never return*/
    while (1)
    {
    }
    return 0;
}

static void vHandlerTask(void* funcParams)
{
    /* As per most tasks, this task is implemented within an infinite loop. */
    for (;;)
    {
        /* Use the semaphore to wait for the event.  The semaphore was created
        before the scheduler was started so before this task ran for the first
        time.  The task blocks indefinitely meaning this function call will only
        return once the semaphore has been successfully obtained - so there is no
        need to check the returned value. */
        xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);

        /* To get here the event must have occurred.  Process the event (in this
        case we just print out a message). */
        sprintf(userText, "Handler task - Processing event.\r\n");
        USART_SendText(USART3, userText);
    }
}

static void vPeriodicTask(void* funcParams)
{
    /* As per most tasks, this task is implemented within an infinite loop. */
    for (;;)
    {
        /* This task is just used to 'simulate' an interrupt.  This is done by
        periodically generating a software interrupt. */
        vTaskDelay(pdMS_TO_TICKS(500));

        /* Generate the interrupt, printing a message both before hand and
        afterwards so the sequence of execution is evident from the output. */
        sprintf(userText, "Periodic task - Pending the interrupt.\r\n");
        USART_SendText(USART3, userText);

        // pend the interrupt
        NVIC_SetPendingIRQ(EXTI15_10_IRQn);

        sprintf(userText, "Periodic task - Resuming.\r\n");
        USART_SendText(USART3, userText);
    }
}

static void Peripheral_Setup(void)
{
    RCC_Setup();
    GPIO_Setup();
    VirtualCOMPort_Setup();
}

static void RCC_Setup()
{
    /*Initialize GPIOB for toggling LEDs*/
    /*USART_2 (USART_B_RX: PD6 D52 on CN9, USART_B_TX: PD5 D53 on CN9) & USART_3 (USART_A_TX:
     * PD8, USART_A_RX: PD9)*/
    /* Initiate clock for GPIOB, GPIOC, GPIOD, and GPIOG */
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    /*Initialize USART3 clock*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /*Initialize SYSCFG clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

static void GPIO_Setup()
{
    /* Initialize GPIOB*/
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Reset every member element of the structure*/
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Initiate GPIOC */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure interrupt for PC13*/
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    /* Initialize EXTI*/
    EXTI_InitTypeDef EXTI_InitStruct;

    EXTI_InitStruct.EXTI_Line = EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;

    EXTI_Init(&EXTI_InitStruct);

    /* Initialize GPIOD*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void prvSetupSoftwareInterrupt()
{
    /* here were simulating the button interrupt by manually setting the interrupt enable bit
     * in the NVIC enable register*/

    /* The interrupt service routine uses an (interrupt safe) FreeRTOS API
    function so the interrupt priority must be at or below the priority defined
    by configSYSCALL_INTERRUPT_PRIORITY. */
    NVIC_SetPriority(EXTI15_10_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    /* Enable the interrupt. */
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* 'Give' the semaphore multiple times.  The first will unblock the handler
    task, the following 'gives' are to demonstrate that the semaphore latches
    the events to allow the handler task to process them in turn without any
    events getting lost.  This simulates multiple interrupts being taken by the
    processor, even though in this case the events are simulated within a single
    interrupt occurrence.*/
    sprintf(userText, "==>Button_Handler\r\n");
    USART_SendText(USART3, userText);

    xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);

    /* Clear the software interrupt bit using the interrupt controllers  */

    /* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.

    NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void VirtualCOMPort_Setup(void)
{
    /*USART_3 (USART_C_TX: PD8, USART_C_RX: PD9) has virtual COM port capability*/

    /*Configure USART3*/
    USART_InitTypeDef USART_InitStruct;

    /*Reset every member element of the structure*/
    memset(&USART_InitStruct, 0, sizeof(USART_InitStruct));

    /*Connect GPIOD pins to AF for USART3*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    /*Configure USART3*/
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    /*Initialize USART3*/
    USART_Init(USART3, &USART_InitStruct);

    /*Enable USART3*/
    USART_Cmd(USART3, ENABLE);

    // /*Enable interrupt for UART3*/
    // USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    // /*Enable interrupt to UART3*/
    // NVIC_EnableIRQ(USART3_IRQn);
}

// void USART3_IRQHandler(void)
// {
//     if (USART_GetITStatus(USART3, USART_IT_RXNE))
//     {
//         if (USART_ReceiveData(USART3) == 'K')
//         {
//             GPIO_ToggleBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14);

//             USART_SendText(USART3, "LED Toggled\n");
//         }
//     }
// }

void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText)
{
    while (*sendText)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) != SET)
            ;
        USART_SendData(USARTx, *sendText);
        *sendText++;
    }
}

void USART_SendNumber(USART_TypeDef* USARTx, uint32_t sendNumber)
{
    /*A temp array to build results of conversion*/
    char value[10];
    /*Loop index*/
    int i = 0;

    do
    {
        /*Convert integer to character*/
        value[i++] = (char)(sendNumber % 10) + '0';
        sendNumber /= 10;
    } while (sendNumber);

    /*Send data*/
    while (i)
    {
        // USART_SendNumber8b(USARTx, value[--i]);
        /*Wait until data register is empty*/
        while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE))
            ;
        USART_SendData(USARTx, value[--i]);
    }
}

void rtosDelayMs(uint32_t ms)
{
    uint32_t curretTickCount = xTaskGetTickCount();

    uint32_t msTicks = (ms * configTICK_RATE_HZ) / 1000;

    while (xTaskGetTickCount() < (curretTickCount + msTicks))
        ;
}