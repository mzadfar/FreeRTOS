#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// MUTEX switch : un comment the below line to use mutex
// #define USE_MUTEX

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;

/* Dimensions the buffer into which messages destined for stdout are placed. */
#define mainMAX_MSG_LEN (80)

/* The task to be created.  Two instances of this task are created. */
static void prvPrintTask(void* pvParameters);

/* The function that uses a mutex to control access to standard out. */
static void prvNewPrintString(const portCHAR* pcString);

#ifdef USE_MUTEX
/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
mutex type semaphore that is used to ensure mutual exclusive access to UART. */
xSemaphoreHandle xMutex;
#endif

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

    sprintf(userText, "Demo of mutual exclusion using Mutex APIs\r\n");
    USART_SendText(USART3, userText);

#ifdef USE_MUTEX
    /* Before a semaphore is used it must be explicitly created.  In this example
    a mutex type semaphore is created. */
    xMutex = xSemaphoreCreateMutex();
#endif

    /* The tasks are going to use a pseudo random delay, seed the random number
generator. */
    srand(567);

#ifdef USE_MUTEX
    /* Only create the tasks if the semaphore was created successfully. */
    if (xMutex != NULL)
    {
#endif
        /* Create two instances of the tasks that attempt to write stdout.  The
        string they attempt to write is passed in as the task parameter.  The tasks
        are created at different priorities so some pre-emption will occur. */
        status = xTaskCreate(
            prvPrintTask,
            "Print1",
            240,
            "Task 1 ******************************************\r\n",
            1,
            NULL);
        configASSERT(status == pdPASS);

        /* Create a employee task with less priority than manager */
        status = xTaskCreate(
            prvPrintTask,
            "Print2",
            240,
            "Task 2 ------------------------------------------\r\n",
            2,
            NULL);
        configASSERT(status == pdPASS);

        /*Start the scheduler*/
        vTaskStartScheduler();
#ifdef USE_MUTEX
    }
#endif

    /*By addig scheduler the below code will never be executed
    since above scheduler always runs, resulting in functions
    runs where functions will never return*/
    while (1)
    {
    }
    return 0;
}

static void prvNewPrintString(const portCHAR* pcString)
{
    static char cBuffer[mainMAX_MSG_LEN];

#ifdef USE_MUTEX
    /* The semaphore is created before the scheduler is started so already
    exists by the time this task executes.

    Attempt to take the semaphore, blocking indefinitely if the mutex is not
    available immediately.  The call to xSemaphoreTake() will only return when
    the semaphore has been successfully obtained so there is no need to check the
    return value.  If any other delay period was used then the code must check
    that xSemaphoreTake() returns pdTRUE before accessing the resource (in this
    case standard out. */
    xSemaphoreTake(xMutex, portMAX_DELAY);
    {
#endif
        /* The following line will only execute once the semaphore has been
        successfully obtained - so standard out can be accessed freely. */
        sprintf(userText, "%s", pcString);
        USART_SendText(USART3, userText);
#ifdef USE_MUTEX
    }
    xSemaphoreGive(xMutex);
#endif
}

static void prvPrintTask(void* funcParams)
{
    char* pcStringToPrint;

    /* Two instances of this task are created so the string the task will send
    to prvNewPrintString() is passed in the task parameter.  Cast this to the
    required type. */
    pcStringToPrint = (char*)funcParams;

    for (;;)
    {
        /* Print out the string using the newly defined function. */
        prvNewPrintString(pcStringToPrint);

        /* Wait a pseudo random time.  Note that rand() is not necessarily
        re-entrant, but in this case it does not really matter as the code does
        not care what value is returned.  In a more secure application a version
        of rand() that is known to be re-entrant should be used - or calls to
        rand() should be protected using a critical section. */
        vTaskDelay(rand() & 0XF);
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
    /*USART_2 (USART_B_RX: PD6 D52 on CN9, USART_B_TX: PD5 D53 on CN9) & USART_3
     * (USART_A_TX: PD8, USART_A_RX: PD9)*/
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

    // /* Initiate GPIOC */
    // GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    // GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    // GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    // GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

    // GPIO_Init(GPIOC, &GPIO_InitStruct);

    // /*Configure interrupt for PC13*/
    // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    // /* Initialize EXTI*/
    // EXTI_InitTypeDef EXTI_InitStruct;

    // EXTI_InitStruct.EXTI_Line = EXTI_Line13;
    // EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    // EXTI_InitStruct.EXTI_LineCmd = ENABLE;

    // EXTI_Init(&EXTI_InitStruct);

    // /*Configure IRQ*/
    // NVIC_SetPriority(EXTI15_10_IRQn, 5);
    // NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Initialize GPIOD*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// void EXTI15_10_IRQHandler(void)
// {
//     traceISR_ENTER();
//     /*Cleas the interrupt pending register (PR) bit*/
//     EXTI_ClearITPendingBit(EXTI_Line13);
//     switchPriority = TRUE;
//     traceISR_EXIT();
// }

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