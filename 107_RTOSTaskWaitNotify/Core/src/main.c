#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;

/*Tasks function prototypes*/
void LED_task_Function(void* funcParams);
void PushButton_task_Function(void* funcParams);

#ifdef SEMI_HOSTING
/*Semi-hosting*/
extern void initialise_monitor_handles();
#endif

/*Private setup for hardware peripherals*/
static void Peripheral_Setup(void);
static void RCC_Setup(void);
static void GPIO_Setup(void);
static void VirtualCOMPort_Setup(void);
void rtosDelayMs(uint32_t ms);

void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText);

/*Macros section*/
#define TRUE                    1
#define FALSE                   0
#define PUSH_BUTTON_PRESSED     TRUE
#define PUSH_BUTTON_NOT_PRESSED FALSE

/*Global variables section*/
uint8_t pushButtonStatusFlag = PUSH_BUTTON_NOT_PRESSED;

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

    /*Initialize peripherals*/
    Peripheral_Setup();

    // Start Recording
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

    /*Create LED_task for RTOS*/
    /*Increase stack size to 500 wods instead of 130words=configMINIMAL_STACK_SIZE
    since we are going to use FreeRTOS API,and tasks use higher stack memory*/
    xTaskCreate(LED_task_Function, "LED_task", 500, NULL, 2, &xTask1_Handler);

    xTaskCreate(PushButton_task_Function, "PushButton_task", 500, NULL, 2, &xTask2_Handler);

    /*Start the scheduler*/
    vTaskStartScheduler();

    /*By addig scheduler the below code will never be executed
    since above scheduler always runs, resulting in functions
    runs where functions will never return*/
    while (1)
    {
    }
    return 0;
}

void LED_task_Function(void* funcParams)
{
    while (1)
    {
        /*Wait until any notification event from button task received*/
        if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
        {
            /*We received the notification*/
            GPIO_ToggleBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14);
            USART_SendText(USART3, "Notification is received\r\n");
        }
    }
}

void PushButton_task_Function(void* funcParams)
{
    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
        {
            /*Botton was pressed*/
            rtosDelayMs(100);
            xTaskNotify(xTask1_Handler, 0x0, eNoAction);
            USART_SendText(USART3, "Notification is sent\r\n");
        }
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
#if 0
  /*Configure interrupt for PC13*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

  /* Initialize EXTI*/
  EXTI_InitTypeDef EXTI_InitStruct;

  EXTI_InitStruct.EXTI_Line = EXTI_Line13;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStruct);

  /*Configure IRQ*/
  NVIC_SetPriority(EXTI15_10_IRQn, 5);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
    /* Initialize GPIOD*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStruct);
}
#if 0
void EXTI15_10_IRQHandler(void)
{
    traceISR_ENTER();
    /*Cleas the interrupt pending register (PR) bit*/
    EXTI_ClearITPendingBit(EXTI_Line13);
    PushButton_Handler(NULL);
    traceISR_EXIT();
}
#endif
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

    /*Enable interrupt for UART3*/
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /*Enable interrupt to UART3*/
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        if (USART_ReceiveData(USART3) == 'K')
        {
            GPIO_ToggleBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14);

            USART_SendText(USART3, "LED Toggled\n");
        }
    }
}

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

void rtosDelayMs(uint32_t ms)
{
    uint32_t curretTickCount = xTaskGetTickCount();

    uint32_t msTicks = (ms * configTICK_RATE_HZ) / 1000;

    while (xTaskGetTickCount() < (curretTickCount + msTicks))
        ;
}