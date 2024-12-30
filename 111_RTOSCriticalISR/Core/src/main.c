#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;
TaskHandle_t xTask3_Handler = NULL;
TaskHandle_t xTask4_Handler = NULL;

TaskHandle_t volatile nextTaskHandle = NULL;

/*Tasks function prototypes*/
void vTask1_Function(void* funcParams); // Green LED
void vTask2_Function(void* funcParams); // Blue LED
void vTask3_Function(void* funcParams); // Red LED

/*Private setup for hardware peripherals*/
static void Peripheral_Setup(void);
static void RCC_Setup(void);
static void GPIO_Setup(void);
static void VirtualCOMPort_Setup(void);

void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText);
void rtosDelayMs(uint32_t ms);
void PushButton_Handler(void* funcParams);

// extern void SEGGER_UART_init(uint32_t);

char userText[99];

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
    // SEGGER_UART_init(500000);

    /*Create tasks for RTOS*/
    status = xTaskCreate(
        vTask1_Function, "Task1", configMINIMAL_STACK_SIZE, NULL, 3, &xTask1_Handler);
    configASSERT(status == pdPASS);

    nextTaskHandle = xTask1_Handler;

    status = xTaskCreate(
        vTask2_Function, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, &xTask2_Handler);
    configASSERT(status == pdPASS);

    status = xTaskCreate(
        vTask3_Function, "Task3", configMINIMAL_STACK_SIZE, NULL, 1, &xTask3_Handler);
    configASSERT(status == pdPASS);

    /*Start FreeRTOS scheduler*/
    vTaskStartScheduler();

    /*By addig scheduler the below code will never be executed
    since above scheduler always runs, resulting in functions
    runs where functions will never return*/
    while (1)
    {
    }
    return 0;
}

void vTask1_Function(void* funcParams)
{
    BaseType_t status;
    while (1)
    {
        SEGGER_SYSVIEW_PrintfTarget("Toggling green LED");
        GPIO_ToggleBits(GPIOB, GPIO_Pin_0);
        status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1200));
        if (status == pdTRUE)
        {
            portENTER_CRITICAL();
            nextTaskHandle = xTask2_Handler;
            GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
            SEGGER_SYSVIEW_PrintfTarget("Delete green LED task");
            portEXIT_CRITICAL();
            vTaskDelete(NULL);
        }
    }
}

void vTask2_Function(void* funcParams)
{
    BaseType_t status;
    while (1)
    {
        SEGGER_SYSVIEW_PrintfTarget("Toggling blue LED");
        GPIO_ToggleBits(GPIOB, GPIO_Pin_7);
        status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(800));
        if (status == pdTRUE)
        {
            portENTER_CRITICAL();
            nextTaskHandle = xTask3_Handler;
            GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
            SEGGER_SYSVIEW_PrintfTarget("Delete blue LED task");
            portEXIT_CRITICAL();
            vTaskDelete(NULL);
        }
    }
}

void vTask3_Function(void* funcParams)
{
    BaseType_t status;
    while (1)
    {
        SEGGER_SYSVIEW_PrintfTarget("Toggling red LED");
        GPIO_ToggleBits(GPIOB, GPIO_Pin_14);
        status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(400));
        if (status == pdTRUE)
        {
            portENTER_CRITICAL();
            nextTaskHandle = NULL;
            GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
            SEGGER_SYSVIEW_PrintfTarget("Delete red LED task");
            portEXIT_CRITICAL();
            vTaskDelete(NULL);
        }
    }
}

void PushButton_Handler(void* funcParams)
{
    BaseType_t pxHigherPriorityTaskWoken;
    pxHigherPriorityTaskWoken = pdFALSE;

    traceISR_ENTER();
    xTaskNotifyFromISR(nextTaskHandle, 0, eNoAction, &pxHigherPriorityTaskWoken);

    /* once the ISR exits, the below macro makes higher priority task which got unblocked to
     * resume on the CPU */
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

    traceISR_EXIT();
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

    /*Configure IRQ*/
    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Initialize GPIOD*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void EXTI15_10_IRQHandler(void)
{
    traceISR_ENTER();
    /*Clears the interrupt pending register (PR) bit*/
    EXTI_ClearITPendingBit(EXTI_Line13);
    PushButton_Handler(NULL);
    traceISR_EXIT();
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

void vApplicationIdleHook(void)
{
    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
}