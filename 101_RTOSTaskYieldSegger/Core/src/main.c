#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

TaskHandle_t xTask1_Handler = NULL;
TaskHandle_t xTask2_Handler = NULL;

/*Tasks function prototypes*/
void vTask1_Function(void* funcParams);
void vTask2_Function(void* funcParams);

/*Private setup for hardware peripherals*/
static void prvSetupUart(void);
static void prvSetupHardware(void);

void USART_SendText(USART_TypeDef* USARTx, const char* sendText);

// extern void SEGGER_UART_init(uint32_t);

char userText[99];

int main(void)
{
    /*Reset the RCC clock configuration to the default reset state.
    Only HSI  is turned on, and uses as system clock source as HSI= 16MHz*/
    RCC_DeInit();

    /*Update SytemCoreClock variable to 16MHz*/
    SystemCoreClockUpdate();

    prvSetupHardware();

    sprintf(userText, "This is the message to be sent va Virtual COM PORT\r\n");
    USART_SendText(USART3, userText);

    BaseType_t status;

    // Enable the CYCCNT counter.
    DWT_CTRL |= (1 << 0);

    // Start Recording
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();
    // SEGGER_UART_init(500000);

    /*Create tasks for RTOS*/
    status = xTaskCreate(
        vTask1_Function, "Task1", configMINIMAL_STACK_SIZE, NULL, 2, &xTask1_Handler);
    configASSERT(status == pdPASS);

    status = xTaskCreate(
        vTask2_Function, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, &xTask2_Handler);
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
    // char userText[100];

    while (1)
    {
        snprintf(userText, 100, "%s\n", (char*)funcParams);
        SEGGER_SYSVIEW_PrintfTarget(userText);
        taskYIELD();
    }
}

void vTask2_Function(void* funcParams)
{
    // char userText[100];
    while (1)
    {
        snprintf(userText, 100, "%s\n", (char*)funcParams);
        SEGGER_SYSVIEW_PrintfTarget(userText);
        taskYIELD();
    }
}

static void prvSetupUart(void)
{
    /*Virtual COM Port: USART_3 (USART_C_TX: PD8, USART_C_RX: PD9)*/
    /* Initiate clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Initiate GPIOD */
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Reset every member element of the structure*/
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Connect GPIOD pins to AF for USART3*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    /*Initialize USART3 clock*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /*Configure USART3*/
    USART_InitTypeDef USART_InitStruct;

    /*Reset every member element of the structure*/
    memset(&USART_InitStruct, 0, sizeof(USART_InitStruct));

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

static void prvSetupHardware(void)
{
    prvSetupUart();
}

void USART_SendText(USART_TypeDef* USARTx, const char* sendText)
{
    for (uint32_t i = 0; i < strlen(sendText); i++)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) != SET)
            ;
        USART_SendData(USARTx, sendText[i]);
    }
}
