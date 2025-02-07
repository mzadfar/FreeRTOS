#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"

#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"
#include "timers.h"

/*Macros section*/
#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

#define TRUE  1
#define FALSE 0

/*Private setup for hardware peripherals*/
static void Peripheral_Setup(void);
static void RCC_Setup(void);
static void GPIO_Setup(void);
static void VirtualCOMPort_Setup(void);
static void RTC_Setup(void);

void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText);
void rtosDelayMs(uint32_t ms);

void getArguments(uint8_t* buffer);
uint8_t getCommandCode(uint8_t* buffer);

// prototypes command helper functions
void make_led_on(void);
void make_led_off(void);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void read_led_status(char* task_msg);
void read_rtc_info(char* task_msg);
void print_error_message(char* task_msg);

// tasks prototypes
void vTask1_menu_display(void* params);
void vTask2_cmd_handling(void* params);
void vTask3_cmd_processing(void* params);
void vTask4_uart_write(void* params);

// Software timer callback function prototype
void led_toggle(TimerHandle_t xTimer);

// global space for some variable
char usr_msg[250] = {0};

// task handles
TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;
TaskHandle_t xTaskHandle4 = NULL;

// Queue handle
QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

// softwar timer handler
TimerHandle_t led_timer_handle = NULL;

// command structure
typedef struct APP_CMD
{
    uint8_t COMMAND_NUM;
    uint8_t COMMAND_ARGS[10];
} APP_CMD_t;

uint8_t command_buffer[20];
uint8_t command_len = 0;

// This is the menu
char menu[] = {
    "\
\r\nLED_ON             ----> 1 \
\r\nLED_OFF            ----> 2 \
\r\nLED_TOGGLE         ----> 3 \
\r\nLED_TOGGLE_OFF     ----> 4 \
\r\nLED_READ_STATUS    ----> 5 \
\r\nRTC_PRINT_DATETIME ----> 6 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};

#define LED_ON_COMMAND             1
#define LED_OFF_COMMAND            2
#define LED_TOGGLE_COMMAND         3
#define LED_TOGGLE_STOP_COMMAND    4
#define LED_READ_STATUS_COMMAND    5
#define RTC_READ_DATE_TIME_COMMAND 6

// /*Global variables*/
RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;

int main(void)
{
    DWT_CTRL |= (1 << 0); // Enable CYCCNT in DWT_CTRL.

    /*Reset the RCC clock configuration to the default reset state.
    Only HSI  is turned on, and uses as system clock source as HSI= 16MHz*/
    RCC_DeInit();

    /*Update SytemCoreClock variable to 16MHz*/
    SystemCoreClockUpdate();

    /*Initialize peripherals*/
    Peripheral_Setup();

    sprintf(usr_msg, "\r\nThis is Queue Command Processing Demo\r\n");
    USART_SendText(USART3, usr_msg);

    BaseType_t status;

    // lets create command queue
    command_queue = xQueueCreate(10, sizeof(APP_CMD_t*));

    // lets create the write queue
    uart_write_queue = xQueueCreate(10, sizeof(char*));

    if ((command_queue != NULL) && (uart_write_queue != NULL))
    {
        // lets create task-1
        status = xTaskCreate(vTask1_menu_display, "TASK1-MENU", 500, NULL, 1, &xTaskHandle1);
        configASSERT(status == pdPASS);
        // lets create task-2
        status = xTaskCreate(
            vTask2_cmd_handling, "TASK2-CMD-HANDLING", 500, NULL, 2, &xTaskHandle2);
        configASSERT(status == pdPASS);
        // lets create task-3
        status = xTaskCreate(
            vTask3_cmd_processing, "TASK3-CMD-PROCESS", 500, NULL, 2, &xTaskHandle3);
        configASSERT(status == pdPASS);
        // lets create task-3
        status =
            xTaskCreate(vTask4_uart_write, "TASK4-UART-WRITE", 500, NULL, 2, &xTaskHandle4);
        configASSERT(status == pdPASS);
        // lets start the scheduler
        vTaskStartScheduler();
    }
    else
    {
        sprintf(usr_msg, "Queue creation failed\r\n");
        USART_SendText(USART3, usr_msg);
    }

    // char strTimeDate[20];

    while (1)
    {

        // RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
        // RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

        // sprintf(
        //     strTimeDate,
        //     "%2.2u:%2.2u:%2.2u\n",
        //     RTC_TimeStruct.RTC_Hours,
        //     RTC_TimeStruct.RTC_Minutes,
        //     RTC_TimeStruct.RTC_Seconds);

        // USART_SendText(USART3, "Time: ");
        // USART_SendText(USART3, strTimeDate);
        // // rtosDelayMs(1000);

        // sprintf(
        //     strTimeDate,
        //     "%2.2u:%2.2u:%2.2u\n",
        //     RTC_DateStruct.RTC_Year,
        //     RTC_DateStruct.RTC_Month,
        //     RTC_DateStruct.RTC_WeekDay);

        // USART_SendText(USART3, "Date: ");
        // USART_SendText(USART3, strTimeDate);
        // // rtosDelayMs(1000);
    }
    return 0;
}

// Task handler implementations
void vTask1_menu_display(void* params)
{

    char* pData = menu;

    while (1)
    {
        xQueueSend(uart_write_queue, &pData, portMAX_DELAY);

        // lets wait here until someone notifies.
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    }
}

void vTask2_cmd_handling(void* params)
{
    uint8_t command_code = 0;

    APP_CMD_t* new_cmd;

    while (1)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        // 1. send command to queue
        new_cmd = (APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));

        taskENTER_CRITICAL();
        command_code = getCommandCode(command_buffer);
        new_cmd->COMMAND_NUM = command_code;
        getArguments(new_cmd->COMMAND_ARGS);
        taskEXIT_CRITICAL();

        // send the command to the command queue
        xQueueSend(command_queue, &new_cmd, portMAX_DELAY);
    }
}

void vTask3_cmd_processing(void* params)
{
    APP_CMD_t* new_cmd;
    char task_msg[50];

    uint32_t toggle_duration = pdMS_TO_TICKS(500);

    while (1)
    {
        xQueueReceive(command_queue, (void*)&new_cmd, portMAX_DELAY);

        if (new_cmd->COMMAND_NUM == LED_ON_COMMAND)
        {
            make_led_on();
        }
        else if (new_cmd->COMMAND_NUM == LED_OFF_COMMAND)
        {
            make_led_off();
        }
        else if (new_cmd->COMMAND_NUM == LED_TOGGLE_COMMAND)
        {
            led_toggle_start(toggle_duration);
        }
        else if (new_cmd->COMMAND_NUM == LED_TOGGLE_STOP_COMMAND)
        {
            led_toggle_stop();
        }
        else if (new_cmd->COMMAND_NUM == LED_READ_STATUS_COMMAND)
        {
            read_led_status(task_msg);
        }
        else if (new_cmd->COMMAND_NUM == RTC_READ_DATE_TIME_COMMAND)
        {
            read_rtc_info(task_msg);
        }
        else
        {
            print_error_message(task_msg);
        }

        // lets free the allocated memory for the new command
        vPortFree(new_cmd);
    }
}

void vTask4_uart_write(void* params)
{
    char* pData = NULL;
    while (1)
    {

        xQueueReceive(uart_write_queue, &pData, portMAX_DELAY);
        USART_SendText(USART3, pData);
    }
}

static void Peripheral_Setup(void)
{
    RCC_Setup();
    GPIO_Setup();
    VirtualCOMPort_Setup();
    RTC_Setup();
}

static void RCC_Setup(void)
{
    /*Initialize GPIOB for toggling LEDs*/
    /*USART_2 (USART_B_RX: PD6 D52 on CN9, USART_B_TX: PD5 D53 on CN9) & USART_3
     * (USART_A_TX: PD8, USART_A_RX: PD9)*/
    /* Initiate clock for GPIOB and GPIOD*/
    /*Initialize GPIOA for PA3/ADC123_IN3 clock*/
    /*Initialize DMA2 clock*/
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD
            | RCC_AHB1Periph_DMA2,
        ENABLE);

    /*Initialize USART2 and USART3 clock*/
    /*Initialize TIM3 and TIM4 clock*/
    /*Initialize DAC clock*/
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM3
            | RCC_APB1Periph_TIM4 | RCC_APB1Periph_DAC,
        ENABLE);

    /*Initialize ADC1 clock*/
    /*Initialize SPI1 clock*/
    /*Initialize TIM1 clock*/
    // #ifdef DAC_PA4_PA5
    //     RCC_APB2PeriphClockCmd(
    //         RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1, ENABLE);
    // #else
    //     RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);
    // #endif

    /*Initialize RTC clock*/
    PWR_BackupAccessCmd(ENABLE);

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET)
        ;

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
}

static void GPIO_Setup(void)
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
    // traceISR_ENTER();
    /*Cleas the interrupt pending register (PR) bit*/
    EXTI_ClearITPendingBit(EXTI_Line13);
    // switchPriority = TRUE;
    // traceISR_EXIT();
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
    USART_InitStruct.USART_BaudRate = 9600;
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
    uint16_t data_byte;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        // a data byte is received from the user
        data_byte = USART_ReceiveData(USART3);
        command_buffer[command_len++] = (data_byte & 0xFF);

        if (data_byte == '\r')
        {
            // then user is finished entering the data

            // reset the command_len variable
            command_len = 0;

            // lets notify the command handling task
            xTaskNotifyFromISR(xTaskHandle2, 0, eNoAction, &xHigherPriorityTaskWoken);

            xTaskNotifyFromISR(xTaskHandle1, 0, eNoAction, &xHigherPriorityTaskWoken);
        }
    }

    // if the above freertos apis wake up any higher priority task, then yield the processor to
    // the
    // higher priority task which is just woken up.

    if (xHigherPriorityTaskWoken)
    {
        taskYIELD();
    }
}

uint8_t getCommandCode(uint8_t* buffer)
{
    if (buffer[0] == '0' || buffer[0] == '1' || buffer[0] == '2' || buffer[0] == '3'
        || buffer[0] == '4' || buffer[0] == '5' || buffer[0] == '6')
    {
        GPIO_ToggleBits(GPIOB, GPIO_Pin_7 | GPIO_Pin_14);
    }

    return buffer[0] - 48;
}

void getArguments(uint8_t* buffer)
{
    GPIO_ToggleBits(GPIOB, GPIO_Pin_7 | GPIO_Pin_14);
}

void make_led_on(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
}

void make_led_off(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
}

void led_toggle(TimerHandle_t xTimer)
{
    GPIO_ToggleBits(GPIOB, GPIO_Pin_0);
}

void led_toggle_start(uint32_t duration)
{

    if (led_timer_handle == NULL)
    {
        // 1. lets create the software timer
        led_timer_handle = xTimerCreate("LED-TIMER", duration, pdTRUE, NULL, led_toggle);

        // 2. start the software timer
        xTimerStart(led_timer_handle, portMAX_DELAY);
    }
    else
    {
        // start the software timer
        xTimerStart(led_timer_handle, portMAX_DELAY);
    }
}

void led_toggle_stop(void)
{
    xTimerStop(led_timer_handle, portMAX_DELAY);
}

void read_led_status(char* task_msg)
{
    sprintf(task_msg, "\r\nLED status is : %d\r\n", GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0));
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

void read_rtc_info(char* task_msg)
{
    // RTC_TimeTypeDef RTC_time;
    // RTC_DateTypeDef RTC_date;
    // read time and date from RTC peripheral of the microcontroller
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

    sprintf(
        task_msg,
        "\r\nTime: %02d:%02d:%02d \r\n Date : %02d-%2d-%2d \r\n",
        RTC_TimeStruct.RTC_Hours,
        RTC_TimeStruct.RTC_Minutes,
        RTC_TimeStruct.RTC_Seconds,
        RTC_DateStruct.RTC_Date,
        RTC_DateStruct.RTC_Month,
        RTC_DateStruct.RTC_Year);
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

static void RTC_Setup(void)
{
    RTC_InitTypeDef RTC_InitStruct;

    /* Initialize the RTC member */
    RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
    RTC_InitStruct.RTC_AsynchPrediv = (uint32_t)0x7F;
    RTC_InitStruct.RTC_SynchPrediv = (uint32_t)0xFF;

    while (RTC_Init(&RTC_InitStruct) != SUCCESS)
        ;

    /* Time = 00h:00min:00sec */
    RTC_TimeStruct.RTC_H12 = RTC_H12_AM;
    RTC_TimeStruct.RTC_Hours = 0;
    RTC_TimeStruct.RTC_Minutes = 0;
    RTC_TimeStruct.RTC_Seconds = 0;

    while (RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct) != SUCCESS)
        ;

    /* Monday, January 01 xx00 */
    RTC_DateStruct.RTC_WeekDay = RTC_Weekday_Monday;
    RTC_DateStruct.RTC_Date = 1;
    RTC_DateStruct.RTC_Month = RTC_Month_January;
    RTC_DateStruct.RTC_Year = 0;

    while (RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct) != SUCCESS)
        ;

    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
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

void print_error_message(char* task_msg)
{
    sprintf(task_msg, "\r\nInvalid command received\r\n");
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}
