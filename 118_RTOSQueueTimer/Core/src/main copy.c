#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/*Global variables section*/
RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;

/*Macros section*/
#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)

#define TRUE  1
#define FALSE 0

TaskHandle_t handle_cmd_task = NULL;
TaskHandle_t handle_menu_task = NULL;
TaskHandle_t handle_print_task = NULL;
TaskHandle_t handle_led_task = NULL;
TaskHandle_t handle_rtc_task = NULL;

QueueHandle_t q_data;
QueueHandle_t q_print;

// software timer handles
TimerHandle_t handle_led_timer[4];
TimerHandle_t rtc_timer;

volatile uint8_t user_data;

// state variable
state_t curr_state = sMainMenu;

void led_effect_callback(TimerHandle_t xTimer);
void rtc_report_callback(TimerHandle_t xTimer);

// /*Tasks function prototypes*/
// void LED_100ms_task_Function(void* funcParams);
// void LED_1s_task_Function(void* funcParams);

/*Private setup for hardware peripherals*/
static void Peripheral_Setup(void);
static void RCC_Setup(void);
static void GPIO_Setup(void);
static void VirtualCOMPort_Setup(void);
static void RTC_Setup(void);

void rtosDelayMs(uint32_t ms);
void USART_SendText(USART_TypeDef* USARTx, volatile char* sendText);
void USART_SendNumber(USART_TypeDef* USARTx, uint32_t sendNumber);

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

    BaseType_t status;

    char strTimeDate[20];

    // Start Recording
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

    /*Create LED_task for RTOS*/
    status = xTaskCreate(menu_task, "menu_task", 250, NULL, 2, &handle_menu_task);
    configASSERT(status == pdPASS);

    status = xTaskCreate(cmd_handler_task, "cmd_task", 250, NULL, 2, &handle_cmd_task);
    configASSERT(status == pdPASS);

    status = xTaskCreate(print_task, "print_task", 250, NULL, 2, &handle_print_task);

    configASSERT(status == pdPASS);

    status = xTaskCreate(led_task, "led_task", 250, NULL, 2, &handle_led_task);

    configASSERT(status == pdPASS);

    status = xTaskCreate(rtc_task, "rtc_task", 250, NULL, 2, &handle_rtc_task);

    configASSERT(status == pdPASS);

    q_data = xQueueCreate(10, sizeof(char));

    configASSERT(q_data != NULL);

    q_print = xQueueCreate(10, sizeof(size_t));

    configASSERT(q_print != NULL);

    // Create software timers for LED effects
    for (int i = 0; i < 4; i++)
        handle_led_timer[i] = xTimerCreate(
            "led_timer", pdMS_TO_TICKS(500), pdTRUE, (void*)(i + 1), led_effect_callback);

    rtc_timer = xTimerCreate(
        "rtc_report_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, rtc_report_callback);

    // HAL_UART_Receive_IT(&huart2, (uint8_t*)&user_data, 1);

    /*Start the scheduler*/
    vTaskStartScheduler();

    /*By addig scheduler the below code will never be executed
    since above scheduler always runs, resulting in functions
    runs where functions will never return*/
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
        // rtosDelayMs(1000);

        // sprintf(
        //     strTimeDate,
        //     "%2.2u:%2.2u:%2.2u\n",
        //     RTC_DateStruct.RTC_Year,
        //     RTC_DateStruct.RTC_Month,
        //     RTC_DateStruct.RTC_WeekDay);

        // USART_SendText(USART3, "Date: ");
        // USART_SendText(USART3, strTimeDate);
        // rtosDelayMs(1000);
    }
    return 0;
}

void rtc_report_callback(TimerHandle_t xTimer)
{
    show_time_date_itm();
}

void led_effect_callback(TimerHandle_t xTimer)
{
    int id;
    id = (uint32_t)pvTimerGetTimerID(xTimer);

    switch (id)
    {
    case 1:
        LED_effect1();
        break;
    case 2:
        LED_effect2();
        break;
    case 3:
        LED_effect3();
        break;
    case 4:
        LED_effect4();
    }
}

/* This function called from UART interrupt handler , hence executes in interrupt context */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    uint8_t dummy;

    for (uint32_t i = 0; i < 4000; i++)
        ;

    if (!xQueueIsQueueFullFromISR(q_data))
    {
        /*Enqueue data byte */
        xQueueSendFromISR(q_data, (void*)&user_data, NULL);
    }
    else
    {
        if (user_data == '\n')
        {
            /*Make sure that last data byte of the queue is '\n' */
            xQueueReceiveFromISR(q_data, (void*)&dummy, NULL);
            xQueueSendFromISR(q_data, (void*)&user_data, NULL);
        }
    }

    /*Send notification to command handling task if user_data = '\n' */
    if (user_data == '\n')
    {
        /*send notification to command handling task */
        xTaskNotifyFromISR(handle_cmd_task, 0, eNoAction, NULL);
    }

    /* Enable UART data byte reception again in IT mode */
    // HAL_UART_Receive_IT(&huart2, (uint8_t*)&user_data, 1);
}

static void Peripheral_Setup(void)
{
    RCC_Setup();
    GPIO_Setup();
    VirtualCOMPort_Setup();
    RTC_Setup();
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

    /*Initialize RTC clock*/
    PWR_BackupAccessCmd(ENABLE);

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET)
        ;

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
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

void led_effect_stop(void)
{
    for (int i = 0; i < 4; i++)
        xTimerStop(handle_led_timer[i], portMAX_DELAY);
}

void led_effect(int n)
{
    led_effect_stop();
    xTimerStart(handle_led_timer[n - 1], portMAX_DELAY);
}

void turn_off_all_leds(void)
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
}

void turn_on_all_leds(void)
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
}

void turn_on_odd_leds(void)
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LED1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LED2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LED3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD6_GPIO_Port, LED4, GPIO_PIN_RESET);
}

void turn_on_even_leds(void)
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LED1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LED2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LED3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD6_GPIO_Port, LED4, GPIO_PIN_SET);
}

void LED_control(int value)
{
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(LD3_GPIO_Port, (LED1 << i), ((value >> i) & 0x1));
}

void LED_effect1(void)
{
    static int flag = 1;
    (flag ^= 1) ? turn_off_all_leds() : turn_on_all_leds();
}

void LED_effect2(void)
{
    static int flag = 1;
    (flag ^= 1) ? turn_on_even_leds() : turn_on_odd_leds();
}

void LED_effect3(void)
{
    static int i = 0;
    LED_control(0x1 << (i++ % 4));
}

void LED_effect4(void)
{
    static int i = 0;
    LED_control(0x08 >> (i++ % 4));
}

void show_time_date_itm(void)
{
    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    memset(&rtc_date, 0, sizeof(rtc_date));
    memset(&rtc_time, 0, sizeof(rtc_time));

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

    char* format;
    format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

    printf("%02d:%02d:%02d [%s]", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, format);
    printf("\t%02d-%02d-%2d\n", rtc_date.Month, rtc_date.Date, 2000 + rtc_date.Year);
}

void show_time_date(void)
{
    static char showtime[40];
    static char showdate[40];

    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    static char* time = showtime;
    static char* date = showdate;

    memset(&rtc_date, 0, sizeof(rtc_date));
    memset(&rtc_time, 0, sizeof(rtc_time));

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

    char* format;
    format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

    /* Display time Format : hh:mm:ss [AM/PM] */
    sprintf(
        (char*)showtime,
        "%s:\t%02d:%02d:%02d [%s]",
        "\nCurrent Time&Date",
        rtc_time.Hours,
        rtc_time.Minutes,
        rtc_time.Seconds,
        format);
    xQueueSend(q_print, &time, portMAX_DELAY);

    /* Display date Format : date-month-year */
    sprintf(
        (char*)showdate,
        "\t%02d-%02d-%2d\n",
        rtc_date.Month,
        rtc_date.Date,
        2000 + rtc_date.Year);
    xQueueSend(q_print, &date, portMAX_DELAY);
}

void rtc_configure_time(RTC_TimeTypeDef* time)
{

    time->TimeFormat = RTC_HOURFORMAT12_AM;
    time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time->StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
}

void rtc_configure_date(RTC_DateTypeDef* date)
{
    HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
}

int validate_rtc_information(RTC_TimeTypeDef* time, RTC_DateTypeDef* date)
{
    if (time)
    {
        if ((time->Hours > 12) || (time->Minutes > 59) || (time->Seconds > 59))
            return 1;
    }

    if (date)
    {
        if ((date->Date > 31) || (date->WeekDay > 7) || (date->Year > 99)
            || (date->Month > 12))
            return 1;
    }

    return 0;
}
