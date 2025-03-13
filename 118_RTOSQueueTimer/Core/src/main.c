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

// /*Global variables*/
RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void menu_task(void* param);
void led_task(void* param);
void rtc_task(void* param);
void print_task(void* param);
void cmd_handler_task(void* param);

void led_effect_stop(void);
void led_effect(int n);

void LED_effect1(void);
void LED_effect2(void);
void LED_effect3(void);
void LED_effect4(void);

void show_time_date(void);
void show_time_date_itm(void);
// void rtc_configure_time(RTC_TimeTypeDef* time);
// void rtc_configure_date(RTC_DateTypeDef* date);
int validate_rtc_information(RTC_TimeTypeDef* time, RTC_DateTypeDef* date);

// Software timer callback function prototype
void led_toggle(TimerHandle_t xTimer);

// global space for some variable
char usr_msg[250] = {0};

// task handles
TaskHandle_t handle_cmd_task = NULL;
TaskHandle_t handle_menu_task = NULL;
TaskHandle_t handle_print_task = NULL;
TaskHandle_t handle_led_task = NULL;
TaskHandle_t handle_rtc_task = NULL;

// Queue handle
QueueHandle_t q_data;
QueueHandle_t q_print;

// software timer handles
TimerHandle_t handle_led_timer[4];
TimerHandle_t rtc_timer;

volatile uint8_t user_data;

typedef struct
{
    uint8_t payload[10];
    uint32_t len;
} command_t;

// state variable
typedef enum
{
    sMainMenu = 0,
    sLedEffect,
    sRtcMenu,
    sRtcTimeConfig,
    sRtcDateConfig,
    sRtcReport,
} state_t;

state_t curr_state = sMainMenu;

void led_effect_callback(TimerHandle_t xTimer);
void rtc_report_callback(TimerHandle_t xTimer);

int extract_command(command_t* cmd);
void process_command(command_t* cmd);

const char* msg_inv = "////Invalid option////\n";

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

    sprintf(usr_msg, "\r\nThis is Queue Command Processing Demo\r\n");
    USART_SendText(USART3, usr_msg);

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

    USART_SendData(USART3, (uint8_t)user_data); // USART_SendText(USART3, user_data);

    vTaskStartScheduler();

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

void menu_task(void* param)
{
    uint32_t cmd_addr;

    command_t* cmd;

    int option;

    const char* msg_menu =
        "\n========================\n"
        "|         Menu         |\n"
        "========================\n"
        "LED effect    ----> 0\n"
        "Date and time ----> 1\n"
        "Exit          ----> 2\n"
        "Enter your choice here : ";

    while (1)
    {
        xQueueSend(q_print, &msg_menu, portMAX_DELAY);

        // wait for menu commands
        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

        if (cmd->len == 1)
        {
            option = cmd->payload[0] - 48;
            switch (option)
            {
            case 0:
                curr_state = sLedEffect;
                xTaskNotify(handle_led_task, 0, eNoAction);
                break;
            case 1:
                curr_state = sRtcMenu;
                xTaskNotify(handle_rtc_task, 0, eNoAction);
                break;
            case 2: /*implement exit */
                break;
            default:
                xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                continue;
            }
        }
        else
        {
            // invalid entry
            xQueueSend(q_print, &msg_inv, portMAX_DELAY);
            continue;
        }

        // wait to run again when some other task notifies
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    } // while super loop
}

void led_task(void* param)
{
    uint32_t cmd_addr;
    command_t* cmd;
    const char* msg_led =
        "========================\n"
        "|      LED Effect     |\n"
        "========================\n"
        "(none,e1,e2,e3,e4)\n"
        "Enter your choice here : ";

    while (1)
    {
        /*Wait for notification (Notify wait) */
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        /*Print LED menu */
        xQueueSend(q_print, &msg_led, portMAX_DELAY);

        /*wait for LED command (Notify wait) */
        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

        if (cmd->len <= 4)
        {
            if (!strcmp((char*)cmd->payload, "none"))
                led_effect_stop();
            else if (!strcmp((char*)cmd->payload, "e1"))
                led_effect(1);
            else if (!strcmp((char*)cmd->payload, "e2"))
                led_effect(2);
            else if (!strcmp((char*)cmd->payload, "e3"))
                led_effect(3);
            else if (!strcmp((char*)cmd->payload, "e4"))
                led_effect(4);
            else
                xQueueSend(q_print, &msg_inv, portMAX_DELAY); /*print invalid message */
        }
        else
            xQueueSend(q_print, &msg_inv, portMAX_DELAY);

        /* update state variable */
        curr_state = sMainMenu;

        /*Notify menu task */
        xTaskNotify(handle_menu_task, 0, eNoAction);
    }
}

uint8_t getnumber(uint8_t* p, int len)
{

    int value;

    if (len > 1)
        value = (((p[0] - 48) * 10) + (p[1] - 48));
    else
        value = p[0] - 48;

    return value;
}

void rtc_task(void* param)
{
    const char* msg_rtc1 =
        "========================\n"
        "|         RTC          |\n"
        "========================\n";

    const char* msg_rtc2 =
        "Configure Time            ----> 0\n"
        "Configure Date            ----> 1\n"
        "Enable reporting          ----> 2\n"
        "Exit                      ----> 3\n"
        "Enter your choice here : ";

    const char* msg_rtc_hh = "Enter hour(1-12):";
    const char* msg_rtc_mm = "Enter minutes(0-59):";
    const char* msg_rtc_ss = "Enter seconds(0-59):";

    const char* msg_rtc_dd = "Enter date(1-31):";
    const char* msg_rtc_mo = "Enter month(1-12):";
    const char* msg_rtc_dow = "Enter day(1-7 sun:1):";
    const char* msg_rtc_yr = "Enter year(0-99):";

    const char* msg_conf = "Configuration successful\n";
    const char* msg_rtc_report = "Enable time&date reporting(y/n)?: ";

    uint32_t cmd_addr;
    command_t* cmd;

    static int rtc_state = 0;
    int menu_code;

    // RTC_TimeTypeDef time;
    // RTC_DateTypeDef date;

#define HH_CONFIG 0
#define MM_CONFIG 1
#define SS_CONFIG 2

#define DATE_CONFIG  0
#define MONTH_CONFIG 1
#define YEAR_CONFIG  2
#define DAY_CONFIG   3

    while (1)
    {
        /*Notify wait (wait till someone notifies) */
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        /*Print the menu and show current date and time information */
        xQueueSend(q_print, &msg_rtc1, portMAX_DELAY);
        show_time_date();
        xQueueSend(q_print, &msg_rtc2, portMAX_DELAY);

        while (curr_state != sMainMenu)
        {

            /*Wait for command notification (Notify wait) */
            xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
            cmd = (command_t*)cmd_addr;

            switch (curr_state)
            {
            case sRtcMenu:
            {
                /*process RTC menu commands */
                if (cmd->len == 1)
                {
                    menu_code = cmd->payload[0] - 48;
                    switch (menu_code)
                    {
                    case 0:
                        curr_state = sRtcTimeConfig;
                        xQueueSend(q_print, &msg_rtc_hh, portMAX_DELAY);
                        break;
                    case 1:
                        curr_state = sRtcDateConfig;
                        xQueueSend(q_print, &msg_rtc_dd, portMAX_DELAY);
                        break;
                    case 2:
                        curr_state = sRtcReport;
                        xQueueSend(q_print, &msg_rtc_report, portMAX_DELAY);
                        break;
                    case 3:
                        curr_state = sMainMenu;
                        break;
                    default:
                        curr_state = sMainMenu;
                        xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                    }
                }
                else
                {
                    curr_state = sMainMenu;
                    xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                }
                break;
            }

            case sRtcTimeConfig:
            {
                /*get hh, mm, ss infor and configure RTC */
                /*take care of invalid entries */
                switch (rtc_state)
                {
                case HH_CONFIG:
                {
                    uint8_t hour = getnumber(cmd->payload, cmd->len);
                    RTC_TimeStruct.RTC_Hours = hour;
                    rtc_state = MM_CONFIG;
                    xQueueSend(q_print, &msg_rtc_mm, portMAX_DELAY);
                    break;
                }
                case MM_CONFIG:
                {
                    uint8_t min = getnumber(cmd->payload, cmd->len);
                    RTC_TimeStruct.RTC_Minutes = min;
                    rtc_state = SS_CONFIG;
                    xQueueSend(q_print, &msg_rtc_ss, portMAX_DELAY);
                    break;
                }
                case SS_CONFIG:
                {
                    uint8_t sec = getnumber(cmd->payload, cmd->len);
                    RTC_TimeStruct.RTC_Seconds = sec;
                    if (!validate_rtc_information(&RTC_TimeStruct, NULL))
                    {
                        // rtc_configure_time(&RTC_TimeStruct);
                        xQueueSend(q_print, &msg_conf, portMAX_DELAY);
                        show_time_date();
                    }
                    else
                        xQueueSend(q_print, &msg_inv, portMAX_DELAY);

                    curr_state = sMainMenu;
                    rtc_state = 0;
                    break;
                }
                }

                break;
            }

            case sRtcDateConfig:
            {

                /*get date, month, day , year info and configure RTC */

                /*take care of invalid entries */
                switch (rtc_state)
                {
                case DATE_CONFIG:
                {
                    uint8_t d = getnumber(cmd->payload, cmd->len);
                    RTC_DateStruct.RTC_Date = d;
                    rtc_state = MONTH_CONFIG;
                    xQueueSend(q_print, &msg_rtc_mo, portMAX_DELAY);
                    break;
                }
                case MONTH_CONFIG:
                {
                    uint8_t month = getnumber(cmd->payload, cmd->len);
                    RTC_DateStruct.RTC_Month = month;
                    rtc_state = DAY_CONFIG;
                    xQueueSend(q_print, &msg_rtc_dow, portMAX_DELAY);
                    break;
                }
                case DAY_CONFIG:
                {
                    uint8_t day = getnumber(cmd->payload, cmd->len);
                    RTC_DateStruct.RTC_WeekDay = day;
                    rtc_state = YEAR_CONFIG;
                    xQueueSend(q_print, &msg_rtc_yr, portMAX_DELAY);
                    break;
                }
                case YEAR_CONFIG:
                {
                    uint8_t year = getnumber(cmd->payload, cmd->len);
                    RTC_DateStruct.RTC_Year = year;

                    if (!validate_rtc_information(NULL, &RTC_DateStruct))
                    {
                        // rtc_configure_date(&RTC_DateStruct);
                        xQueueSend(q_print, &msg_conf, portMAX_DELAY);
                        show_time_date();
                    }
                    else
                        xQueueSend(q_print, &msg_inv, portMAX_DELAY);

                    curr_state = sMainMenu;
                    rtc_state = 0;
                    break;
                }
                }

                break;
            }

            case sRtcReport:
            {
                /*enable or disable RTC current time reporting over ITM printf */
                if (cmd->len == 1)
                {
                    if (cmd->payload[0] == 'y')
                    {
                        if (xTimerIsTimerActive(rtc_timer) == pdFALSE)
                            xTimerStart(rtc_timer, portMAX_DELAY);
                    }
                    else if (cmd->payload[0] == 'n')
                    {
                        xTimerStop(rtc_timer, portMAX_DELAY);
                    }
                    else
                    {
                        xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                    }
                }
                else
                    xQueueSend(q_print, &msg_inv, portMAX_DELAY);

                curr_state = sMainMenu;
                break;
            }

            } // switch end

        } // while end

        /*Notify menu task */
        xTaskNotify(handle_menu_task, 0, eNoAction);

    } // while super loop end
}

void print_task(void* param)
{

    uint32_t* msg;
    while (1)
    {
        xQueueReceive(q_print, &msg, portMAX_DELAY);
        USART_SendText(USART3, (uint8_t*)msg); // HAL_UART_Transmit(&huart2, (uint8_t*)msg,
                                               // strlen((char*)msg), HAL_MAX_DELAY);
    }
}

void cmd_handler_task(void* param)
{
    BaseType_t ret;
    command_t cmd;

    while (1)
    {
        /*Implement notify wait */
        ret = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        if (ret == pdTRUE)
        {
            /*process the user data(command) stored in input data queue */
            process_command(&cmd);
        }
    }
}

void process_command(command_t* cmd)
{
    extract_command(cmd);

    switch (curr_state)
    {
    case sMainMenu:
        xTaskNotify(handle_menu_task, (uint32_t)cmd, eSetValueWithOverwrite);
        break;

    case sLedEffect:
        xTaskNotify(handle_led_task, (uint32_t)cmd, eSetValueWithOverwrite);
        break;

    case sRtcMenu:
    case sRtcTimeConfig:
    case sRtcDateConfig:
    case sRtcReport:
        xTaskNotify(handle_rtc_task, (uint32_t)cmd, eSetValueWithOverwrite);
        break;
    }
}

int extract_command(command_t* cmd)
{
    uint8_t item;
    BaseType_t status;

    status = uxQueueMessagesWaiting(q_data);
    if (!status)
        return -1;
    uint8_t i = 0;

    do
    {
        status = xQueueReceive(q_data, &item, 0);
        if (status == pdTRUE)
            cmd->payload[i++] = item;
    } while (item != '\n');

    cmd->payload[i - 1] = '\0';
    cmd->len = i - 1; /*save  length of the command excluding null char */

    return 0;
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
    USART_SendData(
        USART3, (uint8_t)user_data); // HAL_UART_Receive_IT(&huart2, (uint8_t*)&user_data, 1);
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
////////////////////////////////////////////////////////////
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
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
}

void turn_on_all_leds(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
}

void turn_on_odd_leds(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
}

void turn_on_even_leds(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
}

void LED_control(int value)
{
    for (int i = 0; i < 4; i++)
        GPIO_WriteBit(GPIOB, (GPIO_Pin_7 << i), ((value >> i) & 0x1));
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
///////////////////////////////////////////////////////////////
void show_time_date_itm(void)
{
    // RTC_TimeTypeDef RTC_time;
    // RTC_DateTypeDef RTC_date;

    memset(&RTC_TimeStruct, 0, sizeof(RTC_TimeStruct));
    memset(&RTC_DateStruct, 0, sizeof(RTC_DateStruct));

    // read time and date from RTC peripheral of the microcontroller
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

    char* format;
    format = (RTC_TimeStruct.RTC_H12 == RTC_H12_AM) ? "AM" : "PM";

    printf(
        "%02d:%02d:%02d [%s]",
        RTC_TimeStruct.RTC_Hours,
        RTC_TimeStruct.RTC_Minutes,
        RTC_TimeStruct.RTC_Seconds,
        format);
    printf(
        "\t%02d-%02d-%2d\n",
        RTC_DateStruct.RTC_Month,
        RTC_DateStruct.RTC_Date,
        2000 + RTC_DateStruct.RTC_Year);

    // sprintf(
    //     task_msg,
    //     "\r\nTime: %02d:%02d:%02d \r\n Date : %02d-%2d-%2d \r\n",
    //     RTC_TimeStruct.RTC_Hours,
    //     RTC_TimeStruct.RTC_Minutes,
    //     RTC_TimeStruct.RTC_Seconds,
    //     RTC_DateStruct.RTC_Date,
    //     RTC_DateStruct.RTC_Month,
    //     RTC_DateStruct.RTC_Year);
    // xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

void show_time_date(void)
{
    static char showtime[40];
    static char showdate[40];

    // RTC_DateTypeDef rtc_date;
    // RTC_TimeTypeDef rtc_time;

    static char* time = showtime;
    static char* date = showdate;

    memset(&RTC_DateStruct, 0, sizeof(RTC_DateStruct));
    memset(&RTC_TimeStruct, 0, sizeof(RTC_TimeStruct));

    /* Get the RTC current Time */
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    /* Get the RTC current Date */
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

    char* format;
    format = (RTC_TimeStruct.RTC_H12 == RTC_H12_AM) ? "AM" : "PM";

    /* Display time Format : hh:mm:ss [AM/PM] */
    sprintf(
        (char*)showtime,
        "%s:\t%02d:%02d:%02d [%s]",
        "\nCurrent Time&Date",
        RTC_TimeStruct.RTC_Hours,
        RTC_TimeStruct.RTC_Minutes,
        RTC_TimeStruct.RTC_Seconds,
        format);
    xQueueSend(q_print, &time, portMAX_DELAY);

    /* Display date Format : date-month-year */
    sprintf(
        (char*)showdate,
        "\t%02d-%02d-%2d\n",
        RTC_DateStruct.RTC_Month,
        RTC_DateStruct.RTC_Date,
        2000 + RTC_DateStruct.RTC_Year);
    xQueueSend(q_print, &date, portMAX_DELAY);
}

int validate_rtc_information(RTC_TimeTypeDef* time, RTC_DateTypeDef* date)
{
    if (time)
    {
        if ((time->RTC_Hours > 12) || (time->RTC_Minutes > 59) || (time->RTC_Seconds > 59))
            return 1;
    }

    if (date)
    {
        if ((date->RTC_Date > 31) || (date->RTC_WeekDay > 7) || (date->RTC_Year > 99)
            || (date->RTC_Month > 12))
            return 1;
    }

    return 0;
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
/*
void print_error_message(char* task_msg)
{
    sprintf(task_msg, "\r\nInvalid command received\r\n");
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}
*/
