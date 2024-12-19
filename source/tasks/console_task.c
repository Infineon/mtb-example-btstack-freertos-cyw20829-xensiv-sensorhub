/******************************************************************************
 *
 * (c) (2022-2024), Infineon Technologies AG or an affiliate of Infineon Technologies AG.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Infineon
 * Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions. Therefore, you may
 * use this Software only as provided in the license agreement accompanying the software package from which you obtained
 * this Software ("EULA").
 *
 * If no EULA applies, Infineon hereby grants you a personal, non-exclusive, non-transferable license to copy, compile,
 * modify and use the Software source code solely for use in connection with Infineon's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Infineon.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT
 * LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Infineon
 * reserves the right to make changes to the Software without notice. Infineon does not assume any liability arising
 * out of the application or use of the Software or any product or circuit described in the Software. Infineon does
 * not authorize its products for use in any products where a malfunction or failure of the Infineon product may
 * reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including
 * Infineon's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such
 * use and in doing so agrees to indemnify Infineon against all liability.
 *
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "console_task.h"
#include "task_handles.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "cy_retarget_io.h"

#include "notification_types.h"
#include "cycfg_gatt_db.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool exit_task = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void print_menu();
sensor_notification_types_t convert_console_input_to_sensor_notification(uint8_t input);
sensor_notification_types_t convert_ble_message_to_sensor_notification(uint8_t input);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
void console_input_callback(void* handler_arg, cyhal_uart_event_t event)
{
    (void)handler_arg;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    /* For some reason this interrupt event is the only one that triggers */
    if ((event & CYHAL_UART_IRQ_RX_DONE) == CYHAL_UART_IRQ_RX_DONE)
    {
        exit_task = true;

        xTaskNotifyFromISR(console_task_handle, CONSOLE_INPUT, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void console_task_ble_app_callback()
{
    /* Exit current task */
    exit_task = true;

    /* Alert console task to enable sensor */
    xTaskNotify(console_task_handle, BLE_APP, eSetValueWithOverwrite);
}

void console_task(void *pvParameters)
{
    uint8_t console_rx_buffer[1];

    /* The UART callback handler registration */
    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, console_input_callback, NULL);

    /* Enable required UART events */
    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_DONE, 7U, true);

    /* Wait for data */
    cyhal_uart_read_async(&cy_retarget_io_uart_obj, console_rx_buffer, sizeof(console_rx_buffer));

    print_menu();

    uint32_t notification;
    sensor_notification_types_t sensor_to_activate = INVALID;



    for (;;)
    {
        /* Wait for control input */
        xTaskNotifyWait(pdFALSE,
                        0xFFFFFFFF,     /* Clear bits on Exit */
                        &notification,  /* Notification value set */
                        portMAX_DELAY);

        switch (notification)
        {
            case CONSOLE_INPUT:
                sensor_to_activate = convert_console_input_to_sensor_notification(console_rx_buffer[0]);

                /* Clear Rx Buffer */
                Cy_SCB_UART_ClearRxFifo(cy_retarget_io_uart_obj.base);

                /* Restart UART transfer */
                cyhal_uart_read_async(&cy_retarget_io_uart_obj, console_rx_buffer, sizeof(console_rx_buffer));
                break;

            case BLE_APP:
                printf("BLE app enabled a sensor.\r\n");

                sensor_to_activate = convert_ble_message_to_sensor_notification(app_xensiv_sensor_shield_sensorenable[0]);
                break;

            default:
                // Shouldn't get here
                break;
        }

        /* Add slight delay to wait for task previous task to exit */
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Notify Sensors Task so it knows which sensor to run */
        xTaskNotify(sensors_task_handle,
                    sensor_to_activate,
                    eSetValueWithOverwrite);
    }
}

void print_menu()
{
    /* Print out what commands enable what sensor? */
    printf("\n\nEnter number to enable sensor:\r\n");
    printf("1: BGT60LTR11\r\n");
    printf("2: SHT35\r\n");
    printf("3: BMI270\r\n");
    printf("4: BMM350\r\n");
    printf("5: DPS368\r\n");
    printf("6: PASC02\r\n");
}

sensor_notification_types_t convert_console_input_to_sensor_notification(uint8_t input)
{
    switch (input)
    {
        case '1': return BGT60LTR11;
        case '2': return SHT35;
        case '3': return BMI270;
        case '4': return BMM350;
        case '5': return DPS368;
        case '6': return PASCO2;
        default:
            print_menu();
            return INVALID;
    }
}

sensor_notification_types_t convert_ble_message_to_sensor_notification(uint8_t input)
{
    sensor_notification_types_t sensor = (sensor_notification_types_t) input;

    switch (sensor)
    {
        case BGT60LTR11:
        case SHT35:
        case BMI270:
        case BMM350:
        case DPS368:
        case PASCO2:
            return sensor;

        default:
            return INVALID;
    }
}
