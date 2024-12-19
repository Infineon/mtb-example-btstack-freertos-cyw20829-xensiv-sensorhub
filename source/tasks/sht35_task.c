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
#include <stdio.h>
#include "cybsp.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "sht35_task.h"
#include "mtb_sht3x.h"
#include "task_handles.h"

#include "console_task.h"
#include "util.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_MS                         (1000U)

#define TEMPERATURE_START_INDEX          (0)
#define RELATIVE_HUMIDITY_START_INDEX    (4)

/*******************************************************************************
 *        Variable Definitions
*******************************************************************************/
static cyhal_i2c_t *i2c_obj;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t sht35_initialize(cyhal_i2c_t *i2c_instance)
{
    cy_rslt_t result;

    /* Assign pointer to global i2c object */
    i2c_obj = i2c_instance;

    /* Initialize SHT35 */
    result = mtb_sht3x_init(i2c_obj, MTB_SHT35_ADDRESS_DEFAULT);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing SHT35 sensor\r\n");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

void sht35_sensor_task()
{
    cy_rslt_t result;
    wiced_bt_gatt_status_t gatt_status;

    struct sensor_data_t sht3x_data;
    float temperature_f, humidity_f;
    uint32_t decimal, fraction;

    /* Start periodic measurement */
    result = mtb_sht3x_start_periodic_measurement(i2c_obj,
                                                  REPEAT_MEDIUM,
                                                  MPS_ONE_PER_SEC);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error starting SHT35 periodic measurement\r\n");
        return;
    }

    /* Set exit_task to false initially */
    exit_task = false;

    for (;;)
    {
        sht3x_data = mtb_sht3x_read(i2c_obj);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("Error reading from SHT35 sensor\r\n");
            continue;
        }

        convert_float((float) sht3x_data.temperature, &decimal, &fraction);
        printf("Temperature: %" PRIu32 ".%02" PRIu32 " degC\r\n", decimal, fraction);

        convert_float((float) sht3x_data.humidity, &decimal, &fraction);
        printf("Relative Humidity: %" PRIu32 ".%02" PRIu32 " %%\r\n", decimal, fraction);

        temperature_f = (float) sht3x_data.temperature;
        humidity_f    = (float) sht3x_data.humidity;

        memcpy(&app_xensiv_sensor_shield_sht35[TEMPERATURE_START_INDEX], &temperature_f, sizeof(temperature_f));
        memcpy(&app_xensiv_sensor_shield_sht35[RELATIVE_HUMIDITY_START_INDEX], &humidity_f, sizeof(humidity_f));

        /* Send Notification to BLE App */
        gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_SHT35_VALUE,
                                         app_xensiv_sensor_shield_sht35,
                                         app_xensiv_sensor_shield_sht35_len);
        printf("Sent notification status 0x%x\r\n\n", gatt_status);

        cyhal_system_delay_ms(DELAY_MS);

        /* Exit task if flag is set */
        if (exit_task)
        {
            /* Stop measurement */
            result = mtb_sht3x_stop_measurement(i2c_obj);
            if (CY_RSLT_SUCCESS != result)
            {
                printf("Error stopping SHT35 periodic measurement\r\n");
            }

            exit_task = false;
            break;
        }
    }
}
