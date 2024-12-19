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
#include <inttypes.h>
#include "cybsp.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "dps368_task.h"
#include "xensiv_dps3xx_mtb.h"

#include "console_task.h"
#include "util.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_MS                        (1000U)

#define PRESSURE_START_INDEX            (0)
#define TEMPERATURE_START_INDEX         (4)

/******************************************************************************
* Types
*******************************************************************************/
static xensiv_dps3xx_t dps368_obj;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t dps368_initialize(cyhal_i2c_t *i2c_instance)
{
    cy_rslt_t result;

    /* Dereference in i2c obj from i2c_sensor_task */
    cyhal_i2c_t *i2c_obj = i2c_instance;

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&dps368_obj, i2c_obj, XENSIV_DPS3XX_I2C_ADDR_ALT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Pressure sensor initialization failed!\r\n");
        return result;
    }

    return result;
}

void dps368_sensor_task()
{
    cy_rslt_t result;
    wiced_bt_gatt_status_t gatt_status;

    bool pressure_ready = false;
    bool temperature_ready = false;

    float pressure = 0.0f;
    float temperature = 0.0f;
    uint32_t decimal, fraction;

    /* Set exit_task to false initially */
    exit_task = false;

    for (;;)
    {
        result = xensiv_dps3xx_check_ready(&dps368_obj, &pressure_ready, &temperature_ready);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("Error checking if DPS368 is ready\r\n");
            cyhal_system_delay_ms(DELAY_MS);
            continue;
        }

        if (!pressure_ready && !temperature_ready)
        {
            /* Data not ready, re-check ready */
            cyhal_system_delay_ms(DELAY_MS);
            continue;
        }

        result = xensiv_dps3xx_read(&dps368_obj, &pressure, &temperature);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("Error reading from DPS368\r\n");
            cyhal_system_delay_ms(DELAY_MS);
            continue;
        }

        convert_float(pressure, &decimal, &fraction);
        printf("Pressure: %" PRIu32 ".%02" PRIu32 " hPa\r\n", decimal, fraction);

        convert_float(temperature, &decimal, &fraction);
        printf("Temperature: %" PRIu32 ".%02" PRIu32 " degC\r\n", decimal, fraction);

        memcpy(&app_xensiv_sensor_shield_dps368[PRESSURE_START_INDEX], &pressure, sizeof(pressure));
        memcpy(&app_xensiv_sensor_shield_dps368[TEMPERATURE_START_INDEX], &temperature, sizeof(temperature));

        /* Send Notification to BLE App */
        gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_DPS368_VALUE,
                                         app_xensiv_sensor_shield_dps368,
                                         app_xensiv_sensor_shield_dps368_len);
        printf("Sent notification status 0x%x\r\n\n", gatt_status);

        cyhal_system_delay_ms(DELAY_MS);

        /* Exit task if flag is set */
        if (exit_task)
        {
            exit_task = false;
            break;
        }
    }
}
