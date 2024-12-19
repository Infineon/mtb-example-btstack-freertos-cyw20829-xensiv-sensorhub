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
#include <stdio.h>
#include "cybsp.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bmm350_task.h"
#include "mtb_bmm350.h"
#include "task_handles.h"

#include "console_task.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define BMM350_MAG_TEMP_X_INDEX            (0)
#define BMM350_MAG_TEMP_Y_INDEX            (1)
#define BMM350_MAG_TEMP_Z_INDEX            (2)
#define BMM350_MAG_TEMP_TEMP_INDEX         (3)

#define DELAY_MS                         (1000U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Structure holding bmm350 variables */
static mtb_bmm350_t bmm350_obj;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t bmm350_initialize(cyhal_i2c_t *i2c_instance)
{
    cy_rslt_t result;

    /* Initialize BMM350 Driver */
    result = mtb_bmm350_init_i2c(&bmm350_obj, i2c_instance, MTB_BMM350_ADDRESS_DEFAULT);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing BMM350 driver\r\n");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

void bmm350_sensor_task()
{
    cy_rslt_t result;
    wiced_bt_gatt_status_t gatt_status;

    /* BMM350 Structures */
    mtb_bmm350_data_t data;

    /* Set exit_task to false initially */
    exit_task = false;

    for (;;)
    {
        result = mtb_bmm350_read(&bmm350_obj, &data);
        if (CY_RSLT_SUCCESS == result)
        {
            int abs_x, abs_y, abs_z, abs_temp;

            abs_x = abs((int) data.sensor_data.x);
            abs_y = abs((int) data.sensor_data.y);
            abs_z = abs((int) data.sensor_data.z);
            abs_temp = abs((int) data.sensor_data.temperature);

            printf("x: %d, y: %d, z: %d, temperature: %d\r\n", abs_x,
                                                                               abs_y,
                                                                               abs_z,
                                                                               abs_temp);

            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_X_INDEX] = (uint8_t) abs_x;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_Y_INDEX] = (uint8_t) abs_y;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_Z_INDEX] = (uint8_t) abs_z;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_TEMP_INDEX] = (uint8_t) abs_temp;

            /* Send Notification to BLE App */
            gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_BMM350_VALUE,
                                             app_xensiv_sensor_shield_bmm350,
                                             app_xensiv_sensor_shield_bmm350_len);
            printf("Sent notification status 0x%x\r\n\n", gatt_status);

            vTaskDelay(DELAY_MS);
        }

        /* Exit task if flag is set */
        if (exit_task)
        {
            exit_task = false;
            break;
        }
    }
}
