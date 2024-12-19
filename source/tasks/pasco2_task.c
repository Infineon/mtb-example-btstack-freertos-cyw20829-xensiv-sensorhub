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
#include "cybsp.h"
#include "cyhal.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pasco2_task.h"
#include "xensiv_pasco2_mtb.h"
#include "task_handles.h"

#include "console_task.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define PIN_XENSIV_PASCO2_PWR_EN        CYBSP_A3

/* Wait time for sensor ready (milliseconds) */
#define WAIT_SENSOR_RDY_MS              (1000)

/* The CO2 concentration value acquired by the sensor depends on the external atmospheric pressure.
   To compensate for this effect, pressure values can be acquired from a pressure sensor such as an
   Infineon XENSIV&trade; DPS3xx. (https://github.com/Infineon/sensor-xensiv-dps3xx) */
#define DEFAULT_PRESSURE_REF_HPA        (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */

/******************************************************************************
* Globals
*******************************************************************************/
static xensiv_pasco2_t xensiv_pasco2;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t pasco2_initialize(cyhal_i2c_t *i2c_instance)
{
    cy_rslt_t result;

    /* Initialize XENSIV PAS CO2 power enable */
    result = cyhal_gpio_init(PIN_XENSIV_PASCO2_PWR_EN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("PASCO2 Power Enable initialization error\r\n");
        return result;
    }

    /* Dereference in i2c obj from i2c_sensor_task */
    cyhal_i2c_t *i2c_obj = i2c_instance;

    /* Wait for sensor to be ready */
    cyhal_system_delay_ms(WAIT_SENSOR_RDY_MS);

    /* Initialize PAS CO2 sensor with default parameter values */
    result = xensiv_pasco2_mtb_init_i2c(&xensiv_pasco2, i2c_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("PASCO2 device initialization error\r\n");
        return result;
    }

    return result;
}

void pasco2_sensor_task()
{
    cy_rslt_t result;
    wiced_bt_gatt_status_t gatt_status;

    /* Set exit_task to false initially */
    exit_task = false;

    uint16_t ppm;

    for (;;)
    {
        /* Read from PASC02 Sensor */
        result = xensiv_pasco2_mtb_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &ppm);
        if (CY_RSLT_SUCCESS == result)
        {
            printf("PASCO2 PPM: %d\r\n", ppm);

            memcpy(&app_xensiv_sensor_shield_pasco2[0], &ppm, sizeof(ppm));

            /* Send Notification to BLE App */
            gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_PASCO2_VALUE,
                                             app_xensiv_sensor_shield_pasco2,
                                             app_xensiv_sensor_shield_pasco2_len);
            printf("Sent notification status 0x%x\r\n\n", gatt_status);
        }

        cyhal_system_delay_ms(WAIT_SENSOR_RDY_MS);

        /* Exit task */
        if (exit_task)
        {
            exit_task = false;
            break;
        }
    }
}
