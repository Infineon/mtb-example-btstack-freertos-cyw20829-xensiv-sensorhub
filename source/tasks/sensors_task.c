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
#include <inttypes.h>
#include "cybsp.h"
#include "i2c.h"
#include "stdio.h"

#include "sensors_task.h"
#include "notification_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_handles.h"
#include "bgt60ltr11_task.h"
#include "sht35_task.h"
#include "bmi270_task.h"
#include "bmm350_task.h"
#include "dps368_task.h"
#include "pasco2_task.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define RADAR_SEL                       (CYBSP_D6)

/*******************************************************************************
* Global Variables
*******************************************************************************/
cyhal_i2c_t i2c_obj;

sensor_status_t sensor_status;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void sensor_status_init();
void sensor_status_set_status(sensor_notification_types_t sensor, bool status);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t sensors_initialize()
{
    cy_rslt_t result;

    /* Initialize sensor status flags */
    sensor_status_init();

    i2c_pins_t i2c_pins =
    {
        .sda = CYBSP_I2C_SDA,
        .scl = CYBSP_I2C_SCL
    };

    /* Initialize I2C Instance */
    result = i2c_init(&i2c_obj, &i2c_pins);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing I2C %" PRIu32 "\r\n", result);
        CY_ASSERT(0);
    }

    /* Initialize SHT35 */
    result = sht35_initialize(&i2c_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing SHT35 sensor\r\n");
        return result;
    }

    /* Set SHT35 status to true */
    sensor_status_set_status(SHT35, true);

    /* Initialize BMI270 */
    result = bmi270_initialize(&i2c_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error starting BMI270 sensor\r\n");
        return result;
    }

    /* Set BMI270 status to true */
    sensor_status_set_status(BMI270, true);

    /* Initialize BMM350 */
    result = bmm350_initialize(&i2c_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error starting BMM350 sensor\r\n");
        return result;
    }

    /* Set BMM350 status to true */
    sensor_status_set_status(BMM350, true);

    /* Initialize DPS368 */
    result = dps368_initialize(&i2c_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error starting DPS368 sensor\r\n");
        return result;
    }

    /* Set DPS368 status to true */
    sensor_status_set_status(DPS368, true);

    /* Initialize PASCO2 */
    result = pasco2_initialize(&i2c_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error starting PASCO2 sensor\r\n");
        return result;
    }

    /* Set PASCO2 status to true */
    sensor_status_set_status(PASCO2, true);

    /* Initialize LTR11 last, so in case all other sensors aren't detected (XENSIV
     * SENSOR SHIELD is not connected), then we can say LTR11 isn't detected either.
     */

    /* Initialize RadarSEL pin */
    result = cyhal_gpio_init(RADAR_SEL, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing RADAR_SEL pin\r\n");
        return result;
    }

    /* Initialize LTR11 GPIOs */
    result = bgt60ltr11_initialize();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing BGT60LTR11 sensor\r\n");
        return result;
    }

    /* Set BGT60LTR11 status to true */
    sensor_status_set_status(BGT60LTR11, true);

    return CY_RSLT_SUCCESS;
}

void sensors_task(void *pvParameters)
{
    cy_rslt_t result;

    (void) pvParameters;


    /* Initialize all Sensors */
    result = sensors_initialize();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing some sensors\r\n");
        // Don't ASSERT, we can run app without them
    }

    uint32_t sensor_notification;

    for (;;)
    {
        /* Wait for notification */
         xTaskNotifyWait(pdFALSE,
                         0xFFFFFFFF,             /* Clear bits on Exit */
                         &sensor_notification,   /* Notification value set */
                         portMAX_DELAY);

        switch (sensor_notification)
        {
            case BGT60LTR11:
                if (sensor_status_get_status(BGT60LTR11))
                {
                    printf("Enabling BGT60LTR11...\r\n");
                    bgt60ltr11_sensor_task();
                }
                else
                {
                    printf("BGT60LTR11 not detected!\r\n");
                }
                break;

            case SHT35:
                if (sensor_status_get_status(SHT35))
                {
                    printf("Enabling SHT35...\r\n");
                    sht35_sensor_task();
                }
                else
                {
                    printf("SHT35 not detected!\r\n");
                }
                break;

            case BMI270:
                if (sensor_status_get_status(BMI270))
                {
                    printf("Enabling BMI270...\r\n");
                    bmi270_sensor_task();
                }
                else
                {
                    printf("BMI270 not detected!\r\n");
                }
                break;

            case BMM350:
                if (sensor_status_get_status(BMM350))
                {
                    printf("Enabling BMM350...\r\n");
                    bmm350_sensor_task();
                }
                else
                {
                    printf("BMM350 not detected!\r\n");
                }
                break;

            case DPS368:
                if (sensor_status_get_status(DPS368))
                {
                    printf("Enabling DPS368...\r\n");
                    dps368_sensor_task();
                }
                else
                {
                    printf("DPS368 not detected!\r\n");
                }
                break;

            case PASCO2:
                if (sensor_status_get_status(PASCO2))
                {
                    printf("Enabling PASCO2...\r\n");
                    pasco2_sensor_task();
                }
                else
                {
                    printf("PASCO2 not detected!\r\n");
                }
                break;

            case INVALID:
            default:
                sensor_notification = BGT60LTR11;
                break;
        }
    }
}

void sensor_status_init()
{
    sensor_status.bgt60ltr11 = false;
    sensor_status.sht35      = false;
    sensor_status.bmi270     = false;
    sensor_status.bmm350     = false;
    sensor_status.dps368     = false;
    sensor_status.pasco2     = false;
}

void sensor_status_set_status(sensor_notification_types_t sensor, bool status)
{
    switch (sensor)
    {
        case BGT60LTR11:  sensor_status.bgt60ltr11   = status; return;
        case SHT35:       sensor_status.sht35        = status; return;
        case BMI270:      sensor_status.bmi270       = status; return;
        case BMM350:      sensor_status.bmm350       = status; return;
        case DPS368:      sensor_status.dps368       = status; return;
        case PASCO2:      sensor_status.pasco2       = status; return;

        default: return; // shouldn't get here
    }
}

bool sensor_status_get_status(sensor_notification_types_t sensor)
{
    switch (sensor)
    {
        case BGT60LTR11:  return sensor_status.bgt60ltr11;
        case BMM350:      return sensor_status.bmm350;
        case BMI270:      return sensor_status.bmi270;
        case SHT35:       return sensor_status.sht35;
        case PASCO2:      return sensor_status.pasco2;
        case DPS368:      return sensor_status.dps368;

        default: return false; // shouldn't get here
    }
}
