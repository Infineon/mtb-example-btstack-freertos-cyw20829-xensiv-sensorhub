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

#include "cybsp.h"
#include "cyhal_gpio.h"
#include "cyhal_system.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bgt60ltr11_task.h"
#include "console_task.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define BGT60LTR11_PDET             (CYBSP_D11)
#define BGT60LTR11_TDET             (CYBSP_D12)

#define TARGET_DETECT_INDEX         (0)
#define PHASE_DETECT_INDEX          (1)

#define TARGET_DETECTED             (0)
#define TARGET_NOT_DETECTED         (1)

#define TARGET_APPROACHING          (1)
#define TARGET_DEPARTING            (0)

#define DELAY_MS                    (500u)

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t bgt60ltr11_initialize()
{
    cy_rslt_t result;

    result = cyhal_gpio_init(BGT60LTR11_PDET, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing PDET\r\n");
        return result;
    }

    result = cyhal_gpio_init(BGT60LTR11_TDET, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing TDET\r\n");
        return result;
    }

    return result;
}


void bgt60ltr11_sensor_task()
{
    wiced_bt_gatt_status_t gatt_status;

    int32_t tdet_status = TARGET_NOT_DETECTED;
    int32_t pdet_status = TARGET_DEPARTING;

    /* TDET = Target Detected
     * if TDET = 0 (low), Target detected
     * if TDET = 1 (high), Target not detected
     * */

    /* PDET = Phase Detection
     * if PDET = 0 (low), Target departing
     * if PDET = 1 (high), Target approaching
     * */

    /* Set exit_task to false initially */
    exit_task = false;

    for (;;)
    {
        tdet_status = cyhal_gpio_read(BGT60LTR11_TDET);
        pdet_status = cyhal_gpio_read(BGT60LTR11_PDET);

        if (TARGET_DETECTED == tdet_status)
        {
            printf("Presence Detected: ");

            /* Target detected, check if target approaching or departing */
            if (TARGET_DEPARTING == pdet_status)
            {
                printf("Target Departing!\r\n");
            }
            else
            {
                printf("Target Approaching!\r\n");
            }
        }
        else
        {
            /* No target detected, don't check for PDET */
            printf("No presence detected\r\n");
        }

        app_xensiv_sensor_shield_bgt60ltr11[TARGET_DETECT_INDEX] = tdet_status;
        app_xensiv_sensor_shield_bgt60ltr11[PHASE_DETECT_INDEX] = pdet_status;

        /* Send Notification to BLE App */
        gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_BGT60LTR11_VALUE,
                                         app_xensiv_sensor_shield_bgt60ltr11,
                                         app_xensiv_sensor_shield_bgt60ltr11_len);
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
