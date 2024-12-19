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
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bmi270_task.h"
#include "mtb_bmi270.h"
#include "task_handles.h"

#include "console_task.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_MS                (100U)

/******************************************************************************
* Types
*******************************************************************************/
typedef enum
{
    ORIENTATION_NULL            = 0,    /* Default orientation state used for initialization purposes */
    ORIENTATION_TOP_EDGE        = 1,    /* Top edge of the board points towards the ceiling */
    ORIENTATION_BOTTOM_EDGE     = 2,    /* Bottom edge of the board points towards the ceiling */
    ORIENTATION_LEFT_EDGE       = 3,    /* Left edge of the board (USB connector side) points towards the ceiling */
    ORIENTATION_RIGHT_EDGE      = 4,    /* Right edge of the board points towards the ceiling */
    ORIENTATION_DISP_UP         = 5,    /* Display faces up (towards the sky/ceiling) */
    ORIENTATION_DISP_DOWN       = 6     /* Display faces down (towards the ground) */
} orientation_t;

/******************************************************************************
* Globals
*******************************************************************************/
/* Structure holding bmi270 variables */
static mtb_bmi270_t bmi270_obj;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* These static functions are used by the Motion Sensor. These are not
 * available outside this file. See the respective function definitions for
 * more details.
 */
static cy_rslt_t motionsensor_update_orientation(mtb_bmi270_t *dev, orientation_t *orientation);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
cy_rslt_t bmi270_initialize(cyhal_i2c_t *i2c_instance)
{
    cy_rslt_t result;

    /* Initialize the BMI270 motion sensor*/
    result = mtb_bmi270_init_i2c(&bmi270_obj, i2c_instance, MTB_BMI270_ADDRESS_SEC);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing BMI270\r\n");
        return result;
    }

    /* Configure the BMI270 motion sensor */
    result = mtb_bmi270_config_default(&bmi270_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error configuring BMI270\r\n");
        return result;
    }

    return result;
}

void bmi270_sensor_task()
{
    wiced_bt_gatt_status_t gatt_status;

    /* Set exit_task to false initially */
    exit_task = false;

    orientation_t orientation;

    for (;;)
    {
        motionsensor_update_orientation(&bmi270_obj, &orientation);

        app_xensiv_sensor_shield_bmi270[0] = orientation;

        /* Send Notification to BLE App */
        gatt_status = app_bt_gatt_notify(HDLC_XENSIV_SENSOR_SHIELD_BMI270_VALUE,
                                         app_xensiv_sensor_shield_bmi270,
                                         app_xensiv_sensor_shield_bmi270_len);
        printf("Sent notification status 0x%x\r\n\n", gatt_status);

        cyhal_system_delay_ms(DELAY_MS);

        /* Exit task */
        if (exit_task)
        {
            exit_task = false;
            break;
        }
    }
}

/*******************************************************************************
 * Function Name: motionsensor_update_orientation
 ********************************************************************************
 * Summary:
 *  Function that updates the orientation status to one of the 6 types, see
 *  'orientation'. This functions detects the axis that is most perpendicular
 *  to the ground based on the absolute value of acceleration in that axis.
 *  The sign of the acceleration signifies whether the axis is facing the ground
 *  or the opposite.
 *
 * Return:
 *  CY_RSLT_SUCCESS upon successful orientation update, else a non-zero value
 *  that indicates the error.
 *
 *******************************************************************************/
static cy_rslt_t motionsensor_update_orientation(mtb_bmi270_t *dev, orientation_t *orientation)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int abs_x, abs_y, abs_z;

    mtb_bmi270_data_t data;

    /* Read x, y, z components of acceleration */
    result = mtb_bmi270_read(&bmi270_obj, &data);
    if (CY_RSLT_SUCCESS != result)
    {
    	printf("Error reading from BMI270 sensor\r\n");
        *orientation = ORIENTATION_NULL;
        return result;
    }

    abs_x = abs((int) data.sensor_data.acc.x);
    abs_y = abs((int) data.sensor_data.acc.y);
    abs_z = abs((int) data.sensor_data.acc.z);

    printf("x: %d, y: %d, z: %d\r\n", abs_x, abs_y, abs_z);

    if ((abs_z > abs_x) && (abs_z > abs_y))
    {
        if (data.sensor_data.acc.z < 0)
        {
            /* Display faces down (towards the ground) */
            printf("Orientation = ORIENTATION_DISP_DOWN\r\n");
            *orientation = ORIENTATION_DISP_DOWN;
        }
        else
        {
            /* Display faces up (towards the sky/ceiling) */

            printf("Orientation = ORIENTATION_DISP_UP\r\n");
            *orientation = ORIENTATION_DISP_UP;
        }
    }
    /* Y axis (parallel with shorter edge of board) is most aligned with
     * gravity.
     */
    else if ((abs_y > abs_x) && (abs_y > abs_z))
    {
        if (data.sensor_data.acc.y > 0)
        {
            /* Display has an inverted landscape orientation */

            printf("Orientation = ORIENTATION_BOTTOM_EDGE\r\n");
            *orientation = ORIENTATION_BOTTOM_EDGE;
        }
        else
        {
            /* Display has landscape orientation */

            printf("Orientation = ORIENTATION_TOP_EDGE\r\n");
            *orientation = ORIENTATION_TOP_EDGE;
        }
    }
    /* X axis (parallel with longer edge of board) is most aligned with
     * gravity.
     */
    else
    {
        if (data.sensor_data.acc.x< 0)
        {
            /* Display has an inverted portrait orientation */

            printf("Orientation = ORIENTATION_RIGHT_EDGE\r\n");
            *orientation = ORIENTATION_RIGHT_EDGE;
        }
        else
        {
            /* Display has portrait orientation */

            printf("Orientation = ORIENTATION_LEFT_EDGE\r\n");
            *orientation = ORIENTATION_LEFT_EDGE;
        }
    }

    return result;
}
