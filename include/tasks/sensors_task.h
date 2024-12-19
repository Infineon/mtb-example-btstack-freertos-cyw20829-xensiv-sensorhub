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

#ifndef INCLUDE_TASKS_SENSORS_TASK_H_
#define INCLUDE_TASKS_SENSORS_TASK_H_

#include "cyhal.h"
#include "notification_types.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define SENSORS_TASK_NAME               "sensors_task"
#define SENSORS_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE * 4)
#define SENSORS_TASK_PRIORITY           (configMAX_PRIORITIES - 2)

/*******************************************************************************
 * Types
 ******************************************************************************/
typedef struct
{
    bool bgt60ltr11;
    bool bmm350;
    bool bmi270;
    bool sht35;
    bool pasco2;
    bool dps368;
} sensor_status_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
bool sensor_status_get_status(sensor_notification_types_t sensor);
void sensors_task(void *pvParameters);

#endif /* INCLUDE_TASKS_SENSORS_TASK_H_ */