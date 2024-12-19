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
* Includes
*******************************************************************************/
#include "cybsp.h"
#include "i2c.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define I2C_CLK_FREQ_HZ                 (400000U)

/****************************************************************************
* Globals
*****************************************************************************/
/* Handle for the semaphore that locks the I2C resource */
static SemaphoreHandle_t i2c_semaphore;

/****************************************************************************
* Function Definitions
*****************************************************************************/
int32_t i2c_init(cyhal_i2c_t *i2c_obj, i2c_pins_t *i2c_pins)
{
    cy_rslt_t result;

    /* I2C Configuration Structure */
    cyhal_i2c_cfg_t i2c_config = { false, 0, I2C_CLK_FREQ_HZ };

    /* Initialize I2C */
    result = cyhal_i2c_init(i2c_obj, i2c_pins->sda, i2c_pins->scl, NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        return -1;
    }

    /* Configure I2C */
    result = cyhal_i2c_configure(i2c_obj, &i2c_config);
    if (CY_RSLT_SUCCESS != result)
    {
        return -2;
    }

    /* Initialize I2C Semaphore to lock resource during use */
    i2c_semaphore = xSemaphoreCreateBinary();
    if (NULL == i2c_semaphore)
    {
        printf("Error creating i2c binary semaphore\r\n");
        CY_ASSERT(0);
    }
    xSemaphoreGive(i2c_semaphore);

    return result;
}

int32_t i2c_master_mem_write(cyhal_i2c_t *obj, uint16_t address, uint16_t mem_addr, uint16_t mem_addr_size, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    cy_rslt_t result;

    /* Block I2C resource before using */
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);

    result = cyhal_i2c_master_mem_write(obj, address, mem_addr, mem_addr_size, data, size, timeout);

    /* Release I2C resource */
    xSemaphoreGive(i2c_semaphore);

    return result;
}

int32_t i2c_master_mem_read(cyhal_i2c_t *obj, uint16_t address, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    cy_rslt_t result;

    /* Block I2C resource before using */
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);

    result = cyhal_i2c_master_mem_read(obj, address, mem_addr, mem_addr_size, data, size, timeout);

    /* Release I2C resource */
    xSemaphoreGive(i2c_semaphore);

    return result;
}

int32_t i2c_master_write(cyhal_i2c_t *obj, uint16_t dev_addr, const uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    cy_rslt_t result;

    /* Block I2C resource before using */
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);

    result = cyhal_i2c_master_write(obj, dev_addr, data, size, timeout, send_stop);

    /* Release I2C resource */
    xSemaphoreGive(i2c_semaphore);

    return result;
}

int32_t i2c_master_read(cyhal_i2c_t *obj, uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    cy_rslt_t result;

    /* Block I2C resource before using */
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);

    result = cyhal_i2c_master_read(obj, dev_addr, data, size, timeout, send_stop);

    /* Release I2C resource */
    xSemaphoreGive(i2c_semaphore);

    return result;
}
