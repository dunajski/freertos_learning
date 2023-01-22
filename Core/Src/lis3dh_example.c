/*
 * lis3dh_example.c
 *
 *  Created on: Jan 18, 2023
 *      Author: dnj
 * STM lis3dh driver:
 * https://github.com/STMicroelectronics/lis3dh-pid/tree/master
 * STM lis3dh examples to driver mentioned above:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3dh_STdC/examples
 * Adafruit LIS3DH kit:
 * https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/pinouts
 */

/* NUCLEO_G071RB: Define communication interface */
#define SENSOR_BUS hspi1

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis3dh_reg.h"
#include <lis3dh_example.h>
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

#include "stm32g0xx_hal.h"
#include "stm32g071xx.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);

/* Main Example --------------------------------------------------------------*/

static stmdev_ctx_t dev_ctx;

accel_state_t lis3dh_init(void)
{
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Check device ID */
  lis3dh_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS3DH_ID)
  {
    sprintf((char *)tx_buffer, "ERROR! Accelerometer not found %d\n", whoamI);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    while (1)
    {
      return ACC_ERROR;
    }
  }

  /*  Enable Block Data Update */
  lis3dh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate to 10 hz */
  lis3dh_data_rate_set(&dev_ctx, LIS3DH_ODR_10Hz);
  /* Set full scale to 2 g */
  lis3dh_full_scale_set(&dev_ctx, LIS3DH_2g);
  /* Set operating mode to high resolution */
  lis3dh_operating_mode_set(&dev_ctx, LIS3DH_HR_12bit);
  /* Set FIFO watermark to 25 samples */
  lis3dh_fifo_watermark_set(&dev_ctx, 25);
  /* Set FIFO mode to Stream mode: Accumulate samples and
   * override old data */
  lis3dh_fifo_mode_set(&dev_ctx, LIS3DH_DYNAMIC_STREAM_MODE);
  /* Enable FIFO */
  lis3dh_fifo_set(&dev_ctx, PROPERTY_ENABLE);

  return ACC_RUNNING;
}

bool lis3dh_is_samples_ready(void)
{
  uint8_t flags;
  /* Check if FIFO level over threshold */
  lis3dh_fifo_fth_flag_get(&dev_ctx, &flags);

  if (flags) return true;
  else return false;
}

void lis3dh_read_fifo(void)
{
  uint8_t num;
  /* Read number of sample in FIFO */
  lis3dh_fifo_data_level_get(&dev_ctx, &num);

  while (num-- > 0)
  {
  /* Read XL samples */
  lis3dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  acceleration_mg[0] =
      lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
  acceleration_mg[1] =
      lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
  acceleration_mg[2] =
      lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);
  sprintf((char *)tx_buffer,
          "Acceleration [mg]:%5.0f\t%5.0f\t%5.0f\r\n",
          acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
  tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  reg |= 0xC0;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}
