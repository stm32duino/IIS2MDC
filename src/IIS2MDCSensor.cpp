/**
 ******************************************************************************
 * @file    IIS2MDCSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    January 2020
 * @brief   Implementation of an IIS2MDC 3 axes gyroscope sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "IIS2MDCSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
IIS2MDCSensor::IIS2MDCSensor(TwoWire *i2c) : dev_i2c(i2c)
{
  reg_ctx.write_reg = IIS2MDC_io_write;
  reg_ctx.read_reg = IIS2MDC_io_read;
  reg_ctx.handle = (void *)this;
  address = IIS2MDC_I2C_ADD;
  mag_is_enabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::begin()
{
  /* Enable BDU */
  if (iis2mdc_block_data_update_set(&(reg_ctx), PROPERTY_ENABLE) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  /* Operating mode selection - power down */
  if (iis2mdc_operating_mode_set(&(reg_ctx), IIS2MDC_POWER_DOWN) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  /* Output data rate selection */
  if (iis2mdc_data_rate_set(&(reg_ctx), IIS2MDC_ODR_100Hz) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  /* Self Test disabled. */
  if (iis2mdc_self_test_set(&(reg_ctx), PROPERTY_DISABLE) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  mag_is_enabled = 0U;

  return IIS2MDC_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::end()
{
  /* Disable mag */
  if (Disable() != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::ReadID(uint8_t *Id)
{
  if (iis2mdc_device_id_get(&reg_ctx, Id) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}


/**
 * @brief Enable the IIS2MDC magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::Enable()
{
  /* Check if the component is already enabled */
  if (mag_is_enabled == 1U) {
    return IIS2MDC_OK;
  }

  /* Output data rate selection. */
  if (iis2mdc_operating_mode_set(&reg_ctx, IIS2MDC_CONTINUOUS_MODE) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  mag_is_enabled = 1;

  return IIS2MDC_OK;
}

/**
 * @brief Disable the IIS2MDC magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::Disable()
{
  /* Check if the component is already disabled */
  if (mag_is_enabled == 0U) {
    return IIS2MDC_OK;
  }

  /* Output data rate selection - power down. */
  if (iis2mdc_operating_mode_set(&reg_ctx, IIS2MDC_POWER_DOWN) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  mag_is_enabled = 0;

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC magnetometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetSensitivity(float *Sensitivity)
{
  *Sensitivity = IIS2MDC_MAG_SENSITIVITY_FS_50GAUSS;

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC magnetometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetOutputDataRate(float *Odr)
{
  IIS2MDCStatusTypeDef ret = IIS2MDC_OK;
  iis2mdc_odr_t odr_low_level;

  /* Get current output data rate. */
  if (iis2mdc_data_rate_get(&reg_ctx, &odr_low_level) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  switch (odr_low_level) {
    case IIS2MDC_ODR_10Hz:
      *Odr = 10.0f;
      break;

    case IIS2MDC_ODR_20Hz:
      *Odr = 20.0f;
      break;

    case IIS2MDC_ODR_50Hz:
      *Odr = 50.0f;
      break;

    case IIS2MDC_ODR_100Hz:
      *Odr = 100.0f;
      break;

    default:
      ret = IIS2MDC_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the IIS2MDC magnetometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::SetOutputDataRate(float Odr)
{
  iis2mdc_odr_t new_odr;

  new_odr = (Odr <= 10.000f) ? IIS2MDC_ODR_10Hz
            : (Odr <= 20.000f) ? IIS2MDC_ODR_20Hz
            : (Odr <= 50.000f) ? IIS2MDC_ODR_50Hz
            :                    IIS2MDC_ODR_100Hz;

  if (iis2mdc_data_rate_set(&reg_ctx, new_odr) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC magnetometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetFullScale(int32_t *FullScale)
{
  *FullScale = 50;

  return IIS2MDC_OK;
}

/**
 * @brief  Set the IIS2MDC magnetometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::SetFullScale(int32_t FullScale)
{
  (void)FullScale;
  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC magnetometer sensor axes
 * @param  MagneticField pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetAxes(int32_t *MagneticField)
{
  axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (iis2mdc_magnetic_raw_get(&reg_ctx, data_raw.u8bit) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  /* Get IIS2MDC actual sensitivity. */
  GetSensitivity(&sensitivity);

  /* Calculate the data. */
  MagneticField[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  MagneticField[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  MagneticField[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC magnetometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetAxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (iis2mdc_magnetic_raw_get(&reg_ctx, data_raw.u8bit) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC register value for magnetic sensor
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::ReadReg(uint8_t Reg, uint8_t *Data)
{
  if (iis2mdc_read_reg(&reg_ctx, Reg, Data, 1) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}

/**
 * @brief  Set the IIS2MDC register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::WriteReg(uint8_t Reg, uint8_t Data)
{
  if (iis2mdc_write_reg(&reg_ctx, Reg, &Data, 1) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of self_test in reg CFG_REG_C
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::SetSelfTest(uint8_t val)
{
  if (iis2mdc_self_test_set(&reg_ctx, val) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}

/**
 * @brief  Get the IIS2MDC MAG data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
IIS2MDCStatusTypeDef IIS2MDCSensor::GetDRDYStatus(uint8_t *Status)
{
  if (iis2mdc_mag_data_ready_get(&reg_ctx, Status) != IIS2MDC_OK) {
    return IIS2MDC_ERROR;
  }

  return IIS2MDC_OK;
}



int32_t IIS2MDC_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((IIS2MDCSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t IIS2MDC_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((IIS2MDCSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
