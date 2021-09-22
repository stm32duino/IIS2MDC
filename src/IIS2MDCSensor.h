/**
 ******************************************************************************
 * @file    IIS2MDCSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    January 2020
 * @brief   Abstract Class of an IIS2MDC 3 axes magnetometer sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __IIS2MDCSensor_H__
#define __IIS2MDCSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "iis2mdc_reg.h"

/* Defines -------------------------------------------------------------------*/


#define IIS2MDC_MAG_SENSITIVITY_FS_50GAUSS  1.500f  /**< Sensitivity value for 50 gauss full scale [mgauss/LSB] */

/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  IIS2MDC_OK = 0,
  IIS2MDC_ERROR = -1
} IIS2MDCStatusTypeDef;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} IIS2MDC_AxesRaw_t;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} IIS2MDC_Axes_t;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an IIS2MDC Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class IIS2MDCSensor {
  public:
    IIS2MDCSensor(TwoWire *i2c);
    IIS2MDCStatusTypeDef begin();
    IIS2MDCStatusTypeDef end();
    IIS2MDCStatusTypeDef ReadID(uint8_t *Id);
    IIS2MDCStatusTypeDef Enable();
    IIS2MDCStatusTypeDef Disable();
    IIS2MDCStatusTypeDef GetSensitivity(float *sensitivity);
    IIS2MDCStatusTypeDef GetOutputDataRate(float *odr);
    IIS2MDCStatusTypeDef SetOutputDataRate(float odr);
    IIS2MDCStatusTypeDef GetFullScale(int32_t *fullscale);
    IIS2MDCStatusTypeDef SetFullScale(int32_t fullscale);
    IIS2MDCStatusTypeDef GetAxes(int32_t *magnetic_field);
    IIS2MDCStatusTypeDef GetAxesRaw(int16_t *value);
    IIS2MDCStatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
    IIS2MDCStatusTypeDef WriteReg(uint8_t reg, uint8_t data);
    IIS2MDCStatusTypeDef SetSelfTest(uint8_t val);
    IIS2MDCStatusTypeDef GetDRDYStatus(uint8_t *status);

    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:

    /* Helper classes. */
    TwoWire *dev_i2c;

    /* Configuration */
    uint8_t address;

    uint8_t mag_is_enabled;

    iis2mdc_ctx_t reg_ctx;

};

#ifdef __cplusplus
extern "C" {
#endif
int32_t IIS2MDC_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t IIS2MDC_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
