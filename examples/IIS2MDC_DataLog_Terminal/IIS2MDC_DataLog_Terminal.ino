/*
   @file    IIS2MDC_DataLogTerminal.ino
   @author  Frederic Pillon <frederic.pillon@st.com>
   @brief   Example to use the IIS2MDC high-performance 3-axis magnetometer
 *******************************************************************************
   Copyright (c) 2021, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/


// Includes
#include <IIS2MDCSensor.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PNUM_NOT_DEFINED
#warning "LED_BUILTIN is not defined."
#endif

#define SerialPort  Serial

#if defined(ARDUINO_B_U585I_IOT02A)
#define IIS2MDC_I2C_SCL     PH4
#define IIS2MDC_I2C_SDA     PH5
#else
// Uncomment to set I2C pins to use else default instance will be used
// #define IIS2MDC_I2C_SCL  PYn
// #define IIS2MDC_I2C_SDA  PYn
#endif
#if defined(IIS2MDC_I2C_SCL) && defined(IIS2MDC_I2C_SDA)
TwoWire dev_i2c(IIS2MDC_I2C_SDA, IIS2MDC_I2C_SCL);
#else
// X-NUCLEO-IKS02A1 uses default instance
#define dev_i2c       Wire
#endif

IIS2MDCSensor Magneto(&dev_i2c);

void setup() {
  // Led
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output
  SerialPort.begin(9600);

  // Initialize bus interface
  dev_i2c.begin();

  // Initlialize component
  Magneto.begin();
  Magneto.Enable();
}

void loop() {
  // Led blinking
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read magnetometer
  int32_t magnetometer[3];
  Magneto.GetAxes(magnetometer);

  SerialPort.print("Mag[mGauss]:");
  Serial.print(magnetometer[0]);
  Serial.print(", ");
  Serial.print(magnetometer[1]);
  Serial.print(", ");
  Serial.println(magnetometer[2]);
}
