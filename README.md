# IIS2MDC
Arduino library to support the IIS2MDC high-performance 3-axis magnetometer

## API

This sensor uses I2C to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    IIS2MDCSensor Magneto(&dev_i2c);
    Magneto.begin();
    Magneto.Enable();

The access to the sensor values is done as explained below:  

  Read magnetometer.  

    int32_t magnetometer[3];
    Magneto.GetAxes(magnetometer);

## Documentation

You can find the source files at  
https://github.com/stm32duino/IIS2MDC

The IIS2MDC datasheet is available at  
https://www.st.com/en/mems-and-sensors/iis2mdc.html
