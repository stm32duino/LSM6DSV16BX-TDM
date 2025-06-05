# LSM6DSV16BX TDM
Arduino library to support the TDM feature of the LSM6DSV16BX MEMS sensor.
Currently, this library works only with the STEVAL-MKI237KA and a NUCLEO-U575ZI-Q.
It requires a [STM32 Core](https://github.com/stm32duino/Arduino_Core_STM32) equal to or greater than version 2.0.0.

## API
This library acquires PCM.

### PCM
Before acquire PCM it is required to initialize it:

`   PCM.begin();`

Thanks  to `PCM` instance, it is possible to start to acquire the data, stop, pause the acquisition and resume.
- Start to acquire the PCM data:
    
`   PCM.Record(buffer);`
- Stop to acquire the PCM data:
    
`   PCM.Stop();`
- Pause the acquisition:
    
`   PCM.Pause();`
- Resume the acquisition.
    
`   PCM.Resume();`
- To perform a function to process the data (foo in the example), it can call the function below:
    
`   PCM.onReceive(foo);`

## Examples

* LSM6DSV16BX_TDM_Python_Wave_Serial_Encoder: This application shows how to record some TDM data with [LSM6DSV16BX](https://www.st.com/en/mems-and-sensors/lsm6dsv16bx.html) IMU sensor and send them through serial to a Python application which will write data to a wave file (Python application can be downloaded at https://github.com/stm32duino/Python-Wave-Serial-Encoder)

You need to connect the STEVAL-MKI237KA eval board directly to the NUCLEO-U575ZI-Q board with wires as explained below:
 - pin 1 (VDD) of the STEVAL-MKI237KA eval board connected to pin 3V3 of the NUCLEO-U575ZI-Q board
 - pin 2 (VDDIO) of the STEVAL-MKI237KA eval board connected to pin IOREF of the NUCLEO-U575ZI-Q board
 - pin 3 (WCLK) of the STEVAL-MKI237KA eval board connected to pin FS of SAI_B of the NUCLEO-U575ZI-Q board
 - pin 4 (BCLK) of the STEVAL-MKI237KA eval board connected to pin SCK of SAI_B of the NUCLEO-U575ZI-Q board
 - pin 6 (TDM) of the STEVAL-MKI237KA eval board connected to pin SD of SAI_B of the NUCLEO-U575ZI-Q board
 - pin 13 (GND) of the STEVAL-MKI237KA eval board connected to GND of the NUCLEO-U575ZI-Q board
 - pin 19 (CS) of the STEVAL-MKI237KA eval board connected to pin AVDD of the NUCLEO-U575ZI-Q board
 - pin 20 (SCL) of the STEVAL-MKI237KA eval board connected to pin D15 SCL of the NUCLEO-U575ZI-Q board
 - pin 21 (SDA) of the STEVAL-MKI237KA eval board connected to pin D14 SDA of the NUCLEO-U575ZI-Q board

# Dependencies

The LSM6DSV16BX-TDM library requires the following STM32duino library:

* STM32duino LSM6DSV16BX: https://github.com/stm32duino/LSM6DSV16BX

# Documentation

You can find the source files at
https://github.com/stm32duino/LSM6DSV16BX-TDM

The LSM6DSV16BX datasheet is available at
https://www.st.com/en/mems-and-sensors/lsm6dsv16bx.html
