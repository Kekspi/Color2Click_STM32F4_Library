# Color2Click_STM32F4_Library
A stm32f4 library to design,read and send it with serial CDC for color 16 click spectral sensor. It also involves a python code to read, configurate and create a live graph from the datas of the sensor.
///////////////////////////////
You can set the gain, integration time for all the channels of Color 16 Click/AS7343 spectral sensor and also enable/disable the led and change led's power.
You configurate the values from python. Configuration parameters are the sensors data while it is pointing to a white led. Sensor does not add the flicker detection and visible light to the live graph however, you can still acess these datas with the serial communication.
