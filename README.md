# FP_Examples

This library provides several Function Packs that combine the usage of several X-NUCLEO boards together with a NUCLEO board.

## Examples

There are several examples with the FP_Examples library.
* NucleoCar: This application provides a funny example of usage of a NUCLEO board (it was tested with NUCLEO-F401RE) with 
X-NUCLEO-IDB05A1, X-NUCLEO-IHM02A1 and X-NUCLEO-6180XA1 Expansion Boards. The application implements two operative modes. With the 
first mode the car can move using the main VL6180X sensor; the closer we put the hand on the sensor, the higher is the speed of the car. 
The VL6180X satellites instead are used to avoid the obstacles. With the second mode the car can be controlled via BTLE using a dedicated 
Android App. Also in this case the VL6180X satellites try to avoid the obstacles. In order to change the car mode, you can push the User 
Button of the Nucleo board.
* GestureDirSwipeLed_53L0A1_LED61A1: This application provides a simple example of usage of a NUCLEO board (it was tested with NUCLEO-L476RG) 
with X-NUCLEO-53L0A1 and X-NUCLEO-LED61A1 Expansion Boards. The application allows to switch on/off a strip of LEDs keeping the hand on the 3
proximity sensors at least for 3 seconds. When the LEDs are switched on, the user can increase or decrease the intensity of the LEDs performing
a swipe movement respectively from left to right and from right to left.
* MemsMotorControl_IKS01A2_IHM02A1: This application provides a simple example of usage of a NUCLEO board (it was tested with NUCLEO-F401RE) 
with X-NUCLEO-IKS01A2 and X-NUCLEO-IHM02A1 Expansion Boards. The application allows to run a motor clockwise or counter-clockwise rotating the 
stack of boards accordingly; the speed of motor increases when the angle of rotation increases as well. 
* BleSensors_SensiBLE: This application provides an example of usage of a SensiBLE board. The application uses BLE along with environmental and motion sensors (humidity, temperature, pressure, accelerometer, gyroscope). The user can connect an Android or IOS device with BlueNRG application to SensiBLE and see sensors data. The data are also printed to the serial port.


## Dependencies

The FP_Examples library requires the following STM32duino libraries (link to the source files):

* STM32duino LSM6DSL: https://github.com/stm32duino/LSM6DSL
* STM32duino Proximity_Gesture: https://github.com/stm32duino/Proximity_Gesture
* STM32duino VL6180X: https://github.com/stm32duino/VL6180X
* STM32duino VL53L0X: https://github.com/stm32duino/VL53L0X
* STM32duino SPBTLE-RF: https://github.com/stm32duino/SPBTLE-RF
* STM32duino X-NUCLEO-6180XA1: https://github.com/stm32duino/X-NUCLEO-6180XA1
* STM32duino X-NUCLEO-53L0A1: https://github.com/stm32duino/X-NUCLEO-53L0A1
* STM32duino X-NUCLEO-IHM02A1: https://github.com/stm32duino/X-NUCLEO-IHM02A1
* STM32duino X-NUCLEO-LED61A1: https://github.com/stm32duino/X-NUCLEO-LED61A1
* STM32duino HTS221: https://github.com/stm32duino/HTS221
* STM32duino LPS25HB: https://github.com/stm32duino/LPS25HB
* STM32duino LSM6DS3: https://github.com/stm32duino/LSM6DS3


## Documentation

The datasheets of the several components are available at  
 * http://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsl.html
 * http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl6180x.html
 * http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l0x.html
 * http://www.st.com/content/st_com/en/products/wireless-connectivity/bluetooth-bluetooth-low-energy/spbtle-rf.html
 * http://www.st.com/content/st_com/en/products/motor-drivers/stepper-motor-drivers/l6470.html
 * http://www.st.com/content/st_com/en/products/power-management/led-drivers/boost-current-regulators-for-led/led6001.html
 * http://www.st.com/content/st_com/en/products/mems-and-sensors/humidity-sensors/hts221.html
 * http://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps25hb.html
 * http://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6ds3.html





