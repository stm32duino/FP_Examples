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
* BleSensors_SensiBLE: This application provides an example of usage of a SensiBLE board. The application uses BLE along with environmental and 
motion sensors (humidity, temperature, pressure, accelerometer, gyroscope). The user can connect an Android or IOS device with BlueNRG application 
to SensiBLE and see sensors data. The data are also printed to the serial port.
* Flight1: This application provides an example of usage of a NUCLEO board (it was tested with NUCLEO-F401RE, NUCLEO-L476RG, NUCLEO-L152RE) with
X-NUCLEO-IKS01A2 or X-NUCLEO-IKS01A3, X-NUCLEO-IDB05A1 and X-NUCLEO-53L1A1 Expansion Boards. The application gather data from the sensor mounted and then communicates
them via bluetooth to a suitable Android app (such as ST BLE Sensor). The data gathered include enviromental data (temperature, pression, humidity), 
distance, gesture recognition (tap and directional swipe), acceleration (with accelerometer hardware events) and a gyroscope.
* IKS01A3_S2LP_P2P_Demo: This application provides a simple example of usage of two NUCLEO boards (it was tested with NUCLEO-F401RE and NUCLEO-L053R8) 
with a X-NUCLEO-IKS01A3 and one among X-NUCLEO-S2868A1, X-NUCLEO-S2868A2 and X-NUCLEO-S2915A1. It shows how to send and receive all the sensor data 
provided by the X-NUCLEO-IKS01A3 between two NUCLEO boards connected each other through S2-LP based Expansion Boards; in order to send the sensor data 
you just need to push the button of the NUCLEO board; in order to see the received sensor data you just need to open a hyperterminal connected to 
the Virtual COM port of the Nucleo board; the LED of the NUCLEO board will blink after every successful transmission or reception.

## Dependencies

The FP_Examples library requires the following STM32duino libraries (link to the source files):

* STM32duino LSM6DSL: https://github.com/stm32duino/LSM6DSL
* STM32duino LSM303AGR: https://github.com/stm32duino/LSM303AGR
* STM32duino LPS22HB: https://github.com/stm32duino/LPS22HB
* STM32duino Proximity Gesture: https://github.com/stm32duino/Proximity_Gesture
* STM32duino VL6180X: https://github.com/stm32duino/VL6180X
* STM32duino VL53L0X: https://github.com/stm32duino/VL53L0X
* STM32duino VL53L1X: https://github.com/stm32duino/VL53L1X
* STM32duino SPBTLE-RF: https://github.com/stm32duino/SPBTLE-RF
* STM32duino X-NUCLEO-6180XA1: https://github.com/stm32duino/X-NUCLEO-6180XA1
* STM32duino X-NUCLEO-53L0A1: https://github.com/stm32duino/X-NUCLEO-53L0A1
* STM32duino X-NUCLEO-53L1A1: https://github.com/stm32duino/X-NUCLEO-53L1A1
* STM32duino X-NUCLEO-IHM02A1: https://github.com/stm32duino/X-NUCLEO-IHM02A1
* STM32duino X-NUCLEO-LED61A1: https://github.com/stm32duino/X-NUCLEO-LED61A1
* STM32duino HTS221: https://github.com/stm32duino/HTS221
* STM32duino LPS25HB: https://github.com/stm32duino/LPS25HB
* STM32duino LSM6DS3: https://github.com/stm32duino/LSM6DS3
* STM32duino LSM6DSO: https://github.com/stm32duino/LSM6DSO
* STM32duino LIS2DW12: https://github.com/stm32duino/LIS2DW12
* STM32duino LIS2MDL: https://github.com/stm32duino/LIS2MDL
* STM32duino LPS22HH: https://github.com/stm32duino/LPS22HH
* STM32duino STTS751: https://github.com/stm32duino/STTS751
* STM32duino S2-LP: https://github.com/stm32duino/S2-LP
* STM32duino M95640-R: https://github.com/stm32duino/M95640-R


## Documentation

The datasheets of the several components are available at  
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsl.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/e-compasses/lsm303agr.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hb.html
 * https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl6180x.html
 * https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l0x.html
 * https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l1x.html
 * https://www.st.com/content/st_com/en/products/wireless-connectivity/bluetooth-bluetooth-low-energy/spbtle-rf.html
 * https://www.st.com/content/st_com/en/products/motor-drivers/stepper-motor-drivers/l6470.html
 * https://www.st.com/content/st_com/en/products/power-management/led-drivers/boost-current-regulators-for-led/led6001.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/humidity-sensors/hts221.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps25hb.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6ds3.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dso.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis2dw12.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/e-compasses/lis2mdl.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hh.html
 * https://www.st.com/content/st_com/en/products/mems-and-sensors/temperature-sensors/stts751.html
 * https://www.st.com/content/st_com/en/products/wireless-transceivers-mcus-and-modules/sub-1ghz-rf/s2-lp.html
 * https://www.st.com/content/st_com/en/products/memories/serial-eeprom/standard-serial-eeprom/standard-spi-eeprom/m95640-r.html





