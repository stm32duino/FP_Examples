/**
 ******************************************************************************
   @file    Flight1v2.ino
   @author  STMicroelectronics
   @version V1.0.0
   @date    15 March 2023
   @brief   Arduino demo application for the STMicrolectronics
            X-NUCLEO-IKS01A3, X-NUCLEO-53L1A1
            and X-NUCLEO-IDB05A1
 ******************************************************************************
   @attention

   <h2><center>&copy; COPYRIGHT(c) 2023 STMicroelectronics</center></h2>

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of STMicroelectronics nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
*/

// Note: make sure your STM32duinoBLE is updated to the latest version!

// Includes ------------------------------------------------------------------

#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>
#include <stmpe1600_class.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>
#include <tof_gestures_TAP_1.h>

#include <HTS221Sensor.h>
#include <LPS22HHSensor.h>
#include <LIS2DW12Sensor.h>
#include <LIS2MDLSensor.h>
#include <LSM6DSOSensor.h>
#include <STTS751Sensor.h>

#include <STM32duinoBLE.h>

#define DEV_I2C Wire
#define SerialPort Serial

//#define DEBUG_MODE

#define INT_1 4
#define INT_2 5

// BLE boards
/* Shield IDB05A2 with SPI clock on D3 */
SPIClass SpiHCI(D11, D12, D3);
HCISpiTransportClass HCISpiTransport(SpiHCI, BLUENRG_M0, A1, A0, D7, 8000000, SPI_MODE0);
#if !defined(FAKE_BLELOCALDEVICE)
BLELocalDevice BLEObj(&HCISpiTransport);
BLELocalDevice &BLE = BLEObj;
#endif

// Interrupts
volatile int mems_event = 0;

// Package Version only numbers 0->9
#define FLIGHT1_VERSION_MAJOR '5'
#define FLIGHT1_VERSION_MINOR '0'
#define FLIGHT1_VERSION_PATCH '0'

#ifdef DEBUG_MODE
#define FLIGHT1_PRINTF(...) \
  do {\
    char report[130];\
    snprintf(report, sizeof(report), __VA_ARGS__);\
    SerialPort.print(report);\
  } while(0);
#else
#define FLIGHT1_PRINTF(...) while(0){;}
#endif
// Define the FLIGHT1 Name MUST be 7 char long
#define NAME_FLIGHT1 'F','L','2','V',FLIGHT1_VERSION_MAJOR,FLIGHT1_VERSION_MINOR,FLIGHT1_VERSION_PATCH

// SPI Configuration
#define IDB0XA1_PIN_SPI_MOSI   (11)
#define IDB0XA1_PIN_SPI_MISO   (12)
#define IDB0XA1_PIN_SPI_SCK    (3)

#define IDB0XA1_PIN_SPI_nCS    (A1)
#define IDB0XA1_PIN_SPI_RESET  (7)
#define IDB0XA1_PIN_SPI_IRQ    (A0)

#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

// Macros for conversion from float to int
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

#define FEATURE_MASK_ACC_EVENTS 0x00000400u

// Distance components
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_15, (0x42 * 2));
STMPE1600DigiOut xshutdown_left(&DEV_I2C, GPIO_14, (0x43 * 2));
STMPE1600DigiOut xshutdown_right(&DEV_I2C, GPIO_15, (0x43 * 2));
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_top(&DEV_I2C, &xshutdown_top);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_left(&DEV_I2C, &xshutdown_left);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_right(&DEV_I2C, &xshutdown_right);

// Gesture structure
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;
Gesture_TAP_1_Data_t gestureTapData;
// Range values
uint16_t distance_top, distance_left, distance_right;

// MEMS sensors
LSM6DSOSensor AccGyr(&DEV_I2C);
LIS2DW12Sensor Acc2(&DEV_I2C);
LIS2MDLSensor Mag(&DEV_I2C);
LPS22HHSensor PressTemp(&DEV_I2C);
HTS221Sensor HumTemp(&DEV_I2C);
STTS751Sensor Temp(&DEV_I2C);

// STRING UUIDs
const char *uuidSensorService =   "00000000-0001-11e1-9ab4-0002a5d5c51b";
const char *uuidConfigService =   "00000000-000f-11e1-9ab4-0002a5d5c51b";
const char *uuidPressChar =       "00100000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidHumChar =         "00080000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidTempChar =        "00040000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidAccChar =         "00800000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidGyroChar =        "00400000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidMagChar =         "00200000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidProxChar =        "02000000-0001-11e1-ac36-0002a5d5c51b";
const char *uuidGestureChar =     "00000004-0001-11e1-ac36-0002a5d5c51b";
const char *uuidAccEventChar =    "00000400-0001-11e1-ac36-0002a5d5c51b";
const char *uuidConfigChar =      "00000002-000f-11e1-ac36-0002a5d5c51b";

#define LEN_DISTANCE 4
#define LEN_GESTURE 3
#define LEN_MAG 8
#define LEN_GYRO 8
#define LEN_ACC 8
#define LEN_ACCEVENT 5      // This might become 3 or 4, make sure to check!
#define LEN_PRESS 6
#define LEN_HUM 4
#define LEN_TEMP 4
#define LEN_CFG 20

// ABLE services and characteristics
BLEService sensorService(uuidSensorService);
BLEService configService(uuidConfigService);

BLECharacteristic distanceC(uuidProxChar, BLERead | BLENotify, LEN_DISTANCE);
BLECharacteristic gestureC(uuidGestureChar, BLERead | BLENotify, LEN_GESTURE);

BLECharacteristic pressC(uuidPressChar, BLERead | BLENotify, LEN_PRESS);
BLECharacteristic humC(uuidHumChar, BLERead | BLENotify, LEN_HUM);
BLECharacteristic tempC(uuidTempChar, BLERead | BLENotify, LEN_TEMP);

BLECharacteristic accC(uuidAccChar, BLERead | BLENotify, LEN_ACC);
BLECharacteristic accEventC(uuidAccEventChar, BLERead | BLENotify, LEN_ACCEVENT);
BLECharacteristic gyroC(uuidGyroChar, BLERead | BLENotify, LEN_GYRO);
BLECharacteristic magC(uuidMagChar, BLERead | BLENotify, LEN_MAG);

BLECharacteristic configC(uuidConfigChar, BLENotify | BLEWrite, LEN_CFG);

// Class for bluetooth communication and services
class Flight1Service {
  public:

    uint16_t globalSteps = 0;
    bool shortMode = false;

    Flight1Service(void)
    {
    }

    // Initializes the sensor with the default params
    int begin()
    {
      const char BoardName[8] = {NAME_FLIGHT1, 0};
      int ret;
      ret = BLE.begin();

      BLE.setLocalName(BoardName);
      BLE.setAdvertisedService(sensorService);

      uint8_t addr[6] = {0xff};
      BLE.getRandomAddress(addr);

#define FEATURE_MASK 0x02,0xfc,0x04,0x15

      uint8_t data [14] = {0x0d, 0xff, 0x01, 0x80, FEATURE_MASK};

      data[8] = addr[5];
      data[9] = addr[4];
      data[10] = addr[3];
      data[11] = addr[2];
      data[12] = addr[1];
      data[13] = addr[0];

      BLEAdvertisingData adv;
      adv.setRawData(data, 14);
      BLE.setAdvertisingData(adv);

      BLE.advertise();

      Add_HWServW2ST_Service();

      return ret;
    }

    // Update enviromental data
    int Environmental_Update(int32_t Press, uint16_t Hum, int16_t Temp)
    {
      uint8_t pBuff[LEN_PRESS];
      uint8_t hBuff[LEN_HUM];
      uint8_t tBuff[LEN_TEMP];

      STORE_LE_16(pBuff, millis());
      STORE_LE_32(pBuff + 2, (Press));

      STORE_LE_16(hBuff, millis());
      STORE_LE_16(hBuff + 2, (Hum));

      STORE_LE_16(tBuff, millis());
      STORE_LE_16(tBuff + 2, (Temp));

      int ret = 0;
      ret += pressC.writeValue(pBuff, LEN_PRESS);
      ret += humC.writeValue(hBuff, LEN_HUM);
      ret += tempC.writeValue(tBuff, LEN_TEMP);

      return ret;     // Should be 3 (true) on return
    }

    //Update the proximity sensor distance
    int FlightSense_Distance_Update(uint16_t Distance)
    {
      uint8_t buff[LEN_DISTANCE];

      /* To discriminate the long proximity range from 53L1A1*/
      Distance= Distance | (1 << 15);

      STORE_LE_16(buff, millis());
      STORE_LE_16(buff + 2, Distance);

      int err = 0;
      err += distanceC.writeValue(buff, LEN_DISTANCE);

      return err;
    }

    // Update the accelerometer and gyroscope
    // Order of axes: X, Y, Z
    int AccGyroMag_Update(int32_t *Accel, int32_t *Gyros, int32_t *Mag)
    {
      int32_t AXIS_X;
      int32_t AXIS_Y;
      int32_t AXIS_Z;

      uint8_t aBuff[LEN_ACC];
      uint8_t gBuff[LEN_GYRO];
      uint8_t mBuff[LEN_MAG];

      STORE_LE_16(aBuff, millis());
      STORE_LE_16(aBuff + 2, Accel[0]);
      STORE_LE_16(aBuff + 4, Accel[1]);
      STORE_LE_16(aBuff + 6, Accel[2]);

      STORE_LE_16(mBuff, millis());
      STORE_LE_16(mBuff + 2, Mag[0]);
      STORE_LE_16(mBuff + 4, Mag[1]);
      STORE_LE_16(mBuff + 6, Mag[2]);

      AXIS_X = (Gyros[0]) / 100;
      AXIS_Y = (Gyros[1]) / 100;
      AXIS_Z = (Gyros[2]) / 100;

      STORE_LE_16(gBuff, millis());
      STORE_LE_16(gBuff + 2, AXIS_X);
      STORE_LE_16(gBuff + 4, AXIS_Y);
      STORE_LE_16(gBuff + 6, AXIS_Z);

      int ret = 0;
      ret += gyroC.writeValue(gBuff, LEN_GYRO);
      ret += accC.writeValue(aBuff, LEN_ACC);
      ret += magC.writeValue(mBuff, LEN_MAG);

      return ret;
    }

    // Update the gesture recognition
    int Gestures_Update(uint8_t gest_code)
    {
      uint8_t buff[LEN_GESTURE];

      STORE_LE_16(buff, millis());
      buff[2] = gest_code;

      int ret = 0;
      ret += gestureC.writeValue(buff, LEN_GESTURE);

      return ret;
    }

    int AccEvent_Notify(uint16_t steps, uint8_t event)
    {
      uint8_t buff_2[2 + 1];    // Only event
      uint8_t buff_4[5];        // Event and pedometer

      int ret = 0;

      if (shortMode) {
        STORE_LE_16(buff_4, millis());
        buff_2[2] = event;
        ret += accEventC.writeValue(buff_2, 2 + 1);
      } else {
        STORE_LE_16(buff_4, millis());
        buff_4[2] = event;
        STORE_LE_16(buff_4 + 3, steps);
        ret += accEventC.writeValue(buff_4, LEN_ACCEVENT);
      }
      return ret;
    }

    int Config_Notify(uint8_t Feature [4], uint8_t Command, uint8_t data)
    {
      uint8_t buff[2 + 4 + 1 + 1];
      int ret = 0;

      STORE_LE_16(buff, millis());
      buff[2] = Feature[0];
      buff[3] = Feature[1];
      buff[4] = Feature[2];
      buff[5] = Feature[3];
      buff[6] = Command;
      buff[7] = data;

      ret = configC.writeValue(buff, 2 + 4 + 1 + 1);

      return ret;
    }

  private:

    // Add the services
    void Add_HWServW2ST_Service(void)
    {
      sensorService.addCharacteristic(gestureC);
      sensorService.addCharacteristic(distanceC);
      sensorService.addCharacteristic(pressC);
      sensorService.addCharacteristic(humC);
      sensorService.addCharacteristic(tempC);
      sensorService.addCharacteristic(accC);
      sensorService.addCharacteristic(accEventC);
      sensorService.addCharacteristic(gyroC);
      sensorService.addCharacteristic(magC);

      configService.addCharacteristic(configC);

      BLE.addService(sensorService);
      BLE.addService(configService);

      return;
    }
};

Flight1Service Flight1;

void enableAllFunc()
{
  AccGyr.Enable_Pedometer();
  AccGyr.Enable_Tilt_Detection(LSM6DSO_INT1_PIN);
  AccGyr.Enable_Free_Fall_Detection(LSM6DSO_INT1_PIN);
  AccGyr.Enable_Single_Tap_Detection(LSM6DSO_INT1_PIN);
  AccGyr.Enable_Double_Tap_Detection(LSM6DSO_INT1_PIN);
  AccGyr.Enable_6D_Orientation(LSM6DSO_INT1_PIN);
  AccGyr.Step_Counter_Reset();
}

void disableAllFunc()
{
  AccGyr.Disable_Pedometer();
  AccGyr.Disable_Tilt_Detection();
  AccGyr.Disable_Free_Fall_Detection();
  AccGyr.Disable_Single_Tap_Detection();
  AccGyr.Disable_Double_Tap_Detection();
  AccGyr.Disable_6D_Orientation();
}

// Setup distance sensors for gesture detection
void SetupSingleShot(VL53L1X_X_NUCLEO_53L1A1 *sensor)
{
  int status;

  //Change distance mode to short range
  status = sensor->VL53L1X_SetDistanceMode(1);
  if (status) {
    SerialPort.println("SetDistanceMode failed");
  }

  //Change timing budget to 20 ms
  status = sensor->VL53L1X_SetTimingBudgetInMs(20);
  if (status) {
    SerialPort.println("SetMeasurementTimingBudgetMicroSeconds failed");
  }
  status = sensor->VL53L1X_SetInterMeasurementInMs(20);
  if (status) {
    SerialPort.println("SetInterMeasurementPeriodMilliSeconds failed");
  }

}

void INT1Event_cb()
{
  mems_event = 1;
}

void INT2Event_cb()
{
  mems_event = 1;
}



void configCB(BLEDevice unused1, BLECharacteristic unused2)
{
  uint8_t buf[LEN_CFG];
  (void)unused1;
  (void)unused2;

  configC.readValue(buf, LEN_CFG);
  char command = buf[4];
  uint8_t data = buf[5];
  uint8_t signalEvents [4] = {0x00, 0x00, 0x04, 0x00};

  if (!memcmp(buf, signalEvents, 4) && data) {
    enableAllFunc();
    switch (command) {
      case 'o':
        Flight1.shortMode = true;
        break;
      default:
        Flight1.shortMode = false;
        break;
    }
  } else {
    disableAllFunc();
  }

  // Either way, respond by repeating the command
  uint8_t feature [4] = {0x00};
  memcpy(feature, buf, 4);
  Flight1.Config_Notify(feature, buf[4], buf[5]);
}

int gestureGuard = 0;

bool envEnable = false;
bool proxEnable = false;
bool eventEnable = false;
bool accEnable = false;
bool gyroEnable = false;
bool magEnable = false;

void setup()
{
  SerialPort.begin(115200);
  DEV_I2C.begin();

  pinMode(LED_BUILTIN, OUTPUT); //D13 LED

  //Interrupts.
  attachInterrupt(INT_1, INT1Event_cb, RISING);
  attachInterrupt(INT_2, INT2Event_cb, RISING);

  if (!Flight1.begin()) {
    SerialPort.println("Bluetooth services configuration error!");
    while (1);
  }
  SerialPort.println("Bluetooth configuration done!");

  // Create VL53L1X top component.
  sensor_vl53l1x_top.begin();

  // Switch off VL53L1X top component.
  sensor_vl53l1x_top.VL53L1X_Off();

  // Create (if present) VL53L1X left component.
  sensor_vl53l1x_left.begin();

  //Switch off (if present) VL53L1X left component.
  sensor_vl53l1x_left.VL53L1X_Off();

  // Create (if present) VL53L1X right component.
  sensor_vl53l1x_right.begin();

  // Switch off (if present) VL53L1X right component.
  sensor_vl53l1x_right.VL53L1X_Off();

  //Initialize the sensor
  sensor_vl53l1x_top.InitSensor(0x10);
  sensor_vl53l1x_left.InitSensor(0x12);
  sensor_vl53l1x_right.InitSensor(0x14);

  //Change Distance mode and timings
  SetupSingleShot(&sensor_vl53l1x_top);
  SetupSingleShot(&sensor_vl53l1x_left);
  SetupSingleShot(&sensor_vl53l1x_right);


  //Top sensor should be in long distance mode
  sensor_vl53l1x_top.VL53L1X_SetDistanceMode(2);

  // Initialize VL53L1X gesture library.
  tof_gestures_initDIRSWIPE_1(400, 0, 500, &gestureDirSwipeData);
  tof_gestures_initTAP_1(&gestureTapData);

  //Start measurement
  sensor_vl53l1x_top.VL53L1X_StartRanging();
  sensor_vl53l1x_left.VL53L1X_StartRanging();
  sensor_vl53l1x_right.VL53L1X_StartRanging();

  //Setup MEMS sensors
  Temp.begin();
  Temp.Enable();
  Acc2.begin();
  Acc2.Enable_X();
  HumTemp.begin();
  HumTemp.Enable();
  PressTemp.begin();
  PressTemp.Enable();
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Set_X_ODR(4.0f);
  AccGyr.Enable_G();
  Mag.begin();
  Mag.Enable();

  configC.setEventHandler(BLEWritten, configCB);
}

void loop()
{

  int status;
  uint8_t ready = 0;
  uint16_t distance;
  int32_t decPart, intPart;
  int32_t PressToSend = 0;
  uint16_t HumToSend = 0;
  int16_t TempToSend = 0;
  int gesture_code;
  int left_done = 0;
  int right_done = 0;
  uint8_t NewDataReady = 0;
  uint8_t RangeStatus;

  BLE.poll();

  envEnable = pressC.subscribed() || tempC.subscribed() || humC.subscribed();
  accEnable = accC.subscribed();
  gyroEnable = gyroC.subscribed();
  magEnable = magC.subscribed();
  eventEnable = accEventC.subscribed();
  proxEnable = distanceC.subscribed() || gestureC.subscribed();

  if (envEnable) {
    //Get enviroment data
    float humidity, temperature, pressure;
    HumTemp.GetHumidity(&humidity);
    Temp.GetTemperature(&temperature);
    PressTemp.GetPressure(&pressure);
    MCR_BLUEMS_F2I_2D(pressure, intPart, decPart);
    PressToSend = intPart * 100 + decPart;
    MCR_BLUEMS_F2I_1D(humidity, intPart, decPart);
    HumToSend = intPart * 10 + decPart;
    MCR_BLUEMS_F2I_1D(temperature, intPart, decPart);
    TempToSend = intPart * 10 + decPart;
  }

  int32_t accelerometer[3];
  int32_t gyroscope[3];
  int32_t magnetometer[3];

  if (accEnable) {
    // Read accelerometer
    AccGyr.Get_X_Axes(accelerometer);
  }
  if (gyroEnable) {
    // Read gyroscope
    AccGyr.Get_G_Axes(gyroscope);
  }
  if (magEnable) {
    // Read magnetometer
    Mag.GetAxes(magnetometer);
  }

  if (mems_event) {
    mems_event = 0;
    LSM6DSO_Event_Status_t Astatus;
    AccGyr.Get_X_Event_Status(&Astatus);
    uint8_t stat = 0;

    if (Astatus.StepStatus) {
      uint16_t step_count = 0;
      AccGyr.Get_Step_Count(&step_count);
      FLIGHT1_PRINTF("Step %d\n", step_count);

      Flight1.globalSteps = step_count;
    }
    if (Astatus.FreeFallStatus) {
      FLIGHT1_PRINTF("Free fall\n");
      stat = stat | 0x10u;
    }

    if (Astatus.TapStatus) {
      FLIGHT1_PRINTF("Single tap\n");
      stat = stat | 0x20u;
    }

    if (Astatus.DoubleTapStatus) {
      FLIGHT1_PRINTF("Double tap\n");
      stat = stat | 0x40u;
    }

    if (Astatus.TiltStatus) {
      FLIGHT1_PRINTF("Tilt\n");
      stat = stat | 0x08u;
    }

    if (Astatus.D6DOrientationStatus) {
      FLIGHT1_PRINTF("6D Interrupt\n");
      uint8_t xl = 0;
      uint8_t xh = 0;
      uint8_t yl = 0;
      uint8_t yh = 0;
      uint8_t zl = 0;
      uint8_t zh = 0;
      uint8_t OrientationResult = 0;
      AccGyr.Get_6D_Orientation_XL(&xl);
      AccGyr.Get_6D_Orientation_XH(&xh);
      AccGyr.Get_6D_Orientation_YL(&yl);
      AccGyr.Get_6D_Orientation_YH(&yh);
      AccGyr.Get_6D_Orientation_ZL(&zl);
      AccGyr.Get_6D_Orientation_ZH(&zh);
      if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0) {
        OrientationResult = 0x04u;
      } else if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
        OrientationResult = 0x01u;
      } else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0) {
        OrientationResult = 0x03u;
      } else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
        OrientationResult = 0x02u;
      } else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1) {
        OrientationResult = 0x05u;
      } else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0) {
        OrientationResult = 0x06u;
      }
      stat = stat | OrientationResult;
    }

    if (Astatus.WakeUpStatus) {
      FLIGHT1_PRINTF("Wake Up\n");
      stat = stat | 0x80u;
    }
    Flight1.AccEvent_Notify(Flight1.globalSteps, stat);
  }

  if (proxEnable) {
    //Get top sensor distance and transmit
    do {
      sensor_vl53l1x_top.VL53L1X_CheckForDataReady(&ready);
    } while (!ready);

    status = sensor_vl53l1x_top.VL53L1X_GetRangeStatus(&RangeStatus);
    status = sensor_vl53l1x_top.VL53L1X_GetDistance(&distance);

    if (status == VL53L1X_ERROR_NONE) {
      Flight1.FlightSense_Distance_Update(distance);
    }

    //Clear interrupt
    status = sensor_vl53l1x_top.VL53L1X_ClearInterrupt();

    distance = (RangeStatus == 0 && distance < 1400) ? distance : 1400;

    // Launch gesture detection algorithm.
    gesture_code = tof_gestures_detectTAP_1(distance, &gestureTapData);

    // Check the result of the gesture detection algorithm.
    switch (gesture_code) {
      case GESTURES_SINGLE_TAP:
        Flight1.Gestures_Update(1);
        gestureGuard = 0;
        break;
      default:
        // Do nothing
        break;
    }


    // Wait for data ready
    do {
      //if left not done
      if (left_done == 0) {
        NewDataReady = 0;
        //check measurement data ready
        int status = sensor_vl53l1x_left.VL53L1X_CheckForDataReady(&NewDataReady);

        if (status) {
          SerialPort.println("GetMeasurementDataReady left sensor failed");
        }
        //if ready
        if (NewDataReady) {
          //get status
          status = sensor_vl53l1x_left.VL53L1X_GetRangeStatus(&RangeStatus);
          if (status) {
            SerialPort.println("GetRangeStatus left sensor failed");
          }

          //if distance < 1.3 m
          if (RangeStatus == 0) {
            // we have a valid range.
            status = sensor_vl53l1x_left.VL53L1X_GetDistance(&distance_left);
            if (status) {
              SerialPort.println("GetDistance left sensor failed");
            }
          } else {
            distance_left = 1400;   //default distance
          }

          //restart measurement
          status = sensor_vl53l1x_left.VL53L1X_ClearInterrupt();
          if (status) {
            SerialPort.println("Restart left sensor failed");
          }

          left_done = 1 ;
        }
      }

      //if right not done
      if (right_done == 0) {
        NewDataReady = 0;
        //check measurement data ready
        int status = sensor_vl53l1x_right.VL53L1X_CheckForDataReady(&NewDataReady);

        if (status) {
          SerialPort.println("GetMeasurementDataReady right sensor failed");
        }
        //if ready
        if (NewDataReady) {
          //get status
          status = sensor_vl53l1x_right.VL53L1X_GetRangeStatus(&RangeStatus);
          if (status) {
            SerialPort.println("GetRangeStatus right sensor failed");
          }
          //if distance < 1.3 m
          if (RangeStatus == 0) {
            // we have a valid range.
            status = sensor_vl53l1x_right.VL53L1X_GetDistance(&distance_right);
            if (status) {
              SerialPort.println("GetDistance right sensor failed");
            }
          } else {
            distance_right = 1400;   //default distance
          }

          //restart measurement
          status = sensor_vl53l1x_right.VL53L1X_ClearInterrupt();
          if (status) {
            SerialPort.println("Restart right sensor failed");
          }

          right_done = 1 ;
        }
      }
    } while (left_done == 0 || right_done == 0);


    // Launch gesture detection algorithm.
    gesture_code = tof_gestures_detectDIRSWIPE_1(distance_left, distance_right, &gestureDirSwipeData);

    // Check the result of the gesture detection algorithm.
    switch (gesture_code) {
      case GESTURES_SWIPE_LEFT_RIGHT:
        Flight1.Gestures_Update(3);
        gestureGuard = 0;
        break;
      case GESTURES_SWIPE_RIGHT_LEFT:
        Flight1.Gestures_Update(2);
        gestureGuard = 0;
        break;
      default:
        if (!gestureGuard) {
          Flight1.Gestures_Update(0);
          gestureGuard = 1;
        }
        break;
    }
  }

  //Send all mems sensors data
  if (envEnable) {
    Flight1.Environmental_Update(PressToSend, HumToSend, TempToSend);
  }

  if (gyroEnable || accEnable || magEnable) {
    Flight1.AccGyroMag_Update(accelerometer, gyroscope, magnetometer);
  }
}
