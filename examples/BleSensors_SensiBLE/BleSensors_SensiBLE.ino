/*

 BleSensors_SensiBLE

 This sketch provides an example of using the following SensiBLE features:
  - Environment sensors (humidity, temperature, pressure)
  - Motion sensors (accelerometer and gyroscope)
  - BLE communication
  
 You can use BlueNRG application  provided by STMICROELECTRONICS to test this sketch. 
 The name of bluetooth device is "BlueNRG". Scan devices with the application and
 connect your device.

 Updating periods:
 Accelerometer and gyroscope values via BLE - 100 ms
 Environmental sensors - 1 second
 Printing values to the serial therminal - 1 second

 More information about SensiBLE at https://www.sensiedge.com/

 */

#include <SPI.h>
#include <SPBTLE_RF.h>
#include <sensor_service.h>

#include <LSM6DS3Sensor.h>
#include <HTS221Sensor.h>
#include <LPS25HBSensor.h>

#define PIN_SENSIBLE_LED_GRN    (13)
#define PIN_BLE_SPI_MOSI   (11)
#define PIN_BLE_SPI_MISO   (12)
#define PIN_BLE_SPI_SCK    (3)

#define PIN_BLE_SPI_nCS    (A1)
#define PIN_BLE_SPI_RESET  (7)
#define PIN_BLE_SPI_IRQ    (A0)

#define PIN_BLE_LED    (0xFF)

#define SerialPort  Serial
#define DEV_I2C     Wire

// Configure BTLE_SPI
SPIClass BTLE_SPI(PIN_BLE_SPI_MOSI, PIN_BLE_SPI_MISO, PIN_BLE_SPI_SCK);

// Configure BTLE pins
SPBTLERFClass BTLE(&BTLE_SPI, PIN_BLE_SPI_nCS, PIN_BLE_SPI_IRQ, PIN_BLE_SPI_RESET, PIN_BLE_LED);

const char *name = "BlueNRG";
uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};

// Pointers for sensors objects
LSM6DS3Sensor  *AccGyr; 
HTS221Sensor   *HumTemp; 
LPS25HBSensor  *PressTemp;

// Variables for sensors data
float humidity, temperature;
float pressure, temperatureP;
int32_t accelerometer[3];
int32_t gyroscope[3];
AxesRaw_t axes_data;

// Variables for update periods calculation
uint32_t prevUpdMsec = 0;
uint32_t prevLedMsec = 0;
uint32_t envSensorsSkipCnt = 0;
uint32_t printSkipCnt = 0;

void setup() {
  int ret;

  pinMode(PIN_SENSIBLE_LED_GRN, OUTPUT);

  SerialPort.begin(115200);

  if(BTLE.begin() == SPBTLERF_ERROR)
  {
    SerialPort.println("Bluetooth module configuration error!");
    while(1);
  }

  if(SensorService.begin(name, SERVER_BDADDR))
  {
    SerialPort.println("Sensor service configuration error!");
    while(1);
  }

  /* Configure the User Button in GPIO Mode */
  pinMode(USER_BTN, INPUT);

  ret = SensorService.Add_Acc_Service();

  if(ret == BLE_STATUS_SUCCESS)
    SerialPort.println("Acc service added successfully.");
  else
    SerialPort.println("Error while adding Acc service.");

  ret = SensorService.Add_Environmental_Sensor_Service();

  if(ret == BLE_STATUS_SUCCESS)
    SerialPort.println("Environmental Sensor service added successfully.");
  else
    SerialPort.println("Error while adding Environmental Sensor service.");

  /* Instantiate Timer Service with two characteristics:
   * - seconds characteristic (Readable only)
   * - minutes characteristics (Readable and Notifiable )
   */
  ret = SensorService.Add_Time_Service();

  if(ret == BLE_STATUS_SUCCESS)
    SerialPort.println("Time service added successfully.");
  else
    SerialPort.println("Error while adding Time service.");

  // Initialize I2C bus.
  DEV_I2C.begin(); 
  SerialPort.println("Init I2C");

  // Initlialize Components.
  AccGyr = new LSM6DS3Sensor(&DEV_I2C, LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW);
  AccGyr->Enable_X();
  AccGyr->Enable_G(); 
  SerialPort.println("Init Acc&Gyr");

  HumTemp = new HTS221Sensor (&DEV_I2C);
  HumTemp->Enable(); 
  SerialPort.println("Init Hum&Temp");

  PressTemp = new LPS25HBSensor (&DEV_I2C);
  PressTemp->Enable();
  SerialPort.println("Init Pressure&Temp");
}

void loop() {

  Led_Blink();
  
  BTLE.update();

  // Blink LED depending on the connection state
  if(SensorService.isConnected() != TRUE)
  {
    // Keep the Bluetooth module in discoverable mode
    SensorService.setConnectable();
  }

  // Update sensors values
  if((millis() - prevUpdMsec) >= 100)
  {
    prevUpdMsec = millis();
    // Update environment data every N cycles
    if(++envSensorsSkipCnt >= 10)
    {
      envSensorsSkipCnt = 0;
      // Read humidity and temperature.
      HumTemp->GetHumidity(&humidity);
      HumTemp->GetTemperature(&temperature); 

      // Read pressure and temperature.
      PressTemp->GetPressure(&pressure);
      PressTemp->GetTemperature(&temperatureP);
    }

    // Read accelerometer and gyroscope
    AccGyr->Get_X_Axes(accelerometer);
    AccGyr->Get_G_Axes(gyroscope); 

    if(++printSkipCnt >= 10)
    {
      printSkipCnt = 0;
      // Print sensors values to serial port
      Print_Sensors();
    }

    // Update BLE characteristics
    if(SensorService.isConnected() == TRUE)
    {
      // Update time
      SensorService.Update_Time_Characteristics();
      
      // Update environnemental data
      SensorService.Temp_Update((temperature*10 + temperatureP*10)/2);
      SensorService.Press_Update(pressure*100);
      SensorService.Humidity_Update(humidity*10);

      // Update accelerometer data
      axes_data.AXIS_X = accelerometer[0];
      axes_data.AXIS_Y = accelerometer[1];
      axes_data.AXIS_Z = accelerometer[2];
      SensorService.Acc_Update(&axes_data);
    }
  }
}

void Print_Sensors()
{
  // Print HTS221 data
  SerialPort.println("------------------------");
  SerialPort.println("------------------------");
  SerialPort.print("Hum[%]: ");
  SerialPort.println(humidity, 2);
  SerialPort.print("Temp[C] (HTS221): ");
  SerialPort.println(temperature, 2);
  // Print LPS25 data
  SerialPort.print("Pres[hPa]: ");
  SerialPort.println(pressure, 2);
  SerialPort.print("TempP[C] (LPS25): ");
  SerialPort.println(temperatureP, 2);
  // Print accelerometer data
  SerialPort.println("------------------------");
  SerialPort.println("Acc");
  SerialPort.print("X: ");
  SerialPort.println(accelerometer[0]);
  SerialPort.print("Y: ");
  SerialPort.println(accelerometer[1]);
  SerialPort.print("Z: ");
  SerialPort.println(accelerometer[2]);
  // Print gyroscope data
  SerialPort.println("------------------------");
  SerialPort.println("Gyr");
  SerialPort.print("X: ");
  SerialPort.println(gyroscope[0]);
  SerialPort.print("Y: ");
  SerialPort.println(gyroscope[1]);
  SerialPort.print("Z: ");
  SerialPort.println(gyroscope[2]);
}

void Led_Blink()
{
  // Blink LED depending on the connection state
  if(SensorService.isConnected() == TRUE)
  {
    if(millis() - prevLedMsec > 1000)
    {
      prevLedMsec = millis();
      digitalWrite(PIN_SENSIBLE_LED_GRN, HIGH);
      delay(10);
      digitalWrite(PIN_SENSIBLE_LED_GRN, LOW);
    }
  }
  else
  {    
    if(millis() - prevLedMsec > 300)
    {
      prevLedMsec = millis();
      digitalWrite(PIN_SENSIBLE_LED_GRN, HIGH);
      delay(10);
      digitalWrite(PIN_SENSIBLE_LED_GRN, LOW);
    }
  }
}
