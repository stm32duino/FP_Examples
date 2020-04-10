/**
 ******************************************************************************
 * @file    IKS01A3_S2LP_P2P_Demo.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    April 2020
 * @brief   Arduino demo application for the STMicrolectronics
 *          X-NUCLEO-IKS01A3 and X-NUCLEO-S2868A1 or X-NUCLEO-S2868A2
 *          or X-NUCLEO-S2915A1
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

#include "SPI.h"
#include "M95640R.h"
#include "S2LP.h"
#include "LSM6DSOSensor.h"
#include "LIS2DW12Sensor.h"
#include "LIS2MDLSensor.h"
#include "LPS22HHSensor.h"
#include "STTS751Sensor.h"
#include "HTS221Sensor.h"

#define SerialPort Serial
#define DEV_I2C Wire
#define PA_CSD_PIN A0
#define PA_CPS_PIN A2
#define PA_CTX_PIN A3

SPIClass *devSPI;
M95640R *myM95640R;
S2LP *myS2LP;
// Components
LSM6DSOSensor *AccGyr;
LIS2DW12Sensor *Acc2;
LIS2MDLSensor *Mag;
LPS22HHSensor *PressTemp;
HTS221Sensor *HumTemp;
STTS751Sensor *Temp3;
volatile uint8_t receive_packet = 0;
const int buttonPin = PC13; // set buttonPin to digital pin PC13 */
int pushButtonState = LOW;

static uint8_t send_buf[FIFO_SIZE] ={0};
static uint8_t read_buf[FIFO_SIZE] ={0};

void callback_func(void);
void recv_data(void);
void blink_led(void);
uint8_t eeprom_identification(void);
void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender);
uint32_t get_frequency_band(uint8_t s_RfModuleBand);


/* Setup ---------------------------------------------------------------------*/

void setup() {
  uint32_t s_frequency = 868000000;
  uint32_t s_RfXtalFrequency = 50000000;
  PAInfo_t paInfo;

  memset(&paInfo, 0, sizeof(PAInfo_t));

  // Setup CSD, CPS and CTX pins
  paInfo.paSignalCSD_MCU = PA_CSD_PIN;
  paInfo.paSignalCPS_MCU = PA_CPS_PIN;
  paInfo.paSignalCTX_MCU = PA_CTX_PIN;

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Button
  pinMode(buttonPin, INPUT);
  pushButtonState = (digitalRead(buttonPin)) ?  LOW : HIGH;

  // Put S2-LP in Shutdown
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialize SPI
  devSPI = new SPIClass(D11, D12, D3);
  devSPI->begin();

  AccGyr = new LSM6DSOSensor (&DEV_I2C);
  AccGyr->Enable_X();
  AccGyr->Enable_G();
  Acc2 = new LIS2DW12Sensor (&DEV_I2C);
  Acc2->Enable_X();
  Mag = new LIS2MDLSensor (&DEV_I2C);
  Mag->Enable();
  PressTemp = new LPS22HHSensor(&DEV_I2C);
  PressTemp->Enable();
  HumTemp = new HTS221Sensor (&DEV_I2C);
  HumTemp->Enable();
  Temp3 = new STTS751Sensor (&DEV_I2C);
  Temp3->Enable();

  // Initialize M95640-R
  myM95640R = new M95640R(devSPI, D5);
  myM95640R->begin();

  // Read X-NUCLEO-S2915A1 EEPROM
  if(eeprom_identification())
  {
    SerialPort.println("EEPROM present");
    read_eeprom_content(&s_frequency, &s_RfXtalFrequency, &paInfo.paRfRangeExtender);
  } else
  {
    SerialPort.println("EEPROM not present");
  }

  // Initialize S2-LP
  myS2LP = new S2LP(devSPI, A1, D7, A5, s_frequency, s_RfXtalFrequency, paInfo);
  myS2LP->begin();
  myS2LP->attachS2LPReceive(callback_func);
}

/* Loop ----------------------------------------------------------------------*/

void loop() {
  if(digitalRead(buttonPin) == pushButtonState)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the button is released */
    while (digitalRead(buttonPin) == pushButtonState);

    /* Debouncing */
    HAL_Delay(50);

    // Read humidity and temperature.
    float humidity = 0, temperature = 0;
    HumTemp->GetHumidity(&humidity);
    HumTemp->GetTemperature(&temperature);

    // Read pressure and temperature.
    float pressure = 0, temperature2 = 0;
    PressTemp->GetPressure(&pressure);
    PressTemp->GetTemperature(&temperature2);

    //Read temperature
    float temperature3 = 0;
    Temp3->GetTemperature(&temperature3);

    // Read accelerometer and gyroscope.
    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr->Get_X_Axes(accelerometer);
    AccGyr->Get_G_Axes(gyroscope);

    //Read accelerometer
    int32_t accelerometer2[3];
    Acc2->Get_X_Axes(accelerometer2);

    //Read magnetometer
    int32_t magnetometer[3];
    Mag->GetAxes(magnetometer);

    memcpy((void *)&send_buf[0], (void *)&humidity, sizeof(float));
    memcpy((void *)&send_buf[sizeof(float)], (void *)&temperature, sizeof(float));
    memcpy((void *)&send_buf[2*sizeof(float)], (void *)&pressure, sizeof(float));
    memcpy((void *)&send_buf[3*sizeof(float)], (void *)&temperature2, sizeof(float));
    memcpy((void *)&send_buf[4*sizeof(float)], (void *)&temperature3, sizeof(float));
    memcpy((void *)&send_buf[5*sizeof(float)], (void *)accelerometer, 3*sizeof(int32_t));
    memcpy((void *)&send_buf[5*sizeof(float) + 3*sizeof(int32_t)], (void *)gyroscope, 3*sizeof(int32_t));
    memcpy((void *)&send_buf[5*sizeof(float) + 6*sizeof(int32_t)], (void *)accelerometer2, 3*sizeof(int32_t));
    memcpy((void *)&send_buf[5*sizeof(float) + 9*sizeof(int32_t)], (void *)magnetometer, 3*sizeof(int32_t));

    if(!myS2LP->send(send_buf, (5*sizeof(float) + 12*sizeof(int32_t)), 0x44, true))
    {
      /* Blink LED */
      blink_led();

      /* Print message */
      SerialPort.print("Transmitted ");
      SerialPort.print((5*sizeof(float) + 12*sizeof(int32_t)));
      SerialPort.println(" bytes successfully");
    } else
    {
      SerialPort.println("Error in transmission");
    }
  }

  if(receive_packet)
  {
    receive_packet = 0;
    recv_data();

    /* Blink LED */
    blink_led();
  }
}

void recv_data(void)
{
  uint8_t data_size = myS2LP->getRecvPayloadLen();

  myS2LP->read(read_buf, data_size);

  // Output data.
  SerialPort.print("| Hum[%]: ");
  SerialPort.print(*((float *)(&read_buf[0])), 2);
  SerialPort.print(" | Temp[C]: ");
  SerialPort.print(*((float *)(&read_buf[sizeof(float)])), 2);
  SerialPort.print(" | Pres[hPa]: ");
  SerialPort.print(*((float *)(&read_buf[2*sizeof(float)])), 2);
  SerialPort.print(" | Temp2[C]: ");
  SerialPort.print(*((float *)(&read_buf[3*sizeof(float)])), 2);
  SerialPort.print(" | Temp3[C]: ");
  SerialPort.print(*((float *)(&read_buf[4*sizeof(float)])), 2);
  SerialPort.print(" | Acc[mg]: ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 2*sizeof(int32_t)])));
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 3*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 4*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 5*sizeof(int32_t)])));
  SerialPort.print(" | Acc2[mg]: ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 6*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 7*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 8*sizeof(int32_t)])));
  SerialPort.print(" | Mag[mGauss]: ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 9*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 10*sizeof(int32_t)])));
  SerialPort.print(" ");
  SerialPort.print(*((int32_t *)(&read_buf[5*sizeof(float) + 11*sizeof(int32_t)])));
  SerialPort.println(" |");
}

void callback_func(void)
{
  receive_packet = 1;
}

void blink_led(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}

uint8_t eeprom_identification(void)
{
  uint8_t status=0;

  status = myM95640R->EepromStatus();

  if((status&0xF0) == EEPROM_STATUS_SRWD) {
    /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
    status=1;
  }
  else
  {
    myM95640R->EepromWriteEnable();
    delay(10);
    /* Else the bit may be not set (first time we see this EEPROM), try to set it*/
    status = myM95640R->EepromSetSrwd();
    delay(10);
    /*check again*/
    status = myM95640R->EepromStatus();

    if((status&0xF0) == EEPROM_STATUS_SRWD) { // 0xF0 mask [SRWD 0 0 0]
      /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
      status=1;
    }
    else
    {
      /* Else no EEPROM is present */
      status = 0;
    }
  }

  return status;
}

void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender)
{
  float foffset = 0;
  uint8_t tmpBuffer[32];
  uint8_t s_RfModuleBand = 0;
  int32_t xtal_comp_value = 0;

  /* Read the EEPROM */
  myM95640R->EepromRead(0x0000, 32, tmpBuffer);

  /* Data in EEPROM is not valid ... */
  if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
    *s_RfXtalFrequency = 50000000;

    /* If EEPROM fails, set no EXT_PA by default */
    *s_RfRangeExtender = RANGE_EXT_NONE;

    return;
  }

  switch(tmpBuffer[1]) {
  case 0:
    *s_RfXtalFrequency = 24000000;
    break;
  case 1:
    *s_RfXtalFrequency = 25000000;
    break;
  case 2:
    *s_RfXtalFrequency = 26000000;
    break;
  case 3:
    *s_RfXtalFrequency = 48000000;
    break;
  case 4:
    *s_RfXtalFrequency = 50000000;
    break;
  case 5:
    *s_RfXtalFrequency = 52000000;
    break;
  default:
    *s_RfXtalFrequency = 50000000;
    break;
  }

  s_RfModuleBand = tmpBuffer[3];

  myM95640R->EepromRead(0x0021,4,tmpBuffer);

  for(uint8_t i=0;i<4;i++)
  {
    ((uint8_t*)&foffset)[i]=tmpBuffer[3-i];
  }

  xtal_comp_value = 0;

  /* foffset is a value measured during manufacturing as follows:
  *
  * foffset = fnominal-fmeasured.
  * To compensate such value it should be reported to xtal freq
  * and then subtracted
  *
  */
  if (foffset != 0xFFFFFFFF) {
    uint32_t frequency = get_frequency_band(s_RfModuleBand);

    if (frequency != 0)
    {
    uint32_t xtal_frequency = *s_RfXtalFrequency;

    /* This is the value to be added to the xtal nominal value
    to compensate the xtal offset */
    xtal_comp_value = (int32_t) ((xtal_frequency*(-foffset))/frequency);

      *s_frequency = frequency;
    }
  }

  *s_RfXtalFrequency = *s_RfXtalFrequency + xtal_comp_value;

  *s_RfRangeExtender = (RangeExtType)tmpBuffer[5];
}

uint32_t get_frequency_band(uint8_t s_RfModuleBand)
{
  uint32_t frequency = 0;
  const uint32_t band_frequencies[] = {
    169000000,
    315000000,
    433000000,
    868000000,
    915000000,
    450000000
  };

  if (s_RfModuleBand < (sizeof(band_frequencies)/sizeof(uint32_t))) {
    frequency = band_frequencies[s_RfModuleBand];
  }

  return frequency;
}
