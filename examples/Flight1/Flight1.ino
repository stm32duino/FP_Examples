/**
 ******************************************************************************
 * @file    Flight1.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    21 December 2018
 * @brief   Arduino demo application for the STMicrolectronics
 *          X-NUCLEO-IKS01A2 or X-NUCLEO-IKS01A3, X-NUCLEO-53L1A1
 *          and X-NUCLEO-IDB05A1
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "hci.h"
#include "hci_le.h"
#include "hci_const.h"
#include "sm.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gap.h"
#include "ble_status.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "link_layer.h"
#include <SPBTLE_RF.h>


#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>
#include <stmpe1600_class.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>
#include <tof_gestures_TAP_1.h>


//NOTE: In order to use this example with the IKS01A2 board uncomment the
//      USE_IKS01A2 define and comment the USE_IKS01A3 define
#define USE_IKS01A3
//#define USE_IKS01A2


#ifdef USE_IKS01A3
#include <HTS221Sensor.h>
#include <LPS22HHSensor.h>
#include <LIS2DW12Sensor.h>
#include <LIS2MDLSensor.h>
#include <LSM6DSOSensor.h>
#include <STTS751Sensor.h>
#elif defined(USE_IKS01A2)
#include <HTS221Sensor.h>
#include <LPS22HBSensor.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <LSM6DSLSensor.h>
#endif

#define DEV_I2C Wire
#define SerialPort Serial

#define INT_1 4
#define INT_2 5

//Interrupts.
volatile int mems_event = 0;

volatile uint8_t AccGyroMag_Enable = 0;
volatile uint8_t Enviroment_Enable = 0;
volatile uint8_t Distance_Enable = 0;
volatile uint8_t Gestures_Enable = 0;

/* Package Version only numbers 0->9 */
#define FLIGHT1_VERSION_MAJOR '3'
#define FLIGHT1_VERSION_MINOR '3'
#define FLIGHT1_VERSION_PATCH '0'

//#define DEBUG_MODE

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
/* Define the FLIGHT1 Name MUST be 7 char long */
#define NAME_FLIGHT1 'F','L','1','V',FLIGHT1_VERSION_MAJOR,FLIGHT1_VERSION_MINOR,FLIGHT1_VERSION_PATCH
#define AD_TYPE_COMPLETE_LOCAL_NAME          (0x09)


/*SPI Configuration*/
#define IDB0XA1_PIN_SPI_MOSI   (11)
#define IDB0XA1_PIN_SPI_MISO   (12)
#define IDB0XA1_PIN_SPI_SCK    (3)

SPIClass BTLE_SPI(IDB0XA1_PIN_SPI_MOSI, IDB0XA1_PIN_SPI_MISO, IDB0XA1_PIN_SPI_SCK);

#define IDB0XA1_PIN_SPI_nCS    (A1)
#define IDB0XA1_PIN_SPI_RESET  (7)
#define IDB0XA1_PIN_SPI_IRQ    (A0)

SPBTLERFClass BTLE(&BTLE_SPI, IDB0XA1_PIN_SPI_nCS, IDB0XA1_PIN_SPI_IRQ, IDB0XA1_PIN_SPI_RESET);

/*Macros for UUID*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}

#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PROX_W2ST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x02,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_GESTURE_W2ST_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x04,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

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

/*Macros for conversion from float to int*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

#define FEATURE_MASK_ACC_EVENTS 0x00000400


uint16_t AccEventCharHandle;
uint16_t ConfigCharHandle;
uint16_t EnvironmentalCharHandle;
uint16_t AccGyroMagCharHandle;
uint16_t ProxCharHandle;
uint16_t GestureDetCharHandle;

// Distance components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_top;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_left;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_right;


// Gesture structure.
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;
Gesture_TAP_1_Data_t gestureTapData;
// Range values
uint16_t distance_top, distance_left, distance_right;

//MEMS sensors
#ifdef USE_IKS01A3
HTS221Sensor  *HumTemp;
LPS22HHSensor  *PressTemp;
LSM6DSOSensor *AccGyr;
LIS2DW12Sensor *Acc2;
LIS2MDLSensor *Mag;
STTS751Sensor *Temp;
#elif defined (USE_IKS01A2)
HTS221Sensor  *HumTemp;
LPS22HBSensor  *PressTemp;
LSM6DSLSensor *AccGyr;
LSM303AGR_ACC_Sensor *Acc2;
LSM303AGR_MAG_Sensor *Mag;
#endif


//Callback function
void Flight1_HCI_Event_CB(void *pckt);

//Class for bluetooth communication and services
class Flight1Service
{
public:
   Flight1Service(void)
   {
      StartBlueNRG = FALSE;
      set_connectable= FALSE;
      connected = FALSE;
   }

   /*Initializes the sensor with the default params*/
   int begin()
   {
      const char BoardName[8] = {NAME_FLIGHT1,0};
      int ret;
      uint8_t  hwVersion;
      uint16_t fwVersion;
      HCI_Init();

      /*Attach callback function*/
      attach_HCI_CB(Flight1_HCI_Event_CB);

      BlueNRG_RST();
      /* get the BlueNRG HW and FW versions */
      getBlueNRGVersion(&hwVersion, &fwVersion);

      BlueNRG_RST();

      /*Generate MAC Address*/
      uint8_t data_len_out;
      ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

      ret = aci_gatt_init();
      if(ret)
      {
         FLIGHT1_PRINTF("\r\nGATT_Init failed\r\n");
         goto fail;
      }

      ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

      if(ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("\r\nGAP_Init failed\r\n");
         goto fail;
      }
      ret = hci_le_set_random_address(bdaddr);

      if(ret)
      {
         FLIGHT1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
         goto fail;
      }
      ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                       7/*strlen(BoardName)*/, (uint8_t *)BoardName);
      if(ret)
      {
         FLIGHT1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
         while(1);
      }

      ret=aci_gap_clear_security_database();
      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("\r\nGAP clear security database failed\r\n");
         goto fail;
      }
      ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                         OOB_AUTH_DATA_ABSENT,
                                         NULL, 7, 16,
                                         USE_FIXED_PIN_FOR_PAIRING, PinForParing,
                                         NO_BONDING);
      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("\r\nGAP setting Authentication failed\r\n");
         goto fail;
      }
      /*Print device data*/
      FLIGHT1_PRINTF("SERVER: BLE Stack Initialized \r\n\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n\t\tBoardName= %s\r\n\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n",
                     "IDB05A1",
                     hwVersion,
                     fwVersion>>8,
                     (fwVersion>>4)&0xF,
                     ('a'+(fwVersion&0xF)-1),
                     BoardName,
                     bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

      FLIGHT1_PRINTF("\n");

      /* Set output power level */
      aci_hal_set_tx_power_level(1,4);

      /*Add services*/
      ret = Add_HWServW2ST_Service();
      if(ret == BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("HW      Service W2ST added successfully\r\n");
      }
      else
      {
         FLIGHT1_PRINTF("\r\nError while adding HW Service W2ST\r\n");
      }


      StartBlueNRG = TRUE;
      set_connectable = TRUE;

      return ret;

fail:
      return ret;
   }

   /*Enables the device for the connection*/
   int setConnectable()
   {
      char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_FLIGHT1};
      manuf_data[20] = bdaddr[5];
      manuf_data[21] = bdaddr[4];
      manuf_data[22] = bdaddr[3];
      manuf_data[23] = bdaddr[2];
      manuf_data[24] = bdaddr[1];
      manuf_data[25] = bdaddr[0];

      manuf_data[26] =  PinForParing     &0xFF;
      manuf_data[27] = (PinForParing>>8 )&0xFF;
      manuf_data[28] = (PinForParing>>16)&0xFF;
      manuf_data[29] = (PinForParing>>24)&0xFF;

      manuf_data[16] |= 0x02; /* Proximity*/
      manuf_data[17] |= 0x04; /* One Temperature value*/
      manuf_data[17] |= 0x08; /* Humidity */
      manuf_data[17] |= 0x10; /* Pressure value*/
      manuf_data[17] |= 0x40; /* Gyroscope value*/
      manuf_data[17] |= 0x80; /* Accellerometer value*/
      manuf_data[18] |= 0x04; /* Acc event */
      manuf_data[19] |= 0x04; /* Gesture */

      hci_le_set_scan_resp_data(0,NULL);
      aci_gap_set_discoverable(ADV_IND, 0, 0, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
      aci_gap_update_adv_data(26, manuf_data);
      set_connectable=FALSE;

      return 0;
   }

   /*Update enviromental data (press*/
   tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp)
   {
      tBleStatus ret;
      uint8_t BuffPos;

      uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp1*/];

      STORE_LE_16(buff  ,millis());
      BuffPos=2;

      STORE_LE_32(buff+BuffPos,Press);
      BuffPos+=4;

      STORE_LE_16(buff+BuffPos,Hum);
      BuffPos+=2;

      STORE_LE_16(buff+BuffPos,Temp);
      BuffPos+=2;

      ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating Environmental Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }

   /*Update de proximity sensor distance*/
   tBleStatus FlightSense_Distance_Update(uint16_t Distance)
   {
      tBleStatus ret;
      uint8_t buff[2+2];

      /* To discriminate the long proximity range from 53L1A1*/
      Distance= Distance | (1 << 15);

      STORE_LE_16(buff  ,millis());
      STORE_LE_16(buff+2,Distance);

      ret = aci_gatt_update_char_value(HWServW2STHandle, ProxCharHandle, 0, 2+2,buff);

      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating Distance Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }

   /*Update the accelerometer and gyroscope*/
   tBleStatus AccGyroMag_Update(int32_t *Accel,int32_t *Gyros)
   {
      tBleStatus ret;
      int32_t AXIS_X;
      int32_t AXIS_Y;
      int32_t AXIS_Z;

      uint8_t buff[2+3*2*2];

      STORE_LE_16(buff   ,millis());

      STORE_LE_16(buff+2 ,Accel[0]);
      STORE_LE_16(buff+4 ,Accel[1]);
      STORE_LE_16(buff+6 ,Accel[2]);

      AXIS_X=(Gyros[0])/100;
      AXIS_Y=(Gyros[1])/100;
      AXIS_Z=(Gyros[2])/100;

      STORE_LE_16(buff+8 ,AXIS_X);
      STORE_LE_16(buff+10,AXIS_Y);
      STORE_LE_16(buff+12,AXIS_Z);

      ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*2*2, buff);

      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }

   /*Update the gesture recognition*/
   tBleStatus Gestures_Update(int32_t gest_code)
   {
      tBleStatus ret;

      uint8_t buff[2+ 1];

      STORE_LE_16(buff  ,millis());
      buff[2] = gest_code;

      ret = aci_gatt_update_char_value(HWServW2STHandle, GestureDetCharHandle, 0, 2+1, buff);

      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating Gesture Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }

   tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte)
   {
      tBleStatus ret= BLE_STATUS_SUCCESS;
      uint8_t buff_2[2+2];
      uint8_t buff_3[2+3];

      switch(dimByte)
      {
      case 2:
         STORE_LE_16(buff_2  ,millis());
         STORE_LE_16(buff_2+2,Command);
         ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff_2);
         break;
      case 3:
         STORE_LE_16(buff_3  ,millis());
         buff_3[2]= 0;
         STORE_LE_16(buff_3+3,Command);
         ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+3,buff_3);
         break;
      }

      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating AccEvent_Notify Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }

   tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
   {
      tBleStatus ret;
      uint8_t buff[2+4+1+1];

      STORE_LE_16(buff  ,millis());
      STORE_BE_32(buff+2,Feature);
      buff[6] = Command;
      buff[7] = data;

      ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
      if (ret != BLE_STATUS_SUCCESS)
      {
         FLIGHT1_PRINTF("Error Updating Configuration Char\r\n");
         return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
   }



   /*Public variables*/
   uint8_t set_connectable;
   int connected;
   uint8_t StartBlueNRG;


private:

   /*Add the services*/
   tBleStatus Add_HWServW2ST_Service(void)
   {
      tBleStatus ret;
      int32_t NumberChars = 6;

      uint8_t uuid[16];

      COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
      ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                              1+3*NumberChars,
                              &HWServW2STHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }

      /* Fill the Environmental BLE Characteristc */
      COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
      uuid[14] |= 0x04; /* One Temperature value*/
      EnvironmentalCharSize+=2;

      uuid[14] |= 0x08; /* Humidity */
      EnvironmentalCharSize+=2;

      uuid[14] |= 0x10; /* Pressure value*/
      EnvironmentalCharSize+=4;

      ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                               CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 0, &EnvironmentalCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }

      /*Add gyroscope and accelerometer*/
      COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
      ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                               CHAR_PROP_NOTIFY,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 0, &AccGyroMagCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }

      /*Add proximity*/
      COPY_PROX_W2ST_CHAR_UUID(uuid);
      ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                               CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 0, &ProxCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }

      /* Code for Gesture Detection integration - Start Section */
      COPY_GESTURE_W2ST_CHAR_UUID(uuid);
      ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                               CHAR_PROP_NOTIFY,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 0, &GestureDetCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }

      COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
      ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3, //2+2,
                               CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 1, &AccEventCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }


      COPY_CONFIG_SERVICE_UUID(uuid);
      ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);
      if (ret != BLE_STATUS_SUCCESS)
         goto fail;
      COPY_CONFIG_W2ST_CHAR_UUID(uuid);

      ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                               CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                               ATTR_PERMISSION_NONE,
                               GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                               16, 1, &ConfigCharHandle);

      if (ret != BLE_STATUS_SUCCESS)
      {
         goto fail;
      }


      return BLE_STATUS_SUCCESS;

fail:
      FLIGHT1_PRINTF("Error while adding HW's Characteristcs service.\n");
      return BLE_STATUS_ERROR;
   }





   /*Private variables*/
   uint8_t bdaddr[6];
   uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
   uint32_t PinForParing;
   uint32_t ForceReCalibration    =0;
   uint32_t FirstConnectionConfig =0;
   uint16_t HWServW2STHandle;
   uint16_t ConfigServW2STHandle;
   uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */
   uint8_t manuf_data[30] =
   {
      2  /* lenght*/,0x0A,0x00 /* 0 dBm */, // Trasmission Power
      8  /* lenght*/,0x09,NAME_FLIGHT1, // Complete Name
      13 /* lenght*/ ,0xFF,0x01,/*SKD version */
      0x80, /* NUCLEO-Board */
      0x02, /* Prox */
      0xE0, /* ACC+Gyro+Mag+Pres+Hum+Temp */
      0x00, /* SensorFusionShort */
      0x00, /* SensorFusionFloat */
      0x00, /* BLE MAC start */
      0x00,
      0x00,
      0x00,
      0x00,
      0x00, /* BLE MAC stop */
      0x00, /* BLE PIN start */
      0x00,
      0x00,
      0x00  /* BLE PIN stop */
   };
};

Flight1Service Flight1;

void enableAllFunc()
{
   AccGyr->Enable_Pedometer();
#ifdef USE_IKS01A3
   AccGyr->Enable_Tilt_Detection(LSM6DSO_INT1_PIN);
   AccGyr->Enable_Free_Fall_Detection(LSM6DSO_INT1_PIN);
   AccGyr->Enable_Single_Tap_Detection(LSM6DSO_INT1_PIN);
   AccGyr->Enable_Double_Tap_Detection(LSM6DSO_INT1_PIN);
   AccGyr->Enable_6D_Orientation(LSM6DSO_INT1_PIN);
   AccGyr->Step_Counter_Reset();
#elif defined (USE_IKS01A2)
   AccGyr->Enable_Tilt_Detection();
   AccGyr->Enable_Free_Fall_Detection();
   AccGyr->Enable_Single_Tap_Detection();
   AccGyr->Enable_Double_Tap_Detection();
   AccGyr->Enable_6D_Orientation();
   AccGyr->Reset_Step_Counter();
#endif
}

void disableAllFunc()
{
   AccGyr->Disable_Pedometer();
   AccGyr->Disable_Tilt_Detection();
   AccGyr->Disable_Free_Fall_Detection();
   AccGyr->Disable_Single_Tap_Detection();
   AccGyr->Disable_Double_Tap_Detection();
   AccGyr->Disable_6D_Orientation();
}

void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
   uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
   uint8_t Command = att_data[4];
   uint8_t Data    = att_data[5];
   (void)data_length;
   FLIGHT1_PRINTF("Parsing command: ");
   switch(FeatureMask)
   {
   case FEATURE_MASK_ACC_EVENTS:
      switch(Command)
      {
      case 'm':
         /* Multiple Events */
         switch(Data)
         {
         case 1:
            enableAllFunc();
            FLIGHT1_PRINTF("Multi enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            disableAllFunc();
            FLIGHT1_PRINTF("Multi disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 'f':
         /* FreeFall */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_Free_Fall_Detection(LSM6DSO_INT1_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_Free_Fall_Detection();
#endif
            FLIGHT1_PRINTF("Free fall enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Free_Fall_Detection();
            FLIGHT1_PRINTF("Free fall disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 'd':
         /* Double Tap */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_Double_Tap_Detection(LSM6DSO_INT1_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_Double_Tap_Detection();
#endif
            FLIGHT1_PRINTF("Double tap enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Double_Tap_Detection();
            FLIGHT1_PRINTF("Double tap disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 's':
         /* Single Tap */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_Single_Tap_Detection(LSM6DSO_INT1_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_Single_Tap_Detection();
#endif
            FLIGHT1_PRINTF("Single tap enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Single_Tap_Detection();
            FLIGHT1_PRINTF("Single tap disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 'p':
         /* Pedometer */
         switch(Data)
         {
         case 1:
            AccGyr->Enable_Pedometer();
#ifdef USE_IKS01A3
            AccGyr->Step_Counter_Reset();
#elif defined (USE_IKS01A2)
            AccGyr->Reset_Step_Counter();
#endif
            FLIGHT1_PRINTF("Pedometer enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Pedometer();
            FLIGHT1_PRINTF("Pedometer disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 'w':
         /* Wake UP */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_Wake_Up_Detection(LSM6DSO_INT2_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_Wake_Up_Detection();
#endif
            FLIGHT1_PRINTF("Wake up enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Wake_Up_Detection();
            FLIGHT1_PRINTF("Wake up disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 't':
         /* Tilt */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_Tilt_Detection(LSM6DSO_INT1_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_Tilt_Detection();
#endif
            FLIGHT1_PRINTF("Tilt enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_Tilt_Detection();
            FLIGHT1_PRINTF("Tilt disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      case 'o' :
         /* Tilt */
         switch(Data)
         {
         case 1:
#ifdef USE_IKS01A3
            AccGyr->Enable_6D_Orientation(LSM6DSO_INT1_PIN);
#elif defined(USE_IKS01A2)
            AccGyr->Enable_6D_Orientation();
#endif
            FLIGHT1_PRINTF("Orientation enabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         case 0:
            AccGyr->Disable_6D_Orientation();
            FLIGHT1_PRINTF("Orientation disabled\n");
            Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
         break;
      }
      break;
   }
}


void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
   if(attr_handle == AccEventCharHandle + 2)
   {
      if (att_data[0] == 01)
      {
         enableAllFunc();
         Flight1.AccEvent_Notify(0, 3);
         Flight1.Config_Notify(FEATURE_MASK_ACC_EVENTS,'m',1);
      }
      else if (att_data[0] == 0)
      {
         disableAllFunc();
      }
   }
   else if (attr_handle == ConfigCharHandle + 1)
   {
      /* Received one write command from Client on Configuration characteristc */
      ConfigCommandParsing(att_data, data_length);
   }
   else if(attr_handle == AccGyroMagCharHandle + 2)
   {
      if (att_data[0] == 01)
      {
         AccGyroMag_Enable = 1;
         FLIGHT1_PRINTF("Acc, Gyro and Mag enabled\n");
      }
      else if (att_data[0] == 0)
      {
         AccGyroMag_Enable = 0;
         FLIGHT1_PRINTF("Acc, Gyro and Mag disabled\n");
      }
   }
   else if(attr_handle == EnvironmentalCharHandle + 2)
   {
      if (att_data[0] == 01)
      {
         Enviroment_Enable = 1;
         FLIGHT1_PRINTF("Enviroment enabled\n");
      }
      else if (att_data[0] == 0)
      {
         Enviroment_Enable = 0;
         FLIGHT1_PRINTF("Enviroment disabled\n");
      }
   }
   else if(attr_handle == GestureDetCharHandle + 2)
   {
      if (att_data[0] == 01)
      {
         Gestures_Enable = 1;
         FLIGHT1_PRINTF("Gestures enabled\n");
      }
      else if (att_data[0] == 0)
      {
         Gestures_Enable = 0;
         FLIGHT1_PRINTF("Gestures disabled\n");
      }
   }
   else if (attr_handle == ProxCharHandle + 2)
   {
      if (att_data[0] == 01)
      {
         Distance_Enable = 1;
         FLIGHT1_PRINTF("Distance enabled\n");
      }
      else if (att_data[0] == 0)
      {
         Distance_Enable = 0;
         FLIGHT1_PRINTF("Distance disabled\n");
      }
   }
}



/*Callback function*/
void Flight1_HCI_Event_CB(void *pckt)
{
   hci_uart_pckt *hci_pckt =(hci_uart_pckt*) pckt;
   hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

   FLIGHT1_PRINTF("In Callback: ");

   if(hci_pckt->type != HCI_EVENT_PKT)
   {
      return;
   }
   /*Switch event*/
   switch(event_pckt->evt)
   {

   /*if disconnected*/
   case EVT_DISCONN_COMPLETE:
   {
      Flight1.connected = FALSE;
      Flight1.set_connectable = TRUE;
      delay(100);
      FLIGHT1_PRINTF("Disconnected\n");
   }
   break;
   case EVT_LE_META_EVENT:
   {
      evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

      switch(evt->subevent)
      {
      /*if connected*/
      case EVT_LE_CONN_COMPLETE:
      {
         Flight1.connected=TRUE;
         disableAllFunc();
         AccGyr->Disable_Wake_Up_Detection();
         delay(100);
         FLIGHT1_PRINTF("Connected\n");
      }
      break;
      }
   }
   break;
   case EVT_VENDOR:
   {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
      if (blue_evt->ecode == EVT_BLUE_GATT_ATTRIBUTE_MODIFIED)
      {
         evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
         FLIGHT1_PRINTF("Attribute modified\n");
         Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
      }
   }
   break;
   default:
      FLIGHT1_PRINTF("Error\n");
      break;
   }
}



/*Setup distance sensors for gesture detection*/
void SetupSingleShot(VL53L1_X_NUCLEO_53L1A1 *sensor)
{
   int status;

   //Change distance mode to short range
   status = sensor->VL53L1X_SetDistanceMode(1);
   if( status )
   {
      SerialPort.println("SetDistanceMode failed");
   }

   //Change timing budget to 20 ms
   status = sensor->VL53L1X_SetTimingBudgetInMs(20);
   if( status )
   {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds failed");
   }
   status = sensor->VL53L1X_SetInterMeasurementInMs(20);
   if( status )
   {
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

void setup()
{
   SerialPort.begin(115200);
   DEV_I2C.begin();

   pinMode(LED_BUILTIN, OUTPUT); //D13 LED

   //Interrupts.
   attachInterrupt(INT_1, INT1Event_cb, RISING);
   attachInterrupt(INT_2, INT2Event_cb, RISING);

   //Initialize bluetooth communication
   if(BTLE.begin() == SPBTLERF_ERROR)
   {
      SerialPort.println("Bluetooth module configuration error!");
      while(1);
   }

   if(Flight1.begin())
   {
      SerialPort.println("Bluetooth services configuration error!");
      while(1);
   }
   SerialPort.println("Bluetooth configuration done!");

   // Create VL53L1X top component.
   xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
   sensor_vl53l1_top = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_top, A2);

   // Switch off VL53L1X top component.
   sensor_vl53l1_top->VL53L1_Off();

   // Create (if present) VL53L1X left component.
   xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
   sensor_vl53l1_left = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_left, D8);

   //Switch off (if present) VL53L1X left component.
   sensor_vl53l1_left->VL53L1_Off();

   // Create (if present) VL53L1X right component.
   xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
   sensor_vl53l1_right = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_right, D2);

   // Switch off (if present) VL53L1X right component.
   sensor_vl53l1_right->VL53L1_Off();

   //Initialize the sensor
   sensor_vl53l1_top->InitSensor(0x10);
   sensor_vl53l1_left->InitSensor(0x12);
   sensor_vl53l1_right->InitSensor(0x14);

   //Change Distance mode and timings
   SetupSingleShot(sensor_vl53l1_top);
   SetupSingleShot(sensor_vl53l1_left);
   SetupSingleShot(sensor_vl53l1_right);


   //Top sensor should be in long distance mode
   sensor_vl53l1_top->VL53L1X_SetDistanceMode(2);

   // Initialize VL53L1X gesture library.
   tof_gestures_initDIRSWIPE_1(400, 0, 500, &gestureDirSwipeData);
   tof_gestures_initTAP_1(&gestureTapData);

   //Start measurement
   sensor_vl53l1_top->VL53L1X_StartRanging();
   sensor_vl53l1_left->VL53L1X_StartRanging();
   sensor_vl53l1_right->VL53L1X_StartRanging();

   //Setup MEMS sensors
#ifdef USE_IKS01A3
   HumTemp  = new HTS221Sensor (&DEV_I2C);
   PressTemp = new LPS22HHSensor (&DEV_I2C);
   AccGyr = new LSM6DSOSensor(&DEV_I2C);
   Acc2 = new LIS2DW12Sensor(&DEV_I2C);
   Mag = new LIS2MDLSensor(&DEV_I2C);
   Temp = new STTS751Sensor(&DEV_I2C);
   Temp->Enable();
   Acc2->Enable_X();
#elif defined (USE_IKS01A2)
   HumTemp  = new HTS221Sensor (&DEV_I2C);
   PressTemp = new LPS22HBSensor (&DEV_I2C);
   AccGyr = new LSM6DSLSensor(&DEV_I2C);
   Acc2 = new LSM303AGR_ACC_Sensor(&DEV_I2C);
   Mag = new LSM303AGR_MAG_Sensor(&DEV_I2C);
   Acc2->Enable();
#endif
   HumTemp->Enable();
   PressTemp->Enable();
   AccGyr->Enable_X();
   AccGyr->Set_X_ODR(4.0f);
   AccGyr->Enable_G();
   Mag->Enable();
}

void loop()
{
   /*If device is connectable setup advertisement*/
   if(Flight1.set_connectable)
   {
      Flight1.setConnectable();
   }
   int status;
   uint8_t ready = 0;
   uint16_t distance;
   int32_t decPart, intPart;
   int32_t PressToSend=0;
   uint16_t HumToSend=0;
   int16_t TempToSend=0;
   int gesture_code;
   int left_done = 0;
   int right_done = 0;
   uint8_t NewDataReady=0;
   uint8_t RangeStatus;
   BTLE.update();
   if(Flight1.connected)
   {
      //Get enviroment data
      float humidity, temperature, pressure;
      if (Enviroment_Enable)
      {
         HumTemp->GetHumidity(&humidity);
#ifdef USE_IKS01A3
         Temp->GetTemperature(&temperature);
#elif defined (USE_IKS01A2)
         HumTemp->GetTemperature(&temperature);
#endif
         PressTemp->GetPressure(&pressure);
         MCR_BLUEMS_F2I_2D(pressure, intPart, decPart);
         PressToSend=intPart*100+decPart;
         MCR_BLUEMS_F2I_1D(humidity, intPart, decPart);
         HumToSend = intPart*10+decPart;
         MCR_BLUEMS_F2I_1D(temperature, intPart, decPart);
         TempToSend = intPart*10+decPart;
      }

      int32_t accelerometer[3];
      int32_t gyroscope[3];

      if (AccGyroMag_Enable)
      {
         // Read accelerometer and gyroscope.
         AccGyr->Get_X_Axes(accelerometer);
         AccGyr->Get_G_Axes(gyroscope);
      }

      //FLIGHT1_PRINTF("Accelerometer:\tX:%d\tY:%d\tZ:%d\n", accelerometer[0], accelerometer[1], accelerometer[2]);
      //FLIGHT1_PRINTF("Gyroscope:\tX:%d\tY:%d\tZ:%d\n", gyroscope[0], gyroscope[1], gyroscope[2]);

      if (mems_event)
      {
         mems_event=0;
#ifdef USE_IKS01A3
         LSM6DSO_Event_Status_t Astatus;
         AccGyr->Get_X_Event_Status(&Astatus);
#elif defined(USE_IKS01A2)
         LSM6DSL_Event_Status_t Astatus;
         AccGyr->Get_Event_Status(&Astatus);
#endif
         if (Astatus.StepStatus)
         {
            // New step detected, so print the step counter
            uint16_t step_count = 0;
#ifdef USE_IKS01A3
            AccGyr->Get_Step_Count(&step_count);
#elif defined(USE_IKS01A2)
            AccGyr->Get_Step_Counter(&step_count);
#endif
            FLIGHT1_PRINTF("Step %d\n", step_count);
            Flight1.AccEvent_Notify(step_count, 3);
         }
         if (Astatus.FreeFallStatus)
         {
            FLIGHT1_PRINTF("Free fall\n");
            Flight1.AccEvent_Notify(16, 2);
         }

         if (Astatus.TapStatus)
         {
            FLIGHT1_PRINTF("Single tap\n");
            Flight1.AccEvent_Notify(32, 2);
         }

         if (Astatus.DoubleTapStatus)
         {
            FLIGHT1_PRINTF("Double tap\n");
            Flight1.AccEvent_Notify(64, 2);
         }

         if (Astatus.TiltStatus)
         {
            FLIGHT1_PRINTF("Tilt\n");
            Flight1.AccEvent_Notify(8, 2);
         }

         if (Astatus.D6DOrientationStatus)
         {
            FLIGHT1_PRINTF("6D Interrupt\n");
            uint8_t xl = 0;
            uint8_t xh = 0;
            uint8_t yl = 0;
            uint8_t yh = 0;
            uint8_t zl = 0;
            uint8_t zh = 0;
            uint8_t OrientationResult = 0;
            AccGyr->Get_6D_Orientation_XL(&xl);
            AccGyr->Get_6D_Orientation_XH(&xh);
            AccGyr->Get_6D_Orientation_YL(&yl);
            AccGyr->Get_6D_Orientation_YH(&yh);
            AccGyr->Get_6D_Orientation_ZL(&zl);
            AccGyr->Get_6D_Orientation_ZH(&zh);
            if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
            {
               OrientationResult = 4;
            }
            else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
            {
               OrientationResult = 1;
            }
            else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
            {
               OrientationResult = 3;
            }
            else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
            {
               OrientationResult = 2;
            }
            else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
            {
               OrientationResult = 5;
            }
            else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
            {
               OrientationResult = 6;
            }
            Flight1.AccEvent_Notify(OrientationResult, 2);
         }

         if (Astatus.WakeUpStatus)
         {
            FLIGHT1_PRINTF("Wake Up\n");
            Flight1.AccEvent_Notify(128, 2);
         }
      }

      if (Distance_Enable || Gestures_Enable)
      {
         //Get top sensor distance and transmit
         do
         {
            sensor_vl53l1_top->VL53L1X_CheckForDataReady(&ready);
         }
         while (!ready);

         status = sensor_vl53l1_top->VL53L1X_GetRangeStatus(&RangeStatus);
         status = sensor_vl53l1_top->VL53L1X_GetDistance(&distance);

         if (status == VL53L1_ERROR_NONE && Distance_Enable)
         {
            Flight1.FlightSense_Distance_Update(distance);
         }

         //Clear interrupt
         status = sensor_vl53l1_top->VL53L1X_ClearInterrupt();

         distance = (RangeStatus == 0 && distance<1400) ? distance : 1400;

         // Launch gesture detection algorithm.
         gesture_code = tof_gestures_detectTAP_1(distance, &gestureTapData);

         // Check the result of the gesture detection algorithm.
         switch(gesture_code)
         {
         case GESTURES_SINGLE_TAP:
            Flight1.Gestures_Update(1);
            break;
         default:
            // Do nothing
            break;
         }


         //wait for data ready
         do
         {
            //if left not done
            if(left_done == 0)
            {
               NewDataReady = 0;
               //check measurement data ready
               int status = sensor_vl53l1_left->VL53L1X_CheckForDataReady(&NewDataReady);

               if( status )
               {
                  SerialPort.println("GetMeasurementDataReady left sensor failed");
               }
               //if ready
               if(NewDataReady)
               {
                  //get status
                  status = sensor_vl53l1_left->VL53L1X_GetRangeStatus(&RangeStatus);
                  if( status )
                  {
                     SerialPort.println("GetRangeStatus left sensor failed");
                  }

                  //if distance < 1.3 m
                  if (RangeStatus == 0)
                  {
                     // we have a valid range.
                     status = sensor_vl53l1_left->VL53L1X_GetDistance(&distance_left);
                     if( status )
                     {
                        SerialPort.println("GetDistance left sensor failed");
                     }
                  }
                  else
                  {
                     distance_left = 1400;   //default distance
                  }

                  //restart measurement
                  status = sensor_vl53l1_left->VL53L1X_ClearInterrupt();
                  if( status )
                  {
                     SerialPort.println("Restart left sensor failed");
                  }

                  left_done = 1 ;
               }
            }

            //if right not done
            if(right_done == 0)
            {
               NewDataReady = 0;
               //check measurement data ready
               int status = sensor_vl53l1_right->VL53L1X_CheckForDataReady(&NewDataReady);

               if( status )
               {
                  SerialPort.println("GetMeasurementDataReady right sensor failed");
               }
               //if ready
               if(NewDataReady)
               {
                  //get status
                  status = sensor_vl53l1_right->VL53L1X_GetRangeStatus(&RangeStatus);
                  if( status )
                  {
                     SerialPort.println("GetRangeStatus right sensor failed");
                  }
                  //if distance < 1.3 m
                  if (RangeStatus == 0)
                  {
                     // we have a valid range.
                     status = sensor_vl53l1_right->VL53L1X_GetDistance(&distance_right);
                     if( status )
                     {
                        SerialPort.println("GetDistance right sensor failed");
                     }
                  }
                  else
                  {
                     distance_right = 1400;   //default distance
                  }

                  //restart measurement
                  status = sensor_vl53l1_right->VL53L1X_ClearInterrupt();
                  if( status )
                  {
                     SerialPort.println("Restart right sensor failed");
                  }

                  right_done = 1 ;
               }
            }
         }
         while(left_done == 0 || right_done == 0);


         // Launch gesture detection algorithm.
         gesture_code = tof_gestures_detectDIRSWIPE_1(distance_left, distance_right, &gestureDirSwipeData);

         // Check the result of the gesture detection algorithm.
         switch(gesture_code)
         {
         case GESTURES_SWIPE_LEFT_RIGHT:
            Flight1.Gestures_Update(3);
            break;
         case GESTURES_SWIPE_RIGHT_LEFT:
            Flight1.Gestures_Update(2);
            break;
         default:
            // Do nothing
            break;
         }
      }

      //Send all mems sensors data
      if (Enviroment_Enable)
      {
         Flight1.Environmental_Update(PressToSend, HumToSend, TempToSend);
         delay(30);
      }
      if (AccGyroMag_Enable)
      {
         Flight1.AccGyroMag_Update(accelerometer, gyroscope);
         delay(30);
      }
   }
}
