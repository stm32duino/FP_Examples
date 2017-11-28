/**
 ******************************************************************************
 * @file    NucleoCar.ino
 * @author  STMicroelectronics
 * @version V1.2.0
 * @date    27 November 2017
 * @brief   Arduino demo application for the STMicrolectronics
 *          X-NUCLEO-IHM02A1, X-NUCLEO-6180XA1 and X-NUCLEO-IDB0XA1
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <SPI.h>

#define DEV_I2C Wire
#define SerialPort Serial

/* ---------------- */
/* X-NUCLEO-IHM02A1 */
/* ---------------- */

#include <XNucleoIHM02A1.h>

/* ---------------- */
/* X-NUCLEO-6180XA1 */
/* ---------------- */

#include <vl6180x_x_nucleo_6180xa1_class.h>
#include <stmpe1600_class.h>

STMPE1600DigiOut *gpio0_top;
STMPE1600DigiOut *gpio0_left;
STMPE1600DigiOut *gpio0_right;
VL6180X_X_NUCLEO_6180XA1 *sensor_vl6180x_top;
VL6180X_X_NUCLEO_6180XA1 *sensor_vl6180x_left;
VL6180X_X_NUCLEO_6180XA1 *sensor_vl6180x_right;

/* ---------------- */
/* X-NUCLEO-IDB0XA1 */
/* ---------------- */

#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_le.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include <SPBTLE_RF.h>
#include <stdlib.h>
#include "bluenrg_hal_aci.h"
#include "hci.h"
#include "hci_le.h"
#include "bluenrg_utils.h"
#include "stm32_bluenrg_ble.h"
#include "osal.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "gp_timer.h"
#include "Arduino.h"

#define IDB0XA1_PIN_SPI_MOSI   (11)
#define IDB0XA1_PIN_SPI_MISO   (12)
#define IDB0XA1_PIN_SPI_SCK    (3)

SPIClass BTLE_SPI(IDB0XA1_PIN_SPI_MOSI, IDB0XA1_PIN_SPI_MISO, IDB0XA1_PIN_SPI_SCK);

#define IDB0XA1_PIN_SPI_nCS    (A1)
#define IDB0XA1_PIN_SPI_RESET  (7)
#define IDB0XA1_PIN_SPI_IRQ    (A0)

SPBTLERFClass BTLE(&BTLE_SPI, IDB0XA1_PIN_SPI_nCS, IDB0XA1_PIN_SPI_IRQ, IDB0XA1_PIN_SPI_RESET);

const char *name = "BlueCar"; // Max 7 chars otherwise error
uint8_t SERVER_BDADDR[] = {0xab, 0xb0, 0x01, 0xE1, 0x80, 0x02};

typedef enum 
{
  CSTS_DISCONNECTED,
  CSTS_CONNECTED,
  CSTS_DISCONNECTED_BY_USER
}Connection_status_t;

/* Public variables ----------------------------------------------------------*/
extern uint8_t bnrg_expansion_board;

/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
  	uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
	uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
	uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
	uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
	}while(0)

/* Private Prototypes --------------------------------------------------------*/
void NucleoCar_HCI_Event_CB(void *pckt);

class NucleoCarServiceClass
{
  public:
    NucleoCarServiceClass(void)
    {
      BTLEArrivedDataLength = 0;
      DEVICE_CONNECTION_STATUS = CSTS_DISCONNECTED;
      DEVICE_CONNECTION_AUTHORIZED = FALSE;
      connection_handle = 0;
      connected = FALSE;
      set_connectable = TRUE;
    }

    tBleStatus begin(const char *name, uint8_t addr[BDADDR_SIZE]) {
      uint8_t bdaddr[BDADDR_SIZE];
      uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

      uint8_t  hwVersion;
      uint16_t fwVersion;

      int ret;

      if((name == NULL) || (addr == NULL)) {
        return BLE_STATUS_NULL_PARAM;
      }

      attach_HCI_CB(NucleoCar_HCI_Event_CB);

      /* get the BlueNRG HW and FW versions */
      ret = getBlueNRGVersion(&hwVersion, &fwVersion);
      if(ret) {
        PRINTF("Reading Version failed.\n");
        return ret;
      }

      /*
       * Reset BlueNRG again otherwise we won't
       * be able to change its MAC address.
       * aci_hal_write_config_data() must be the first
       * command after reset otherwise it will fail.
       */
      BlueNRG_RST();

      if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
        bnrg_expansion_board = IDB05A1;
        /*
         * Change the MAC address to avoid issues with Android cache:
         * if different boards have the same MAC address, Android
         * applications unless you restart Bluetooth on tablet/phone
         */
       addr[5] = 0x02;
      }

      /* The Nucleo board must be configured as SERVER */
      Osal_MemCpy(bdaddr, addr, BDADDR_SIZE);

      ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                      CONFIG_DATA_PUBADDR_LEN,
                                      bdaddr);
      if(ret){
        PRINTF("Setting BD_ADDR failed.\n");
        return ret;
      }

      ret = aci_gatt_init();
      if(ret){
        PRINTF("GATT_Init failed.\n");
        return ret;
      }

      if (bnrg_expansion_board == IDB05A1) {
        ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
      }
      else {
        ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
      }

      if(ret){
        PRINTF("GAP_Init failed.\n");
        return ret;
      }

      ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                       strlen(name), (uint8_t *)name);

      if(ret){
        PRINTF("aci_gatt_update_char_value failed.\n");
        return ret;
      }

      ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                         OOB_AUTH_DATA_ABSENT,
                                         NULL,
                                         7,
                                         16,
                                         USE_FIXED_PIN_FOR_PAIRING,
                                         123456,
                                         BONDING);
      if (ret) {
        PRINTF("BLE Stack Initialization failed.\n");
        return ret;
      }

      /* Set output power level */
      ret = aci_hal_set_tx_power_level(1,4);

      if (ret) {
        PRINTF("Setting Tx Power Level failed.\n");
      }

      return ret;
    }

    void       setConnectable(void) {
      tBleStatus ret;

      const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','C','a','r'};

      if(set_connectable){
        /* disable scan response */
        hci_le_set_scan_resp_data(0,NULL);
        PRINTF("General Discoverable Mode.\n");

        ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                       sizeof(local_name), local_name, 0, NULL, 0, 0);
        if (ret != BLE_STATUS_SUCCESS) {
          PRINTF("Error while setting discoverable mode (%d)\n", ret);
        }
        set_connectable = FALSE;
      }
	}

    int        isConnected(void) {
      return connected;
	}

    tBleStatus Add_Car_Service(void) {
      tBleStatus ret;
  
      /*
      UUIDs:
      D973F2E0-B19E-11E2-9E96-0800200C9A66 (service)
      D973F2E1-B19E-11E2-9E96-0800200C9A66 (tx charac)
      D973F2E2-B19E-11E2-9E96-0800200C9A66 (rx charac)
      */
  
      const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
      const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
      const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
  
      ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &sampleServHandle); /* original is 9?? */
      if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
      ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                               16, 1, &TXCharHandle);
      if (ret != BLE_STATUS_SUCCESS) goto fail;
  
      ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                               16, 1, &RXCharHandle);
      if (ret != BLE_STATUS_SUCCESS) goto fail;
  
      // Sample Service added
      return BLE_STATUS_SUCCESS; 
  
      fail:
        // Error while adding Sample Service
        return BLE_STATUS_ERROR;
	}

    void       GAP_ConnectionComplete_CB(uint8_t addr[BDADDR_SIZE], uint16_t handle) {
      connected = TRUE;
      connection_handle = handle;

      /*
       PRINTF("Connected to device:");
       for(int i = 5; i > 0; i--){
         PRINTF("%02X-", addr[i]);
       }
       PRINTF("%02X\n", addr[0]);
      */
	}

    void       GAP_DisconnectionComplete_CB(void) {
      /* Disconnected */
      connected = FALSE;
      PRINTF("Disconnected\n");
      /* Make the device connectable again */
      set_connectable = TRUE;
      DEVICE_CONNECTION_AUTHORIZED = FALSE;
      DEVICE_CONNECTION_STATUS = CSTS_DISCONNECTED_BY_USER;
	}

    void       Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data) {
      if (handle == RXCharHandle + 1) {
        receiveData(att_data, data_length);
      }
	}

    uint16_t charUuidTX_Handle, charUuidRX_Handle;

    uint8_t BLE_Rx_buffer[260];
    uint16_t BTLEArrivedDataLength;
    Connection_status_t DEVICE_CONNECTION_STATUS;

  private:
    void receiveData(uint8_t* data_buffer, uint8_t Nb_bytes) {
      //uint16_t counter;
      if (BTLEArrivedDataLength == 0) {
        for (BTLEArrivedDataLength = 0; BTLEArrivedDataLength < Nb_bytes; BTLEArrivedDataLength++) 
          BLE_Rx_buffer[BTLEArrivedDataLength] = data_buffer[BTLEArrivedDataLength];
        if (!DEVICE_CONNECTION_AUTHORIZED) {
          if (BLE_CheckAuthorization(BLE_Rx_buffer)) {
            DEVICE_CONNECTION_AUTHORIZED = TRUE;
            DEVICE_CONNECTION_STATUS = CSTS_CONNECTED;
          }
          BTLEArrivedDataLength = 0;
        }
      }
	}

    uint8_t BLE_CheckAuthorization(uint8_t* buffer) {
      // Authorization string [TD&E-BTE-bCAR]
      if (BTLEArrivedDataLength != 13) return FALSE;
      if (buffer[0] != 'T') return FALSE;
      if (buffer[1] != 'D') return FALSE;
      if (buffer[2] != '&') return FALSE;
      if (buffer[3] != 'E') return FALSE;
      if (buffer[4] != '-') return FALSE;
      if (buffer[5] != 'B') return FALSE;
      if (buffer[6] != 'T') return FALSE;
      if (buffer[7] != 'E') return FALSE;
      if (buffer[8] != '-') return FALSE;
      if (buffer[9] != 'b') return FALSE;
      if (buffer[10] != 'C') return FALSE;
      if (buffer[11] != 'A') return FALSE;
      if (buffer[12] != 'R') return FALSE;
  
      return TRUE;
	}

    volatile uint16_t connection_handle;

    volatile bool connected;
    volatile bool set_connectable;

    uint16_t sampleServHandle;
    uint16_t TXCharHandle;
    uint16_t RXCharHandle;

    uint8_t DEVICE_CONNECTION_AUTHORIZED;
};

NucleoCarServiceClass NucleoCarService;

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void NucleoCar_HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      NucleoCarService.GAP_DisconnectionComplete_CB();
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
          NucleoCarService.GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          /* this callback is invoked when a GATT attribute is modified
          extract callback data and pass to suitable handler function */
          if (bnrg_expansion_board == IDB05A1) {
            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
            NucleoCarService.Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
          else {
            evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
            NucleoCarService.Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
        }
        break;
      }
    }
    break;
  }
}


/* Definitions ---------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/

int32_t speed_proxy = 0;
int32_t direction_proxy = 0;

uint8_t speed = 50;
uint8_t direction = 50;

int8_t correction = 0;
uint8_t functionality = 0;   // Drive by BLE or Proximity

/* ---------------- */
/* X-NUCLEO-6180XA1 */
/* ---------------- */

uint32_t range_top;
uint32_t range_left;
uint32_t range_right;

/* ---------------- */
/* X-NUCLEO-IHM02A1 */
/* ---------------- */

// Motor Control Expansion Board.
XNucleoIHM02A1 *x_nucleo_ihm02a1;
SPIClass *dev_spi;
L6470 **motors;

// Initialization parameters of the motors connected to the expansion board.
L6470_init_t L6470_init[L6470DAISYCHAINSIZE] = {
  
    /* First Motor. */
    {
        12.0,                          /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },

    /* Second Motor. */
    {
        12.0,                          /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
    
};


/* Setup ---------------------------------------------------------------------*/

void setup() {

  int status;
  SerialPort.begin(115200);
  DEV_I2C.begin();

  pinMode(LED_BUILTIN, OUTPUT); //D13 LED
  pinMode(USER_BTN, INPUT);     //D23 BUTTON
  
  SerialPort.println("---------------------------");
  
  /* ---------------- */
  /* X-NUCLEO-IHM02A1 */
  /* ---------------- */
  
  // Initializing SPI bus.
  // dev_spi = new SPIClass(D11, D12, D3); // Use SPI1
  dev_spi = new SPIClass(D42, D43, D44); // Use SPI2 to avoid conflict with SPI1 used by BLE Board

  // Initializing Motor Control Expansion Board.
  x_nucleo_ihm02a1 = new XNucleoIHM02A1(&L6470_init[0], &L6470_init[1], A4, A5, D4, A2, dev_spi);

  // Building a list of motor control components.
  motors = x_nucleo_ihm02a1->get_components();

  SerialPort.println("IHM02A1 Configuration Done!");

  /* ---------------- */
  /* X-NUCLEO-6180XA1 */
  /* ---------------- */

  // Create VL6180X top component.
  gpio0_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_12);
  sensor_vl6180x_top = new VL6180X_X_NUCLEO_6180XA1(&DEV_I2C, gpio0_top, 0);
  
  // Switch off VL6180X top component.
  sensor_vl6180x_top->VL6180x_Off();
  
  // Create (if present) VL6180X left component.
  gpio0_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14);
  sensor_vl6180x_left = new VL6180X_X_NUCLEO_6180XA1(&DEV_I2C, gpio0_left, 0);
  
  // Switch off (if present) VL6180X left component.
  sensor_vl6180x_left->VL6180x_Off();
  
  // Create (if present) VL6180X right component.
  gpio0_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15);
  sensor_vl6180x_right = new VL6180X_X_NUCLEO_6180XA1(&DEV_I2C, gpio0_right, 0);
  
  // Switch off (if present) VL6180X right component.
  sensor_vl6180x_right->VL6180x_Off();
  
  // Initialize VL6180X top component.
  status = sensor_vl6180x_top->InitSensor(0x10);
  if(status)
  {
    SerialPort.println("Init sensor_vl6180x_top failed...");
  }
  sensor_vl6180x_top->StartInterleavedMode();

  // Initialize VL6180X left component.
  status = sensor_vl6180x_left->InitSensor(0x11);
  if(status)
  {
    SerialPort.println("Init sensor_vl6180x_left failed...");
  }
  sensor_vl6180x_left->StartInterleavedMode();

  // Initialize VL6180X right component.
  status = sensor_vl6180x_right->InitSensor(0x12);
  if(status)
  {
    SerialPort.println("Init sensor_vl6180x_right failed...");
  }
  sensor_vl6180x_right->StartInterleavedMode();

  SerialPort.println("6180XA1 Configuration Done!");

  /* ---------------- */
  /* X-NUCLEO-IDB0XA1 */
  /* ---------------- */

  int ret;

  if(BTLE.begin() == SPBTLERF_ERROR)
  {
    SerialPort.println("Bluetooth module configuration error!");
    while(1);
  }

  if(NucleoCarService.begin(name, SERVER_BDADDR))
  {
    SerialPort.println("NucleoCar service configuration error!");
    while(1);
  }
  
  ret = NucleoCarService.Add_Car_Service();

  if(ret == BLE_STATUS_SUCCESS)
    SerialPort.println("Car service added successfully.");
  else
    SerialPort.println("Error while adding Car service.");

  //randomSeed(analogRead(A0));

  SerialPort.println("IDB0XA1 Configuration Done!");

  /* ---------------- */
  /* Wait USER Button */
  /* ---------------- */

  // Read the state of USER_BTN pin
  int buttonState = digitalRead(USER_BTN);

  SerialPort.println("Press USER Button");
  do {
    buttonState = digitalRead(USER_BTN);
  } while (buttonState == HIGH);

  SerialPort.println("Start - Set Wheels Home Position");
  motors[0]->set_home(); // Set Home for Turning Wheels
  
}

/* Loop ----------------------------------------------------------------------*/

void loop() {

  int buttonState = digitalRead(USER_BTN);
  
  // Check if the button is pressed
  if (buttonState == LOW) {
    // Wait until the button is released
    do {
      // Read the state of USER_BTN pin
      buttonState = digitalRead(USER_BTN);
    } while(buttonState == LOW);

    // Check the LED pin status
    int ledState = digitalRead(LED_BUILTIN);
    if(ledState == LOW) {
      // If the LED is off, we switch it on
      digitalWrite(LED_BUILTIN, HIGH);
      functionality=1;
      speed = 50;
      direction = 50;
      SerialPort.println("Proxy Mode");   
    } else {
      // If the LED is on, we switch it off
      digitalWrite(LED_BUILTIN, LOW);
      functionality=2;
      speed = 50;
      direction = 50;
      SerialPort.println("BLE Mode");
    }
  }

  BTLE.update();

  if(NucleoCarService.isConnected() == TRUE)
  {

    //if (NucleoCarService.DEVICE_CONNECTION_STATUS == CSTS_CONNECTED)
    //{
    //  SerialPort.println("BLE Connected");
    //}

    if (NucleoCarService.BTLEArrivedDataLength)
    {
      /* Data arrived from BLE: command decode */
      switch (NucleoCarService.BLE_Rx_buffer[0])
      {
        /* Set speed and direction data command */
        case 0:
          // Store last data
          speed = NucleoCarService.BLE_Rx_buffer[1];
          direction = NucleoCarService.BLE_Rx_buffer[2];
          //SerialPort.print("Speed =");
          //SerialPort.println(speed);
          //SerialPort.print("Direction =");
          //SerialPort.println(direction);
          NucleoCarService.BTLEArrivedDataLength = 0;
          break;
          
        default:
          NucleoCarService.BTLEArrivedDataLength = 0;
          break;
      }
    }
    
  }
  else
  {
    //SerialPort.println("BLE Disconnected - Set Discovery Mode");

    if (NucleoCarService.DEVICE_CONNECTION_STATUS == CSTS_DISCONNECTED_BY_USER)
    {
      NucleoCarService.BTLEArrivedDataLength = 0;
      speed = 50;
      direction = 50;
      //SerialPort.print("Speed =");
      //SerialPort.println(speed);
      //SerialPort.print("Direction =");
      //SerialPort.println(direction);
      NucleoCarService.DEVICE_CONNECTION_STATUS = CSTS_DISCONNECTED;

      SerialPort.println("BLE Disconnected - Set Discovery Mode");      
    }

    //Keep the Bluetooth module in discoverable mode
    NucleoCarService.setConnectable();
  }

  // -----------

  switch (functionality)
  {
  
  case 1: // Ignore BLE e Motor controlled by Proximity Sensors

    // --------------------------------------
    //    0 / +200 = Speed value only Forward
    // -200 / +200 = Direction value
    // --------------------------------------

    sensor_vl6180x_top->GetDistance(&range_top);
    sensor_vl6180x_left->GetDistance(&range_left);
    sensor_vl6180x_right->GetDistance(&range_right);
  
    // Speed by Motor1
    // ---------------    
    if (range_top!=255)
    {
      speed_proxy=200-range_top;
      motors[1]->run(StepperMotor::FWD, speed_proxy*2);
    }
    else
    {
      //motors[1]->hard_stop(); // Keep wheel blocked
      motors[1]->hard_hiz();
    }
  
    // Direction by Motor0
    // -------------------    
    if (range_right!=255 || range_left!=255)
    {
      if (range_right>200) { range_right=200; }
      if (range_left>200) { range_left=200; }
      direction_proxy=((200-range_right)-(200-range_left));
      motors[0]->go_to(-direction_proxy*25);
    }
    else
    {
      motors[0]->go_home();
    }
    
    break;

  case 2: // Motor controlled by BLE and Proximity

    // ------------------------------------------------
    // |0 - 100| = Speed value
    //  0 - 49: Backward; 50 = Stop; 51 - 100 = Forward
    // |0 - 100| = Direction value  
    //  0 - 49: Left; 50 = Center; 51 - 100 = Right
    // ------------------------------------------------

    // Speed by Motor1
    // ---------------
    
    if (speed!=50)
    {
      if (speed>50)
      { 
        if (range_right<75 || range_left<75)
        { motors[1]->hard_stop(); }
        else
        { motors[1]->run(StepperMotor::FWD, (speed-50)*4); }
      }
      if (speed<50) { motors[1]->run(StepperMotor::BWD, (50-speed)*4); }
    }
    else
    {
      //motors[1]->hard_stop(); // Keep wheel blocked
      motors[1]->hard_hiz();
    }

    // Direction by Motor0
    // -------------------
    
    sensor_vl6180x_left->GetDistance(&range_left);
    sensor_vl6180x_right->GetDistance(&range_right);

    if (range_right>200) { range_right=200; }
    if (range_left>200) { range_left=200; }
    correction=(range_right-range_left)/4; // -50 / +50

    if (direction!=50)
    {
      if ((direction-50+correction)>50)
        { motors[0]->go_to(50*100); }
      else if ((direction-50+correction)<-50)
        { motors[0]->go_to(-50*100); }
      else
      { motors[0]->go_to((direction-50+correction)*100); }
    }
    else
    {
      motors[0]->go_to(correction*100);
      //motors[0]->go_home();
    }

    break;

  default:
    break;
    
  }

}
