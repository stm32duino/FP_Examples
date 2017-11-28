/**
 ******************************************************************************
 * @file    GestureDirSwipeLed_53L0A1_LED61A1.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    27 November 2017
 * @brief   Arduino test application based on X-NUCLEO-53L0A1 proximity sensor
 *          expansion board and X-NUCLEO-LED61A1 expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
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
#include <vl53l0x_x_nucleo_53l0a1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>
#include <Led6001.h>

// Components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_top;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_left;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_right;
// I2C dev.
#define DEV_I2C Wire
#define SerialPort Serial
// Gesture structure.
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;

// Range values
uint32_t distance_top, distance_left, distance_right;

/* LED Control Component. */
Led6001 *led;

/* LED dimming */
float dimming = 0.0f;

/* Interrupt flags. */
static volatile bool xfault_irq_triggered = false;
bool verbose = false;

int current_channel = 0;
int intensity = 0;
int current_state = 0;
int timestamp_start = -1;

void switchOffLed();
void switchOnLed();
int check_onoff(int r_top, int r_left, int r_right);
void changeColor();
void changeColor();

/**
 *  Setup all sensors for single shot mode
 */
void SetupSingleShot(void) {
  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  status = sensor_vl53l0x_top->StaticInit();
  if( status ) {
    if(verbose) {
      SerialPort.println("StaticInit top sensor failed");
    }
  }

  status = sensor_vl53l0x_top->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefCalibration top sensor failed");
    }
  }

  status = sensor_vl53l0x_top->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefSpadManagement top sensor failed");
    }
  }

  status = sensor_vl53l0x_top->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ) {
    if(verbose) {
      SerialPort.println("SetDeviceMode top sensor failed");
    }
  }

  status = sensor_vl53l0x_top->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ) {
    if(verbose) {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds top sensor failed");
    }
  }
  
  status = sensor_vl53l0x_left->StaticInit();
  if( status ) {
    if(verbose) {
      SerialPort.println("StaticInit left sensor failed");
    }
  }

  status = sensor_vl53l0x_left->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefCalibration left sensor failed");
    }
  }

  status = sensor_vl53l0x_left->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefSpadManagement left sensor failed");
    }
  }

  status = sensor_vl53l0x_left->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ) {
    if(verbose) {
      SerialPort.println("SetDeviceMode left sensor failed");
    }
  }

  status = sensor_vl53l0x_left->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ) {
    if(verbose) {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds left sensor failed");
    }
  }

  status = sensor_vl53l0x_right->StaticInit();
  if( status ) {
    if(verbose) {
      SerialPort.println("StaticInit right sensor failed");
    }
  }

  status = sensor_vl53l0x_right->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefCalibration right sensor failed");
    }
  }

  status = sensor_vl53l0x_right->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ) {
    if(verbose) {
      SerialPort.println("PerformRefSpadManagement right sensor failed");
    }
  }

  status = sensor_vl53l0x_right->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ) {
    if(verbose) {
      SerialPort.println("SetDeviceMode right sensor failed");
    }
  }

  status = sensor_vl53l0x_right->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ) {
    if(verbose) {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds right sensor failed");
    }
  }
}

/**
 * @brief  Interrupt Request for the component's XFAULT interrupt.
 * @param  None.
 * @retval None.
 */
void xfault_irq(void) {
  xfault_irq_triggered = true;
}

/**
 * @brief  Interrupt Handler for the component's XFAULT interrupt.
 * @param  None.
 * @retval None.
 */
void xfault_handler(void) {
  /* Printing to the console. */
  if(verbose) {
    SerialPort.print("XFAULT Interrupt detected! Re-initializing LED driver...");
  }

  /* Re-starting-up LED Control Component. */
  led->start_up();

  /* Printing to the console. */
  if(verbose) {
    SerialPort.println("Done.");
  }
}


/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;

  // Initialize serial for output.
  if(verbose) {
    SerialPort.begin(115200);
  }

  // Initialize I2C bus.
  DEV_I2C.begin();
  DEV_I2C.setClock(400000);

  /* Initializing LED Control Component. */
  led = new Led6001(D4, A3, D6, D5);
  if (led->init() != COMPONENT_OK) {
    if(verbose) {
      SerialPort.println("Led init failed...");
    }
    exit(EXIT_FAILURE);
  }

  /* Attaching interrupt request functions. */
  led->attach_xfault_irq(&xfault_irq);
  
  switchOffLed();
  
  // Create VL53L0X top component.
  xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
  sensor_vl53l0x_top = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_top, A2);

  // Switch off VL53L0X top component.
  sensor_vl53l0x_top->VL53L0X_Off();
  
  // Create VL53L0X left component.
  xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
  sensor_vl53l0x_left = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_left, D8);

  // Switch off VL53L0X left component.
  sensor_vl53l0x_left->VL53L0X_Off();
  
  // Create VL53L0X right component.
  xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
  sensor_vl53l0x_right = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_right, D2);

  // Switch off VL53L0X right component.
  sensor_vl53l0x_right->VL53L0X_Off();
  
  // Initialize VL53L0X top component.
  status = sensor_vl53l0x_top->InitSensor(0x10);
  if(status) {
    if(verbose) {
      SerialPort.println("Init sensor_vl53l0x_top failed...");
    }
  }
  
  // Initialize VL53L0X left component.
  status = sensor_vl53l0x_left->InitSensor(0x12);
  if(status) {
    if(verbose) {
      SerialPort.println("Init sensor_vl53l0x_left failed...");
    }
  }
  
  // Initialize VL53L0X right component.
  status = sensor_vl53l0x_right->InitSensor(0x14);
  if(status) {
    if(verbose) {
      SerialPort.println("Init sensor_vl53l0x_right failed...");
    }
  }
  
  // Initialize VL6180X gesture library.
  tof_gestures_initDIRSWIPE_1(190, 0, 2000, &gestureDirSwipeData);

  SetupSingleShot();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  int gesture_code;
  // Read Range.

  sensor_vl53l0x_top->StartMeasurement();
  sensor_vl53l0x_left->StartMeasurement();
  sensor_vl53l0x_right->StartMeasurement();
  
  int top_done = 0;
  int left_done = 0;
  int right_done = 0;
  uint8_t NewDataReady=0;
  VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

  if (xfault_irq_triggered) {
     xfault_irq_triggered = false;
     xfault_handler();
  }
  
  do {
    if(top_done == 0) {
      NewDataReady = 0;
      int status = sensor_vl53l0x_top->GetMeasurementDataReady(&NewDataReady);

      if( status ) {
        if(verbose) {
          SerialPort.println("GetMeasurementDataReady top sensor failed");
        }
      }
      
      if(NewDataReady) {
        status = sensor_vl53l0x_top->ClearInterruptMask(0);
        if( status ) {
          if(verbose) {
            SerialPort.println("ClearInterruptMask top sensor failed");
          }
        }

        status = sensor_vl53l0x_top->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ) {
          if(verbose) {
            SerialPort.println("GetRangingMeasurementData top sensor failed");
          }
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_top = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_top = 1200;
        }
        
        top_done = 1;
      }
    }

    if(left_done == 0) {
      NewDataReady = 0;
      int status = sensor_vl53l0x_left->GetMeasurementDataReady(&NewDataReady);

      if( status ) {
        if(verbose) {
          SerialPort.println("GetMeasurementDataReady left sensor failed");
        }
      }
      
      if(NewDataReady) {
        status = sensor_vl53l0x_left->ClearInterruptMask(0);
        if( status ) {
          if(verbose) {
            SerialPort.println("ClearInterruptMask left sensor failed");
          }
        }

        status = sensor_vl53l0x_left->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ) {
          if(verbose) {
            SerialPort.println("GetRangingMeasurementData left sensor failed");
          }
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_left = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_left = 1200;
        }
        
        left_done = 1;
      }
    }
    
    if(right_done == 0) {
      NewDataReady = 0;
      int status = sensor_vl53l0x_right->GetMeasurementDataReady(&NewDataReady);

      if( status ) {
        if(verbose) {
          SerialPort.println("GetMeasurementDataReady right sensor failed");
        }
      }
      
      if(NewDataReady) {
        status = sensor_vl53l0x_right->ClearInterruptMask(0);
        if( status ) {
          if(verbose) {
            SerialPort.println("ClearInterruptMask right sensor failed");
          }
        }

        status = sensor_vl53l0x_right->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ) {
          if(verbose) {
            SerialPort.println("GetRangingMeasurementData right sensor failed");
          }
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_right = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_right = 1200;
        }
        
        right_done = 1;
      }
    }
  } while(top_done == 0 || left_done == 0 || right_done == 0);
  
  if(check_onoff(distance_top, distance_left, distance_right) == 1) {
    if(current_state == 1) {
      switchOffLed();
      current_state = 0;
    }else {
      switchOnLed();
      current_state = 1;
      delay(1000);
    }
  }
  
  // Launch gesture detection algorithm.
  gesture_code = tof_gestures_detectDIRSWIPE_1(distance_left, distance_right, &gestureDirSwipeData);

  // Check the result of the gesture detection algorithm.
  switch(gesture_code) {
    case GESTURES_SWIPE_LEFT_RIGHT:
      if(verbose) {
        SerialPort.println("From Left To Right");
      }

      if(current_state == 1) {
        increaseIntensity();
      }
      break;
    case GESTURES_SWIPE_RIGHT_LEFT:
      if(verbose) {
        SerialPort.println("From Right To Left");
      }

      if(current_state == 1) {
        decreaseIntensity();
      }
      break;
    default:
      // Do nothing
      break;
  }
}

void increaseIntensity() {
  dimming = dimming + 0.2f;
  if(dimming > 1.0f) {
    dimming = 1.0f;
  }

  if(verbose) {
    SerialPort.print("increaseIntensity -> set_pwm_dimming: ");
    SerialPort.println(dimming);
  }

  led->set_pwm_dimming(dimming);
}

void decreaseIntensity() {
  dimming = dimming - 0.2f;
  if(dimming < 0.2f) {
    dimming = 0.2f;
  }

  if(verbose) {
    SerialPort.print("decreaseIntensity -> set_pwm_dimming: ");
    SerialPort.println(dimming);
  }

  led->set_pwm_dimming(dimming);
}

void switchOffLed() {
  led->power_off();
}

void switchOnLed() {
  dimming = 0.2f; /* default dimming value */
  led->power_on();
  led->set_pwm_dimming(dimming);
}

int check_onoff(int r_top, int r_left, int r_right) {
  int ret_val = 0;
  
  if(r_top < 190 && r_left < 190 && r_right < 190) {
    if(timestamp_start != -1) {
      int timestamp_final = millis();
      
      if((timestamp_final - timestamp_start) > 3000) {
        ret_val = 1;
        timestamp_start = -1;
      }
    }else {
      timestamp_start = millis();
    }
  }else {
    timestamp_start = -1;
  }

  if(ret_val) {
    if(verbose) {
      SerialPort.println("ON/OFF");
    }
  }
  
  return ret_val;
}

