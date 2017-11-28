/*
 ******************************************************************************
 * @file    MemsMotorControl_IKS01A2_IHM02A1.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    27 November 2017
 * @brief   Arduino vertical application using the STMicroelectronics
 *          X-NUCLEO-IKS01A2 MEMS Inertial and Environmental Sensors Expansion
 *          Board and the X-NUCLEO-IHM02A1 Motor Control Expansion Board to get
 *          a MEMS-based motor control (direction and speed).
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* mbed specific header files. */
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

/* Components and expansion boards specific header files. */
#include "LSM6DSLSensor.h"
#include "XNucleoIHM02A1.h"


/* Definitions ---------------------------------------------------------------*/

/* Preferred acceleration axis. */
#define ACCELERATION_AXIS 1

/* Absolute value of the threshold on the preferred axis' acceleration. */
#define ACCELERATION_TH 50

/* Rotation gain. */
#define ROTATION_SPEED_GAIN 1


/* Variables -----------------------------------------------------------------*/

/* Accelerometer-Gyroscope Component. */
#define DEV_I2C Wire
#define SerialPort Serial
LSM6DSLSensor *acc_gyr;

/* Motor Control Expansion Board. */
SPIClass *dev_spi;
XNucleoIHM02A1 *x_nucleo_ihm02a1;
L6470 **motors;

/* Initialization parameters of the motors connected to the expansion board. */
L6470_init_t L6470_init[L6470DAISYCHAINSIZE] = {
    /* First Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        1000.0,                        /* Motor maximum speed [step/s]. */
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
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        1000.0,                        /* Motor maximum speed [step/s]. */
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

/* Defaults. */
int status;
int speed;


/* Functions -----------------------------------------------------------------*/

/**
 * @brief  Initialization.
 * @param  None.
 * @retval None.
 */
void setup() {
  /* Printing to the console. */
  SerialPort.begin(115200);
  SerialPort.print("Motor Control with MEMS\r\n\n");

  /* Aliveness. */
  pinMode(D13, OUTPUT);

  // Initialize I2C bus.
  DEV_I2C.begin();

  /* Initializing Accelerometer-Gyroscope component. */
  acc_gyr = new LSM6DSLSensor(&DEV_I2C);
  acc_gyr->Enable_X();

  /* Initializing SPI bus. */
  dev_spi = new SPIClass(D11, D12, D3);

  /* Initializing Motor Control Expansion Board. */
  x_nucleo_ihm02a1 = new XNucleoIHM02A1(&L6470_init[0], &L6470_init[1], A4, A5, D4, A2, dev_spi);

  /* Building a list of motor control components. */
  motors = x_nucleo_ihm02a1->get_components();

  /* Set defaults. */
  status = 0;
  speed = 0;
}

/**
 * @brief  Main loop.
 * @param  None.
 * @retval None.
 */
void loop() {
  /* Aliveness. */
  digitalWrite(D13, HIGH);
  delay(250);
  digitalWrite(D13, LOW);
  delay(250);

  /* Main Loop. */
  while(true) {
    /* Reading Accelerometer. */
    int32_t accelerometer_data[3];
    acc_gyr->Get_X_Axes(accelerometer_data);

    /* Motor Control. */
    int module = abs(accelerometer_data[ACCELERATION_AXIS]);
    if (module > ACCELERATION_TH) {
      int sign = accelerometer_data[ACCELERATION_AXIS] < 0 ? -1 : 1;
      speed = module * ROTATION_SPEED_GAIN;

      /* Requesting to run. */
      motors[0]->run(sign == -1 ? StepperMotor::BWD : StepperMotor::FWD, speed);
      status = sign;

      /* Printing to the console. */
      SerialPort.print("Speed: ");
      SerialPort.print(sign == -1 ? '-' : '+');
      SerialPort.print(motors[0]->get_speed());
      SerialPort.print("\r\n");
    } else if (status != 0) {
      /* Requesting to stop. */
      motors[0]->soft_stop();
      status = 0;
      speed = 0;

      /* Printing to the console. */
      SerialPort.print("Stop.\r\n");
    }

    /* Waiting. */
    delay(50);
  }
}
