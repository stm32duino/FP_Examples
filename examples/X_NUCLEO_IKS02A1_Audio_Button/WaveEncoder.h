/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVE_H__
#define __WAVE_H__

/* Includes --------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum {
  WAV_OK = 0,
  WAV_ERROR = -1
} WAVStatus;

/* Class Declaration -----------------------------------------------------------------------------*/
class WaveClass {
  public:
    /* Constructor */
    WaveClass();

    /* Function */
    void header_init(uint8_t *pHeader, uint32_t SampleRate, uint32_t BitsPerSample);
    void header_update(uint8_t *pHeader, uint32_t *byteswritten);

};

extern WaveClass WAV;
#endif
