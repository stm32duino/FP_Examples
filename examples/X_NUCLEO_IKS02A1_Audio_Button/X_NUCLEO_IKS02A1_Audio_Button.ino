/* This example was developed for Nucleo F401RE or L476RG only. If you want to use other Nucleo or Architecture 
 * you have to implement the dedicated file to manage the low level PDM part
 */
#if !defined(ARDUINO_NUCLEO_L476RG) && !defined(ARDUINO_NUCLEO_F401RE)
#error "This example is only for STM32 Nucleo-F401RE or Nucleo-L476RG!"
#endif

#include "Arduino.h"
#include "PDM.h"
#include "pdm2pcm.h"
#include "WaveEncoder.h"
#include "SD.h"
#include <CMSIS_DSP.h>

/* PDM */
uint16_t sampleBuffer[(((AUDIO_IN_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)/2)] = {0};
int count = 1;
String file_name = "";

/* Button */
bool is_start = false; /* true means start, false means stop */
volatile bool button = false;

#ifdef USER_BTN
  const int buttonPin = USER_BTN;
#else
  const int buttonPin = 5;
#endif

int PushButtonState = LOW;

/* PDM2PCM */
#define PCM_BUFLEN    (AUDIO_IN_FREQ/AUDIO_IN_DECIMATOR_FACTOR) /* AUDIO_IN_FREQ = 1280, AUDIO_IN_DECIMATOR_FACTOR = 80  so PCM_BUFLEN = 1280/80 = 16*/
int16_t pcmSamples[PCM_BUFLEN*N_MS_PER_INTERRUPT];

/* SD Card */
File myFile;
uint8_t pHeader[44];
uint32_t byteswritten = 44;

void setup() {
  /* Serial */
  Serial.begin(115200);
  
  /* Button Event Setting */
  pinMode(buttonPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), play_stop, FALLING);

  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  PushButtonState = (digitalRead(buttonPin)) ?  LOW : HIGH;

  /* SD Card Initialization */
  if (!SD.begin(12000000, 10)) {
    Serial.println("Failed to initialize SD!\n");
    while(1);
  }
  
  /* Enable and Start */
  if (PDM.Begin() != PDM_OK)
  {
    Serial.println("Failed to start PDM!\n");
  }
  else
  {
    Serial.println("PDM correctly initialized!\n");
  }

  /* Function to do with data */
  PDM.onReceive(foo);

  /* Initialize PDM2PCM */
  pdm2pcm_init(BYTE_LEFT_MSB, PDM_ENDIANNESS_BE, SINC4);
  if(pdm2pcm_volume(5))  //set volume (0-6) 
    Serial.println("Volume not correct!\n"); 
}

void loop() {
  if(button){
    /* Debouncing */
    delay(50);

    /* Wait until the button is released */
    while ((digitalRead(buttonPin) == PushButtonState));

    /* Debouncing */
    delay(50);
    
    if(!is_start){
      
      Serial.println("Setting File");
      
      /*Inizialization File */
      file_name = "sample_" + String(count++) + ".wav";
      byteswritten = 44;
      
      /*Inizialization File */
      if (SD.exists(file_name)) {
        SD.remove(file_name);
        Serial.println("File Already Exists - File Removed");
      }
    
      if (!(myFile = SD.open(file_name, FILE_WRITE))) {
        Serial.println("Failed to initialize File\n");
        while(1);
      }
    
      WAV.header_init(pHeader, 16000, 16);
    
      myFile.write(pHeader, 44);
      myFile.close();
      Serial.flush();

      is_start = true;
      Serial.println("RECORDING");
      PDM.Record(sampleBuffer);
      
    }else{
      is_start = false;
      if (PDM.Stop() == PDM_ERROR)
      {
        Serial.println("PDM_ERROR");
      }
      Serial.println("STOP");
      
      /* Update and Writing header */
      if (!(myFile = SD.open(file_name, O_WRITE))) {
        Serial.println("Failed to Open File\n");
        while(1);
      }
    
      WAV.header_update(pHeader, &byteswritten);
      if (!myFile.seek(0)) {
        Serial.println("Failed to Update Header\n");
        while(1);
      }
      myFile.write(pHeader, 44);
      myFile.close();   

    }
    button = false;
  }
}


extern "C" void foo()
{
  for (uint32_t i = 0; i < N_MS_PER_INTERRUPT; i++)
  {
    pdm2pcm((uint8_t *) & (sampleBuffer[i * ((BLOCK_SIZE/8)/2)]), &pcmSamples[i * PCM_BUFLEN], BLOCK_SIZE);
  }

  /* Writing on SD */
  myFile = SD.open(file_name, O_WRITE);
  myFile.seek(byteswritten);
  for (uint32_t i = 0; i < PCM_BUFLEN * N_MS_PER_INTERRUPT; i++)
  {
    uint16_t data = (uint16_t)pcmSamples[i];

    myFile.write((uint8_t)data);
    myFile.write((uint8_t)(data>>8));
    
  }
  
  byteswritten += PCM_BUFLEN * N_MS_PER_INTERRUPT * sizeof(int16_t);
  myFile.flush();
  myFile.close();  
}

void play_stop()
{
  button = true;
}
