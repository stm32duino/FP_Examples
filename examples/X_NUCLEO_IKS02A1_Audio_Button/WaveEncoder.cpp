/* Includes ------------------------------------------------------------------------*/
#include <WaveEncoder.h>
#include <string.h>

/* Class Implementation -----------------------------------------------------------------------------*/
/* Constructor */
WaveClass::WaveClass()
{
}

/* Function */

/**
 *  @brief Initialize header
 *  @param pHeader pointer to a buffer with the parameter of the header
 *  @param SampleRate value of the sample rate of the sensor
 *  @param BitsPerSample value of the bits per sample of the sensor
 */
void WaveClass::header_init(uint8_t *pHeader, uint32_t SampleRate, uint32_t BitsPerSample)
{
  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeader[0] = 'R';
  pHeader[1] = 'I';
  pHeader[2] = 'F';
  pHeader[3] = 'F';

  /* Write the file length ---------------------------------------------------
  /* The sampling time: this value will be be written back at the end of the
  *  recording operation.
  *  Example: 44 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC
  */
  pHeader[4] = 0x2C;
  pHeader[5] = 0x00;
  pHeader[6] = 0x00;
  pHeader[7] = 0x00;

  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pHeader[8]  = 'W';
  pHeader[9]  = 'A';
  pHeader[10] = 'V';
  pHeader[11] = 'E';

  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pHeader[12]  = 'f';
  pHeader[13]  = 'm';
  pHeader[14]  = 't';
  pHeader[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pHeader[16]  = 0x10;
  pHeader[17]  = 0x00;
  pHeader[18]  = 0x00;
  pHeader[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pHeader[20]  = 0x01;
  pHeader[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pHeader[22]  = 0x01;
  pHeader[23]  = 0x00;

  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00 */

  pHeader[24]  = (uint8_t)((SampleRate & 0xFF));
  pHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
  pHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
  pHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);

  /* Write the Byte Rate // SampleRate * (BitPerSample/8) -----------------------------------------------------*/

  uint32_t ByteRate = SampleRate * (BitsPerSample / 8);
  pHeader[28]  = (uint8_t)((ByteRate & 0xFF));
  pHeader[29]  = (uint8_t)((ByteRate >> 8) & 0xFF);
  pHeader[30]  = (uint8_t)((ByteRate >> 16) & 0xFF);
  pHeader[31]  = (uint8_t)((ByteRate >> 24) & 0xFF);

  /* Write the block alignment -----------------------------------------------*/
  uint32_t BlockAlign = (BitsPerSample / 8);
  pHeader[32]  = BlockAlign; //NbrChannels * (BitPerSample/8)
  pHeader[33]  = 0x00;

  /* Write the number of bits per sample (16)-------------------------------------*/
  pHeader[34]  = BitsPerSample;
  pHeader[35]  = 0x00;

  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pHeader[36]  = 'd';
  pHeader[37]  = 'a';
  pHeader[38]  = 't';
  pHeader[39]  = 'a';

  /* Write the number of sample data NumSamples * Ch * (BitsPerSample/8)-----------------------------------------*/
  /* This variable will be written back at the end of the recording operation -> 0 */
  pHeader[40]  = 0x00;
  pHeader[41]  = 0x00;
  pHeader[42]  = 0x00;
  pHeader[43]  = 0x00;
}

/**
 *  @brief Update header
 *  @param pHeader pointer to a buffer with the parameter of the header
 *  @param byteswritten pointer to the value of the bytes written
 */
void WaveClass::header_update(uint8_t *pHeader, uint32_t *byteswritten)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
  recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = (uint8_t) * byteswritten; //<-- man mano che scrivo salva qua dentro i byte scritti
  pHeader[5] = (uint8_t)(*byteswritten >> 8);
  pHeader[6] = (uint8_t)(*byteswritten >> 16);
  pHeader[7] = (uint8_t)(*byteswritten >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pHeader[40] = (uint8_t)(*byteswritten - 44);
  pHeader[41] = (uint8_t)((*byteswritten - 44) >> 8);
  pHeader[42] = (uint8_t)((*byteswritten - 44) >> 16);
  pHeader[43] = (uint8_t)((*byteswritten - 44) >> 24);

}

WaveClass WAV;
