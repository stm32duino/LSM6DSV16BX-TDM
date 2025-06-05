/**
******************************************************************************
* @file    LSM6DSV16BX_TDM_Python_Wave_Serial_Encoder.ino
* @author  STMicroelectronics
* @version V1.0.0
* @date    5 June 2025
* @brief   Arduino test application for STMicroelectronics LSM6DSV16BX
*          IMU sensor with TDM.
*          This application makes use of C++ classes obtained from the C
*          components' drivers.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
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
/*
 * To use this example you need to connect the STEVAL-MKI237KA eval board to the NUCLEO-U575ZI-Q board with wires as explained below:
 * pin 1 (VDD) of the STEVAL-MKI237KA eval board connected to pin 3V3 of the NUCLEO-U575ZI-Q board
 * pin 2 (VDDIO) of the STEVAL-MKI237KA eval board connected to pin IOREF of the NUCLEO-U575ZI-Q board
 * pin 3 (WCLK) of the STEVAL-MKI237KA eval board connected to pin FS of SAI_B of the NUCLEO-U575ZI-Q board
 * pin 4 (BCLK) of the STEVAL-MKI237KA eval board connected to pin SCK of SAI_B of the NUCLEO-U575ZI-Q board
 * pin 6 (TDM) of the STEVAL-MKI237KA eval board connected to pin SD of SAI_B of the NUCLEO-U575ZI-Q board
 * pin 13 (GND) of the STEVAL-MKI237KA eval board connected to GND of the NUCLEO-U575ZI-Q board
 * pin 19 (CS) of the STEVAL-MKI237KA eval board connected to pin AVDD of the NUCLEO-U575ZI-Q board
 * pin 20 (SCL) of the STEVAL-MKI237KA eval board connected to pin D15 SCL of the NUCLEO-U575ZI-Q board
 * pin 21 (SDA) of the STEVAL-MKI237KA eval board connected to pin D14 SDA of the NUCLEO-U575ZI-Q board
 */

/* This example was developed for NUCLEO-U575ZI-Q only. If you want to use other Nucleo or Architecture
 * you have to implement the dedicated file to manage the low level PCM part
 */
#if !defined(ARDUINO_NUCLEO_U575ZI_Q)
  #error "This example is only for STM32 NUCLEO-U575ZI-Q!"
#endif

#include "Arduino.h"
#include "LSM6DSV16BXSensor.h"
#include "PCM.h"
#include "iir2.h"

/* I2C_ADD_L means SA0 connected to GND */
LSM6DSV16BXSensor sensor(&Wire, LSM6DSV16BX_I2C_ADD_H);
/* Enable only z-axis */
lsm6dsv16bx_tdm_xl_axis_t tdm_axis = {.x = 0, .y = 0, .z = 1};

struct iir2 iir2;
const float b[2][3] = {
  { 0.972613898499844f, -1.945227796999688f, 0.972613898499844f },
  { 0.986211924627082f, -1.972423849254165f, 0.986211924627082f },
};
const float a[2][3] = {
  { 1.000000000000000f, -1.944477657767094f, 0.945977936232282f },
  { 1.000000000000000f, -1.972233729195266f, 0.972613969313063f },
};

static float audio_gain = 64.0f;

#define TDM_RATE_8KHZ 0
#define TDM_RATE_16KHZ 1

/* PCM */
int16_t sampleBuffer[(((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * N_MS_PER_INTERRUPT) / 2)] = {0};
int16_t audio_data[(((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * N_MS_PER_INTERRUPT) / 2)] = {0};
bool recording = false;
byte inByte = 0;

static void tdm_audio_filter_data(struct iir2 *filter, uint16_t len, uint16_t offset);

volatile uint8_t interrupt = 0;

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();

  /* Serial */
  Serial.begin(921600);
  Serial.flush();
  while (Serial.available() > 0) {
    Serial.read();
  }

  iir2_init(&iir2, b[TDM_RATE_16KHZ], a[TDM_RATE_16KHZ]); // init hp filter (fc = 50 Hz)
  sensor.begin();
  sensor.Disable_X();
  sensor.Set_TDM_XL_Axis(tdm_axis);
  sensor.Set_TDM_WCLK_BCLK(LSM6DSV16BX_WCLK_16kHZ_BCLK_2048kHz);
  sensor.Set_TDM_XL_Full_Scale(LSM6DSV16BX_TDM_8g);
  sensor.Set_X_Power_Mode(LSM6DSV16BX_XL_HIGH_PERFORMANCE_TDM_MD);

  /* Enable and Start */
  if (PCM.Begin() != PCM_OK) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  /* Function to execute with data */
  PCM.onReceive(foo);
}

void loop()
{
  while (Serial.available() > 0) {
    inByte = Serial.read();
  }

  if (inByte == 0x00) {
    recording = 0;
    PCM.Stop();
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (inByte == 0x01 && recording == 0) {
    PCM.Record(sampleBuffer);
    recording = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (interrupt == 1 && recording == 1) {
    interrupt = 0;
    Serial.write((const char *)audio_data, sizeof(audio_data));
    Serial.flush();
  }
}

static void tdm_audio_filter_data(struct iir2 *filter, uint16_t len, uint16_t offset)
{
  for (uint16_t i = 0; i < len; i++) {
    if (audio_gain == 0.0f) {
      audio_data[offset + i] = sampleBuffer[offset + i];
    } else {
      float zf = iir2_run(filter, sampleBuffer[offset + i]) * audio_gain;
      zf = zf > INT16_MAX ? INT16_MAX : zf;
      zf = zf < INT16_MIN ? INT16_MIN : zf;
      audio_data[offset + i] = (int16_t)zf;
    }
  }
}

extern "C" void foo()
{
  tdm_audio_filter_data(&iir2, (((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * N_MS_PER_INTERRUPT) / 2), 0);
  interrupt = 1;
}
