/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PCM_H__
#define __PCM_H__

/* Includes -----------------------------------------------------------------------*/
#include <Arduino.h>
#include "pcm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Weak Functions - They must be implemented by the user */
int pcm_low_level_init(void);
int pcm_low_level_deinit(void);
int pcm_low_level_record(void);
int pcm_low_level_stop(void);
int pcm_low_level_pause(void);
int pcm_low_level_resume(void);
void pcm_low_level_copy_half_cplt(int16_t *pBuf);
void pcm_low_level_copy_cplt(int16_t *pBuf);

#ifdef __cplusplus
}
#endif


/* Defines ------------------------------------------------------------------------*/

/* Typedefs -----------------------------------------------------------------------*/
/* Microphone Parameter*/
typedef struct {
  uint32_t Device;
  uint32_t SampleRate;
  int16_t *pBuff;
  uint32_t State;
} AudioIn_Ctx_t;


#define PCM_OK      0
#define PCM_ERROR   1

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/* Class Declaration --------------------------------------------------------------*/

class PCMClass {
  public:
    /* Constructor */
    PCMClass();

    /* Functions */
    int Begin();
    int End();
    int Record(int16_t *pBuf);
    int Stop();
    int Pause();
    int Resume();
    void GetState(int *status);

    /* Interrupt */
    void onReceive(void(*)(void));

    /* private */
    void DMAHalfCpltCallback(void);
    void DMACpltCallback(void);

  private:

    void (*_onReceive)(void);

    AudioIn_Ctx_t mic;
};

extern PCMClass PCM;

#endif
