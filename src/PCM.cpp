/* Includes ------------------------------------------------------------------------*/
#include <PCM.h>

/* Define---------------------------------------------------------------------------*/

/* Class Implementation -------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Constructor */
PCMClass::PCMClass()
{
}

/* Methods */

/**
 *  @brief Initialize microphone and I2S/SAI parameter
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::Begin()
{
  int ret = PCM_OK;

  /* Context */
  mic.Device = 0;
  mic.SampleRate = AUDIO_IN_SAMPLING_FREQUENCY; /*16k*/
  mic.State = AUDIO_IN_STATE_RESET;

  ret = pcm_low_level_init();

  // Update State to STOP
  mic.State = AUDIO_IN_STATE_STOP;

  return ret;
}

/**
 *  @brief Deinizialize microphone and I2S/SAI parameter
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::End()
{
  int ret = PCM_OK;

  ret = pcm_low_level_deinit();

  mic.State = AUDIO_IN_STATE_RESET;

  return ret;
}

/**
 *  @brief  DMA Half Callback
 *  @param None
 *  @retval None
 */
void PCMClass::DMAHalfCpltCallback(void)
{
  pcm_low_level_copy_half_cplt(mic.pBuff);

  _onReceive();
}

/**
 *  @brief  DMA Complete Callback
 *  @param None
 *  @retval None
 */
void PCMClass::DMACpltCallback(void)
{
  pcm_low_level_copy_cplt(mic.pBuff);

  _onReceive();
}

/**
 *  @brief Start recording
 *  @param pBuf: pointer to a buffer to store data
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::Record(int16_t *pBuf)
{
  int ret = PCM_OK;

  mic.pBuff = pBuf;

  ret = pcm_low_level_record();

  /* Update State */
  mic.State = AUDIO_IN_STATE_RECORDING;

  return ret;
}

/**
 *  @brief Stop recording
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::Stop()
{
  int ret = PCM_OK;

  ret = pcm_low_level_stop();

  /* Update AUDIO IN state */
  mic.State = AUDIO_IN_STATE_STOP;
  /* Return Status */
  return ret;
}

/**
 *  @brief Pause recording
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::Pause()
{
  int ret = PCM_OK;

  ret = pcm_low_level_pause();

  /* Update AUDIO IN state */
  mic.State = AUDIO_IN_STATE_PAUSE;

  /* Return Status */
  return ret;
}

/**
 *  @brief Resume recording
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int PCMClass::Resume()
{
  int ret = PCM_OK;

  ret = pcm_low_level_resume();

  /* Update AUDIO IN state */
  mic.State = AUDIO_IN_STATE_RECORDING;

  /* Return Status */
  return ret;
}

/**
 *  @brief Star recording
 *  @param status: pointer where to save the status
 *  @retval None
 */
void PCMClass::GetState(int *status)
{
  *status = mic.State;
}

/**
 *  @brief Set user function
 *  @param function: pointer to a function
 *  @retval None
 */
void PCMClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

#ifdef __cplusplus
extern "C" {
#endif

/* Low Level Functions: This functions are empty and must be implemented in the platform dependent low level driver */
__attribute__((weak)) int pcm_low_level_init(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_low_level_deinit(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_low_level_record(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_low_level_stop(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_low_level_pause(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_low_level_resume(void)
{
  return PCM_ERROR;
}

__attribute__((weak)) void pcm_low_level_copy_half_cplt(int16_t *pBuf)
{
  (void)pBuf;
}

__attribute__((weak)) void pcm_low_level_copy_cplt(int16_t *pBuf)
{
  (void)pBuf;
}

#ifdef __cplusplus
}
#endif


PCMClass PCM;
