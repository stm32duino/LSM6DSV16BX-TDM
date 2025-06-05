#ifndef __PCM_CONFIG_DEFAULT_H__
#define __PCM_CONFIG_DEFAULT_H__

/*Number of millisecond of audio at each DMA interrupt*/
#ifndef N_MS_PER_INTERRUPT
  #define N_MS_PER_INTERRUPT              (2)
#endif /* N_MS_PER_INTERRUPT */

#ifndef AUDIO_IN_SAMPLING_FREQUENCY
  #define AUDIO_IN_SAMPLING_FREQUENCY     (16000)  /* It could be 16000 or 8000 */
#endif /* AUDIO_IN_SAMPLING_FREQUENCY */

#endif /* __PCM_CONFIG_DEFAULT_H__ */
