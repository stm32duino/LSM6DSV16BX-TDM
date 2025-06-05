/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PCM_CONFIG_H__
#define __PCM_CONFIG_H__

#if __has_include("pcm_config_custom.h")
  #include "pcm_config_custom.h"
#else
  #include "pcm_config_default.h"
#endif

#endif /* __PCM_CONFIG_H__ */
