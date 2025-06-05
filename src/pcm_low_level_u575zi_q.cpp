#if defined(ARDUINO_NUCLEO_U575ZI_Q)

#include "PCM.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t SAI1_client = 0;
SAI_HandleTypeDef hsai_BlockB1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
int16_t SAI_InternalBuffer[((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * N_MS_PER_INTERRUPT)];
uint32_t SAI_InternalBufferSize = ((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * N_MS_PER_INTERRUPT);

static void MX_GPDMA1_Init(void);
static int MX_SAI1_Init(void);

/* Low Level Functions implementation */

/**
 *  @brief PCM initialization
 *  @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_init(void)
{
  int ret = PCM_OK;
  MX_GPDMA1_Init();
  ret = MX_SAI1_Init();
  return ret;
}

/**
 *  @brief PCM deinitialization
  * @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_deinit(void)
{
  if (HAL_SAI_DeInit(&hsai_BlockB1) != HAL_OK) {
    return PCM_ERROR;
  }

  return PCM_OK;
}

/**
 *  @brief Start recording
  * @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_record(void)
{
  if (HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)SAI_InternalBuffer, (uint16_t)SAI_InternalBufferSize) != HAL_OK) {
    return PCM_ERROR;
  }

  return PCM_OK;
}

/**
 *  @brief Stop recording
  * @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_stop(void)
{
  if (HAL_SAI_DMAStop(&hsai_BlockB1) != HAL_OK) {
    return PCM_ERROR;
  }

  return PCM_OK;
}

/**
 *  @brief Pause recording
  * @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_pause(void)
{
  if (HAL_SAI_DMAPause(&hsai_BlockB1) != HAL_OK) {
    return PCM_ERROR;
  }

  return PCM_OK;
}

/**
 *  @brief Resume recording
  * @param None
 *  @retval 1 if success, 0 otherwise
 */
int pcm_low_level_resume(void)
{
  if (HAL_SAI_DMAResume(&hsai_BlockB1) != HAL_OK) {
    return PCM_ERROR;
  }

  return PCM_OK;
}

/**
 *  @brief Half Callback
 *  @param pBuf: pointer to a buffer to store data
 *  @retval None
 */
void pcm_low_level_copy_half_cplt(int16_t *pBuf)
{
  uint32_t index;
  int16_t *DataTempSAI = SAI_InternalBuffer;

  for (index = 0; index < (SAI_InternalBufferSize / 2U); index++) {
    pBuf[index] = (DataTempSAI[index]);
  }
}

/**
 *  @brief Complet Callback
 *  @param pBuf: pointer to a buffer to store data
 *  @retval None
 */
void pcm_low_level_copy_cplt(int16_t *pBuf)
{
  uint32_t index;
  int16_t *DataTempSAI = &SAI_InternalBuffer[SAI_InternalBufferSize / 2U];

  for (index = 0; index < (SAI_InternalBufferSize / 2U); index++) {
    pBuf[index] = (DataTempSAI[index]);
  }
}

/* This function is defined weak in the Arduino Core, so we can override it */

/**
 *  @brief System Clock Parameters Configuration
  * @param None
  * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    while (1);
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI
                                     | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    while (1);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC | RCC_PERIPHCLK_DAC1
                                       | RCC_PERIPHCLK_CLK48;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
  PeriphClkInit.Dac1ClockSelection = RCC_DAC1CLKSOURCE_LSE;
  PeriphClkInit.IclkClockSelection = RCC_CLK48CLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    while (1);
  }
  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static int MX_SAI1_Init(void)
{
  int ret = PCM_OK;

  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockB1.Init.Mckdiv = 0;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MckOutput = SAI_MCK_OUTPUT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  if (AUDIO_IN_SAMPLING_FREQUENCY == 16000) { // 16 KHz
    hsai_BlockB1.FrameInit.FrameLength = 128;
  } else { // 8 KHz
    hsai_BlockB1.FrameInit.FrameLength = 256;
  }
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 1;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000001;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK) {
    ret = PCM_ERROR;
  }

  return ret;
}

/**
* @brief SAI MSP Initialization
* This function configures the hardware resources used in this example
* @param hsai: SAI handle pointer
* @retval None
*/
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DMA_NodeConfTypeDef NodeConfig;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /* SAI1 */
  if (hsai->Instance == SAI1_Block_B) {
    /* Peripheral clock enable */

    /** Initializes the peripherals clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
    PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLL2.PLL2M = 3;
    PeriphClkInit.PLL2.PLL2N = 16;
    PeriphClkInit.PLL2.PLL2P = 125;
    PeriphClkInit.PLL2.PLL2Q = 2;
    PeriphClkInit.PLL2.PLL2R = 2;
    PeriphClkInit.PLL2.PLL2RGE = RCC_PLLVCIRANGE_1;
    PeriphClkInit.PLL2.PLL2FRACN = 0;
    PeriphClkInit.PLL2.PLL2ClockOut = RCC_PLL2_DIVP;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      while (1);
    }

    if (SAI1_client == 0) {
      __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client ++;

    /* GPIO SAI Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
    NodeConfig.Init.Request = GPDMA1_REQUEST_SAI1_B;
    NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
    NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
    NodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
    NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
    NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
    NodeConfig.Init.SrcBurstLength = 1;
    NodeConfig.Init.DestBurstLength = 1;
    NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    NodeConfig.Init.Mode = DMA_NORMAL;
    NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
    NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
    NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel0) != HAL_OK) {
      while (1);
    }

    if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK) {
      while (1);
    }

    if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK) {
      while (1);
    }

    handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel0.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
    handle_GPDMA1_Channel0.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    handle_GPDMA1_Channel0.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel0) != HAL_OK) {
      while (1);
    }

    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &List_GPDMA1_Channel0) != HAL_OK) {
      while (1);
    }

    __HAL_LINKDMA(hsai, hdmarx, handle_GPDMA1_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK) {
      while (1);
    }

  }
}

/**
* @brief SAI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hsai: SAI handle pointer
* @retval None
*/
void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  /* SAI1 */
  if (hsai->Instance == SAI1_Block_B) {
    SAI1_client --;
    if (SAI1_client == 0) {
      /* Peripheral clock disable */
      __HAL_RCC_SAI1_CLK_DISABLE();
    }

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_8 | GPIO_PIN_9);

    /* SAI1 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
  }
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{
  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hSai)
{
  (void)hSai;
  PCM.DMAHalfCpltCallback();
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hSai)
{
  (void)hSai;
  PCM.DMACpltCallback();
}

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_NUCLEO_U575ZI_Q */
