/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // sprintf için gerekli
#include <string.h> // strlen için gerekli
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */


char tx_buffer[256];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Bu satır, projenin başka yerlerinde tanımlanan SPI1 yapılandırmasını
// bu dosyada kullanabilmemizi sağlar. CubeIDE bunu otomatik oluşturur.
extern SPI_HandleTypeDef hspi1;
/* USER CODE END PV */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Bu tanımlamalar, RFM98W modülünün (içindeki SX1278 çipi) datasheet'inden alınmıştır.
// Karmaşık register adreslerini ezberlemek yerine onlara isim vererek kod okunabilirliğini artırırız.

// RFM98W Register Adresleri
#define REG_VERSION              0x42 // Versiyon bilgisi için
#define REG_OP_MODE              0x01 // Çalışma modunu ayarlamak için
#define REG_FRF_MSB              0x06 // Frekans ayarı (En anlamlı byte)
#define REG_FRF_MID              0x07 // Frekans ayarı (Orta byte)
#define REG_FRF_LSB              0x08 // Frekans ayarı (En anlamsız byte)
#define REG_DIO_MAPPING_1        0x40 // DIO pinlerinin görevlerini atamak için

// RFM98W Çalışma Modları
#define MODE_SLEEP               0x80 // Uyku modu (LoRa modunda)
#define MODE_STDBY               0x81 // Bekleme modu (LoRa modunda)
#define MODE_TX                  0x83 // Gönderme modu (LoRa modunda)
#define MODE_RX_CONTINUOUS       0x85 // Sürekli dinleme modu (LoRa modunda)


// Sadece Alıcı (Receiver) için Gerekli Register'lar
#define REG_RX_NB_BYTES          0x13 // Alınan paketteki byte sayısını tutar
#define REG_FIFO_RX_CURRENT_ADDR 0x10 // Alınan son paketin FIFO'daki başlangıç adresini tutar
#define REG_PKT_SNR_VALUE        0x19 // Sinyal-Gürültü Oranı
#define REG_PKT_RSSI_VALUE       0x1A // Sinyal Gücü




#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_FIFO 0x00
#define REG_IRQ_FLAGS            0x12

// Tablo 3.35'e göre 60 byte'lık Telemetri Veri Paketi Yapısı
typedef struct __attribute__((packed)) {
    uint16_t packet_number;            // 2 byte
    uint8_t  flight_state;             // 1 byte

    // OBC Verileri
    int16_t  obc_roll;                 // 2 byte
    int16_t  obc_pitch;                // 2 byte
    int16_t  obc_yaw;                  // 2 byte
    uint8_t  obc_abs_acceleration;     // 1 byte
    uint8_t  obc_max_abs_acceleration; // 1 byte
    uint8_t  obc_pressure;             // 1 byte
    int16_t  obc_vertical_velocity;    // 2 byte
    int16_t  obc_altitude;             // 2 byte
    int16_t  obc_max_altitude;         // 2 byte
    int8_t   obc_temperature;          // 1 byte

    // BPC Verileri
    int16_t  bpc_roll;                 // 2 byte
    int16_t  bpc_pitch;                // 2 byte
    int16_t  bpc_yaw;                  // 2 byte
    uint8_t  bpc_abs_acceleration;     // 1 byte
    uint8_t  bpc_max_abs_acceleration; // 1 byte
    uint8_t  bpc_pressure;             // 1 byte
    int16_t  bpc_vertical_velocity;    // 2 byte
    int16_t  bpc_altitude;             // 2 byte
    int16_t  bpc_max_altitude;         // 2 byte

    // DAQ Verileri
    uint16_t daq_pressure_1;           // 2 byte
    uint16_t daq_pressure_2;           // 2 byte
    int16_t  daq_temperature;          // 2 byte

    // PRC Verileri
    int16_t  prc_current;              // 2 byte
    int16_t  prc_voltage;              // 2 byte

    // GPS ve Sistem Zamanı
    uint32_t system_time;              // 4 byte
    float    latitude;                 // 4 byte
    float    longitude;                // 4 byte
    uint32_t gps_utc_time;             // 4 byte
} TelemetryPacket_t;






volatile uint8_t rx_done_flag = 0;

TelemetryPacket_t received_data;


void RFM98_Reset()
{
  // Reset pinini 1ms boyunca LOW'a çek
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  // Reset pinini HIGH'a çekerek modülün normal çalışmasına izin ver
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // Modülün kendine gelmesi için kısa bir bekleme
}


void RFM98_Write(uint8_t address, uint8_t value)
{
  // Yazma işlemi için adresin en anlamlı bitini (MSB) 1 yapmamız gerekiyor.
  // Bu, SX1278 datasheet'inde belirtilen bir kuraldır.
  uint8_t write_address = address | 0x80;

  // 1. NSS pinini LOW'a çekerek LoRa modülüyle haberleşmeyi başlat.
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  // 2. Önce yazılacak register'ın adresini gönder.
  HAL_SPI_Transmit(&hspi1, &write_address, 1, HAL_MAX_DELAY);

  // 3. Sonra o adrese yazılacak veriyi gönder.
  HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);

  // 4. NSS pinini HIGH'a çekerek haberleşmeyi sonlandır.
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}








uint8_t RFM98_Read(uint8_t address)
{
  // Okuma işlemi için adresin en anlamlı bitini (MSB) 0 yapmamız gerekiyor.
  // Bu, SX1278 datasheet'inde belirtilen bir kuraldır.
  uint8_t read_address = address & 0x7F;
  uint8_t received_data;

  // 1. NSS pinini LOW'a çekerek LoRa modülüyle haberleşmeyi başlat.
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  // 2. Önce okunacak register'ın adresini gönder.
  HAL_SPI_Transmit(&hspi1, &read_address, 1, HAL_MAX_DELAY);

  // 3. Veriyi okumak için STM32'den anlamsız bir veri (0x00) gönderilir.
  //    Bu gönderim sırasında MISO hattından gelen veri okunur.
  HAL_SPI_Receive(&hspi1, &received_data, 1, HAL_MAX_DELAY);

  // 4. NSS pinini HIGH'a çekerek haberleşmeyi sonlandır.
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

  return received_data;
}










uint8_t LoRa_Init()
{
  // Modülü resetle
  RFM98_Reset();

  // 1. İletişimi doğrula: Versiyon numarasını kontrol et
  if (RFM98_Read(REG_VERSION) != 0x12)
  {
    return 0; // Hata: Modül bulunamadı veya SPI çalışmıyor
  }

  // 2. Modülü yapılandırmak için ayarları YAZmaya başla

  // Ayarları değiştirebilmek için önce Uyku (Sleep) moduna al.
  // Bu bir datasheet kuralıdır: LoRa modu ayarları sadece uyku modunda değiştirilebilir.
  RFM98_Write(REG_OP_MODE, MODE_SLEEP);
  HAL_Delay(10); // Modun değişmesi için kısa bir bekleme

  // Frekansı ayarla (Örnek: 868 MHz)
  // Bu hex değerleri datasheet'teki formülle hesaplanır: Freq = F_STEP * Değer
  // F_STEP = 32MHz / 2^19 ≈ 61.035 Hz
  // Frekansı 433 MHz'e ayarla
  // Hesaplama: 433,000,000 / 61.035 ≈ 7094272 = 0x6C8000
  RFM98_Write(REG_FRF_MSB, 0x6C); // 433 MHz için MSB
  RFM98_Write(REG_FRF_MID, 0x80); // 433 MHz için MID
  RFM98_Write(REG_FRF_LSB, 0x00); // 433 MHz için LSB
  // Diğer temel ayarlar... (Şimdilik standart değerleri kullanabiliriz)
  // - Çıkış gücü: +17dBm
  // - Spreading Factor: 7, Bandwidth: 125kHz (En yaygın ayarlardan biri)
  // - CRC Aktif
  RFM98_Write(0x09, 0xFF); // RegPaConfig -> PA_BOOST, MaxPower, OutputPower
  RFM98_Write(0x1D, 0x72); // RegModemConfig1 -> BW=125kHz, CR=4/5
  RFM98_Write(0x1E, 0x74); // RegModemConfig2 -> SF=7, CRC On
  RFM98_Write(0x26, 0x04); // RegModemConfig3 -> AgcAutoOn


  RFM98_Write(REG_DIO_MAPPING_1,  0x00);


  // Ayarları bitirdikten sonra, bir sonraki komuta hazır olması için Bekleme (Standby) moduna al.
  RFM98_Write(REG_OP_MODE, MODE_STDBY);
  HAL_Delay(10);

  return 1; // Başlatma başarılı
}










/**
  * @brief  Verilen metin mesajını LoRa ile gönderir.
  * @param  message: Gönderilecek karakter dizisi (string).
  */




void LoRa_ReceiveMode()
{
  // DIO0 pinini RxDone kesmesi için ayarla (Init fonksiyonunda zaten yapıldı)
  // RFM98_Write(REG_DIO_MAPPING_1, 0x00);

  // Modülü Sürekli Alım moduna geçir
  RFM98_Write(REG_OP_MODE, MODE_RX_CONTINUOUS);
}






void LoRa_ReadPacket()
{
  // 1. Alınan paketin boyutunu kontrol et
  uint8_t packet_size = RFM98_Read(REG_RX_NB_BYTES);
  if (packet_size != sizeof(TelemetryPacket_t))
  {
      // Hatalı boyutta paket geldi, işlemi iptal et
      return;
  }

  // 2. FIFO okuma başlangıç adresini al
  uint8_t current_fifo_addr = RFM98_Read(REG_FIFO_RX_CURRENT_ADDR);

  // 3. FIFO işaretçisini bu adrese ayarla
  RFM98_Write(REG_FIFO_ADDR_PTR, current_fifo_addr);

  // 4. FIFO'dan veriyi byte byte oku ve struct'a kopyala
  uint8_t* data_ptr = (uint8_t*)&received_data;
  for (int i = 0; i < packet_size; i++)
  {
      data_ptr[i] = RFM98_Read(REG_FIFO);
  }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // LoRa modülünü başlat
  if (LoRa_Init() == 0)
  {
    // Hata durumunda terminale mesaj gönder
    int len = sprintf(tx_buffer, "HATA: LoRa modülü başlatılamadı!\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
    while(1); // Hata durumunda burada bekle
  }

  // Terminale başlangıç mesajı gönderelim
  int len = sprintf(tx_buffer, "LoRa Alıcı Başlatıldı. Veri bekleniyor...\r\n");
  HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);

  // LoRa'yı dinleme moduna al
  LoRa_ReceiveMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    // Bir paket geldi mi diye kontrol et
    if (rx_done_flag)
    {
      // Bayrağı hemen sıfırla ki aynı paketi tekrar işlemeyelim
      rx_done_flag = 0;

      // 1. Gelen verileri sprintf ile buffer'a yazdır
      int message_len = 0;
      message_len += sprintf(tx_buffer + message_len, "--- YENI PAKET ALINDI ---\r\n");
      message_len += sprintf(tx_buffer + message_len, "Paket No: %u\r\n", received_data.packet_number);
      message_len += sprintf(tx_buffer + message_len, "Uçuş Durumu: %u\r\n", received_data.flight_state);
      message_len += sprintf(tx_buffer + message_len, "PRC Voltaj: %.2f V\r\n", received_data.prc_voltage / 100.0);
      message_len += sprintf(tx_buffer + message_len, "Enlem: %f\r\n", received_data.latitude);
      message_len += sprintf(tx_buffer + message_len, "-------------------------\r\n\r\n");

      // 2. Dolu olan buffer'ı tek seferde UART üzerinden gönder
      HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, message_len, HAL_MAX_DELAY);

      // 3. Bir sonraki paketi almak için tekrar dinleme moduna geç
      LoRa_ReceiveMode();
    }
    // Bayrak 1 değilse, işlemci burada bekleyebilir veya başka işler yapabilir.

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO0_Pin PC5 */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RESET_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LORA_DIO0_Pin)
  {
    // RxDone kesmesi geldiğini ana döngüye bildir
    rx_done_flag = 1;

    // Gelen veriyi FIFO'dan oku
    LoRa_ReadPacket();

    // LoRa modülünün içindeki IRQ bayraklarını temizle
    RFM98_Write(REG_IRQ_FLAGS, 0xFF);
  }
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
