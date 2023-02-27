#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"

typedef struct{
  uint8_t FrameType;
  uint8_t FrameSubType;
  uint8_t Channel;
  uint8_t TxAddr[6];
  int8_t RSSI;
  uint8_t SSID[32]
}wifi_rx_packet_t;

void printPacket(wifi_rx_packet_t msg)
{
  printf("Type: %x, SubType: %x, Channel: %x, RSSI: %d\n",msg.FrameType, msg.FrameSubType, msg.Channel, msg.RSSI);
  printf("txAddr:");
  for(uint8_t i=0; i<6; i++)
  {
    printf("%02x",msg.TxAddr[i]);
  }
  printf(" SSID:");
  for(uint8_t i=0; i<32; i++)
  {
    //printf("%02x",msg.SSID[i]);
    printf("%c",);
  }
  printf("\n");
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

void wifi_promiscuous_callback(void *buf, wifi_promiscuous_pkt_type_t type)
{
  uint8_t lenSSID = 0;
  wifi_rx_packet_t rxPacket;
  for(uint8_t i=0; i<32; i++)
  {
    rxPacket.SSID[i] = 0x0;
  }
  wifi_promiscuous_pkt_t *rxBuffer = (wifi_promiscuous_pkt_t *)buf;
  if (type == WIFI_PKT_MGMT)
  {
    rxPacket.FrameType = (rxBuffer->payload[0] & 0xC) >> 2;
    rxPacket.FrameSubType = rxBuffer->payload[0] >> 4;
    rxPacket.Channel = rxBuffer->rx_ctrl.channel;
    rxPacket.RSSI = rxBuffer->rx_ctrl.rssi;
    for(uint8_t i=0; i<6; i++)
    {
      rxPacket.TxAddr[i] = rxBuffer->payload[i+11];
    }
    lenSSID = rxBuffer->payload[37]; // SSID length is in location 37 in the payload
    for(uint8_t i=0; i<lenSSID; i++) // SSID can have a max of 32 octets
    {
      rxPacket.SSID[i] = rxBuffer->payload[i+38];
    }
    if(rxPacket.FrameSubType == 0x5)
    {
      printf("Length: %d\n", rxBuffer->rx_ctrl.sig_len);
      printPacket(rxPacket);
      /*
      for (uint16_t i = 0; i < rxBuffer->rx_ctrl.sig_len; i++)
      {
        printf("%02x", rxBuffer->payload[i]);
        //printf("Data %d : %x\n",i , (rxBuffer->payload[i]));
      }
      */
      printf("\r\n");
    }
  }
}

void app_main(void)
{
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_callback));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  int level = 0;

  if (0 != esp_wifi_set_channel(1, 0))
  {
    printf(" wifi channel : 1\r\n");
  }

  while (true)
  {
    gpio_set_level(GPIO_NUM_2, level);
    level = !level;
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}