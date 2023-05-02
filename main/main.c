//---------- CODE LIBRARIES ---------------
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"

//---------- QUEUE FOR DATA TO BE SENT VIA GRPS ---------------
QueueHandle_t queue; // Queue handler

//---------- CODE DEFINITIONS ---------------

static const int RX_BUF_SIZE = 1024;
static uint8_t gprs_state = 0;

#define TXD_PIN 16 //UART TX PIN
#define RXD_PIN 17 //UART RX PIN 

//STRUCT FOR WIFI PACKAGE RECEIVED
typedef struct{
  uint8_t FrameType;
  uint8_t FrameSubType;
  uint8_t Channel;
  uint8_t TxAddr[6];
  int8_t RSSI;
  uint8_t lenSSID;
  uint8_t SSID[32];
}wifi_rx_packet_t;

//---------- CODE HELP FUNCTIONS ---------------

//Function that will pront a received package
//for TB only
void printPacket(wifi_rx_packet_t msg)
{
  ESP_LOGI("wifi", "Type: %x, SubType: %x, Channel: %x, RSSI: %d",msg.FrameType, msg.FrameSubType, msg.Channel, msg.RSSI);
  /*
  printf("Type: %x, SubType: %x, Channel: %x, RSSI: %d\n",msg.FrameType, msg.FrameSubType, msg.Channel, msg.RSSI);
  printf("txAddr:");
  for(uint8_t i=0; i<6; i++)
  {
    printf("%02x",msg.TxAddr[i]);
  }
  printf(" SSID:");
  for(uint8_t i=0; i<32; i++)
  {
    printf("%02x",msg.SSID[i]);
  }
  printf("\n");
  */
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

//### TX UART function ###
static void sendData(const char* data)
{
    //uart_flush_input(UART_NUM_1);
    const uint8_t len = strlen(data);
    const uint8_t txBytes = uart_write_bytes(UART_NUM_1, data, len);
    printf("Sent: %s", data);
    //ESP_LOGI("UART TX", "Wrote %d bytes", txBytes);
}

//### Function to handle the AT commands State Machine ###
void gprs_validation(char* data)
{
  if(strchr((char*) data,'K'))
  {
    switch (gprs_state)
    {
      case 0:
        gprs_state = 1;
        sendData("AT\r"); // check connection is ok
      break;
      case 1:
        sendData("AT+CPIN?\r"); // check sim card is in
        gprs_state = 2;
      break;
      case 2:
        sendData("AT+CGREG?\r"); // check is connected to network
        gprs_state = 3;
      break;
      case 3:
        sendData("AT+COPS?\r"); // check the network the device is connected to
        gprs_state = 4;
      break;
      case 4:
        sendData("AT+CGATT=1\r"); // perform GPRS attach
        gprs_state = 5;
      break;
      case 5:
        sendData("AT+CGPADDR=1\r"); // check IP address
        gprs_state = 0;
      break;
      case 6:
        sendData('AT+CIPCSGP=1,"internet.movistar.com.ec"."movistar","movistar"\r');
        // set APN
        gprs_state = 7;
      break;
      case 7:
        sendData('AT+CIPSTART="TCP","google.com.vn","80"\r'); // set destination URL
        gprs_state = 8;
      break;
      case 8:
        sendData('AT+CIPSEND\r'); // send Data
        gprs_state = 0;
      break;
    }
  }
  else
  {
    gprs_state = 0;
    sendData("AT\r");
  }
}

//### RX UART function ###
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            printf("\nReceived: %s\n",data);
            gprs_validation((char*) data);
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

//### Function to handle when a new promiscuous package arrives ###
void wifi_promiscuous_callback(void *buf, wifi_promiscuous_pkt_type_t type)
{
  uint8_t ssid_length = 0;
  wifi_rx_packet_t rxPacket;
  wifi_promiscuous_pkt_t *rxBuffer = (wifi_promiscuous_pkt_t *)buf;

  //Zero all informacion in the SSID slot
  for(uint8_t i=0; i<32; i++)
  {
    rxPacket.SSID[i] = 0x0;
  }
  
  //Filter for MANAGMENT packets only
  if (type == WIFI_PKT_MGMT)
  {
    //Copy 802.11 data to the RX packet struct
    rxPacket.FrameType = (rxBuffer->payload[0] & 0xC) >> 2;
    rxPacket.FrameSubType = rxBuffer->payload[0] >> 4;
    rxPacket.Channel = rxBuffer->rx_ctrl.channel;
    rxPacket.RSSI = rxBuffer->rx_ctrl.rssi;
    for(uint8_t i=0; i<6; i++)
    {
      rxPacket.TxAddr[i] = rxBuffer->payload[i+11];
    }
    ssid_length = rxBuffer->payload[37]; // SSID length is in location 37 in the payload
    rxPacket.lenSSID = ssid_length;
    for(uint8_t i=0; i<ssid_length; i++) // SSID can have a max of 32 octets
    {
      rxPacket.SSID[i] = rxBuffer->payload[i+38];
    }
    //---> Filter to execute only for Probe Requests Received
    //---> Here is where an MQTT update must happen
    if(rxPacket.FrameSubType == 0x5)
    {
      //printPacket(rxPacket);
      //sendData("AT\r");
      if (xQueueSend(queue, &rxPacket, 1000 / portTICK_PERIOD_MS)) // Added new data to queue
      {
        //printf("added message to queue\n");
      }
      else
      {
        printf("failed to add message to queue\n");
      }
      /* p
      for (uint16_t i = 0; i < rxBuffer->rx_ctrl.sig_len; i++)
      {
        printf("%02x", rxBuffer->payload[i]);
        //printf("Data %d : %x\n",i , (rxBuffer->payload[i]));
      }
      */
    }
  }
}

//### LED blink function ###
static void blink(void *arg)
{
  // Blinky LED
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  uint8_t level = 0;
  esp_log_level_set("UART TX", ESP_LOG_INFO);
  
  wifi_rx_packet_t rxPacket;

  while (1)
  {
    gpio_set_level(GPIO_NUM_2, level);
    level = !level;
    //sendData("AT+COPS?\r");

    // Queue receiver
    if (xQueueReceive(queue, &rxPacket, 5000 / portTICK_PERIOD_MS))
    {
      printPacket(rxPacket);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  //Initiate FLASH memory
  nvs_flash_init();

  //Initiate modules for WiFi handling
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_callback));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  //Initiate UART configurations
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  if (0 != esp_wifi_set_channel(1, 0))
  {
    printf(" wifi channel : 1\r\n");
  }
  
  //Create queue for data to be sent via GPRS
  queue = xQueueCreate(15, sizeof(wifi_rx_packet_t));

  //Create task for main functionallity
  xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(blink, "blink_task", 1024, NULL, configMAX_PRIORITIES-1, NULL);

  //Start GPRS state machine
  gprs_validation("OK");
}
