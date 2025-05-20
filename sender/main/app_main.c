/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// -------ADDED -------------//
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "esp_system.h"


/*
SPI sender (master) example.

This example is supposed to work together with the SPI receiver. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. The slave makes this pin high as soon as it is
ready to receive/send data. This code connects this line to a GPIO interrupt which gives the rdySem semaphore. The main
task waits for this semaphore to be given before queueing a transmission.
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GPIO_HANDSHAKE      1
#define GPIO_MOSI           4
#define GPIO_MISO           5
#define GPIO_SCLK           6
#define GPIO_CS             7

#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST HSPI_HOST
#else
#define SENDER_HOST SPI2_HOST
#endif

//-----------------------ADDED------------------------//
//#define EOF                  -1
#define FRE                  -2

#define BUF_SIZE            128
#define CMD_SIZE            1

static const char *TAG = "spiMaster";

int binary_file_length = 0;

char file_name[] = "/spiffs/simple_ota.bin";

static FILE* open_file()
{
    ESP_LOGI(TAG, "Reading file");

    // Open for reading hello.txt
    FILE* file = fopen("/spiffs/simple_ota.bin", "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file");
        return NULL;
    }

    return file;
}

static uint8_t read_file_chunk(FILE* file, uint8_t* sendbuf){

    //allocating memory on core and clearing previous
    memset(sendbuf, 0, BUF_SIZE - CMD_SIZE);

    //CMD handling headers here
    //cmd to end ota
    sendbuf[0] = 129;

    //read in chunk at current fread pointer   
    fread(sendbuf + CMD_SIZE, 1, BUF_SIZE - CMD_SIZE, file);

    #ifdef DEBUG_SENDBUFFER
    printf("Last byte: %02X\n", sendbuf[BUF_SIZE-1]);
    printf("First data byte: %02X\n", sendbuf[CMD_SIZE]);
    printf("First byte: %02X\n", sendbuf[0]);
    #endif

    binary_file_length += sizeof(sendbuf);

    //check if at the end of the file
    if (sizeof(sendbuf) < BUF_SIZE - CMD_SIZE) {
        if (feof(file)) {
            printf("EOF reached\n");
            return EOF;
        } else if (ferror(file)) {
            printf("File read error\n");
            return FRE;
        }
    }

    //return no error
    return 0;

}

static void close_file(FILE* file) {
        fclose(file);
}
//-----------------------ADDED------------------------//


//Main application
void app_main(void)
{
    printf("Entering app_main()\n");
    //esp_err_t ret;
    spi_device_handle_t handle;
    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128,      //50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 1
    };

    //GPIO config for the handshake line.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };
    

    int n = 0;
    uint8_t sendbuf[BUF_SIZE] = {0};
    uint8_t recvbuf[BUF_SIZE] = {0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

//-----------------------ADDED------------------------//

    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 1,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

//-----------------------ADDED------------------------//

    //Create the semaphore.
    //rdySem = xSemaphoreCreateBinary();
    //Set up handshake line interrupt.
    
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_pull_mode(GPIO_HANDSHAKE, GPIO_PULLDOWN_ONLY);
   
    //Initialize the SPI bus and add the device we want to send stuff to.

    ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(SENDER_HOST, &devcfg, &handle);
    assert(ret == ESP_OK);


    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect
    //positive edge on the handshake line.
    // xSemaphoreGive(rdySem);

//-----------------------ADDED------------------------//
//while slave hasn't initiallized anything yet,
    do {
        //tell the slave to get in image download mode
        printf("Telling slave to setup for ota\n");

        t.length = BUF_SIZE * 8;
        binary_file_length = 0;

        //cmd to start ota
        sendbuf[0] = 128;

        //setup buffers
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        //Wait for slave to be ready for next byte before sending
        printf("Now waiting for slave\n");
        //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
        while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
            vTaskDelay(1);
        }
        ret = spi_device_transmit(handle, &t);
        printf("Received: %hhn\n", recvbuf);

    } while (recvbuf[0] == 0);
    

    //try and open the file
    printf("Opening file..... \n");
    FILE* file = open_file();

    //this section grabs the next chunk of the file and sends it to the slave
    //until the end of file is reached or error
    while (1) {
        printf("-------Entering reading a new chunk: CMD 129------\n");
        
        //read buffer call
        if (read_file_chunk(file, sendbuf) != 0 ){
            printf("error reading chunk\n");
            break;
            //need more error handling here
        }

        //setup buffers
        t.length = BUF_SIZE * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        
        //Wait for slave to be ready for next byte before sending
        printf("Now waiting for slave\n");
        //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
        while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
            vTaskDelay(1);
        }
        ret = spi_device_transmit(handle, &t);
        printf("Sent the %d chunk of file\n", n);
        //printf("Received: %s\n", recvbuf);
        n++;
    }
    
    printf("Closing file..... \n");
    close_file(file);


    //now tell slave its at the end of the file
    printf("Telling slave EOF\n");

    t.length = BUF_SIZE * 8;

    //cmd to end ota
    sendbuf[0] = 130;

    //setup buffers
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    //Wait for slave to be ready for next byte before sending
    printf("Now waiting for slave\n");
    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
        vTaskDelay(1);
    }

    ret = spi_device_transmit(handle, &t);
    printf("Total file size: %d\n", binary_file_length);

//-----------------------ADDED------------------------//

    ret = spi_bus_remove_device(handle);
    assert(ret == ESP_OK);

    //esp_restart();
}
