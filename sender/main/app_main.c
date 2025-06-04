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
#include <esp_partition.h>


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

#define FRE                  -2

#define DATA_SIZE           1021
#define CMD_SIZE            3
#define TRANS_SIZE          1024

#define HASH_LEN 32

//Debug ifdefs below 
//#define PART_FILE
#define PART_FILE_NUM       3
//#define DEBUG_SENDBUFFER
//#define DEBUG_SENDBUFFER_LAST
//#define DEBUG_FREAD_SIZE


static const char *TAG = "spiMaster";

//file shouldn't be more than 2MB, both of these are in bytes
int binary_file_length;
uint32_t file_size;

char file_name[] = "/spiffs/simple_ota.bin";

static void open_file(FILE** file)
{
    file_size = 0;
    binary_file_length = 0;
    
    ESP_LOGI(TAG, "Reading file");

    // Open for reading hello.txt
    *file = fopen("/spiffs/simple_ota.bin", "r");
    if (*file == NULL) {
        ESP_LOGE(TAG, "Failed to open file");
        return;
    }

    #ifdef PART_FILE
        file_size = PART_FILE_NUM*DATA_SIZE;
    #else  
        //gets how many bytes in a file
        fseek(*file, 0, SEEK_END);      // Move to end of file
        file_size = ftell(*file);
        rewind(*file);  //restart pointer to bg of file 
    #endif
    
    printf("Opened file of size: %ld\n", file_size);
}

static uint16_t read_file_chunk(FILE* file, uint8_t* sendbuf, int data_size){

    //allocating memory on core and clearing previous
    memset(sendbuf, 0, data_size);

    //read in chunk at current fread pointer
    uint16_t read_bytes = fread(sendbuf + CMD_SIZE, 1, data_size, file);  

    //CMD handling headers here
    //cmd to end ota
    sendbuf[0] = 129;
    sendbuf[1] = read_bytes >> 8;
    sendbuf[2] = read_bytes & 0x00FF;

    if ((read_bytes) > 0){
        binary_file_length += read_bytes;
        //check if at the end of the file and can transmit a part of a chunk
        if (read_bytes < data_size) {
            //in case of left over bytes
            printf("EOF reached winthin bounds\n");
            return read_bytes;
        } else if (read_bytes == data_size) {
            return read_bytes; //or data_size
        } else {
            return FRE;
        }

    } else if (ferror(file)) {
        printf("File read error\n");
        return FRE;
    }

    //return no error but no data, file is done
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
    uint8_t sendbuf[TRANS_SIZE] = {0};
    uint8_t recvbuf[TRANS_SIZE] = {0};
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
    binary_file_length = 0;

    //setup transaction
    t.length = TRANS_SIZE * 8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    //send dummy command
    //tell the slave to get in image download mode
    #ifdef DEBUG
    printf("-------Telling slave to setup for ota: CMD 128------\n");
    #endif
    //cmd to start ota
    sendbuf[0] = 128;

    //Wait for slave to be ready for next byte before sending
    printf("Now waiting for slave\n");
    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
        vTaskDelay(1);
    }
    ret = spi_device_transmit(handle, &t);
    printf("Received: %hhn\n", recvbuf);

    do {
        //tell the slave to get in image download mode
        #ifdef DEBUG
        printf("-------Telling slave to setup for ota: CMD 128------\n");
        #endif
        //cmd to start ota
        sendbuf[0] = 128;

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
    FILE* file = NULL;

    open_file(&file);
/*
    //tell the slave how big a file is coming
    printf("-------Telling slave ota file size: CMD 131------\n");
 

    //Wait for slave to be ready for next byte before sending
    printf("Now waiting for slave\n");
    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
        vTaskDelay(1);
    }
    ret = spi_device_transmit(handle, &t);
    //printf("Received: %hhn\n", recvbuf);
*/
    int err = 0;
    //this section grabs the next chunk of the file and sends it to the slave
    //until the end of file is reached or error
    while (
    #ifdef PART_FILE
    n < PART_FILE_NUM
    #else
    1
    #endif 
    ) {
        #ifdef DEBUG
        printf("-------Entering reading a new chunk: CMD 129------\n");
        #endif

        //read buffer call
        uint16_t read_bytes = read_file_chunk(file, sendbuf, DATA_SIZE);

        if ( read_bytes > 0 ){
            #ifdef DEBUG
            printf("Chunk of file: %d\n", n);
            #endif

            #ifdef DEBUG_FREAD_SIZE
            printf("Full Size: %02X\n", read_bytes);
            printf("First Size byte: %02X\n", sendbuf[1]);
            printf("Last Size byte: %02X\n", sendbuf[2]);
            #endif

            //setup buffers
            t.length = TRANS_SIZE * 8;
            t.tx_buffer = sendbuf;
            t.rx_buffer = recvbuf;

            #ifdef DEBUG_SENDBUFFER
            for (int i = 0; i < read_bytes; i++) {
                printf("%d byte: %02X\n", i, sendbuf[i]);
            }
            #endif

            //Wait for slave to be ready for next byte before sending
            #ifdef DEBUG 
            printf("Now waiting for slave\n"); 
            #endif
            //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
            while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
                vTaskDelay(1);
            }

            ret = spi_device_transmit(handle, &t);

            n++;

            if (read_bytes < DATA_SIZE) {
                #ifdef DEBUG
                printf("End of file reached, last chunk comming of bytes: %d\n", read_bytes);
                #endif

                #ifdef DEBUG_SENDBUFFER_LAST
                for (int i = 900; i < TRANS_SIZE; i++) {
                    printf("%d byte: %02X\n", i, sendbuf[i]);
                }
                #endif
                break;
            }
            //otherwise, keep looping
            continue;

        } else if ( err != 0 ) {
            printf("Error reading\n");
            break;
        } else {
            printf("File is done, this is awkward\n");
            break;
        }
    }
    
    //now tell slave its at the end of the file
    #ifdef DEBUG
    printf("-------Telling slave EOF: CMD 130------\n");
    #endif

    //cmd to end ota
    sendbuf[0] = 130;
    sendbuf[1] = 0;
    sendbuf[2] = 0;
    //sendbuf[1] = final_chunk_size; //how many bits need to be written to OTA

    //file sendbuf with file chunk after CMD+1
    //read_file_chunk(file, sendbuf+1, final_chunk_size);

   // binary_file_length += final_chunk_size;


    t.length = TRANS_SIZE * 8;

    //setup buffers
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    //Wait for slave to be ready for next byte before sending
    #ifdef DEBUG
    printf("Now waiting for slave\n");
    #endif

    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while(gpio_get_level(GPIO_HANDSHAKE) != 1) {
        vTaskDelay(1);
    }

    ret = spi_device_transmit(handle, &t);
    printf("Measured file size: %d\n", binary_file_length);
    printf("Actual file size: %ld\n", file_size);

    //close file
    printf("Closing file..... \n");
    close_file(file);

    //ret = spi_bus_remove_device(handle);
    //assert(ret == ESP_OK);

    //esp_restart();

    //ret = spi_bus_remove_device(handle);
    //assert(ret == ESP_OK);
}
