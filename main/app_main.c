/* ========================================
 *
 * Copyright © 2019, Dana H. Myers.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ========================================
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"

#include "lwip/apps/sntp.h"

#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "axmodem.h"
#include "ac101.h"
#include "igate/igate.h"

#include <math.h>

#include "driver/uart.h"

static const char* TAG = "axmodem_main";

/*
 * XXX:
 */
void dspOutProcess(void *x);

/*
 * local data
 */
#define	DISK_BUFFERS	(3 * SAMPLE_RATE / DSP_BLOCK_LEN / 4)	// 750mS of buffer
#define	NEXT_BUFFER(x)	((x) + 1 >= DISK_BUFFERS ? (x) + 1 - DISK_BUFFERS : (x) + 1)

static int16_t rxBuffer[DMA_BLOCK_LEN * 2];
static float inBuffer[DSP_BLOCK_LEN];

#ifdef DISK_LOG
static int16_t diskBuffers[DISK_BUFFERS][DSP_BLOCK_LEN];
static uint16_t diskIn, diskOut;
SemaphoreHandle_t diskMutex;
#endif

/*
 * CODEC access
 * XXX: wrap interface over this for portability
 * XXX: liberally lifted from esp-adf
 */

static const char *ES_TAG = "AC101_DRIVER";

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(ES_TAG, format, ##__VA_ARGS__); \
        return b;\
    }


/*
 *
 */
static i2c_config_t es_i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 33,
    .scl_io_num = 32,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

/*
 *
 */
static void
i2c_init()
{
    /*
     * XXX: pins specific to board
     */

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &es_i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, es_i2c_cfg.mode, 0, 0, 0));
}


/*
 * XXX:
 */
static void
i2c_example_master_read_slave(uint8_t DevAddr,
  uint8_t reg,uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( DevAddr << 1 ) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( DevAddr << 1 ) | READ_BIT, ACK_CHECK_EN));		//check or not
    ESP_ERROR_CHECK(i2c_master_read(cmd, data_rd, size, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}

static void
AC101_Write_Reg(uint8_t reg, uint16_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	uint8_t send_buff[4];

	send_buff[0] = (AC101_ADDR << 1);
	send_buff[1] = reg;
	send_buff[2] = (val>>8) & 0xff;
	send_buff[3] = val & 0xff;

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}

static uint16_t
AC101_Read_Reg(uint8_t reg) {
	uint16_t val = 0;
	uint8_t data_rd[2];

	i2c_example_master_read_slave(AC101_ADDR,reg, data_rd, 2);
	val = (data_rd[0] << 8) + data_rd[1];
	return val;
}

/*
 * I2S configuration
 */
static const i2s_config_t i2s_config = {
     .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
     .sample_rate = SAMPLE_RATE * 4,
     .bits_per_sample = 16,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
     .intr_alloc_flags = 0,
     .dma_buf_count = 4,
     .dma_buf_len = DMA_BLOCK_LEN * 2,    // 
     .use_apll = true
};

static const i2s_pin_config_t i2s_pin_config = {
    .bck_io_num = GPIO_NUM_27,
    .ws_io_num = GPIO_NUM_26,
    .data_out_num = GPIO_NUM_25,
    .data_in_num = GPIO_NUM_35
};

/*
 *
 */
static int
codecStart()
{
    gpio_config_t  io_conf;
    esp_err_t rv;

    // init i2c
    i2c_init();


	AC101_Write_Reg(CHIP_AUDIO_RS, 0x123);
    /* XXX: 20mS reset delay; is this long enough? */
	vTaskDelay(20 / portTICK_PERIOD_MS);
	printf("chip ID: %x\n", AC101_Read_Reg(CHIP_AUDIO_RS));

	AC101_Write_Reg(SPKOUT_CTRL, 0xe880);

	//Enable the PLL from 32 * 48kHz  BCLK source
	AC101_Write_Reg(PLL_CTRL1, 0x014f);
	AC101_Write_Reg(PLL_CTRL2, 0x8300);
	AC101_Write_Reg(SYSCLK_CTRL, 0xab08);
	AC101_Write_Reg(MOD_CLK_ENA, 0x800c);
	AC101_Write_Reg(MOD_RST_CTRL, 0x800c);

	// AC101_Write_Reg(I2S_SR_CTRL, 0x8000);			// 48ks/s
	AC101_Write_Reg(I2S_SR_CTRL, 0x2000);			// 12ks/s

	//AIF config
	AC101_Write_Reg(I2S1LCK_CTRL, 0x8850);			//BCLK/LRCK
	AC101_Write_Reg(I2S1_SDOUT_CTRL, 0xc000);		//
	AC101_Write_Reg(I2S1_SDIN_CTRL, 0xc000);
	AC101_Write_Reg(I2S1_MXR_SRC, 0x2200);			//

	AC101_Write_Reg(ADC_SRCBST_CTRL, 0xccc4);
	// AC101_Write_Reg(ADC_SRC, 0x2020);
    AC101_Write_Reg(ADC_SRC, 0x0408);
    AC101_Write_Reg(ADC_DIG_CTRL, 0x8000);

    /* ADC gain */
//	AC101_Write_Reg(ADC_APC_CTRL, 0xbbc0);   // L, R: +0dB
    AC101_Write_Reg(ADC_APC_CTRL, 0xffc0);  // L, R: +6dB

    /* ADC volume: what does this do? */
    AC101_Write_Reg(ADC_VOL_CTRL, 0xa0a0);  // L, R: +0dB

	//Path Configuration
	AC101_Write_Reg(DAC_MXR_SRC, 0xcc00);
	AC101_Write_Reg(DAC_DIG_CTRL, 0x8000);
	AC101_Write_Reg(OMIXER_SR, 0x0081);
	AC101_Write_Reg(OMIXER_DACA_CTRL, 0xf080);//}

    //I2S1_SDOUT_CTRL
    AC101_Write_Reg(MOD_CLK_ENA, 0x800c);
    AC101_Write_Reg(MOD_RST_CTRL, 0x800c);

    //* Enable Headphoe output
    AC101_Write_Reg(OMIXER_DACA_CTRL, 0xff80);
    AC101_Write_Reg(HPOUT_CTRL, 0xc3c1);
    AC101_Write_Reg(HPOUT_CTRL, 0xcb00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    AC101_Write_Reg(HPOUT_CTRL, 0xfbc0); // XXX:
    
    //* Enable Speaker output
    AC101_Write_Reg(SPKOUT_CTRL, 0xeabd);
    vTaskDelay(10 / portTICK_PERIOD_MS);


    // no need to configure MCLK output pin
    // XXX:
#if 0
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
#endif

    // configure i2s
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &i2s_pin_config));
    
    /*
     * Specific to A1S board
     */
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT(GPIO_NUM_21) |
      BIT(GPIO_NUM_22) | BIT(GPIO_NUM_23);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_21, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_22, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_23, 0)); // PTT

    return (ESP_OK);
}


/*
 *
 */

#define DEFAULT_SSID    "SSID"
#define DEFAULT_PWD     "PASSWORD"
#define DEFAULT_PS_MODE WIFI_PS_NONE

// #define IP4_CONNECTED_BIT   BIT0
// #define IP6_CONNECTED_BIT   BIT1
#define	WIFI_CONNECTED_BIT	BIT2
#define	EXAMPLE_ESP_MAXIMUM_RETRY	10

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

/*
 * Use system default event loop
 */
static void
event_handler(void* arg, esp_event_base_t event_base,
  int32_t event_id, void* event_data)
{
	//ESP_LOGI(TAG, "EVT: %d\n", event->event_id);

	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/*
 *
 */
static void
networkStart()
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD
        },
    };

    tcpip_adapter_init();

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
      ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
      IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "esp_wifi_set_ps().");
    esp_wifi_set_ps(DEFAULT_PS_MODE);

    /* start SNTP */
    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "PST+8PDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

#ifdef	DISK_LOG
/*
 * create an audio file
 */
static int
createRawFile()
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    static char nameBuf[40];

    time(&now);
    localtime_r(&now, &timeinfo);
    snprintf(nameBuf, sizeof (nameBuf), "/sdcard/raw_%4.4d%2.2d%2.2d_%2.2d%2.2d%2.2d.wav",
    		timeinfo.tm_year + 1900, timeinfo.tm_mon, timeinfo.tm_mday,
			timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    printf("rawf: %s\n", nameBuf);
	return (creat(nameBuf, 0x666));
}

static TaskHandle_t diskWriteTaskHandle;
static int logFile;
static uint32_t dcount;
volatile static TickType_t maxWrite = 0;
volatile static TickType_t avgWrite = 0;

/*
 * Disk write task
 */
IRAM_ATTR static void
diskWriteTask(void *x)
{
	TickType_t t1, t2;
	uint32_t count;
	uint16_t diskIndex;

	while (true) {
		count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		xSemaphoreTake(diskMutex, portMAX_DELAY);
		if (diskOut == diskIn) {
			// buffer is empty, nothing to do
			xSemaphoreGive(diskMutex);
			continue;
		}
		diskIndex = diskOut;
		xSemaphoreGive(diskMutex);

		t1 = xTaskGetTickCount();

		/* capture 10 minutes of audio per file */
		if (dcount++ < (12000 * 60 / DSP_BLOCK_LEN)) {
			if (logFile > 2) {
				write(logFile, &diskBuffers[diskIndex][0], DSP_BLOCK_LEN * sizeof (int16_t));
			}
		} else {
			dcount = 0;
			close(logFile);
			logFile = createRawFile();
			if (logFile > 2) {
				write(logFile, &diskBuffers[diskIndex][0], DSP_BLOCK_LEN * sizeof (int16_t));
			}
			avgWrite /= (12000 * 60 / DSP_BLOCK_LEN);
        	printf("maxWrite: %u %u (no sync)\n", maxWrite, avgWrite);
        	maxWrite = avgWrite = 0;
		}

        t2 = xTaskGetTickCount();
        t2 -= t1;
        if (t2 > maxWrite) {
        	maxWrite = t2;
        }
        avgWrite += t2;

        /* free up the disk buffer */
		xSemaphoreTake(diskMutex, portMAX_DELAY);
		diskOut = NEXT_BUFFER(diskOut);
		xSemaphoreGive(diskMutex);

		// XXX: do an fsync() here?
	}

}
#endif

/*
 * DSP processing
 */
static TaskHandle_t dspInTaskHandle, dspOutTaskHandle;
static float inputGain = 4.0f;

//volatile uint64_t dspCycles;
//
/*
 *
 */
IRAM_ATTR static void
dspInProcess(void *x)
{
    size_t bytes_read;
    int16_t *diskp;

    /*
     *
     */
    while (1) {
        (void)i2s_read(I2S_NUM_0, (char *)rxBuffer, sizeof (rxBuffer),
          &bytes_read, 1000);

        if (bytes_read != sizeof (rxBuffer)) {
            ESP_LOGE("CODEC", "short codec read: %d", bytes_read);
        }

#ifdef DISK_LOG
        /*
         * Get disk buffer if available
         */
        xSemaphoreTake(diskMutex, portMAX_DELAY);
        /* check for full buffer */
        if (NEXT_BUFFER(diskIn) == diskOut) {
        	// full
        	diskp = NULL;
        } else {
        	// room available
  			diskp = &diskBuffers[diskIn][0];
        	// handle the wrap case
  			diskIn = NEXT_BUFFER(diskIn);
        }
        xSemaphoreGive(diskMutex);

        if (!diskp) {
        	puts("disk overflow");
        }
#endif
        /*
         * separate right-channel input; note that inBuffer &
         * outBuffer are sized to DSP_BLOCK_LEN * 1 channel,
         *
         * XXX: frames are 4x stereo samples; is there a way to
         *  reduce this 1x stereo sample?
         *
         *  One sample is : [R, L] * 4
         *  even samples are R, odd are L
         */
        for (int i = 0; i < DSP_BLOCK_LEN; i++) {
        	/* process samples into float for DSP */
            // inBuffer[i] = ((float)rxBuffer[i * 8 + 0]) / 32767.0;
            inBuffer[i] = ((float)rxBuffer[i * 8 + 0])  * (float)(1.0 / 32767.0);

#ifdef DISK_LOG
            /* process samples into diskBuffer */
            if (diskp) {
            	diskp[i] = rxBuffer[i * 8 + 0];
            }
#endif

            /* XXX: gain + clip */
            inBuffer[i] *= inputGain;
            if (inBuffer[i] >= 1.0f) {
            	inBuffer[i] = 0.9999f;
            } else if (inBuffer[i] <= -1.0f) {
            	inBuffer[i] = -0.9999f;
            }
        }

        gpio_set_level(GPIO_NUM_22, 0);
        modemProcessInput(inBuffer);
        gpio_set_level(GPIO_NUM_22, 1);

#ifdef DISK_LOG
        // wake up disk writer
        if (diskp) {
        	xTaskNotifyGive(diskWriteTaskHandle);
        }
#endif
    }
}

#ifdef DISK_LOG
#include <unistd.h>

#define	SD_USE_MMC

/*
 *
 */
void
sdcardInitialize()
{
	esp_err_t ret;

#ifdef	SD_USE_MMC
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#else
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
#endif

	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
		.format_if_mount_failed = false,
		.max_files = 5,
		.allocation_unit_size = 16 * 1024
	};
	FATFS *out_fs;
	sdmmc_card_t *card;

//	slot_config.gpio_cd = 34;

#ifdef SD_USE_MMC
    /* XXX: */
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_MTDI_HS2_DATA2);

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.

	gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    // host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    host.flags = SDMMC_HOST_FLAG_4BIT | \
             SDMMC_HOST_FLAG_1BIT | \
             SDMMC_HOST_FLAG_DDR;
#else

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13

    ESP_LOGI(TAG, "Using SPI peripheral");

    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif

	ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount filesystem. "
			  "If you want the card to be formatted, set format_if_mount_failed = true.");
		} else {
			ESP_LOGE(TAG, "Failed to initialize the card (%s). "
			  "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
		}
		return;
	}

	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);
	logFile = createRawFile();
	printf("logFile: %d\n", logFile);
}
#endif

/*
 *
 */
esp_err_t
app_main(void)
{
    esp_err_t err;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_log_level_set("*", ESP_LOG_INFO);

    /*
     * XXX: config UART0
     */
    (void)uart_param_config(UART_NUM_0, &uart_config);
    (void)uart_driver_install(UART_NUM_0,
            2048,   // RX buffer size
            2048,   // TX buffer size
            0,      // evt queue size
            NULL,   // evt qeueue handle
            0);     // irq flags

#ifdef DISK_LOG
    /*
     * Initialize network, codec, modem
     */
    diskMutex = xSemaphoreCreateMutex();
#endif

    /* initialize NVS */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase and retry NVS init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /*
     *
     */
    networkStart();
    // XXX: give the network a little time
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    codecStart();
    modemInitialize();

#if DISK_LOG
    /*
     *
     */
    sdcardInitialize();

    /*
     * Disk write task for audio logging
     */
    err = xTaskCreate(diskWriteTask, "diskWrite",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 2,
	  &diskWriteTaskHandle);
#endif

    /*
     * DSP processing tasks
     * Separate tasks for input and output processing
     * XXX: check for err return
     */
    err = xTaskCreate(dspInProcess, "dspIn",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 1,
	  &dspInTaskHandle);

    err = xTaskCreate(dspOutProcess, "dspOut",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 1,
	  &dspOutTaskHandle);

    /*
     * Start iGate
     */
    igateStart();

    /*
     * All done; return and be deleted
     */
    return (ESP_OK);
}


