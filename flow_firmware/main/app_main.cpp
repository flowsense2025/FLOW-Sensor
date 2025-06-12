/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"


/* Peripherals */
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"


/* OS */
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_sensor.h"


void initializeDevice(){

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    fflush(stdout);
}


static const char *tag = "NimBLE_BLE_HeartRate";

static TimerHandle_t blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "blehr_sensor_1.0";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;
static QueueHandle_t flow_queue;

/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const uint8_t* addr)
{
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void
blehr_tx_hrate_stop(void)
{
    //xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset heart rate measurement */
static void
blehr_tx_hrate_reset(void)
{
    // int rc;

    // if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
    //     rc = 0;
    // } else {
    //     rc = 1;
    // }

    // assert(rc == 0);

}

/* This function simulates heart beat and notifies it to the client */
static void
blehr_tx_hrate(TimerHandle_t ev)
{
    // static uint8_t hrm[2];
    // int rc;
    // struct os_mbuf *om;

    // if (!notify_state) {
    //     blehr_tx_hrate_stop();
    //     heartrate = 90;
    //     return;
    // }

    // hrm[0] = 0x06; /* contact of a sensor */
    // hrm[1] = heartrate; /* storing dummy data */

    // /* Simulation of heart beats */
    // static uint8_t dummy;
    // if (xQueueReceive(flow_queue, &dummy, NULL);
    // heartrate++;
    // if (heartrate == 160) {
    //     heartrate = 90;
    // }

    // om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
    // rc = ble_gatts_notify_custom(conn_handle, hrs_hrm_handle, om);

    // assert(rc == 0);

    blehr_tx_hrate_reset();
}

static int
blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void
blehr_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blehr_advertise();
}

static void
blehr_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void init(){
    initializeDevice();

    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);

    flow_queue = xQueueCreate(128, sizeof(uint8_t));
}

#define GPIO_INPUT_IO gpio_num_t::GPIO_NUM_21
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO)

static const char *TAG = "GPIO_INTERRUPT";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    static const uint8_t count = 1;
    // Just print from ISR (normally keep ISR short)
    ets_printf("GPIO[%d] interrupt triggered: %d\n", gpio_num, count);
    xQueueSendFromISR(flow_queue, &count, NULL);
}

void gpio_init(){
    // Configure GPIO as input
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE, // interrupt on falling edge, change as needed
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Attach ISR handler for specific pin
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);

    ESP_LOGI(TAG, "Interrupt on GPIO %d configured", GPIO_INPUT_IO);

}

typedef struct{
    uint16_t litre = 0;
    uint16_t milliletre_remainder = 0; // represented out of 11 (11 pulses = 1 L)
}WaterVolume_t;

void increment_volume(WaterVolume_t* volume){

    uint8_t new_litre = (volume->milliletre_remainder == 10);
    if (new_litre){
        volume->litre++;
        volume->milliletre_remainder = 0;
    }
    else
    {
        volume->milliletre_remainder++;
    }
}

float get_total_volume(WaterVolume_t* volume){
    return volume->litre + (float)volume->milliletre_remainder/11.0;
}

extern "C" void app_main(void)
{
    init();
    gpio_init();

    static uint32_t frm[2];
    int rc;
    struct os_mbuf *om;

    frm[0] = 0x06; /* device ble id */
    
    WaterVolume_t total_volume = {0};

    // #define DUMMY_DATA
    while (1) {

        
        #ifdef DUMMY_DATA

        vTaskDelay(pdMS_TO_TICKS(1000));
        #else
        uint8_t data = 0;

        if(xQueueReceive(flow_queue, &data, portMAX_DELAY) == pdFALSE){
            // do smt...in 52 days
        }

        do{
            increment_volume(&total_volume);
            vTaskDelay(pdMS_TO_TICKS(10));
        }while(xQueueReceive(flow_queue, &data, 0) == pdTRUE);

        ESP_LOGI(TAG, "GPIO interrupt received");

        #endif 
        // TODO: Should we store total count on device?
        // TODO: May need an external SD card 

        // 11 pulses per litre. 
        // 1/11 L 

        // we only need to send a BLE message if there is a difference in the data
        if (!notify_state) {
            continue;
        }
        else
        {
            // precision of 90 mL
            frm[1] = total_volume.litre; /* storing dummy data */
            //local_count = 0; // "move" data to phone by resetting count
            ESP_LOGI(TAG, "%.2f", get_total_volume(&total_volume));
                
            om = ble_hs_mbuf_from_flat(frm, sizeof(frm));
            rc = ble_gatts_notify_custom(conn_handle, hrs_hrm_handle, om);

            assert(rc == 0);
        }
    }
}

