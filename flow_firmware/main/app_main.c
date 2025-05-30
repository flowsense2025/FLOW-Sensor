/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/timer.h"

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

/* Device I/O */ 
#include "user_spi_interface.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


/* External Sensors */ 
#include "AS6031_Bit_Definition.h"
#include "AS6031_CFG_Macros.h"
#include "AS6031_Coding.h"

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_CLK_FREQ / TIMER_DIVIDER)  // convert counter value to seconds

static QueueHandle_t s_timer_queue;

#define HAL_Delay(x) vTaskDelay(x/portTICK_PERIOD_MS)

#define TIME_ms(x)				(float)(x*1000.0)		// result in [ms]
#define TIME_us(x)				(float)(x*1000000.0)	// result in [us]
#define TIME_ns(x)				(float)(x*1000000000.0)	// result in [ns]

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

AS6031_InitTypeDef DUT; // DUT = Device Under Test

volatile uint32_t  My_INTN_Counter = 0;
volatile uint8_t   My_INTN_State = 1;

volatile int value = 0;
volatile int addr = 0xC0;

volatile float     CLKLS_freq = 32768;             // LS Clock frequency in Hz
volatile float     CLKHS_freq = 4000000;           // HS Clock frequency in Hz
volatile float     CLKHS_freq_cal = 4000000;       // Calibrated HS Clock frequency in Hz
volatile float     CLKHS_freq_corr_fact = 1.000;   // Correction factor for HS Clock frequency

volatile float     RAW_Result = 0;                 // RAW Value in [LSB]
volatile float     Time_Result = 0;                // Result in [s]
volatile float     Time_Result_ns = 0;             // Result in [ns]

volatile float     RAW_CAL_Result = 0;             // RAW Value in [LSB]
volatile float     Time_CAL_Result = 0;            // Result in [s]
volatile float     Time_CAL_Result_ns = 0;         // Result in [ns]

uint8_t   spiRX[7];                       // used for readout of ID
volatile uint64_t  Bit_ID = 0;                     // contains individual ID[55:0] of GP22

volatile float     MAX_Cal_Time_Delay = 0;         // Max. measured calibrated time delay in [s]
volatile float     MAX_Cal_Time_Delay_ns = 0;      // Max. measured calibrated time delay in [ns]

volatile int       N_Measure_Cycles;               // counter for the while loop
volatile int       MAX_N_Measure_Cycles = 100;     // limit for the counter


// Configuration: using plastic spool piece plastic Audiowell New-Design, V-Shape
uint32_t  Reg[20] = {
		0x48DBA399,	// [0xC0] CR_WD_DIS
		0x00800401,	// [0xC1] CR_IFC_CTRL
		0x00111111,	// [0xC2] CR_GP_CTRL
		0x00000001,	// [0xC3] CR_USM_OPT
		0x010703FF,	// [0xC4] CR_IEH
		0x60060C08,	// [0xC5] CR_CPM
		0x01012100,	// [0xC6] CR_MRG_TS
		0x00240000,	// [0xC7] CR_TPM
		0x00680064,	// [0xC8] CR_USM_PRC
		0x60160202,	// [0xC9] CR_USM_FRC
		0x000FEA10,	// [0xCA] CR_USM_TOF
		0x00A7DE81,	// [0xCB] CR_USM_AM
		0x94A0C46C,	// [0xCC] CR_TRIM1
		0x401100C4,	// [0xCD] CR_TRIM2
		0x00A7400F,	// [0xCE] CR_TRIM3
		0x00000001,	// [0xD0] SHR_TOF_RATE
		0x00000D80,	// [0xD1] SHR_USM_RLS_DLY_U
		0x00000D80,	// [0xD2] SHR_USM_RLS_DLY_D
		0x00000041,	// [0xDA] SHR_ZCD_FHL_U
		0x00000041	// [0xDB] SHR_ZCD_FHL_D
};

// Configuration: using plastic spool piece plastic DN8
uint32_t  Reg2[20] = {
		0x48DBA399,	// [0xC0] CR_WD_DIS
		0x00800401,	// [0xC1] CR_IFC_CTRL
		0x00000000,	// [0xC2] CR_GP_CTRL
		0x00000001,	// [0xC3] CR_USM_OPT
		0x0011F7FF,	// [0xC4] CR_IEH
		0x6046EE08,	// [0xC5] CR_CPM
		0x01012100,	// [0xC6] CR_MRG_TS
		0x00E40000,	// [0xC7] CR_TPM
		0x04690564,	// [0xC8] CR_USM_PRC
		0x601E0202,	// [0xC9] CR_USM_FRC
		0x010FEA4E,	// [0xCA] CR_USM_TOF
		0x2320DE61,	// [0xCB] CR_USM_AM
		0x94A0C46C,	// [0xCC] CR_TRIM1
		0x401100C4,	// [0xCD] CR_TRIM2
		0x00A7400F,	// [0xCE] CR_TRIM3
		0x00000001,	// [0xD0] SHR_TOF_RATE
		0x00000ED8,	// [0xD1] SHR_USM_RLS_DLY_U
		0x00000ED8,	// [0xD2] SHR_USM_RLS_DLY_D
		0x00000028,	// [0xDA] SHR_ZCD_FHL_U
		0x00000028	// [0xDB] SHR_ZCD_FHL_D
};

// Firmware Code: <AS6031_AS6040_A1.F1.11.01_DIF_over_PI_sim.hex>
uint8_t FWC[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xA1, 0xF1, 0x11, 0x01, 0xF2, 0xDC, 0x61, 0x1B, 0x64, 0x16, 0xF2, 0xDC, 0x61, 0x13, 0x64, 0x31,
		0xF2, 0xDC, 0x61, 0x23, 0x64, 0x2E, 0xF2, 0xDC, 0x61, 0x2B, 0x64, 0x76, 0xC9, 0x01, 0x1D, 0xF2,
		0xE2, 0x61, 0x37, 0xC9, 0x01, 0x1D, 0xB8, 0x77, 0xF2, 0x80, 0x2D, 0xF2, 0x84, 0x37, 0x7D, 0xCA,
		0x00, 0x66, 0xF2, 0xC1, 0x61, 0x43, 0xCA, 0xF9, 0x11, 0xF2, 0xDD, 0xF1, 0xAB, 0xC9, 0x01, 0x1D,
		0xC9, 0x01, 0x1D, 0xC9, 0x01, 0x1D, 0xF2, 0x80, 0x77, 0xF2, 0x84, 0x7B, 0x88, 0x73, 0xCB, 0x74,
		0xF2, 0xA5, 0x7D, 0x76, 0x88, 0x73, 0xCB, 0x74, 0xF2, 0xA6, 0x7D, 0xF2, 0xA6, 0x73, 0xF2, 0xA5,
		0x33, 0x87, 0x7C, 0xF3, 0x03, 0x77, 0x34, 0x71, 0xF3, 0x05, 0x0F, 0x4B, 0x13, 0x5D, 0x73, 0xF3,
		0x04, 0x13, 0x4D, 0x73, 0xF2, 0x5E, 0x77, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0x75, 0x3B, 0x9A, 0xCA,
		0x00, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0xF3, 0x02, 0x77, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0x82, 0x7C,
		0xCF, 0xF3, 0x67, 0x73, 0xF2, 0xD1, 0x7C, 0xF2, 0xD2, 0x7C, 0xF2, 0x5F, 0x7F, 0x00, 0x3D, 0x09,
		0x00, 0xF2, 0x5E, 0x7F, 0x00, 0x00, 0x04, 0x31, 0xF2, 0xC5, 0x62, 0x03, 0xF2, 0x5F, 0xD3, 0xF2,
		0xC5, 0x62, 0x03, 0xF2, 0x5E, 0xC3, 0xF2, 0xCA, 0x73, 0xCB, 0x80, 0x00, 0x00, 0x1F, 0x00, 0xCE,
		0x70, 0x88, 0x7C, 0xF2, 0xC6, 0x73, 0xCB, 0x80, 0x00, 0x00, 0x1F, 0xFF, 0x09, 0xCB, 0xA5, 0x00,
		0x00, 0x00, 0x1F, 0xF2, 0xD0, 0x7B, 0xCB, 0x86, 0xCB, 0x54, 0xCE, 0x58, 0xF2, 0xE3, 0x63, 0x47,
		0x75, 0x00, 0x01, 0x06, 0x24, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0xF2, 0x9E, 0x7C, 0xF3, 0x5B, 0x73,
		0xF3, 0x5C, 0x77, 0xF2, 0x9E, 0x7B, 0xCA, 0xF3, 0x1C, 0x8A, 0x7C, 0x64, 0x01, 0xF2, 0xDC, 0x0B,
		0xCD
};

int FWC_Length = sizeof(FWC);  // Calculates the size of the array in bytes.

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: %lu, %lu\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

void read_sensor();
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

    for (int i = 3; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    fflush(stdout);
}


static const char *tag = "NimBLE_BLE_FLOW";

static TimerHandle_t blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "flow_sensor_1.0";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;

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
    xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset heart rate measurement */
static void
blehr_tx_hrate_reset(void)
{
    int rc;

    if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
        rc = 0;
    } else {
        rc = 1;
    }

    assert(rc == 0);

}

/* This function simulates heart beat and notifies it to the client */
static void
blehr_tx_hrate(TimerHandle_t ev)
{
    static uint8_t hrm[2];
    int rc;
    struct os_mbuf *om;

    if (!notify_state) {
        blehr_tx_hrate_stop();
        heartrate = 90;
        return;
    }

    read_sensor();

    hrm[0] = 0x06; /* contact of a sensor */
    hrm[1] = N_Measure_Cycles; /* storing dummy data */

    /* Simulation of heart beats */
    heartrate++;
    if (heartrate == 160) {
        heartrate = 90;
    }

    om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
    rc = ble_gatts_notify_custom(conn_handle, hrs_hrm_handle, om);

    assert(rc == 0);

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

void scisense_begin(){

  AS6031_Init_CFG(&DUT, Reg);

  // Reset DUT
  Write_Opcode(RC_SYS_RST);
  DUT.State = AS6031_STATE_RESET;

  HAL_Delay(3); // Datasheet -> Delay = 1ms... BUT at least 3ms are needed _MH

  // Write Configuration (0xC0 - 0xCE, 0xD0 - 0xD2, 0xDA - 0xDB)
  int offset = 0;
  int i, j = 0;

  for (i = 0; i <= 19; i++) {
	  if (i == 0) {
		  offset = 0xC0;
		  j = 0;
	  }
	  if (i == 15) {
		  offset = 0xD0;
		  j = 0;
	  }
	  if (i == 18) {
		  offset = 0xDA;
		  j = 0;
	  }
	  Write_Dword(RC_RAA_WR, (offset+j), DUT.CR[i]);
	  j++;
  }

  Write_Opcode(RC_MCT_ON);

/*
  for (i = 0; i < 200; i++) {
	  // update CR9 register with new parameter
	  DUT.Param.CR9.FBG_CLK_DIV = (addr);
	  AS6031_Update_CFG(&DUT); // Function to update CFG for uploading into DUT!

	  Write_Dword(RC_RAA_WR, 0xC9, DUT.CR[9]); // Write Register into AS6031!
	  i++;
  }
*/

/*
  // update CR5 register with new parameter
  DUT.Param.CR5.HSC_DIV_MODE = 1; // Recommended setting for 8MHz
  DUT.Param.CR5.HSC_DIV = 1; // Recommended setting for 8MHz

  AS6031_Update_CFG(&DUT); // Function to update CFG for uploading into DUT!

  Write_Dword(RC_RAA_WR, 0xC5, DUT.CR[5]); // Write Register into AS6031!
*/

  // FW Handling Procedures
  // Datasheet Appendix, section 15.7
  // Phase 1: Wait time (dependent on start option)
  // Phase 2: Preparation (common for all procedures)
  // Phase 3: FW Update (different for procedures [A], [B], [C], [D] )
  // Phase 4: FW Retention Check (common for all procedures)

  // Phase1: Initial Wait Time
  Write_Opcode(RC_SYS_RST);
  DUT.State = AS6031_STATE_RESET;

  HAL_Delay(3);

  // Phase 2: Preparation
  Write_Opcode(RC_BM_REQ);
  Write_Dword(RC_RAA_WR, 0xC0, 0x48DBA399);
  Write_Dword(RC_RAA_WR, 0xCD, 0x40100000);
  Write_Dword(RC_RAA_WR, 0xC6, 0x00001000);
  Write_Opcode(RC_SV_INIT);
  Write_Opcode(RC_MCT_OFF);
  HAL_Delay(1);
  Write_Opcode2(RC_MT_REQ, 0x00);
  HAL_Delay(1);
  Write_Dword(RC_RAA_WR, 0xDD, 0x00000007);
  Write_Opcode(RC_RF_CLR);
  Write_Dword(RC_RAA_WR, 0xC4, 0x000AF000);
  Write_Opcode(RC_BM_RLS);

  Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
  Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0) {};
  Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0) {};

  // Phase 3: FW Update
  Read_Dword(RC_RAA_RD, 0xEC);

  // Write FWC
  for (i = 32; i <= FWC_Length; i++) {
	  Write_Byte2(RC_FWC_WR, i, FWC[i]);  // Writing FWC, bytewise with two byte address
  }

  // Write FWD
  Write_Dword(RC_RAA_WR_NVRAM, 0x00, 0x0000AB6A); // Writing Firmware Code User, Checksum
  Write_Dword(RC_RAA_WR_NVRAM, 0x01, 0x00000556); // Writing Firmware Data User, Checksum
  Write_Dword(RC_RAA_WR_NVRAM, 0x02, 0x00010000); // Writing FWD_SIMPLE_SCALE (fd16)
  Write_Dword(RC_RAA_WR_NVRAM, 0x03, 0x00000000); // Writing FWD_ZERO_OFFSET
  Write_Dword(RC_RAA_WR_NVRAM, 0x04, 0x051EB852); // Writing FWD_MAX_TOF_DIFF
  Write_Dword(RC_RAA_WR_NVRAM, 0x05, 0xFAE147AE); // Writing FWD_NEG_TOF_DIFF_LIMIT

  Write_Dword(RC_RAA_WR_NVRAM, 0x5B, 0x0000000A); // Writing FWD_R_PULSE_PER_LITER
  Write_Dword(RC_RAA_WR_NVRAM, 0x5C, 0x000003E8); // Writing FWD_R_PULSE_MAX_FLOW

  Write_Dword(RC_RAA_WR_NVRAM, 0x67, 0x00000000); // Writing FWD_USM_RLS_DLY_INIT

  Write_Dword(RC_RAA_WR_NVRAM, 0x6B, 0xABCD7654); // Writing Boot-Loader Release Code

  // Update Config 0x6C ... 0x77 in NVRAM
  // without CR_TRIMx and without SHR_...
  //Write_Dword(RC_RAA_WR_NVRAM, 0x6C, 0x48DBA399); // CR_WD_DIS Watchdog Disable
  //Write_Dword(RC_RAA_WR_NVRAM, 0x6D, 0x00800101); // CR_IFC_CTRL Interfaces Control
  //Write_Dword(RC_RAA_WR_NVRAM, 0x6E, 0x00100044); // CR_GP_CTRL General Purpose Control
  //Write_Dword(RC_RAA_WR_NVRAM, 0x6F, 0x20000003); // CR_USM_OPT USM Options
  //Write_Dword(RC_RAA_WR_NVRAM, 0x70, 0x001002A7); // CR_IEH Interrupt & Error Handling
  //Write_Dword(RC_RAA_WR_NVRAM, 0x71, 0x2046EE08); // CR_CPM Clock & Power Management
  //Write_Dword(RC_RAA_WR_NVRAM, 0x72, 0x0101A080); // CR_MRG_TS Measure Rate Generator & Task Sequencer
  //Write_Dword(RC_RAA_WR_NVRAM, 0x73, 0x00140000); // CR_TPM Temperature Measurement
  //Write_Dword(RC_RAA_WR_NVRAM, 0x74, 0x207807A4); // CR_USM_PRC USM Processing
  //Write_Dword(RC_RAA_WR_NVRAM, 0x75, 0x60150202); // CR_USM_FRC USM Fire & Receive Control
  //Write_Dword(RC_RAA_WR_NVRAM, 0x76, 0x00002A0E); // CR_USM_TOF Time Of Flight Rate init value
  //Write_Dword(RC_RAA_WR_NVRAM, 0x77, 0x23209E71); // CR_USM_AM Amplitude and FHL

  Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
  Write_Dword(RC_RAA_WR, 0xDE, 0x00010000);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0) {};

  // Phase 4: FW Retention Check
  Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
  Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0) {};
  Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0) {};
  Write_Dword(RC_RAA_WR,  0xD3, 0x0007F000);
  HAL_Delay(offset); // After initialization checksum error flags, delay of at least 34ms are needed _MH
  Write_Opcode(RC_FW_CHKSUM);
  while(Read_Dword_Bits(RC_RAA_RD, 0xE0, 3, 3) == 0) {};
  Read_Dword(RC_RAA_RD, 0xD3);

  // END
  Write_Opcode(RC_SYS_RST);


  /* USER CODE END 2 */



}


void start_ble_task(){
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

    /* name, period/time,  auto reload, timer ID, callback */
    blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, blehr_tx_hrate);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);
}

void read_sensor(){

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  N_Measure_Cycles++;

	  // Wait for INTN
	  // NVIC Functionality to increase the speed of MCU
     Waiting_For_INTN(0);
	  //while ( (My_INTN_State==1)) { }; //timeout is missing

	  // Post Processing

	//   RAW_Result = Read_Dword(RC_RAA_RD, 0x80);  // FDB_US_TOF_SUM_OF_ALL_U
	  int ret = Read_Dword(RC_RAA_RD, 0xED);  // FDB_US_TOF_SUM_OF_ALL_D
      printf("Raw Result10: %d\n", ret);
	  ret = Read_Dword(RC_RAA_RD, 0xE9);  // FDB_US_TOF_SUM_OF_ALL_D
      printf("Raw Result11: %d\n", ret);
	  ret = Read_Dword(RC_RAA_RD, 0xE5);  // FDB_US_TOF_SUM_OF_ALL_D
      printf("Raw Result12: %d\n", ret);
	  ret = Read_Dword(RC_RAA_RD, 0xE7);  // FDB_US_TOF_SUM_OF_ALL_D
      printf("Raw Result14: %d\n", ret);
    // 0x E9
    //0xE5 VCC
    //0xE7 timestamp

	//   RAW_Result = Read_Dword(RC_RAA_RD, 0x88);  // FDB_US_TOF_0_U
      printf("Raw Result0: %.2f\n", RAW_Result);

	//   RAW_Result /= 65536;  //divided by 2^16
	  RAW_Result /= DUT.Param.CR10.TOF_HIT_SUM_NO;  // Divided by number of hits
      printf("Raw Result1: %.2f\n", RAW_Result);
	  Time_Result = RAW_Result * 250 *(1e-9);

      printf("Time Result: %.2f\n", RAW_Result);
	  Time_Result_ns = TIME_ns(Time_Result); // result in [ns]
      printf("Time Result(ns): %d\n", N_Measure_Cycles);


	  // Clear INTN
	  Write_Opcode(RC_IF_CLR);

	//   HAL_Delay(1000); // used for debugging

}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    //uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
    static uint64_t timer_counter_value = 0;
    timer_counter_value++;



    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    // if (!info->auto_reload) {
    //     timer_counter_value += info->alarm_interval * TIMER_SCALE;
    //     timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    // }

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);
    

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}


void app_main(void)
{
    initializeDevice();

    // user_spi_init();
    // scisense_begin();
    printf("Finished initializing flow sensor\n");

    // start_ble_task();

    //ESP_LOGI(tag, "Beginning to read main sensor");
    // while(1){
        // read_sensor();
    // }
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 1);
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    while (1) {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.info.auto_reload) {
            printf("Timer Group with auto reload\n");
            printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

            /* Print the timer values passed by event */
            printf("------- EVENT TIME --------\n");
            print_timer_counter(evt.timer_counter_value);

            /* Print the timer values as visible by this task */
            printf("-------- TASK TIME --------\n");
            double task_counter_value;
            timer_get_counter_time_sec(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
            print_timer_counter(task_counter_value);

        } else {
            printf("Timer Group without auto reload\n");
        }
    }

}