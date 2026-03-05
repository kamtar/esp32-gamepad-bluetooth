#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_hidd_prf_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "usb/usb_host.h"

#if defined(__has_include) && __has_include("usb/hid_host.h")
#include "usb/hid_host.h"
#define HAS_USB_HID_HOST 1
#else
#define HAS_USB_HID_HOST 0
#endif

#include "hid_dev.h"
#include "hidd_le_prf_int.h"

#define APP_TAG "USB_BLE_GAMEPAD"
#define HIDD_DEVICE_NAME "ESP32-S3 USB Gamepad Bridge"

#define GAMEPAD_REPORT_ID HID_RPT_ID_GAMEPAD_IN
#define GAMEPAD_REPORT_LEN 7
#define HIDD_CONN_INTERVAL_MIN 0x0006
#define HIDD_CONN_INTERVAL_MAX 0x0010
#define HIDD_ADV_INTERVAL_MIN 0x0020
#define HIDD_ADV_INTERVAL_MAX 0x0030

static bool s_ble_connected;
static bool s_ble_secured;
static uint16_t s_hid_conn_id;

static uint8_t hidd_service_uuid128[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = HIDD_CONN_INTERVAL_MIN,
    .max_interval = HIDD_CONN_INTERVAL_MAX,
    .appearance = 0x03C0,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = HIDD_ADV_INTERVAL_MIN,
    .adv_int_max = HIDD_ADV_INTERVAL_MAX,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void send_ble_gamepad_report(const uint8_t *data, size_t length)
{
    if (!s_ble_connected || !s_ble_secured || length == 0) {
        return;
    }

    uint8_t report[GAMEPAD_REPORT_LEN] = {0};

    report[0] = (length > 2) ? data[2] : 0;
    report[1] = (length > 3) ? data[3] : 0;
    report[2] = (length > 4) ? data[4] : 0;
    report[3] = (length > 5) ? data[5] : 0;
    report[4] = (length > 6) ? (data[6] & 0x0F) : 0x0F;

    uint16_t buttons = 0;
    if (length > 0) {
        buttons |= data[0];
    }
    if (length > 1) {
        buttons |= ((uint16_t)data[1]) << 8;
    }

    report[5] = (uint8_t)(buttons & 0xFF);
    report[6] = (uint8_t)((buttons >> 8) & 0xFF);

    hid_dev_send_report(hidd_le_env.gatt_if,
                        s_hid_conn_id,
                        GAMEPAD_REPORT_ID,
                        HID_REPORT_TYPE_INPUT,
                        sizeof(report),
                        report);
}

#if HAS_USB_HID_HOST
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
    (void)arg;

    uint8_t data[64] = {0};
    size_t data_length = 0;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        if (hid_host_device_get_raw_input_report_data(hid_device_handle, data, sizeof(data), &data_length) == ESP_OK) {
            send_ble_gamepad_report(data, data_length);
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(APP_TAG, "USB HID device disconnected");
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(APP_TAG, "USB HID transfer error");
        break;

    default:
        break;
    }
}

static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
    (void)arg;

    if (event != HID_HOST_DRIVER_EVENT_CONNECTED) {
        return;
    }

    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    ESP_LOGI(APP_TAG,
             "USB HID connected (subclass=%d protocol=%d)",
             dev_params.sub_class,
             dev_params.proto);

    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback,
        .callback_arg = NULL,
    };

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));

    if (dev_params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
        ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
    }

    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void *arg)
{
    hid_host_device_event(hid_device_handle, event, arg);
}
#endif

static void usb_lib_task(void *arg)
{
    (void)arg;

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));

    while (true) {
        uint32_t event_flags = 0;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH:
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            ESP_ERROR_CHECK(esp_ble_gap_set_device_name(HIDD_DEVICE_NAME));
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&hidd_adv_data));
        }
        break;

    case ESP_HIDD_EVENT_BLE_CONNECT:
        s_ble_connected = true;
        s_hid_conn_id = param->connect.conn_id;
        ESP_LOGI(APP_TAG, "BLE HID connected (conn_id=%u)", s_hid_conn_id);
        break;

    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        s_ble_connected = false;
        s_ble_secured = false;
        ESP_LOGI(APP_TAG, "BLE HID disconnected");
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&hidd_adv_params));
        break;

    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&hidd_adv_params));
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_ERROR_CHECK(esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true));
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            s_ble_secured = true;
            ESP_LOGI(APP_TAG, "BLE security established");
        } else {
            s_ble_secured = false;
            ESP_LOGW(APP_TAG, "BLE security failed (reason=0x%x)", param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    default:
        break;
    }
}

static void init_ble_hid(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ret = esp_hidd_profile_init();
    if (ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "esp_hidd_profile_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hidd_event_callback));

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key)));
}

static void init_usb_hid_host(void)
{
#if HAS_USB_HID_HOST
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL,
    };

    BaseType_t task_created = xTaskCreatePinnedToCore(usb_lib_task,
                                                       "usb_lib",
                                                       4096,
                                                       NULL,
                                                       2,
                                                       NULL,
                                                       0);
    configASSERT(task_created == pdTRUE);

    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
#else
    ESP_LOGW(APP_TAG, "USB HID host support unavailable at compile time and HID bridge is disabled");
#endif
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(APP_TAG, "Starting ESP32-S3 USB-to-BLE gamepad bridge");

    init_ble_hid();
    init_usb_hid_host();
}
