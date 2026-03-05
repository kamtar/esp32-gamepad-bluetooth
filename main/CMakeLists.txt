idf_component_register(SRCS "usb_ble_gamepad_bridge_main.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    PRIV_REQUIRES bt nvs_flash driver usb
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
