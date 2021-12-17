idf_component_register(SRCS "user_ble.c" 
                            "user_battery_service.c" 
                            "user_test_service.c" 
                            "user_deviceInfo_service.c" 
                            "user_sensor_service.c"
                    INCLUDE_DIRS "include" 
                                 "../../../esp-idf/components/bt/include/esp32s3/include"
                                 "../../../esp-idf/components/bt/host/bluedroid/api/include/api")
