#ifndef __USER_BLE_H_
#define __USER_BLE_H_
/**************************************************************************************/
/* include                                                                            */
/**************************************************************************************/
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

/**************************************************************************************/
/* グローバル定数定義                                                                    */
/**************************************************************************************/
/* デバイス名 */
#define DEVICE_NAME "Welsense"
enum
{
    DEVICEINFO_SVC_INST_ID = 0,   /* Device Info Serivce Instance ID */
    BATT_SVC_INST_ID,             /* Battery Serivce Instance ID */
    SENSOR_SVC_INST_ID,           /* Sensor Serivce Instance ID */
    SVC_INST_ID,                  /* Service Instance ID */

    INST_ID_NUM,
};

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 1024
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t  char_prop_read_write_notify  = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  char_prop_read_notify        = ESP_GATT_CHAR_PROP_BIT_READ  | ESP_GATT_CHAR_PROP_BIT_NOTIFY; 
static const uint8_t  char_prop_read               = ESP_GATT_CHAR_PROP_BIT_READ;

/**************************************************************************************/
/* グローバル変数定義                                                                    */
/**************************************************************************************/

/**************************************************************************************/
/* グローバル関数定義                                                                   */
/**************************************************************************************/
extern esp_err_t user_ble_init(void);

#endif /* __USER_BLE_H_ */