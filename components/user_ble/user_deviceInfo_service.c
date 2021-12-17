/**************************************************************************************/
/* ヘッダーファイル定義                                                                  */
/**************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

#include "user_deviceInfo_service.h"
#include "user_ble.h"

/**************************************************************************************/
/* テーブル定義                                                                        */
/**************************************************************************************/
/* Device Infomation Service Table */
enum
{
    DEVICEINFO_IDX_SERVICE = 0,

    /* Manufacture Name String */
    DEVICEINFO_MANUFACTURE_CHAR,
    DEVICEINFO_MANUFACTURE_VAL,

    /* Model Number String */
    DEVICEINFO_MODEL_NUMBER_CHAR,
    DEVICEINFO_MODEL_NUMBER_VAL,

    /* Serial Number String */
    DEVICEINFO_SERIAL_NUMBER_CHAR,
    DEVICEINFO_SERIAL_NUMBER_VAL,

    /* HardWare Revision String */
    DEVICEINFO_HARDWARE_REVISION_CHAR,
    DEVICEINFO_HARDWARE_REVISION_VAL,

    /* Software Revision String */
    DEVICEINFO_SOFTWARE_REVISION_CHAR,
    DEVICEINFO_SOFTWARE_REVISION_VAL,

    DEVICEINFO_IDX_NUM,
};

/**************************************************************************************/
/* 定数定義                                                                             */
/**************************************************************************************/
#define USER_DEVICEINFO_SERVICE_TAG   "USER_DEVICEINFO_SERVICE"

/* サービス・キャラクタリスティック UUID  */
#define ESP_DEVICEINFO_SERVICE_UUID              0x180A
#define ESP_DEVICEINFO_MANUFACTURE_CHAR_UUID     0x2A29
#define ESP_DEVICEINFO_MODEL_NUM_CHAR_UUID       0x2A24
#define ESP_DEVICEINFO_SERIAL_NUM_CHAR_UUID      0x2A25
#define ESP_DEVICEINFO_HARDWARE_REV_CHAR_UUID    0x2A27
#define ESP_DEVICEINFO_SOFTWARE_REV_CHAR_UUID    0x2A26

static const uint16_t DEVICEINFO_SERVICE_UUID           = ESP_DEVICEINFO_SERVICE_UUID;
static const uint16_t DEVICEINFO_MANUFACTURE_CHAR_UUID  = ESP_DEVICEINFO_MANUFACTURE_CHAR_UUID;
static const uint16_t DEVICEINFO_MODEL_NUM_CHAR_UUID    = ESP_DEVICEINFO_MODEL_NUM_CHAR_UUID;
static const uint16_t DEVICEINFO_SERIAL_NUM_CHAR_UUID   = ESP_DEVICEINFO_SERIAL_NUM_CHAR_UUID;
static const uint16_t DEVICEINFO_HARDWARE_REV_CHAR_UUID = ESP_DEVICEINFO_HARDWARE_REV_CHAR_UUID;
static const uint16_t DEVICEINFO_SOFTWARE_REV_CHAR_UUID = ESP_DEVICEINFO_SOFTWARE_REV_CHAR_UUID;

static const char     DEVICEINFO_MANUFACTURE[]  = "TOYOTA";
static const char     DEVICEINFO_MODEL_NUM[]    = "ESP32-DEMO";
static const char     DEVICEINFO_SERIAL_NUM[]   = "Welsense 3";
static const char     DEVICEINFO_HARDWARE_REV[] = "1.0";
static const char     DEVICEINFO_SOFTWARE_REV[] = "2.0";

/* Device Info Service */
static const esp_gatts_attr_db_t s_cst_gatt_deviceInfo_db[DEVICEINFO_IDX_NUM] =
{
    /* Device Info サービス定義 */
    [DEVICEINFO_IDX_SERVICE] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(DEVICEINFO_SERVICE_UUID), sizeof(DEVICEINFO_SERVICE_UUID), (uint8_t *)&DEVICEINFO_SERVICE_UUID}},
    
    /* Manufacture キャラクタリスティック設定 */
    [DEVICEINFO_MANUFACTURE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Manufacture キャラクタリスティック値 */
    [DEVICEINFO_MANUFACTURE_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICEINFO_MANUFACTURE_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(DEVICEINFO_MANUFACTURE), (uint8_t *)&DEVICEINFO_MANUFACTURE}},

    /* Model Number キャラクタリスティック設定 */
    [DEVICEINFO_MODEL_NUMBER_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Model Number キャラクタリスティック値 */
    [DEVICEINFO_MODEL_NUMBER_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICEINFO_MODEL_NUM_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(DEVICEINFO_MODEL_NUM), (uint8_t *)&DEVICEINFO_MODEL_NUM}},

    /* Serial Number キャラクタリスティック設定 */
    [DEVICEINFO_SERIAL_NUMBER_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Model Number キャラクタリスティック値 */
    [DEVICEINFO_SERIAL_NUMBER_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICEINFO_SERIAL_NUM_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(DEVICEINFO_SERIAL_NUM), (uint8_t *)&DEVICEINFO_SERIAL_NUM}},

    /* Hardware Revision キャラクタリスティック設定 */
    [DEVICEINFO_HARDWARE_REVISION_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Hardware Revision キャラクタリスティック値 */
    [DEVICEINFO_HARDWARE_REVISION_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICEINFO_HARDWARE_REV_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(DEVICEINFO_HARDWARE_REV), (uint8_t *)&DEVICEINFO_HARDWARE_REV}},

    /* Software Revision キャラクタリスティック設定 */
    [DEVICEINFO_SOFTWARE_REVISION_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Software Revision キャラクタリスティック値 */
    [DEVICEINFO_SOFTWARE_REVISION_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICEINFO_SOFTWARE_REV_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(DEVICEINFO_SOFTWARE_REV), (uint8_t *)&DEVICEINFO_SOFTWARE_REV}},
};

/**************************************************************************************/
/* データ型定義                                                                      */
/**************************************************************************************/

/**************************************************************************************/
/* static変数定義                                                                  */
/**************************************************************************************/
static uint16_t s_u2_deviceInfo_handle_table[DEVICEINFO_IDX_NUM];

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
// static void user_deviceInfo_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
// static void user_deviceInfo_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);

/**************************************************************************************/
/* 関数名：user_deviceInfo_service_gatt_create_attr_tab                                */
/* 概要 ：アトリビュートテーブル作成                                                       */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_deviceInfo_service_gatt_create_attr_tab(esp_gatt_if_t t_u1_gatts_if)
{
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(s_cst_gatt_deviceInfo_db, t_u1_gatts_if, DEVICEINFO_IDX_NUM, DEVICEINFO_SVC_INST_ID);
    if (create_attr_ret)
    {
        ESP_LOGE(USER_DEVICEINFO_SERVICE_TAG, "create attr table failed, error code = %x", create_attr_ret);
    }
    // return create_attr_ret;
}

/**************************************************************************************/
/* 関数名：user_deviceInfo_service_start_service                                        */
/* 概要 ：サービス開始                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_deviceInfo_service_start_service(esp_ble_gatts_cb_param_t *t_st_param)
{
    if((t_st_param->add_attr_tab.svc_inst_id == DEVICEINFO_SVC_INST_ID) && (t_st_param->add_attr_tab.num_handle == DEVICEINFO_IDX_NUM))
    {
        ESP_LOGI(USER_DEVICEINFO_SERVICE_TAG, "DeviceInfomation create attribute table successfully, the number handle = %d\n", t_st_param->add_attr_tab.num_handle);           
        memcpy(s_u2_deviceInfo_handle_table, t_st_param->add_attr_tab.handles, sizeof(s_u2_deviceInfo_handle_table));
        esp_ble_gatts_start_service(s_u2_deviceInfo_handle_table[DEVICEINFO_IDX_SERVICE]);
    }
}

/**************************************************************************************/
/* 関数名：user_deviceInfo_service_wraite                                              */
/* 概要 ：サービス開始                                                                   */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
// void user_deviceInfo_service_wraite(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
// {
//     uint16_t t_u2_descr_value;

//     if ((s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_CFG] == t_st_param->write.handle) && (t_st_param->write.len == 2))
//     {
//         t_u2_descr_value = (t_st_param->write.value[1] << 8) | t_st_param->write.value[0];
        
//         if (t_u2_descr_value == 0x0001)
//         {
//             user_battery_service_notifyEnable(t_st_param, t_u1_gatts_if);
//         }
//         else if (t_u2_descr_value == 0x0002)
//         {
//             user_battery_service_indicateEnable(t_st_param, t_u1_gatts_if);
//         }
//         else if (t_u2_descr_value == 0x0000)
//         {
//             ESP_LOGI(USER_BATTERY_SERVICE_TAG, "notify/indicate disable ");
//         }
//         else
//         {
//             ESP_LOGE(USER_BATTERY_SERVICE_TAG, "unknown descr value");
//             esp_log_buffer_hex(USER_BATTERY_SERVICE_TAG, t_st_param->write.value, t_st_param->write.len);
//         }
//     }
// }

/**************************************************************************************/
/* 関数名：user_battery_service_notifyEnable                                              */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
// static void user_battery_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
// {
//     ESP_LOGI(USER_BATTERY_SERVICE_TAG, "notify enable");
                
//     uint8_t notify_data[10];

//     for (int i = 0; i < sizeof(notify_data); ++i)
//     {
//         notify_data[i] = (i * 2) % 0xFF;
//     }

//     //the size of notify_data[] need less than MTU size
//     esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_VAL],
//                                     sizeof(notify_data), notify_data, false);
// }

/**************************************************************************************/
/* 関数名：user_battery_service_indicateEnable                                         */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
// static void user_battery_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
// {
//     ESP_LOGI(USER_BATTERY_SERVICE_TAG, "indicate enable");
                
//     uint8_t indicate_data[15];
    
//     for (int i = 0; i < sizeof(indicate_data); ++i)
//     {
//         indicate_data[i] = (i * 10);
//     }

//     //the size of indicate_data[] need less than MTU size
//     esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_VAL],
//                                 sizeof(indicate_data), indicate_data, true);                                            
// }