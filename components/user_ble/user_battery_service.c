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

#include "user_battery_service.h"
#include "user_ble.h"

/**************************************************************************************/
/* テーブル定義                                                                        */
/**************************************************************************************/
/* Battery Service Table */
enum
{
    BATTERY_IDX_SERVICE = 0,
    BATTERY_LEVEL_CHAR,
    BATTERY_LEVEL_NOTIFY_VAL,
    BATTERY_LEVEL_NOTIFY_CFG,
    BATTERY_IDX_NUM,
};

/**************************************************************************************/
/* 定数定義                                                                             */
/**************************************************************************************/
#define USER_BATTERY_SERVICE_TAG   "USER_BATTERY_SERVICE"

/* サービス・キャラクタリスティック UUID  */
#define ESP_BATTERY_SERVICE_UUID        0x180F
#define ESP_BATTERY_LEVEL_CHAR_UUID     0x2A19

static const uint16_t BATTERY_SERVICE_UUID    = ESP_BATTERY_SERVICE_UUID;
static const uint16_t BATTERY_LEVEL_CHAR_UUID = ESP_BATTERY_LEVEL_CHAR_UUID;

static uint8_t        BATTERY_LEVEL_VAL[1] = {0x00};
static const uint8_t  BATTERY_LEVEL_NOTIFY_CCC[2] = {0x00, 0x00};

/* Battery Service */
static const esp_gatts_attr_db_t s_cst_gatt_batt_db[BATTERY_IDX_NUM] =
{
    /* Batteryサービス定義 */
    [BATTERY_IDX_SERVICE] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(BATTERY_SERVICE_UUID), sizeof(BATTERY_SERVICE_UUID), (uint8_t *)&BATTERY_SERVICE_UUID}},
    
    /* Battery Levelキャラクタリスティック設定 */
    [BATTERY_LEVEL_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Battery Levelキャラクタリスティック値 */
    [BATTERY_LEVEL_NOTIFY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&BATTERY_LEVEL_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(BATTERY_LEVEL_VAL), BATTERY_LEVEL_VAL}},

    /* Battery Level設定書き込み定義 */
    [BATTERY_LEVEL_NOTIFY_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(BATTERY_LEVEL_NOTIFY_CCC), (uint8_t *)&BATTERY_LEVEL_NOTIFY_CCC}},
    
};

/**************************************************************************************/
/* データ型定義                                                                      */
/**************************************************************************************/

/**************************************************************************************/
/* static変数定義                                                                  */
/**************************************************************************************/
static uint16_t s_u2_batt_handle_table[BATTERY_IDX_NUM];

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
static void user_battery_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
static void user_battery_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);

/**************************************************************************************/
/* 関数名：user_battery_service_gatt_create_attr_tab                                   */
/* 概要 ：アトリビュートテーブル作成                                                       */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_battery_service_gatt_create_attr_tab(esp_gatt_if_t t_u1_gatts_if)
{
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(s_cst_gatt_batt_db, t_u1_gatts_if, BATTERY_IDX_NUM, BATT_SVC_INST_ID);
    if (create_attr_ret)
    {
        ESP_LOGE(USER_BATTERY_SERVICE_TAG, "create attr table failed, error code = %x", create_attr_ret);
    }
    // return create_attr_ret;
}

/**************************************************************************************/
/* 関数名：user_battery_service_start_service                                          */
/* 概要 ：サービス開始                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_battery_service_start_service(esp_ble_gatts_cb_param_t *t_st_param)
{
    if((t_st_param->add_attr_tab.svc_inst_id == BATT_SVC_INST_ID) && (t_st_param->add_attr_tab.num_handle == BATTERY_IDX_NUM))
    {
        ESP_LOGI(USER_BATTERY_SERVICE_TAG, "battery create attribute table successfully, the number handle = %d\n", t_st_param->add_attr_tab.num_handle);           
        memcpy(s_u2_batt_handle_table, t_st_param->add_attr_tab.handles, sizeof(s_u2_batt_handle_table));
        esp_ble_gatts_start_service(s_u2_batt_handle_table[BATTERY_IDX_SERVICE]);
    }
}

/**************************************************************************************/
/* 関数名：user_battery_service_wraite                                                 */
/* 概要 ：サービス開始                                                                   */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_battery_service_wraite(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    uint16_t t_u2_descr_value;

    if ((s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_CFG] == t_st_param->write.handle) && (t_st_param->write.len == 2))
    {
        t_u2_descr_value = (t_st_param->write.value[1] << 8) | t_st_param->write.value[0];
        
        if (t_u2_descr_value == 0x0001)
        {
            user_battery_service_notifyEnable(t_st_param, t_u1_gatts_if);
        }
        else if (t_u2_descr_value == 0x0002)
        {
            user_battery_service_indicateEnable(t_st_param, t_u1_gatts_if);
        }
        else if (t_u2_descr_value == 0x0000)
        {
            ESP_LOGI(USER_BATTERY_SERVICE_TAG, "notify/indicate disable ");
        }
        else
        {
            ESP_LOGE(USER_BATTERY_SERVICE_TAG, "unknown descr value");
            esp_log_buffer_hex(USER_BATTERY_SERVICE_TAG, t_st_param->write.value, t_st_param->write.len);
        }
    }
}

/**************************************************************************************/
/* 関数名：user_battery_service_notifyEnable                                              */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_battery_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_BATTERY_SERVICE_TAG, "notify enable");
                
    uint8_t notify_data[10];

    for (int i = 0; i < sizeof(notify_data); ++i)
    {
        notify_data[i] = (i * 2) % 0xFF;
    }

    //the size of notify_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_VAL],
                                    sizeof(notify_data), notify_data, false);
}

/**************************************************************************************/
/* 関数名：user_battery_service_indicateEnable                                         */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_battery_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_BATTERY_SERVICE_TAG, "indicate enable");
                
    uint8_t indicate_data[15];
    
    for (int i = 0; i < sizeof(indicate_data); ++i)
    {
        indicate_data[i] = (i * 10);
    }

    //the size of indicate_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_batt_handle_table[BATTERY_LEVEL_NOTIFY_VAL],
                                sizeof(indicate_data), indicate_data, true);                                            
}