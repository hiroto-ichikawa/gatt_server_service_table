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

#include "user_test_service.h"
#include "user_ble.h"

/**************************************************************************************/
/* テーブル定義                                                                        */
/**************************************************************************************/
/* Service Table */
enum
{
    IDX_SVC,                /* サービスインデックス                  */
    IDX_CHAR_A,             /* キャラクタリスティック                */
    IDX_CHAR_VAL_A,         /* キャラクタリスティック value(値)       */
    IDX_CHAR_CFG_A,         /* Clinet charactrisitc Configuration */

    IDX_NB,                 /* テーブル要素数                        */
};

/**************************************************************************************/
/* 定数定義                                                                             */
/**************************************************************************************/
#define USER_TEST_SERVICE_TAG   "USER_TEST_SERVICE"

/* サービス・キャラクタリスティック UUID  */
#define ESP_TEST_SERVICE_UUID  0x00FF
#define ESP_TEST_CHAR_UUID     0xFF01

static const uint16_t GATTS_SERVICE_UUID_TEST  = ESP_TEST_SERVICE_UUID;
static const uint16_t GATTS_CHAR_UUID_TEST_A   = ESP_TEST_CHAR_UUID;

static const uint8_t  heart_measurement_ccc[2] = {0x00, 0x00};
static const uint8_t  char_value[4]            = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t s_cst_gatt_db[IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},
};

/**************************************************************************************/
/* データ型定義                                                                      */
/**************************************************************************************/

/**************************************************************************************/
/* static変数定義                                                                  */
/**************************************************************************************/
static uint16_t s_u2_handle_table[IDX_NB];

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
static void user_test_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
static void user_test_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);

/**************************************************************************************/
/* 関数名：user_test_service_gatt_create_attr_tab                                      */
/* 概要 ：アトリビュートテーブル作成                                                       */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_test_service_gatt_create_attr_tab(esp_gatt_if_t t_u1_gatts_if)
{
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(s_cst_gatt_db, t_u1_gatts_if, IDX_NB, SVC_INST_ID);

    if (create_attr_ret)
    {
        ESP_LOGE(USER_TEST_SERVICE_TAG, "create attr table failed, error code = %x", create_attr_ret);
    }

    // return create_attr_ret;
}

/**************************************************************************************/
/* 関数名：user_test_service_start_service                                             */
/* 概要 ：サービス開始                                                                   */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_test_service_start_service(esp_ble_gatts_cb_param_t *t_st_param)
{
    if((t_st_param->add_attr_tab.svc_inst_id == SVC_INST_ID) && (t_st_param->add_attr_tab.num_handle == IDX_NB))
    {
        ESP_LOGI(USER_TEST_SERVICE_TAG, "create attribute table successfully, the number handle = %d\n", t_st_param->add_attr_tab.num_handle);           
        memcpy(s_u2_handle_table, t_st_param->add_attr_tab.handles, sizeof(s_u2_handle_table));
        esp_ble_gatts_start_service(s_u2_handle_table[IDX_SVC]);
    }
}

/**************************************************************************************/
/* 関数名：user_test_service_wraite                                                    */
/* 概要 ：サービス開始                                                                   */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_test_service_wraite(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    uint16_t t_u2_descr_value;

    if ((s_u2_handle_table[IDX_CHAR_CFG_A] == t_st_param->write.handle) && (t_st_param->write.len == 2))
    {
        t_u2_descr_value = (t_st_param->write.value[1] << 8) | t_st_param->write.value[0];
        
        if (t_u2_descr_value == 0x0001)
        {
            user_test_service_notifyEnable(t_st_param, t_u1_gatts_if);
        }
        else if (t_u2_descr_value == 0x0002)
        {
            user_test_service_indicateEnable(t_st_param, t_u1_gatts_if);
        }
        else if (t_u2_descr_value == 0x0000)
        {
            ESP_LOGI(USER_TEST_SERVICE_TAG, "notify/indicate disable ");
        }
        else
        {
            ESP_LOGE(USER_TEST_SERVICE_TAG, "unknown descr value");
            esp_log_buffer_hex(USER_TEST_SERVICE_TAG, t_st_param->write.value, t_st_param->write.len);
        }
    }
}

/**************************************************************************************/
/* 関数名：user_test_service_notifyEnable                                              */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_test_service_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_TEST_SERVICE_TAG, "notify enable");
                
    uint8_t notify_data[10];

    for (int i = 0; i < sizeof(notify_data); ++i)
    {
        notify_data[i] = i % 0xff;
    }

    //the size of notify_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_handle_table[IDX_CHAR_VAL_A],
                                    sizeof(notify_data), notify_data, false);
}

/**************************************************************************************/
/* 関数名：user_test_service_notifyEnable                                              */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_test_service_indicateEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_TEST_SERVICE_TAG, "indicate enable");
                
    uint8_t indicate_data[15];
    
    for (int i = 0; i < sizeof(indicate_data); ++i)
    {
        indicate_data[i] = (i*10) % 0xff;
    }

    //the size of indicate_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_handle_table[IDX_CHAR_VAL_A],
                                sizeof(indicate_data), indicate_data, true);                                            
}