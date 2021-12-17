/**************************************************************************************/
/* ヘッダーファイル定義                                                                  */
/**************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "user_ble.h"
#include "user_test_service.h"
#include "user_battery_service.h"
#include "user_deviceInfo_service.h"
#include "user_sensor_service.h"

/**************************************************************************************/
/* 定数定義                                                                        */
/**************************************************************************************/
#define BLE_GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

/* アプリケーションプロファイル設定 */
#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)


/**************************************************************************************/
/* データ型定義                                                                      */
/**************************************************************************************/
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

struct gatts_profile_inst{
    esp_gatts_cb_t       gatts_cb;
    uint16_t             gatts_if;
    uint16_t             app_id;
    uint16_t             conn_id;
    uint16_t             service_handle;
    esp_gatt_srvc_id_t   service_id;
    uint16_t             char_handle;
    esp_bt_uuid_t        char_uuid;
    esp_gatt_perm_t      perm;
    esp_gatt_char_prop_t property;
    uint16_t             descr_handle;
    esp_bt_uuid_t        descr_uuid;
};

struct service_talbe_t{
    void (*create_attr_tab)(esp_gatt_if_t t_u1_gatts_if);
    void (*start_service)(esp_ble_gatts_cb_param_t *t_st_param);
    void (*wraite)(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
};

/**************************************************************************************/
/* static変数定義                                                                  */
/**************************************************************************************/
static uint8_t adv_config_done       = 0;

/* サービス UUID */
static uint8_t s_u1_service_uuid[64] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0A, 0x18, 0x00, 0x00,
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00,
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x30, 0x15, 0x00, 0x00,
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* アドバタイジングデータ(31 byte) */
static esp_ble_adv_data_t s_st_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,      //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,   //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* スキャンレスポンスデータ */
static esp_ble_adv_data_t s_st_scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,     //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(s_u1_service_uuid),
    .p_service_uuid      = s_u1_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* アドバタイジングパラメータ */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static struct service_talbe_t s_st_service_table[INST_ID_NUM] = {
    [DEVICEINFO_SVC_INST_ID] = {user_deviceInfo_service_gatt_create_attr_tab, user_deviceInfo_service_start_service, NULL},
    [BATT_SVC_INST_ID]       = {user_battery_service_gatt_create_attr_tab,    user_battery_service_start_service,    user_battery_service_wraite},
    [SENSOR_SVC_INST_ID]     = {user_sensor_service_gatt_create_attr_tab,     user_sensor_service_start_service,     user_sensor_service_wraite},
    [SVC_INST_ID]            = {user_test_service_gatt_create_attr_tab,       user_test_service_start_service,       user_test_service_wraite},
};

static prepare_type_env_t prepare_write_env;

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
static void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void ble_gap_adv_data_set_complete_evt(void);
static void ble_gap_scan_rsp_data_set_complete_evt(void);
static void ble_gap_adv_start_complete_evt(esp_ble_gap_cb_param_t *t_st_param);
static void ble_gap_adv_stop_complete_evt(esp_ble_gap_cb_param_t *t_st_param);
static void ble_gap_update_conn_params_evt(esp_ble_gap_cb_param_t *t_st_param);

static void ble_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void ble_gatts_profile_reg_evt(esp_gatt_if_t t_u1_gatts_if);
static void ble_gatts_profile_read_evt(void);
static void ble_gatts_profile_write_evt(esp_gatt_if_t t_u1_gatts_if, esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_exec_write_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_mtu_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_conf_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_start_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_connect_evt(esp_gatt_if_t t_u1_gatts_if, esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_disconnect_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void ble_gatts_profile_creat_attr_tab_evt(esp_ble_gatts_cb_param_t *t_st_param);
static void example_prepare_write_event_env(esp_gatt_if_t t_u1_gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

/**************************************************************************************/
/* プロファイルテーブル作成                                                          */
/**************************************************************************************/
/* プロファイルテーブル */
static struct gatts_profile_inst s_st_profile_table[PROFILE_NUM] = 
{
    [PROFILE_APP_IDX] = {
        .gatts_cb = ble_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/**************************************************************************************/
/* 関数名：user_ble_init                                                               */
/* 概要 ：初期化関数                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
esp_err_t user_ble_init(void)
{
    esp_err_t t_ret;

    /* コントローラメモリ解放 */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* BTコントローラ初期化 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    t_ret = esp_bt_controller_init(&bt_cfg);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(t_ret));
        return t_ret;
    }

    /* BTコントローラ有効化 */
    t_ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(t_ret));
        return t_ret;
    }

    /* Bluetoothリソースの初期化 */
    t_ret = esp_bluedroid_init();
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(t_ret));
        return t_ret;
    }

    /* Bluetooth有効化 */
    t_ret = esp_bluedroid_enable();
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(t_ret));
        return t_ret;
    }

    /* BTA(ペリフェラル？) GATTSモジュールへイベントハンドラーを登録 */
    t_ret = esp_ble_gatts_register_callback(ble_gatts_event_handler);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "gatts register error, error code = %x", t_ret);
        return t_ret;
    }

    /* GAPイベントのコールバック登録 */
    t_ret = esp_ble_gap_register_callback(ble_gap_event_handler);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "gap register error, error code = %x", t_ret);
        return t_ret;
    }

    /* アプリケーションプロファイル登録 */
    t_ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "gatts app register error, error code = %x", t_ret);
        return t_ret;
    }

    /* ローカルMTU(Maximum Transmission Unit)設定 */
    t_ret = esp_ble_gatt_set_local_mtu(500);
    if (t_ret != ESP_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", t_ret);
        return t_ret;
    }

    return t_ret;
}

/**************************************************************************************/
/* 関数名：user_ble_gatts_event_handler                                                */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* 登録イベントの場合、プロファイルのgatts_ifを保存する */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            s_st_profile_table[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(BLE_GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if ((gatts_if == ESP_GATT_IF_NONE) || (gatts_if == s_st_profile_table[idx].gatts_if))
            {
                if (s_st_profile_table[idx].gatts_cb != NULL)
                {
                    s_st_profile_table[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    }while (0);
}

/**************************************************************************************/
/* 関数名：gap_event_handler                                                           */
/* 概要 ：GAPイベントハンドラー                                                           */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ble_gap_adv_data_set_complete_evt();
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ble_gap_scan_rsp_data_set_complete_evt();
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ble_gap_adv_start_complete_evt(param);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ble_gap_adv_stop_complete_evt(param);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ble_gap_update_conn_params_evt(param);
            break;
        default:
            break;
    }
}

/**************************************************************************************/
/* 関数名：ble_gap_adv_data_set_complete_evt                                           */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_adv_data_set_complete_evt(void)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
    
    adv_config_done &= (~ADV_CONFIG_FLAG);
    if (adv_config_done == 0)
    {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

/**************************************************************************************/
/* 関数名：ble_gap_adv_data_set_complete_evt                                           */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_scan_rsp_data_set_complete_evt(void)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
    
    adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    if (adv_config_done == 0)
    {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

/**************************************************************************************/
/* 関数名：ble_gap_adv_start_complete_evt                                              */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_adv_start_complete_evt(esp_ble_gap_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
    
    /* advertising start complete event to indicate advertising start successfully or failed */
    if (t_st_param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) 
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "advertising start failed");
    }
    else
    {
        ESP_LOGI(BLE_GATTS_TABLE_TAG, "advertising start successfully");
    }
}

/**************************************************************************************/
/* 関数名：ble_gap_adv_stop_complete_evt                                              */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_adv_stop_complete_evt(esp_ble_gap_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
    
    if (t_st_param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "Advertising stop failed");
    }
    else
    {
        ESP_LOGI(BLE_GATTS_TABLE_TAG, "Stop adv successfully\n");
    }
}

/**************************************************************************************/
/* 関数名：ble_gap_update_conn_params_evt                                              */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gap_update_conn_params_evt(esp_ble_gap_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  t_st_param->update_conn_params.status,
                  t_st_param->update_conn_params.min_int,
                  t_st_param->update_conn_params.max_int,
                  t_st_param->update_conn_params.conn_int,
                  t_st_param->update_conn_params.latency,
                  t_st_param->update_conn_params.timeout);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_event_handler                                             */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:  /* アプリケーション登録 */
            ble_gatts_profile_reg_evt(gatts_if);
       	    break;
        case ESP_GATTS_READ_EVT:
            ble_gatts_profile_read_evt();
       	    break;
        case ESP_GATTS_WRITE_EVT:
            ble_gatts_profile_write_evt(gatts_if, param);
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ble_gatts_profile_exec_write_evt(param);
            break;
        case ESP_GATTS_MTU_EVT:
            ble_gatts_profile_mtu_evt(param);
            break;
        case ESP_GATTS_CONF_EVT:
            ble_gatts_profile_conf_evt(param);
            break;
        case ESP_GATTS_START_EVT:
            ble_gatts_profile_start_evt(param);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ble_gatts_profile_connect_evt(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ble_gatts_profile_disconnect_evt(param);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ble_gatts_profile_creat_attr_tab_evt(param);
            break;
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_reg_evt                                                   */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_reg_evt(esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_REG_EVT");
    uint8_t t_u1_cnt;

    /* デバイス名設定 */
    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (set_dev_name_ret)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    }

    /* アドバタイジングデータ登録 */
    esp_err_t ret = esp_ble_gap_config_adv_data(&s_st_adv_data);
    if (ret)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= ADV_CONFIG_FLAG;
    
    /* スキャンレスポンスデータ登録 */
    ret = esp_ble_gap_config_adv_data(&s_st_scan_rsp_data);
    if (ret)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;

    /* Attribute Tableからサービスとキャラクタリスティックの作成 */
    for (t_u1_cnt = 0; t_u1_cnt < INST_ID_NUM; t_u1_cnt++)
    {
        if(s_st_service_table[t_u1_cnt].create_attr_tab != NULL)
        {
            s_st_service_table[t_u1_cnt].create_attr_tab(t_u1_gatts_if);
        }
    }
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_read_evt                                                          */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_read_evt(void)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_write_evt                                                 */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_write_evt(esp_gatt_if_t t_u1_gatts_if, esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT");
    uint8_t t_u1_cnt;
    
    if (!t_st_param->write.is_prep)
    {
        // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(BLE_GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", t_st_param->write.handle, t_st_param->write.len);
        esp_log_buffer_hex(BLE_GATTS_TABLE_TAG, t_st_param->write.value, t_st_param->write.len);

        /* サービス毎にモジュールを追加すること */
        for (t_u1_cnt = 0; t_u1_cnt < INST_ID_NUM; t_u1_cnt++)
        {
            if(s_st_service_table[t_u1_cnt].wraite != NULL)
            {
                s_st_service_table[t_u1_cnt].wraite(t_st_param, t_u1_gatts_if);
            }
        }

        /* send response when param->write.need_rsp is true*/
        if (t_st_param->write.need_rsp == true)
        {
            esp_ble_gatts_send_response(t_u1_gatts_if, t_st_param->write.conn_id, t_st_param->write.trans_id, ESP_GATT_OK, NULL);
        }
    }
    else
    {
        /* handle prepare write */
        example_prepare_write_event_env(t_u1_gatts_if, &prepare_write_env, t_st_param);
    }
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_exec_write_evt                                            */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_exec_write_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    example_exec_write_event_env(&prepare_write_env, t_st_param);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_mtu_evt                                                   */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_mtu_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", t_st_param->mtu.mtu);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_conf_evt                                                  */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_conf_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", 
                                t_st_param->conf.status, t_st_param->conf.handle);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_start_evt                                                 */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_start_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", 
                                t_st_param->start.status, t_st_param->start.service_handle);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_connect_evt                                               */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_connect_evt(esp_gatt_if_t t_u1_gatts_if, esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d, gatts_if = %d", 
                                t_st_param->connect.conn_id, t_u1_gatts_if);

    esp_log_buffer_hex(BLE_GATTS_TABLE_TAG, t_st_param->connect.remote_bda, 6);
    
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, t_st_param->connect.remote_bda, sizeof(esp_bd_addr_t));
    
    /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
    conn_params.latency = 0;
    conn_params.max_int = 0x18;    // max_int = 0x18*1.25ms = 30ms
    conn_params.min_int = 0x0C;    // min_int = 0x0C*1.25ms = 15ms
    conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
    
    //start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_disconnect_evt                                            */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_disconnect_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", 
                                    t_st_param->disconnect.reason);
    esp_ble_gap_start_advertising(&adv_params);
}

/**************************************************************************************/
/* 関数名：ble_gatts_profile_creat_attr_tab_evt                                        */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void ble_gatts_profile_creat_attr_tab_evt(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT");
    uint8_t t_u1_cnt;

    if (t_st_param->add_attr_tab.status != ESP_GATT_OK)
    {
        ESP_LOGE(BLE_GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", 
                                        t_st_param->add_attr_tab.status);
    }
    else
    {
        for (t_u1_cnt = 0; t_u1_cnt < INST_ID_NUM; t_u1_cnt++)
        {
            if(s_st_service_table[t_u1_cnt].start_service != NULL)
            {
                s_st_service_table[t_u1_cnt].start_service(t_st_param);
            }
        }
    }
}

/**************************************************************************************/
/* 関数名：example_prepare_write_event_env                                             */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(BLE_GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(BLE_GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(BLE_GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

/**************************************************************************************/
/* 関数名：example_exec_write_event_env                                                */
/* 概要 ：GATTイベントハンドラー                                                         */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(BLE_GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(BLE_GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}
