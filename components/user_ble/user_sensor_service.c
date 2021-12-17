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

#include "user_sensor_service.h"
#include "user_ble.h"

/**************************************************************************************/
/* テーブル定義                                                                        */
/**************************************************************************************/
/* Sensor Service Table */
enum
{
    SENSOR_IDX_SERVICE = 0,

    /* 6-Axis Sensor */
    SENSOR_6AXIS_CHAR,
    SENSOR_6AXIS_NOTIFY_VAL,
    SENSOR_6AXIS_NOTIFY_CFG,

    /* Quaternion Sensor */
    SENSOR_QUAT_CHAR,
    SENSOR_QUAT_NOTIFY_VAL,
    SENSOR_QUAT_NOTIFY_CFG,

    SENSOR_IDX_NUM,
};

enum{
    NOTIFY_6AXIS = 0,
    NOTIFY_QUAT,
    NOTIFY_NUM,
};

/**************************************************************************************/
/* 定数定義                                                                             */
/**************************************************************************************/
#define USER_SENSOR_SERVICE_TAG   "USER_SENSOR_SERVICE"
#define SENSOR_UUID_SIZE          16
#define AXIS_SEND_BUFFER_SIZE     24
#define QUAT_SEND_BUFFER_SIZE     22

/* サービス・キャラクタリスティック UUID  */
static uint8_t SENSOR_SERVICE_UUID[SENSOR_UUID_SIZE]    = {0x8f, 0x4b, 0x65, 0xb7, 0x9c, 0x73, 0xc3, 0x9c, 0xe7, 0x42, 0x8f, 0x3f, 0x30, 0x15, 0x00, 0x00};
static uint8_t SENSOR_6AXIS_CHAR_UUID[SENSOR_UUID_SIZE] = {0x8f, 0x4b, 0x65, 0xb7, 0x9c, 0x73, 0xc3, 0x9c, 0xe7, 0x42, 0x8f, 0x3f, 0x34, 0x15, 0x00, 0x00};
static uint8_t SENSOR_QUAT_CHAR_UUID[SENSOR_UUID_SIZE]  = {0x8f, 0x4b, 0x65, 0xb7, 0x9c, 0x73, 0xc3, 0x9c, 0xe7, 0x42, 0x8f, 0x3f, 0x36, 0x15, 0x00, 0x00};

static uint8_t        SENSOR_6AXIS_VAL[AXIS_SEND_BUFFER_SIZE]        = {0x00};
static const uint8_t  SENSOR_6AXIS_NOTIFY_CCC[AXIS_SEND_BUFFER_SIZE] = {0x00};

static uint8_t        SENSOR_QUAT_VAL[QUAT_SEND_BUFFER_SIZE]        = {0x00};
static const uint8_t  SENSOR_QUAT_NOTIFY_CCC[QUAT_SEND_BUFFER_SIZE] = {0x00};

/* Sensor Service */
static const esp_gatts_attr_db_t s_cst_gatt_sensor_db[SENSOR_IDX_NUM] =
{
    /* Sensorサービス定義 */
    [SENSOR_IDX_SERVICE] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(SENSOR_SERVICE_UUID), sizeof(SENSOR_SERVICE_UUID), SENSOR_SERVICE_UUID}},
    
    /* 6-Axis キャラクタリスティック設定 */
    [SENSOR_6AXIS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    /* 6-Axis キャラクタリスティック値 */
    [SENSOR_6AXIS_NOTIFY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, SENSOR_6AXIS_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(SENSOR_6AXIS_VAL), SENSOR_6AXIS_VAL}},
    /* 6-Axis 設定書き込み定義 */
    [SENSOR_6AXIS_NOTIFY_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(SENSOR_6AXIS_NOTIFY_CCC), sizeof(SENSOR_6AXIS_NOTIFY_CCC), (uint8_t *)&SENSOR_6AXIS_NOTIFY_CCC}},
    
    /* Quaternion キャラクタリスティック設定 */
    [SENSOR_QUAT_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    /* 6-Axis キャラクタリスティック値 */
    [SENSOR_QUAT_NOTIFY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, SENSOR_QUAT_CHAR_UUID, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(SENSOR_QUAT_VAL), SENSOR_QUAT_VAL}},
    /* 6-Axis 設定書き込み定義 */
    [SENSOR_QUAT_NOTIFY_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(SENSOR_QUAT_NOTIFY_CCC), sizeof(SENSOR_QUAT_NOTIFY_CCC), (uint8_t *)&SENSOR_QUAT_NOTIFY_CCC}},
};

/**************************************************************************************/
/* データ型定義                                                                      */
/**************************************************************************************/
struct notifyFunction{
    void  (*f_enable)(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
    void  (*f_disable)(void);
};

/**************************************************************************************/
/* static変数定義                                                                  */
/**************************************************************************************/
static uint16_t s_u2_sensor_handle_table[SENSOR_IDX_NUM];

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
static void user_sensor_6axis_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
static void user_sensor_6axis_notifyDisable(void);
static void user_sensor_quat_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if);
static void user_sensor_quat_notifyDisable(void);
static void user_sensor_write_error(esp_ble_gatts_cb_param_t *t_st_param);

static struct notifyFunction s_st_notifyFunc[NOTIFY_NUM] = {
    [NOTIFY_6AXIS] = {user_sensor_6axis_notifyEnable, user_sensor_6axis_notifyDisable},
    [NOTIFY_QUAT]  = {user_sensor_quat_notifyEnable, user_sensor_quat_notifyDisable},
};

/**************************************************************************************/
/* 関数名：user_sensor_service_gatt_create_attr_tab                                   */
/* 概要 ：アトリビュートテーブル作成                                                       */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_sensor_service_gatt_create_attr_tab(esp_gatt_if_t t_u1_gatts_if)
{
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(s_cst_gatt_sensor_db, t_u1_gatts_if, SENSOR_IDX_NUM, SENSOR_SVC_INST_ID);
    if (create_attr_ret)
    {
        ESP_LOGE(USER_SENSOR_SERVICE_TAG, "create attr table failed, error code = %x", create_attr_ret);
    }
    // return create_attr_ret;
}

/**************************************************************************************/
/* 関数名：user_sensor_service_start_service                                          */
/* 概要 ：サービス開始                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_sensor_service_start_service(esp_ble_gatts_cb_param_t *t_st_param)
{
    if((t_st_param->add_attr_tab.svc_inst_id == SENSOR_SVC_INST_ID) && (t_st_param->add_attr_tab.num_handle == SENSOR_IDX_NUM))
    {
        ESP_LOGI(USER_SENSOR_SERVICE_TAG, "sensor create attribute table successfully, the number handle = %d\n", t_st_param->add_attr_tab.num_handle);           
        memcpy(s_u2_sensor_handle_table, t_st_param->add_attr_tab.handles, sizeof(s_u2_sensor_handle_table));
        esp_ble_gatts_start_service(s_u2_sensor_handle_table[SENSOR_IDX_SERVICE]);
    }
}

/**************************************************************************************/
/* 関数名：user_sensor_service_wraite                                                 */
/* 概要 ：サービス開始                                                                   */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_sensor_service_wraite(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    uint16_t t_u2_descr_value;
    uint8_t  t_u1_notifyChar = NOTIFY_NUM;

    if(t_st_param->write.len != 2)
    {
        /* 書き込み長さが２でない場合は処理をしない */
        return;
    }

    t_u2_descr_value = (t_st_param->write.value[1] << 8) | t_st_param->write.value[0];

    /* どのキャラクタリスティックに対する命令か */
    if(s_u2_sensor_handle_table[SENSOR_6AXIS_NOTIFY_CFG] == t_st_param->write.handle)
    {
            t_u1_notifyChar = NOTIFY_6AXIS;

    }
    else if(s_u2_sensor_handle_table[SENSOR_QUAT_NOTIFY_CFG] == t_st_param->write.handle)
    {
            t_u1_notifyChar = NOTIFY_QUAT;
    }

    /* enable/disableに応じた処理を実行 */
    if(t_u1_notifyChar != NOTIFY_NUM)
    {
        if (t_u2_descr_value == 0x0001)
        {
            s_st_notifyFunc[t_u1_notifyChar].f_enable(t_st_param, t_u1_gatts_if);
        }
        else if (t_u2_descr_value == 0x0000)
        {
            s_st_notifyFunc[t_u1_notifyChar].f_disable();
        }
        else
        {
            user_sensor_write_error(t_st_param);
        }
    }
}

/**************************************************************************************/
/* 関数名：user_sensor_6axis_notifyEnable                                              */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_sensor_6axis_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_SENSOR_SERVICE_TAG, "6-axis notify enable");
                
    uint8_t t_u8_notifyData[AXIS_SEND_BUFFER_SIZE];

    for (int i = 0; i < sizeof(t_u8_notifyData); ++i)
    {
        t_u8_notifyData[i] = (i * 2) % 0xFF;
    }

    //the size of notify_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_sensor_handle_table[SENSOR_6AXIS_NOTIFY_VAL],
                                    sizeof(t_u8_notifyData), t_u8_notifyData, false);
}

/**************************************************************************************/
/* 関数名：user_sensor_quat_notifyEnable                                               */
/* 概要 ：Notify開始時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_sensor_quat_notifyEnable(esp_ble_gatts_cb_param_t *t_st_param, esp_gatt_if_t t_u1_gatts_if)
{
    ESP_LOGI(USER_SENSOR_SERVICE_TAG, "quaturnion notify enable");
                
    uint8_t t_u8_notifyData[QUAT_SEND_BUFFER_SIZE];
    
    for (int i = 0; i < sizeof(t_u8_notifyData); ++i)
    {
        t_u8_notifyData[i] = (i * 10);
    }

    //the size of indicate_data[] need less than MTU size
    esp_ble_gatts_send_indicate(t_u1_gatts_if, t_st_param->write.conn_id, s_u2_sensor_handle_table[SENSOR_QUAT_NOTIFY_VAL],
                                sizeof(t_u8_notifyData), t_u8_notifyData, true);                                            
}

/**************************************************************************************/
/* 関数名：user_sensor_6axis_notifyDisable                                             */
/* 概要 ：Notify切断時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_sensor_6axis_notifyDisable(void)
{
    ESP_LOGI(USER_SENSOR_SERVICE_TAG, "6-axis notify disable ");
}

/**************************************************************************************/
/* 関数名：user_sensor_quat_notifyDisable                                             */
/* 概要 ：Notify切断時の処理                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_sensor_quat_notifyDisable(void)
{
    ESP_LOGI(USER_SENSOR_SERVICE_TAG, "quartrnion notify disable ");
}

/**************************************************************************************/
/* 関数名：user_sensor_write_error                                                     */
/* 概要 ：write error                                                                 */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_sensor_write_error(esp_ble_gatts_cb_param_t *t_st_param)
{
    ESP_LOGE(USER_SENSOR_SERVICE_TAG, "unknown descr value");
    esp_log_buffer_hex(USER_SENSOR_SERVICE_TAG, t_st_param->write.value, t_st_param->write.len);
}