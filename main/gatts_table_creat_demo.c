/**************************************************************************************/
/* ヘッダーファイル定義                                                                  */
/**************************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "gatts_table_creat_demo.h"

#include "user_analogOutput.h"
#include "user_ble.h"

/**************************************************************************************/
/* 定数定義                                                                            */
/**************************************************************************************/
#define MAIN_ERROR_TAG  "MAIN_ERROR"

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                        */
/**************************************************************************************/
static esp_err_t app_init(void);

/**************************************************************************************/
/* 関数名：app_main                                                                    */
/* 概要 ：メイン関数                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void app_main(void)
{
    esp_err_t t_retErr;

    t_retErr = app_init();
    if(t_retErr != ESP_OK)
    {
        ESP_LOGE(MAIN_ERROR_TAG, "Initialization error!!!");
        return;
    }
    
    while(1)
    {
        // ESP_LOGI(ICHIKAWA_TAG, "porlling counter : %ld", t_u4_cnt);
        // if(st_notifyContorl.t_bl_flg == true)
        // {
        //     t_u1_sendBuff[0] = (t_u4_cnt & 0xFF000000) >> 24;
        //     t_u1_sendBuff[1] = (t_u4_cnt & 0x00FF0000) >> 16;
        //     t_u1_sendBuff[2] = (t_u4_cnt & 0x0000FF00) >>  8;
        //     t_u1_sendBuff[3] = (t_u4_cnt & 0x000000FF);

        //     esp_ble_gatts_send_indicate(st_notifyContorl.t_u2_gatts_if, st_notifyContorl.t_u2_conn_id, st_notifyContorl.t_u2_service_handle,
        //                                 sizeof(t_u1_sendBuff), t_u1_sendBuff, false);

        //     t_u4_cnt++;
        // }
        // else
        // {
        //     t_u4_cnt = 0;
        // }

        user_analogOutput_writePinToggle(ANALOGOUTPUT_RGB_LED);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**************************************************************************************/
/* 関数名：app_init                                                                    */
/* 概要 ：初期化関数                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static esp_err_t app_init(void)
{
    esp_err_t t_ret;

    /* Initialize NVS. */
    ESP_LOGI(MAIN_ERROR_TAG, "nvs_flash_init");
    t_ret = nvs_flash_init();
    if (t_ret == ESP_ERR_NVS_NO_FREE_PAGES || t_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        t_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( t_ret );

    /* アナログ入力初期化 */
    user_analogOutput_init();

    /* BLE初期化 */
    t_ret = user_ble_init();
    if(t_ret != ESP_OK)
    {
        ESP_LOGE(MAIN_ERROR_TAG, "user_ble_init() error");
    }

    return t_ret;
}