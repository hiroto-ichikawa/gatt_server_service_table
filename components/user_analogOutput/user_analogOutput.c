/**************************************************************************************/
/* ヘッダーファイル定義                                                                  */
/**************************************************************************************/
#include <user_analogOutput.h>

/**************************************************************************************/
/* 定数定義                                                                        */
/**************************************************************************************/
#define ANALOGOUTPUT_OUTPUT_PIN_SEL(x)  (1ULL<<x)

/* LOG */
#define ANALOGOUTPUT_DEBUG_TAG "ichikawa_gpio"

/* RGB LED設定 */
static const gpio_config_t s_cst_rgbLed =
{
    .intr_type    = GPIO_INTR_DISABLE,                                       /* 割り込み設定    */
    .mode         = GPIO_MODE_INPUT_OUTPUT,                                  /* 入出力モード設定 */
    .pin_bit_mask = ANALOGOUTPUT_OUTPUT_PIN_SEL(ANALOGOUTPUT_RGB_LED),       /* 設定対象ピン    */
    .pull_down_en = GPIO_PULLDOWN_ENABLE,                                    /* プルダウン設定  */
    .pull_up_en   = GPIO_PULLUP_DISABLE,                                     /* プルアップ設定  */
};

/**************************************************************************************/
/* static変数定義                                                                     */
/**************************************************************************************/

/**************************************************************************************/
/* 関数プロトタイプ宣言（内部関数）                                                    */
/**************************************************************************************/
static void user_analogOutput_outputInit(gpio_config_t t_st_gpioConfig);

/**************************************************************************************/
/* 関数名：user_analogOutput_init                                                      */
/* 概要 ：初期化関数                                                                    */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_analogOutput_init(void)
{
    ESP_LOGI(ANALOGOUTPUT_DEBUG_TAG, "user_analogOutput_init");

    /* RGP LED出力設定 */
    user_analogOutput_outputInit(s_cst_rgbLed);
}

/**************************************************************************************/
/* 関数名：user_analogOutput_writePin                                                  */
/* 概要 ：出力書き込み                                                                  */ 
/* 入力 ：GPIO NUM                                                                    */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_analogOutput_writePin(gpio_num_t t_gpio, uint32_t t_u4_level)
{
    gpio_set_level(t_gpio, t_u4_level);
}

/**************************************************************************************/
/* 関数名：user_analogOutput_writePinToggle                                           */
/* 概要 ：出力書き込み(Toggle)                                                         */ 
/* 入力 ：GPIO NUM                                                                    */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
void user_analogOutput_writePinToggle(gpio_num_t t_gpio)
{
    int t_s4_retLevel = 0;

    t_s4_retLevel = 1 - gpio_get_level(t_gpio);
    gpio_set_level(t_gpio, t_s4_retLevel);
}

/**************************************************************************************/
/* 関数名：user_analogOutput_outputInit                                                */
/* 概要 ：GPIO出力初期化関数                                                             */ 
/* 入力 ：なし                                                                         */
/* 出力 ：なし                                                                         */
/**************************************************************************************/
static void user_analogOutput_outputInit(gpio_config_t t_st_gpioConfig)
{
    gpio_config_t io_conf = {};

    io_conf = t_st_gpioConfig;
    // 設定書き込み
    gpio_config(&io_conf);
}