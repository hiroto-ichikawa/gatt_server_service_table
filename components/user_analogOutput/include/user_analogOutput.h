#ifndef __USER_ANALOGOUTPUT_H_
#define __USER_ANALOGOUTPUT_H_
/**************************************************************************************/
/* include                                                                            */
/**************************************************************************************/
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

/**************************************************************************************/
/* 定数定義                                                                            */
/**************************************************************************************/
#define ANALOGOUTPUT_HIGH   0x01
#define ANALOGOUTPUT_LOW    0x00

#define ANALOGOUTPUT_RGB_LED  GPIO_NUM_48

/**************************************************************************************/
/* グローバル関数定義                                                                   */
/**************************************************************************************/
extern void user_analogOutput_init(void);
extern void user_analogOutput_writePin(gpio_num_t t_gpio, uint32_t t_u4_level);
extern void user_analogOutput_writePinToggle(gpio_num_t t_gpio);

#endif /* __USER_GPIO_H_ */