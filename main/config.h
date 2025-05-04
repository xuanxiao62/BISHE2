#pragma once

#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef CONFIG_GLOBALS
    #define CONFIG_EXT
#else
    #define CONFIG_EXT extern
#endif

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "spi_flash_mmap.h"
#include "esp_log.h"

#include "senser.h"

//----------------------------------------------------------
//定义常量
// typedef uint8_t UINT8;
// typedef uint16_t UINT16;
// typedef uint32_t UINT32;
// typedef uint64_t UINT64;
// typedef bool BOOLEAN;

//定义GPIO
#define GPIO_AHT20_SCL          5
#define GPIO_AHT20_SDA          4

#define GPIO_LED_RED            10
#define GPIO_LED_GREEN          11
#define GPIO_LED_BLUE           12


//定义全局变量
extern int16_t 	global_humi;	//板载湿度
extern int16_t 	global_tem;		//板载温度






//定义全局函数
#define SLEEP_MS(X) vTaskDelay(X / portTICK_PERIOD_MS);		//毫秒延时(最小分度10ms)


#endif /* __file_end__ */
