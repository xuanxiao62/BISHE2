#pragma once
#ifndef __SENSER_H__
#define __SENSER_H__

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "config.h"

#include "mqtt_client.h"
#include "esp_log.h"
#include "cJSON.h"



//线程入口
void senser_task_run(void *arg);

esp_err_t i2c_master_init(void);
void AHT20_init(void);
uint8_t AHT20_Read_Status(void);//读取AHT20的状态寄存器
uint8_t AHT20_Read_Cal_Enable(void);  //查询cal enable位有没有使能
void AHT20_SendAC(void); //向AHT20发送AC命令
void AHT20_Read_CTdata(uint32_t *ct); //没有CRC校验，直接读取AHT20的温度和湿度数据
void JH_Reset_REG(uint8_t addr);///重置寄存器
void AHT20_Start_Init(void);///上电初始化进入正常测量状态


void i2c_write(uint8_t slave_addr, uint8_t addr, uint8_t* buf, uint8_t len);
void i2c_read(uint8_t slave_addr, uint8_t addr, uint8_t* buf, uint8_t len);


/**************** AHT20 ***************/
#define AHT20_ADDR      0x70
/**************************************/

extern esp_mqtt_client_handle_t client;
// extern char g_mqtt_topic_pub[];
// extern char g_mqtt_topic_sub[];
void publish_message(const char *message);

#define MQTT_TOPIC_PUB   "/sys/a15D0zFeGtP/AHT10/thing/event/property/post"   //具体发布和订阅的主题为该前缀和chipid（mac地址的后3个字节）拼接而成，形如：/test/esp32c3/pub/7b6744
#define MQTT_TOPIC_SUB   "/sys/a15D0zFeGtP/AHT10/thing/event/property/post_reply"


#endif
