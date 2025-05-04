#include "senser.h"
#include "common.h"

static const char TAG[] = "senser.c";

#define I2C_MASTER_SCL_IO           5                           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4                           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000



int16_t global_humi = 0;
int16_t global_tem = 0;
uint16_t global_volt = 0;
#define ADC_BUF_LENTH 10

/***************************************************/

void senser_task_run(void *arg){

    uint32_t AHT20_data[2] = {0};
    int16_t tem = 0, hum = 0;
    // char payload[100] = {0};
    char str[10] ={0};
    uint8_t i ;

    AHT20_init();
    SLEEP_MS(10);

    while(1){
    /***************** 温湿度 *****************/
        i = 5;
        tem = 0;
        hum = 0;
        char payload[256] = {0};
        while(i--)
        {
            AHT20_Read_CTdata(AHT20_data);       //不经过CRC校验，直接读取AHT20的温度和湿度数据    推荐每隔大于1S读一次
            hum += AHT20_data[0]*100*10/1024/1024;     //计算得到湿度值(放大10倍)
            tem += AHT20_data[1]*200*10/1024/1024-500; //计算得到温度值(放大10倍)
            // 累加

            SLEEP_MS(100);
        }
        hum /= 5;   // 累加求平均
        tem /= 5;

        global_humi = hum; //传入全局变量
        global_tem = tem;
    /*****************************************/
        // ESP_LOGI(TAG, "tem:%d.%d, hum:%d.%d", global_tem / 10, global_tem % 10, global_humi / 10, global_humi % 10);
        // sprintf(payload, "{\n\"tem\":%d.%d,\"hum\":%d.%d\n}", global_tem / 10, global_tem % 10, global_humi / 10, global_humi % 10);
        // publish_message(payload);


        ESP_LOGI(TAG, "tem:%d.%d, hum:%d.%d", global_tem / 10, global_tem % 10, global_humi / 10, global_humi % 10);
        sprintf(payload, "{\n\"id\":1721639473270,\"params\": {\"KeepFreshTemperature\":%d,\"CurrentHumidity\":%d},\"version\":\"1.0\",\"method\":\"thing.event.property.post\"}\n", global_tem / 10, global_humi / 10);
        publish_message(payload);

        // cJSON* msg_json = cJSON_CreateObject();
        // sprintf(str, "%2d.%1d", global_tem / 10, global_tem % 10);
        // cJSON_AddStringToObject(msg_json, "tem", str);
        // sprintf(str, "%2d.%1d", global_humi / 10, global_humi % 10);
        // cJSON_AddStringToObject(msg_json, "hum", str);
        // char* msg_str = cJSON_Print(msg_json);
        // publish_message(msg_str);
        // printf(msg_str);
        // cJSON_Delete(msg_json);


    /*****************************************/
        SLEEP_MS(900000); //
    }

}

/**************************************************************************************************************************************************/

void publish_message(const char *message) {
    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_PUB, message, 0, 1, 0);
    printf("publish: %s\n", MQTT_TOPIC_PUB);
    ESP_LOGI("MQTT_PUBLISH", "Message published, msg_id=%d", msg_id);
}




/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    static i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}



/*********************** AHT20 驱动部分 **********************/
void AHT20_init(void)
{
    while((AHT20_Read_Status() & 0x18) != 0x18)
    {
        AHT20_Start_Init(); //重新初始化寄存器
        SLEEP_MS(1000);
    }
    ESP_LOGI(TAG, "AHT20 initialized OK.\r\n");
}

uint8_t AHT20_Read_Status(void)//读取AHT20的状态寄存器
{
    uint8_t Byte_first;
    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, &Byte_first, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return Byte_first;
}

uint8_t AHT20_Read_Cal_Enable(void)  //查询cal enable位有没有使能
{
    uint8_t val = 0;
    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, &val, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if((val & 0x68)==0x08)
        return 1;
    else
        return 0;
}

void AHT20_SendAC(void) //向AHT20发送AC命令
{
    uint8_t cmd[] = {0xac, 0x33, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, cmd, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void AHT20_Read_CTdata(uint32_t *ct) //没有CRC校验，直接读取AHT20的温度和湿度数据
{
    uint8_t  Byte[6] = {0};

    uint32_t TempData = 0;
    uint16_t cnt = 0;
    AHT20_SendAC();//向AHT20发送AC命令

    SLEEP_MS(80);//延时80ms左右

    cnt = 0;
    while(((AHT20_Read_Status() & 0x80) == 0x80))//直到状态bit[7]为0，表示为空闲状态，若为1，表示忙状态
    {
        SLEEP_MS(1580);
        if(cnt++ >= 100)
        {
            break;
        }
    }

    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, Byte, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
//    Byte_1th //状态字，查询到状态为0x98,表示为忙状态，bit[7]为1；状态为0x1C，或者0x0C，或者0x08表示为空闲状态，bit[7]为0
//    Byte_2th //湿度
//    Byte_3th //湿度
//    Byte_4th //湿度/温度
//    Byte_5th //温度
//    Byte_6th //温度

    TempData = (TempData|Byte[1])<<8;
    TempData = (TempData|Byte[2])<<8;
    TempData = (TempData|Byte[3]);
    TempData = TempData >>4;
    ct[0] =  TempData;//湿度
    TempData = 0;
    TempData = (TempData|Byte[3])<<8;
    TempData = (TempData|Byte[4])<<8;
    TempData = (TempData|Byte[5]);
    TempData = TempData & 0x0fffff;
    ct[1] = TempData; //温度

}

void JH_Reset_REG(uint8_t addr)
{
    uint8_t Byte[3] = {0};
    uint8_t cmd[] = {addr, 0x00, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, cmd, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    SLEEP_MS(10);//延时10ms左右

    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, Byte, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    SLEEP_MS(10);//延时10ms左右

    cmd[0] = (0xB0|addr);
    cmd[1] = Byte[1];
    cmd[2] = Byte[2];
    i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR >> 1, cmd, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void AHT20_Start_Init(void)
{
    JH_Reset_REG(0x1b);
    JH_Reset_REG(0x1c);
    JH_Reset_REG(0x1e);
}
/********************************** AHT20 END **********************************/





