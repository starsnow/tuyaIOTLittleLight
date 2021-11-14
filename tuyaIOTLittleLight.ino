#include <TuyaWifi.h>

TuyaWifi my_device;

unsigned char led_state = 0;
/* Connect network button pin */

const uint8_t WIFI_RECONNECT_BUTTON_PIN = 3;
const uint8_t TRIGGER_PIN = 12;

/******************************************************************************
                        1:dp数据点序列号重新定义
          **此为自动生成代码,如在开发平台有相关修改请重新下载MCU_SDK**         
******************************************************************************/
//开关(可下发可上报)
//备注:
#define DPID_SWITCH_LED 20
//调节(只下发)
//备注:
#define DPID_CONTROL_DATA 28
//轻触开关(可下发可上报)
//备注:
#define DPID_TAP_ENABLE 101

///* Current device DP values */
unsigned char dp_bool_value = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
unsigned char dp_string_value[21] = {"0"};

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type.
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] = {
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_CONTROL_DATA, DP_TYPE_STRING},
  {DPID_TAP_ENABLE, DP_TYPE_BOOL},
};

unsigned char pid[] = {"************"}; //*********处替换成涂鸦IoT平台自己创建的产品的PID
unsigned char mcu_ver[] = {"1.0.0"};

void setup()
{
    Serial.begin(9600);

    pinMode(WIFI_RECONNECT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
        
    //Initialize led port, turn off led.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    //Initialize networking keys.
    //incoming all DPs and their types array, DP numbers
    //Enter the PID and MCU software version
    my_device.init(pid, mcu_ver);
    // my_device.set_dp_cmd_total(dp_array, 29);
    my_device.set_dp_cmd_total(dp_array, 12);
    //register DP download processing callback function
    my_device.dp_process_func_register(dp_process);
    //register upload all DP callback function
    my_device.dp_update_all_func_register(dp_update_all);
}

void loop()
{
    static unsigned long last_time = 0;
    //Enter the connection network mode when Pin7 is pressed.
    if (digitalRead(WIFI_RECONNECT_BUTTON_PIN) == LOW)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(80);
        digitalWrite(LED_BUILTIN, LOW);

        if (digitalRead(WIFI_RECONNECT_BUTTON_PIN) == LOW)
        {
            my_device.mcu_set_wifi_mode(SMART_CONFIG);
        }
    }
    my_device.uart_service();

    /* LED blinks when network is being connected */
    if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW))
    {
        if (millis() - last_time >= 500)
        {
            last_time = millis();

            if (led_state == LOW)
            {
                led_state = HIGH;
            }
            else
            {
                led_state = LOW;
            }
            digitalWrite(LED_BUILTIN, led_state);
        }
    }
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
    switch (dpid)
    {
    case DPID_TAP_ENABLE:
        // 模块机械震动传感器
        digitalWrite(TRIGGER_PIN, LOW);
        delay(500);
        digitalWrite(TRIGGER_PIN, HIGH);
        break;

    default:
        break;
    }
    return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
}
