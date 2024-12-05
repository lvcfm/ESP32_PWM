//点焊机点焊程-2024/11/16

#include <Arduino.h>
#include <U8g2lib.h>
#include "nvs_flash.h"

// NVS分区的命名空间
#define NVS_NAMESPACE "reboot_counter"
uint32_t myVariable;

// 函数声明
static void initialize_nvs();
static void increment_reboot_counter(uint32_t reboot_count);
static uint32_t get_reboot_counter();

// 初始化 NVS 分区
static void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// 增加重启计数
static void increment_reboot_counter(uint32_t reboot_count) {
    nvs_handle_t my_handle;
    //uint32_t reboot_count = 0;

    // 打开 NVS 命名空间
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));

    // // 读取当前的重启计数
    // ESP_ERROR_CHECK(nvs_get_u32(my_handle, "rebootValue", &reboot_count));

    // // 增加计数
    
    // 写入新的重启计数
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "rebootValue", reboot_count));

    // 提交更改
    ESP_ERROR_CHECK(nvs_commit(my_handle));

    // 关闭 NVS 句柄
    nvs_close(my_handle);
}

// 获取重启计数
static uint32_t get_reboot_counter() {
    nvs_handle_t my_handle;
    uint32_t reboot_count = 0;
    esp_err_t ret;

    // 尝试以只读模式打开 NVS 命名空间
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // 命名空间不存在，以读写模式打开并创建它
        ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
        if (ret != ESP_OK) {
            ESP_LOGE("NVS", "Error opening NVS namespace for read/write: %s", esp_err_to_name(ret));
            return -1; // 或者设置一个默认的重启计数
        }
        // 初始化重启计数
        ret = nvs_set_u32(my_handle, "rebootValue", 0);
        if (ret != ESP_OK) {
            ESP_LOGE("NVS", "Error setting reboot counter: %s", esp_err_to_name(ret));
            nvs_close(my_handle);
            return -1; // 或者设置一个默认的重启计数
        }
        ret = nvs_commit(my_handle);
        if (ret != ESP_OK) {
            ESP_LOGE("NVS", "Error committing reboot counter: %s", esp_err_to_name(ret));
            nvs_close(my_handle);
            return -1; // 或者设置一个默认的重启计数
        }
    } else if (ret != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS namespace for read: %s", esp_err_to_name(ret));
        return -1; // 或者设置一个默认的重启计数
    }

    // 读取重启计数
    ret = nvs_get_u32(my_handle, "rebootValue", &reboot_count);
    if (ret != ESP_OK) {
        ESP_LOGE("NVS", "Error reading reboot counter: %s", esp_err_to_name(ret));
        reboot_count = 0; // 如果读取失败，则设置为0
    }

    // 关闭 NVS 句柄
    nvs_close(my_handle);

    return reboot_count;
}


// 定义按钮和LED的GPIO引脚
#define LED_BUILTIN 2 // 板载led指示灯
#define buttonAPin 32 // 按钮A
#define buttonBPin 33 // 按钮B
#define LED_Pin 12    // 控制灯泡

#define transparent_mode(x) u8g2.setFontMode(x) // 是否开启透明字体模式，0为默认值不开启透明字体，1开启
#define colour_mode(x) u8g2.setDrawColor(x);    // 设置画笔颜色，0为背景透明色，1为实色

#define frame_line1 u8g2.drawRBox(0, 1, 127, 15, 3)  // 以实色画第一行选择框
#define frame_line2 u8g2.drawRBox(0, 17, 127, 16, 3) // 以实色画第二行选择框
#define frame_line3 u8g2.drawRBox(0, 33, 127, 16, 3) // 以实色画第三行选择框
#define frame_line4 u8g2.drawRBox(0, 48, 127, 16, 3) // 以实色画第四行选择框

#define str_line1(x) u8g2.drawUTF8(14, 14, x) // 在第一行写字
#define str_line2(x) u8g2.drawUTF8(14, 30, x) // 在第二行写字
#define str_line3(x) u8g2.drawUTF8(14, 46, x) // 在第三行写字
#define str_line4(x) u8g2.drawUTF8(14, 61, x) // 在第四行写字

int optionIndex = 0; // 选项
int timeValues[3] = {0, 0, 0};

// 按钮状态变量
bool buttonAPressed = false;
unsigned long lastButtonAPressTime = 0;
unsigned long buttonALongPressDuration = 1000; // 长按持续时间1000ms

bool buttonBTriggered = false;
unsigned long lastButtonBPressTime = 0;
unsigned long buttonBDebounceDelay = 50; // 按钮B的消抖时间

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21);

void key_init()
{
  pinMode(LED_Pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonAPin, INPUT_PULLUP); // 使用内部上拉电阻
  pinMode(buttonBPin, INPUT_PULLUP); // 使用内部上拉电阻
}

void controlLED()
{
  if (timeValues[0] > 0)
  {
    // 双脉冲
    if (timeValues[1] > 0 && timeValues[2] > 0)
    {
      delay(3000);
      // 脉冲一
      digitalWrite(LED_Pin, HIGH);     // 点亮LED
      digitalWrite(LED_BUILTIN, HIGH); // 点亮指示灯LED
      delay(timeValues[0]);            // 根据timeValue控制点亮时间
      digitalWrite(LED_Pin, LOW);
      digitalWrite(LED_BUILTIN, LOW);

      // 间隔
      delay(timeValues[1]);

      // 脉冲二
      digitalWrite(LED_Pin, HIGH);     // 点亮LED
      digitalWrite(LED_BUILTIN, HIGH); // 点亮指示灯LED
      delay(timeValues[2]);            // 根据timeValue控制点亮时间
      digitalWrite(LED_Pin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      myVariable++;
    }
    else
    {           
      delay(3000);
      // 单脉冲
      digitalWrite(LED_Pin, HIGH);     // 点亮LED
      digitalWrite(LED_BUILTIN, HIGH); // 点亮指示灯LED
      delay(timeValues[0]);            // 根据timeValue控制点亮时间
      digitalWrite(LED_Pin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      myVariable++;
    }
  }
  delay(5);
}

void setup()
{
  Serial.begin(9600); // 打开串口9600
  u8g2.begin();       // u8g2初始化
  key_init();

  // 打印ESP32的芯片信息
  Serial.println("ESP32 Information:");
  Serial.print("Chip Revision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Flash Size: ");
  Serial.print(ESP.getFlashChipSize());
  Serial.println(" bytes");
  Serial.print("Free Memory: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("CPU Frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.print("SDK Version: ");
  Serial.println(ESP.getSdkVersion());
}

void loop()
{
  initialize_nvs();
  myVariable = get_reboot_counter();

  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.setFontPosBaseline();

  if (digitalRead(buttonAPin) == LOW)
  {
    if (!buttonAPressed)
    {
      buttonAPressed = true;
      lastButtonAPressTime = millis();
    }
  }
  else
  {
    if (buttonAPressed)
    {
      unsigned long duration = millis() - lastButtonAPressTime;
      if (duration > buttonALongPressDuration)
      {
        // 长按切换选项
        optionIndex = (optionIndex + 1) % 3;
      }
      else if (duration < 400)
      {                                                                          // 假设单击时间小于300ms
        timeValues[optionIndex] = max(0, min(200, timeValues[optionIndex] + 1)); // 增加时间值
        // displaySettings(); // 更新显示
      }
      else if (duration < 800)
      {                                                                          // 假设dan击时间小于800ms 双击
        timeValues[optionIndex] = max(0, min(200, timeValues[optionIndex] - 1)); // 减少时间值
        // displaySettings(); // 更新显示
      }
      buttonAPressed = false;
    }
  }
  // 按钮A结束

  // 按钮B
  if (digitalRead(buttonBPin) == LOW)
  {
    if (!buttonBTriggered)
    {
      buttonBTriggered = true;
      lastButtonBPressTime = millis();
    }
  }
  else
  {
    if (buttonBTriggered)
    {
      if ((millis() - lastButtonBPressTime) > buttonBDebounceDelay)
      {
        // 按钮B的消抖逻辑通过，执行LED控制操作
        controlLED();
        buttonBTriggered = false;
      }
    }
  }

  char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
  sprintf(buffer1, "脉冲一:%dms", timeValues[0]); // 格式化字符串
  sprintf(buffer2, "间  隔:%dms", timeValues[1]); // 格式化字符串
  sprintf(buffer3, "脉冲二:%dms", timeValues[2]); // 格式化字符串
  sprintf(buffer4, "总次数:%d", myVariable);      // 格式化字符串

  // 显示//显示
  switch (optionIndex)
  {
  case 0:
    /* code */
    u8g2.clearBuffer();
    transparent_mode(1);
    colour_mode(1);
    frame_line1;
    colour_mode(0);
    str_line1(buffer1);
    colour_mode(1);
    str_line2(buffer2);
    str_line3(buffer3);
    str_line4(buffer4);
    u8g2.sendBuffer();

    break;

  case 1:
    /* code */
    u8g2.clearBuffer();
    transparent_mode(1);
    colour_mode(1);
    frame_line2;
    colour_mode(0);
    str_line2(buffer2);
    colour_mode(1);
    str_line1(buffer1);
    str_line3(buffer3);
    str_line4(buffer4);
    u8g2.sendBuffer();

    break;

  case 2:
    /* code */
    u8g2.clearBuffer();
    transparent_mode(1);
    colour_mode(1);
    frame_line3;
    colour_mode(0);
    str_line3(buffer3);
    colour_mode(1);
    str_line1(buffer1);
    str_line2(buffer2);
    str_line4(buffer4);
    u8g2.sendBuffer();

    break;

  default:
    break;
  }
  increment_reboot_counter(myVariable);
}