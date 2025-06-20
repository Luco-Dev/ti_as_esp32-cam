#include <Arduino.h>
#include "esp_camera.h"

// UNCOMMENT THIS TO SELECT THE AI THINKER MODULE
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#endif

// RTOS event group or flag
volatile bool captureRequested = false;
SemaphoreHandle_t captureSemaphore;

// FRAMESIZE_QQVGA	160 x 120	Very low, fast
// FRAMESIZE_QVGA	320 x 240	Low, fast
// FRAMESIZE_CIF	352 x 288	Low
// FRAMESIZE_VGA	640 x 480	Default, good balance
// FRAMESIZE_SVGA	800 x 600	Medium, higher memory use
// FRAMESIZE_XGA	1024 x 768	Higher, may struggle
// FRAMESIZE_SXGA	1280 x 1024	High, might cause OOM errors
// FRAMESIZE_UXGA	1600 x 1200	Usually too high for AI Thinker

// 1	Best image quality, largest file size, slowest processing
// 10	Very good quality (often recommended)
// 20	Good quality, smaller file size
// 30	Medium quality
// 40+	Lower quality, faster, smallest file size
// 63	Worst quality, fastest capture


void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 24000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size    = FRAMESIZE_QQVGA;
  config.jpeg_quality  = 63;
  config.fb_count      = 2;

  if (!psramFound()) {
    Serial.println("PSRAM not found");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  Serial.println("Camera initialized");
}

void SerialCommandTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'C') {
        xSemaphoreGive(captureSemaphore);  // Trigger capture
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void CameraTask(void *pvParameters) {
    while (true) {
        if (xSemaphoreTake(captureSemaphore, portMAX_DELAY) == pdTRUE) {
            camera_fb_t * fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed");
                continue;
            }

            Serial.write(0xA5);
            Serial.write(0x5A);
            uint16_t len = fb->len;
            Serial.write((uint8_t *)&len, 2);
            Serial.write(fb->buf, fb->len);

            esp_camera_fb_return(fb);
        }
    }
}

void setup() {
  Serial.begin(4000000);
  delay(2000); // Let serial port init

  setupCamera();

  captureSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(SerialCommandTask, "SerialCommandTask", 4096, NULL, 1, NULL);
xTaskCreate(CameraTask, "CameraTask", 8192, NULL, 1, NULL);

  Serial.println("Ready. Send 'C' over serial to capture.");
}

void loop() {
  // Nothing here; FreeRTOS handles the logic
}
