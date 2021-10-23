#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "st7789.h"
#include "fontx.h"
#include "axp192.h"

#include "sht30.h"

#define TAG "sht30_sample"

TFT_t dev;
FontxFile fx16G[2];
FontxFile fx24G[2];
FontxFile fx32G[2];
FontxFile fx16M[2];
FontxFile fx24M[2];
FontxFile fx32M[2];

// M5stickC-Plus stuff
#define CONFIG_WIDTH	  135
#define CONFIG_HEIGHT	  240
#define CONFIG_MOSI_GPIO  15
#define CONFIG_SCLK_GPIO  13
#define CONFIG_CS_GPIO	  5 
#define CONFIG_DC_GPIO	  23
#define CONFIG_RESET_GPIO 18
#define CONFIG_BL_GPIO	  32
#define CONFIG_OFFSETX	  52
#define CONFIG_OFFSETY	  40


void app_setup_spiffs(void)
{
    
	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 8,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total,&used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}
}

void app_setup_fonts(void)
{
  // set font file
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/ILGH32XB.FNT",""); // 16x32Dot Gothic

	InitFontx(fx16M,"/spiffs/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/ILMH32XB.FNT",""); // 16x32Dot Mincyo
}

void app_setup_lcd(void)
{
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
}

void app_main(void)
{
  esp_err_t err;
  uint16_t status;
  uint16_t temperature;
  uint16_t humidity;
  char *temp_title = "Temp: ";
  char *temp_unit = "c\"";
  char *hum_title = "Hum: ";
  char *hum_unit = "%RH";
  uint8_t text[48];

  axp192_init();
  axp192_ldo2(true);
  axp192_ldo3(true);
  app_setup_spiffs();
  app_setup_fonts();
  app_setup_lcd();
  lcdFillScreen(&dev, BLACK);
  lcdSetFontDirection(&dev, DIRECTION90);

  // sht30_deinit();
  ESP_ERROR_CHECK(sht30_init());

  while (true) {
    ESP_LOGI(TAG, "SHT3x status = %x", status);
    err = sht30_read_measured_values(&temperature, &humidity);
    ESP_LOGI(TAG, "SHT3x temperature = %02f, humidity = %02f", sht30_calc_celsius(temperature), sht30_calc_relative_humidity(humidity));
    // ESP_ERROR_CHECK(sht30_softreset());
    if (err != ESP_OK) {
      // sht30_stop_measurement();
      // sht30_softreset();
    } else {
      lcdFillScreen(&dev, BLACK);
      sprintf((char*)text, "%s %0.2f %s",
              temp_title, sht30_calc_celsius(temperature), temp_unit);
      lcdDrawString(&dev, fx24G, 80, 10, text, WHITE);
      sprintf((char*)text, "%s %0.2f %s",
              hum_title, sht30_calc_relative_humidity(humidity), hum_unit);
      lcdDrawString(&dev, fx24G, 40, 10, text, WHITE);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    sht30_read_status(&status);
    if (status == 0) {
      sht30_reset_status();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  // ESP_ERROR_CHECK(sht30_deinit());
}
