#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sht30.h"

#define SHT30_I2C      I2C_NUM_1
#define SHT30_I2C_ADDR 0x44
#define SHT30_I2C_SDA  32
#define SHT30_I2C_SCK  33
#define SHT30_I2C_CLK  400 * 1000

#define SHT30_CRC_LEN 2
#define SHT30_CRC_POLYNOMIAL 0x31

static uint8_t sht30_check_crc(uint8_t *buf, uint8_t crc);

esp_err_t sht30_init(void)
{
  esp_err_t err;
  i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SHT30_I2C_SDA,
    .scl_io_num = SHT30_I2C_SCK,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = SHT30_I2C_CLK,
  };
  err = i2c_param_config(SHT30_I2C, &i2c_config);
  if (err != ESP_OK) {
    return err;
  }
  err = i2c_driver_install(SHT30_I2C, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK) {
    return err;
  }
  i2c_set_timeout(SHT30_I2C, pdMS_TO_TICKS(1000));
  return ESP_OK;
}

esp_err_t sht30_deinit(void)
{
  return i2c_driver_delete(SHT30_I2C);
}

esp_err_t sht30_read_status(uint16_t *out)
{
  esp_err_t err;
  uint8_t status[] = { 0x00, 0x00 };
  uint8_t code[] = { 0xF3, 0x2D };
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, code[0], true);
  i2c_master_write_byte(cmd, code[1], true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, status, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, status+1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(100));
  *out = (status[0] << 8) | status[1];
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t sht30_reset_status(void)
{
  esp_err_t err;
  uint8_t status[] = { 0x00, 0x00 };
  uint8_t code[] = { 0x30, 0x41 };
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, code[0], true);
  i2c_master_write_byte(cmd, code[1], true);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t sht30_softreset(void)
{
  esp_err_t err;;
  uint8_t code[] = { 0x30, 0xA2 };
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, code[0], true);
  i2c_master_write_byte(cmd, code[1], true);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t sht30_read_measured_values(uint16_t *temperature, uint16_t *humidity)
{
  esp_err_t err = ESP_OK;
  uint8_t code[] = { 0x20, 0x32 };
  uint8_t crc[2];
  uint8_t temp[2];
  uint8_t hum[2];
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, code[0], true);
  i2c_master_write_byte(cmd, code[1], true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, temp, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, temp+1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, crc, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, hum, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, hum+1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, crc+1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  ESP_LOGD("sht30", "temp %d(%x, %x), crc %d(%x), check result = %d", (uint8_t)((temp[0]<<8)+temp[1]), temp[0], temp[1], crc[0], crc[0], sht30_check_crc(temp, crc[0]));
  
  ESP_LOGD("sht30", "humdity %d(%x, %x), crc %d(%x), check result = %d", (uint8_t)((hum[0]<<8)+hum[1]), hum[0], hum[1], crc[1], crc[1], sht30_check_crc(hum, crc[1]));
  if (sht30_check_crc(temp, crc[0])) {
    *temperature = (temp[0] << 8) | temp[1];
  } else {
    *temperature = 0;
    err = ESP_ERR_INVALID_RESPONSE;
  }

  if (sht30_check_crc(hum, crc[1])) {
    *humidity = (hum[0] << 8) | hum[1];
  } else {
    *humidity = 0;
    err = ESP_ERR_INVALID_RESPONSE;
  }
  return err;
}

esp_err_t sht30_stop_measurement(void)
{
  esp_err_t err = ESP_OK;
  uint8_t code[] = { 0x30, 0x93 };
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, code[0], true);
  i2c_master_write_byte(cmd, code[1], true);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(300));
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t sht30_heater(bool b)
{
  esp_err_t err = ESP_OK;
  uint8_t c[2] = { 0x30, 0x6d };
  if (!b) {
    c[1] = 0x66;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT30_I2C_ADDR<<1)|I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, c[0], true);
  i2c_master_write_byte(cmd, c[1], true);  
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(SHT30_I2C, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return err;
}

static uint8_t sht30_check_crc(uint8_t *buf, uint8_t crc)
{
  uint8_t v = 0xff;
  for (int i = 0; i < SHT30_CRC_LEN; i++) {
    v ^= *buf++;
    for (int j = 0; j < 8; j++) {
      if (v & 0x80) {
        v <<= 1;
        v ^= SHT30_CRC_POLYNOMIAL;
      } else {
        v <<= 1;
      }
    }
  }
  return v == crc;
}

float sht30_calc_celsius(uint16_t temp)
{
  return -45.0F + 175.0F * (temp/(65536.0F-1));
}

float sht30_calc_relative_humidity(uint16_t hum)
{
  return 100.0F * hum / (65536.0F - 1);
}
