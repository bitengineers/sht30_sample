idf_component_register(SRCS "sht30-sample.c"
  INCLUDE_DIRS "."
  REQUIRES spi_flash spiffs sht30 lcd axp192)

spiffs_create_partition_image(storage ../font FLASH_IN_PROJECT)

