idf_component_register(
  SRCS st7789.c fontx.c
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash spiffs
)
