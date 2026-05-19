# I2C is not used — MCP23S17 is driven via hardware SPI
I2C_DRIVER_REQUIRED = no
SPI_DRIVER_REQUIRED = yes
CUSTOM_MATRIX = yes
SRC += matrix.c
