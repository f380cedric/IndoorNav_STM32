OBJDIR=obj
SRCDIR=STM32F0xx_HAL_Driver/Src
###########################################

CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

INCDIR = CMSIS/Include
INCDIR += CMSIS/Device/ST/STM32F0xx/Include
INCDIR += STM32F0xx_HAL_Driver/Inc

INCLUDE = $(addprefix -I,$(INCDIR))

###########################################

CFLAGS  = -std=gnu99 -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m0 -mthumb-interwork
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -DSTM32F072xB
CFLAGS += --specs=nosys.specs
#CFLAGS += -ffreestanding -nostdlib
#CFLAGS += -include choose_your_mcu_here.h
#CFLAGS += -include stm32f0xx_hal_conf.h
#CFLAGS += -I .
#CFLAGS += -I CMSIS/Include
#CFLAGS += -I CMSIS/Device/ST/STM32F0xx/Include
#CFLAGS += -I STM32F0xx_HAL_Driver/Inc


SRCFILES = stm32f0xx_hal_adc.c           stm32f0xx_hal_pcd.c \
	stm32f0xx_hal_adc_ex.c        stm32f0xx_hal_pcd_ex.c \
	stm32f0xx_hal.c               stm32f0xx_hal_pwr.c \
	stm32f0xx_hal_can.c           stm32f0xx_hal_pwr_ex.c \
	stm32f0xx_hal_cec.c           stm32f0xx_hal_rcc.c \
	stm32f0xx_hal_comp.c          stm32f0xx_hal_rcc_ex.c \
	stm32f0xx_hal_cortex.c        stm32f0xx_hal_rtc.c \
	stm32f0xx_hal_crc.c           stm32f0xx_hal_rtc_ex.c \
	stm32f0xx_hal_crc_ex.c        stm32f0xx_hal_smartcard.c \
	stm32f0xx_hal_dac.c           stm32f0xx_hal_smartcard_ex.c \
	stm32f0xx_hal_dac_ex.c        stm32f0xx_hal_smbus.c \
	stm32f0xx_hal_dma.c           stm32f0xx_hal_spi.c \
	stm32f0xx_hal_flash.c         stm32f0xx_hal_spi_ex.c \
	stm32f0xx_hal_flash_ex.c      stm32f0xx_hal_tim.c \
	stm32f0xx_hal_gpio.c          stm32f0xx_hal_tim_ex.c \
	stm32f0xx_hal_i2c.c           stm32f0xx_hal_tsc.c \
	stm32f0xx_hal_i2c_ex.c        stm32f0xx_hal_uart.c \
	stm32f0xx_hal_i2s.c           stm32f0xx_hal_uart_ex.c \
	stm32f0xx_hal_irda.c          stm32f0xx_hal_usart.c \
	stm32f0xx_hal_iwdg.c          stm32f0xx_hal_wwdg.c \

SRCS = $(addprefix $(SRCDIR)/, $(SRCFILES))
OBJS = $(addprefix $(OBJDIR)/, $(notdir $(SRCS:.c=.o)))

.PHONY: all clean
all: libstm32f0.a

$(OBJDIR)/%.o : $(SRCDIR)/%.c
	$(CC) -MMD $(INCLUDE) $(CFLAGS) -c -o $@ $<

libstm32f0.a: $(OBJS)
	$(AR) rcs $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f0.a

-include $(wildcard $(OBJDIR)/*.d)
