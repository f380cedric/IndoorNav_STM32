NAME=main
STMDIR=lib
SRCDIR=src
OBJDIR=obj
INCDIR=inc
LDDIR=$(INCDIR)

###################################################

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
CP=arm-none-eabi-objcopy
SF=st-flash

###################################################

INCDIR += $(STMDIR)/STM32F0xx_HAL_Driver/Inc
INCDIR += $(STMDIR)/CMSIS/Include
INCDIR += $(STMDIR)/CMSIS/Device/ST/STM32F0xx/Include
INCLUDE = $(addprefix -I,$(INCDIR))

###################################################

CFLAGS  = -std=gnu99 -g -O2 -Wall -Wextra -Wduplicated-cond -Wduplicated-branches -Wlogical-op -TSTM32F072RB_FLASH.ld
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m0
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -DSTM32F072xB
CFLAGS += --specs=nosys.specs

###################################################

SRCS = $(wildcard $(SRCDIR)/*.c)
SRCS += $(wildcard $(SRCDIR)/*.s)

###################################################

OBJS = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(notdir $(SRCS)))))
#SRCS =  main.c \
	dwm1000.c \
	stm32f0xx_it.c \
	system_stm32f0xx.c \
	stm32f0xx_hal_msp.c \
	startup_stm32f072xb.s

#SRCS += stm32f0xx_hal.c \
	stm32f0xx_hal_rcc.c \
	stm32f0xx_hal_rcc_ex.c \
	stm32f0xx_hal_cortex.c \
	stm32f0xx_hal_gpio.c \
	stm32f0xx_hal_spi.c \
	stm32f0xx_hal_uart.c \
	stm32f0xx_hal_dma.c \
	stm32f0xx_hal_uart_ex.c \


###################################################

LDDIR += $(STMDIR)
LDFLAGS = $(addprefix -L, $(LDDIR))
LDFLAGS += -TSTM32F072RB_FLASH.ld
LDFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m0
LDFLAGS += -fsingle-precision-constant -Wdouble-promotion
LDFLAGS += -DSTM32F072xB
LDFLAGS += --specs=nosys.specs


###################################################

.PHONY: all lib  proj clean flash

all: lib proj

proj: $(OBJDIR)/$(NAME).bin

lib:
	$(MAKE) -C $(STMDIR)

%.bin: %.elf
	$(CP) -O binary $< $@

$(OBJDIR)/$(NAME).elf: $(STMDIR)/libstm32f0.a $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) -lm -lstm32f0

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -MMD $(INCLUDE) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.s
	$(CC) -MMD $(INCLUDE) $(CFLAGS) -c $< -o $@


clean:
	rm -f $(OBJDIR)/*

flash: $(OBJDIR)/$(NAME).bin
	$(SF) write $< 0x8000000

-include $(wildcard $(OBJDIR)/*.d)
