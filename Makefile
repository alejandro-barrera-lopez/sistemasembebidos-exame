# Variables xerais
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)as
GDB = gdb-multiarch
OPENOCD = openocd

# Directorios do proxecto
BOARD_DIR = BOARD
DRIVERS_DIR = drivers
UTILITIES_DIR = utilities
INCLUDE_DIR = include

# Fontes
SRCS_C = main.c \
    $(wildcard $(BOARD_DIR)/*.c) \
    $(wildcard $(DRIVERS_DIR)/*.c) \
    $(wildcard $(UTILITIES_DIR)/*.c) \
    $(INCLUDE_DIR)/system_MKL46Z4.c

SRCS_ASM = startup_MKL46Z4.S

# Obxectos
OBJS_C = $(SRCS_C:.c=.o)
OBJS_ASM = $(SRCS_ASM:.S=.o)
OBJS = $(OBJS_C) $(OBJS_ASM)

# Directorios de inclusión
INCLUDES = -I. \
    -I$(BOARD_DIR) \
    -I$(DRIVERS_DIR) \
    -I$(UTILITIES_DIR) \
    -I$(INCLUDE_DIR) \
    -I$(DRIVERS_DIR) \
    -DCPU_MKL46Z256VLL4 \
    -DFRDM_KL46Z \
    -DFREEDOM \
    -DSDK_DEBUGCONSOLE=1

# Nome do executable
TARGET = main

# Flags de linkado e compilado
ARCHFLAGS = -mthumb -mcpu=cortex-m0plus
CFLAGS_BASE = $(ARCHFLAGS) -Wall $(INCLUDES)
CFLAGS_DEBUG = $(CFLAGS_BASE) -O0 -g3 -DDEBUG
CFLAGS_RELEASE = $(CFLAGS_BASE) -O2 -DNDEBUG
ASFLAGS = $(ARCHFLAGS)
LDFLAGS = $(ARCHFLAGS) --specs=nosys.specs -Wl,--gc-sections,-Map=$(TARGET).map,-T,MKL46Z256xxx4_flash.ld

# Se non está definido, definir CFLAGS por defecto
CFLAGS ?= $(CFLAGS_RELEASE)

# Regras principais
all: release

debug: CFLAGS = $(CFLAGS_DEBUG)
debug: $(TARGET).elf

release: CFLAGS = $(CFLAGS_RELEASE)
release: $(TARGET).elf

# Regla para construír o ELF
$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

# Regras para construír os obxectos
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(ASFLAGS) -c $< -o $@

# Regra para depurar con GDB
gdb: $(TARGET).elf
	$(GDB) -ex "target extended-remote localhost:3333" $(TARGET).elf

# Regra para flashear
flash: $(TARGET).elf
	$(OPENOCD) -f openocd.cfg -c "program $(TARGET).elf verify reset exit"

# Regras de limpeza
clean:
	rm -f $(OBJS)

cleanall: clean
	rm -f $(TARGET).elf $(TARGET).map

# Mostrar variables
vars:
	@echo "SRCS_C = $(SRCS_C)"
	@echo "OBJS = $(OBJS)"
	@echo "INCLUDES = $(INCLUDES)"

.PHONY: all debug release clean mrproper gdb flash vars
