NAME=piecyk

MCU=atmega8
F_CPU=4000000UL
FUSE_HIGH=0xd1
FUSE_LOW=0x03
PROGRAMMER_ID=usbasp

CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
AVRDUDE=avrdude
AVRSIZE=avr-size

LIBS=hd44780fw customprocs hd44780
LIBS_PATH=$(foreach n, $(LIBS), ./lib/lib$(n)/lib$(n).a)
LIBS_LINK=$(foreach n, $(LIBS), -L./lib/lib$(n) -l$(n))
INC=-I $(addprefix -I./lib/lib,$(LIBS))

CFLAGS=-c -I $(INC) -mmcu=$(MCU) -Os -Wall -DF_CPU=$(F_CPU) -std=c99
LDFLAGS=-mmcu=$(MCU) $(LIBS_LINK)

all: $(LIBS_PATH) compile elf hex eep fuse size

debug: CFLAGS += -DDEBUG -g
debug: all

compile:
	$(CC) $(CFLAGS) $(NAME).c -o $(NAME).o

elf: compile
	$(CC) $(LDFLAGS) $(NAME).o $(LIBS_LINK) -o $(NAME).elf

hex: elf
	$(OBJCOPY) -j .text -j .data -O ihex $(NAME).elf $(NAME).hex

eep: elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $(NAME).elf $(NAME).eep.hex

fuse: elf
	$(OBJCOPY) -j .fuse -O ihex $(NAME).elf $(NAME).fuse.hex

size: elf
	$(AVRSIZE) $(NAME).elf

write:
	$(AVRDUDE) -c $(PROGRAMMER_ID) -p $(MCU) -e \
		-U flash:w:$(NAME).hex \
		-U eeprom:w:$(NAME).eep.hex \
		-U lfuse:w:$(FUSE_LOW):m -U hfuse:w:$(FUSE_HIGH):m

.PHONY: $(LIBS_PATH)
$(LIBS_PATH):
	make -C $(dir $(@))

clean:
	rm -f $(NAME).o $(NAME).elf $(NAME).hex $(NAME).eep $(NAME).fuse
	rm -f $(foreach n, $(LIBS), ./lib/lib$(n)/*.[ao])
