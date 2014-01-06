# Sources:
C_SRC=hd44780_low.c
H_SRC=hd44780_low.h

# Libs dependency
MY_LIBS = 

# Target MCU
MCU ?= atmega16
F_CPU ?= 10000000UL

# Target library:
TARGET_LIB = libhd44780.a

# Additional includes:
INC=-I../avrutils $(addprefix -I../lib,$(MY_LIBS))

# Release configurations:
RELNAME=libhd44780-$(RELVER)
RELDIR=../../tags/$(RELNAME)
RELFILES=$(C_SRC) $(H_SRC) README COPYING

# Configuration:
HEXFORMAT=ihex
OPTLEVEL=s

# Tools:
CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
SIZE=avr-size
RM=rm -f
AR=avr-ar
CP=cp

# Shorts:
O_FILES=$(subst .c,.o,$(C_SRC))

# Assembler flags:
ASMFLAGS =-I. $(INC) -mmcu=$(MCU) \
	-Wa,-gstabs

# C flags:
CFLAGS=-I. $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
	-fpack-struct -fshort-enums \
	-funsigned-bitfields -funsigned-char \
	-Wall \
	-DF_CPU=$(F_CPU)

# Archiver flags:
ARFLAGS=-rcs

# Phony targets:
.PHONY: clean release

# Archive target:
$(TARGET_LIB): $(O_FILES) $(H_SRC)
	$(AR) $(ARFLAGS) $@ $(O_FILES)	

# Objects from C files:
%.o : %.c
	$(CC) $(CFLAGS) -o $@ -c $<

# Clean target:
clean:
	$(RM) $(TARGET_LIB) $(O_FILES)

# Tag current state and make a release:
release:
	make	# Be sure there are no errors...
	if [ ! -d $(RELDIR) ]; then mkdir $(RELDIR); fi
	$(CP) $(RELFILES) $(RELDIR)
	$(CP) Makefile.release $(RELDIR)/Makefile
	cd ../../tags && tar -cvzf $(RELNAME).tgz $(RELNAME)
	mv ../../tags/$(RELNAME).tgz ../../tags/tarballs
	
