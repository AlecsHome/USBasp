     1	#
     2	#   Makefile for usbasp
     3	#   20061119   Thomas Fischl        original
     4	#   20061120   Hanns-Konrad Unger   help: and TARGET=atmega48 added
     5	#
     6	
     7	# crystal frequency
     8	#F_CPU=12000000UL
     9	#F_CPU=16000000UL
    10	
    11	# TARGET=atmega8    HFUSE=0xc9  LFUSE=0xef
    12	# TARGET=atmega48   HFUSE=0xdd  LFUSE=0xff
    13	# TARGET=at90s2313
    14	TARGET=atmega8
    15	CFLAGS += -DF_CPU=12000000UL
    16	HFUSE=0xc9
    17	LFUSE=0xef
    18	
    19	# ─юсрт№Єх т эрўрыю Makefile ЁрчьхЁ√ ярь Єш фы  Ёрчэ√ї ьшъЁюъюэЄЁюыыхЁют
    20	ifeq ($(TARGET), atmega8)
    21	  FLASH_SIZE = 8192
    22	  RAM_SIZE = 1024
    23	  EEPROM_SIZE = 512
    24	else ifeq ($(TARGET), atmega328p)
    25	  FLASH_SIZE = 32768
    26	  RAM_SIZE = 2048
    27	  EEPROM_SIZE = 1024
    28	else
    29	  FLASH_SIZE = 8192
    30	  RAM_SIZE = 1024
    31	  EEPROM_SIZE = 512
    32	endif
    33	
    34	
    35	
    36	# ISP=bsd      PORT=/dev/parport0
    37	# ISP=ponyser  PORT=/dev/ttyS1
    38	# ISP=stk500   PORT=/dev/ttyS1
    39	# ISP=usbasp   PORT=/dev/usb/ttyUSB0
    40	# ISP=stk500v2 PORT=/dev/ttyUSB0
    41	#PORT=/dev/usb/ttyUSB0
    42	PORT=usb
    43	
    44	help:
    45		@echo "Usage: make                same as make help"
    46		@echo "       make help           same as make"
    47		@echo "       make main.hex       create main.hex"
    48		@echo "       make clean          remove redundant data"
    49		@echo "       make disasm         disasm main"
    50		@echo "       make flash          upload main.hex into flash"
    51		@echo "       make fuses          program fuses"
    52		@echo "       make avrdude        test avrdude"
    53		@echo "Current values:"
    54		@echo "       TARGET=${TARGET}"
    55		@echo "       LFUSE=${LFUSE}"
    56		@echo "       HFUSE=${HFUSE}"
    57		@echo "       CLOCK=12000000"
    58		@echo "       ISP=${ISP}"
    59		@echo "       PORT=${PORT}"
    60	
    61	SRC += tpi.S
    62	#CFLAGS += -fexec-charset=CP866 -finput-charset=UTF-8
    63	
    64	#COMPILE = avr-gcc -Wall -Wextra -Os -flto -std=gnu99 -Iusbdrv -I. -mmcu=$(TARGET) # -DDEBUG_LEVEL=2
    65	# ╬яЎшш ъюьяшы ЄюЁр
    66	CFLAGS  = -Wall -Wextra -Os -std=gnu99 -Iusbdrv -I. -mmcu=$(TARGET)
    67	CFLAGS += -flto -ffunction-sections -fdata-sections
    68	#CFLAGS += -fno-jump-tables  # disabled: switch dispatch is dense; jump tables are smaller and O(1)
    69	CFLAGS += -fmerge-all-constants
    70	# ╙сшЁрхЄ фєсышЁє■∙шхё  ЇєэъЎшш шч шэёЄрэЎшЁютрэшщ °рсыюэют
    71	CFLAGS += -fipa-icf -fipa-icf-functions -fipa-icf-variables
    72	# ═х Ёрчфхы хЄ wide-Єшя√ (uint32_t юёЄр╕Єё  т ЁхушёЄЁрї ярЁрї, ьхэ№°х mov)
    73	CFLAGS += -fno-split-wide-types
    74	
    75	# ╬яЎшш ышэъют∙шър
    76	LDFLAGS = -Wl,--gc-sections -Wl,--relax -flto
    77	
    78	COMPILE = avr-gcc $(CFLAGS)
    79	
    80	OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o isp.o clock.o tpi.o main.o i2c.o microwire.o 
    81	
    82	.c.o:
    83		$(COMPILE) -c $< -o $@
    84	#-Wa,-ahlms=$<.lst
    85	
    86	.S.o:
    87		$(COMPILE) -x assembler-with-cpp -c $< -o $@
    88	# "-x assembler-with-cpp" should not be necessary since this is the default
    89	# file type for the .S (with capital S) extension. However, upper case
    90	# characters are not always preserved on Windows. To ensure WinAVR
    91	# compatibility define the file type manually.
    92	
    93	.c.s:
    94		$(COMPILE) -S $< -o $@
    95	
    96	clean:
    97		rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.bin *.o main.s usbdrv/*.o
    98	
    99	# file targets:
   100	main.bin:	$(OBJECTS)
   101		$(COMPILE) $(OBJECTS) $(LDFLAGS) -Wl,-Map,main.map -o main.bin
   102	
   103	main.hex:	main.bin
   104		rm -f main.hex main.eep.hex
   105		avr-objcopy -j .text -j .data -O ihex main.bin main.hex
   106		@echo ""
   107		@echo "--- Memory Usage for $(TARGET) ---"	
   108		@echo "File sizes:"
   109		@echo "  ELF: `wc -c < main.bin` bytes (with debug info)"
   110		@echo "  HEX: `wc -c < main.hex` bytes (program file)"
   111		@echo ""
   112		@echo "Microcontroller memory usage:"
   113		@avr-size -A main.bin > _size.tmp
   114		@awk -v flash_size=$(FLASH_SIZE) -v ram_size=$(RAM_SIZE) -v eeprom_size=$(EEPROM_SIZE) '\
   115		    /^\.[ \t]*text/   {flash += $$2} \
   116		    /^\.[ \t]*data/   {flash += $$2; ram += $$2} \
   117		    /^\.[ \t]*bss/    {ram += $$2} \
   118		    /^\.[ \t]*eeprom/ {eeprom += $$2} \
   119		    function pct(used, total) { return (total > 0) ? used * 100 / total : 0 } \
   120		    function color(p) { return (p < 70) ? "\033[0;32m" : (p < 85) ? "\033[1;33m" : "\033[0;31m" } \
   121		    END { \
   122		        fp = pct(flash, flash_size); rp = pct(ram, ram_size); ep = pct(eeprom, eeprom_size); \
   123		        printf "  Flash:  %s%5d / %4d bytes (%5.1f%%)\033[0m\n", color(fp), flash, flash_size, fp; \
   124		        printf "  RAM:    %s%5d / %4d bytes (%5.1f%%)\033[0m\n", color(rp), ram, ram_size, rp; \
   125		        printf "  EEPROM: %s%5d / %4d bytes (%5.1f%%)\033[0m\n", color(ep), eeprom, eeprom_size, ep; \
   126		    }' _size.tmp
   127		@echo ""
   128		@echo "Section details:"
   129		@awk '/\.(text|data|bss|eeprom)/ {printf("  %-10s %6d bytes\n", $$1, $$2)}' _size.tmp
   130		@rm -f _size.tmp
   131		@echo "----------------------------------"
   132	
   133	#	./checksize main.bin
   134	# do the checksize script as our last action to allow successful compilation
   135	# on Windows with WinAVR where the Unix commands will fail.
   136	
   137	disasm:	main.bin
   138		avr-objdump -d main.bin
   139	
   140	cpp:
   141		$(COMPILE) -E main.c
   142	
   143	flash:   main.hex
   144		avrdude -c ${ISP} -p ${TARGET} -P ${PORT} -U flash:w:main.hex
   145	
   146	fuses:
   147		avrdude -c ${ISP} -p ${TARGET} -P ${PORT} -u -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m
   148	
   149	avrdude:
   150		avrdude -c ${ISP} -p ${TARGET} -P ${PORT} -v
   151	
   152	# Fuse atmega8 high byte HFUSE:
   153	# 0xc9 = 1 1 0 0   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
   154	#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
   155	#        | | | |   | +-------- BOOTSZ1
   156	#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
   157	#        | | | +-------------- CKOPT (full output swing)
   158	#        | | +---------------- SPIEN (allow serial programming)
   159	#        | +------------------ WDTON (WDT not always on)
   160	#        +-------------------- RSTDISBL (reset pin is enabled)
   161	# Fuse atmega8 low byte LFUSE:
   162	# 0x9f = 1 0 0 1   1 1 1 1
   163	#        ^ ^ \ /   \--+--/
   164	#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
   165	#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
   166	#        | +------------------ BODEN (BrownOut Detector enabled)
   167	#        +-------------------- BODLEVEL (2.7V)
   168	#
   169	# Fuse atmega48 high byte hfuse:
   170	# 0xdf = 1 1 0 1   1 1 1 1     factory setting
   171	#        ^ ^ ^ ^   ^ \-+-/
   172	#        | | | |   |   +------ BODLEVEL (Brown out disabled)
   173	#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
   174	#        | | | +-------------- WDTON (WDT not always on)
   175	#        | | +---------------- SPIEN (allow serial programming)
   176	#        | +------------------ DWEN (debug wire is disabled)
   177	#        +-------------------- RSTDISBL (reset pin is enabled)
   178	# 0xdd = ext.reset, no DW, SPI, no watchdog, no save eeprom, BOD 2.7V
   179	# Fuse atmega48 low byte lfuse:
   180	# 0x62 = 0 1 1 0   0 0 1 0     factory setting
   181	#        ^ ^ \ /   \--+--/
   182	#        | |  |       +------- CKSEL 3..0 (internal 8Mhz Oszillator)
   183	#        | |  +--------------- SUT 1..0 (start-up time)
   184	#        | +------------------ CKOUT (no clock output)
   185	#        +-------------------- CKDIV8 (divide clock by 8)
   186	# 0xdc = divide/1,no clock output,fast raising power,low Pw Oszil. 3..8 Mhz
   187	# 0xe0 = divide/1,no clock output,fast raising power,external Oszil.
   188	# 0xff = divide/1,no clock output,slow raising power,low Pw Oszil 8..  Mhz
   189	
   190	
   191	SERIAL = `echo /dev/tty.USA19QI*`
   192	UISP = uisp -dprog=$S -dserial=$(SERIAL) -dpart=auto
   193	# The two lines above are for "uisp" and the AVR910 serial programmer connected
   194	# to a Keyspan USB to serial converter to a Mac running Mac OS X.
   195	# Choose your favorite programmer and interface.
   196	
   197	uisp:	all
   198		$(UISP) --erase
   199		$(UISP) --upload --verify if=main.hex
   200	
   201	
