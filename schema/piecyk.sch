EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:contrib
LIBS:valves
LIBS:atmel
LIBS:piecyk-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "piecyk"
Date "7 nov 2014"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 78L05 U1
U 1 1 52BB01E4
P 3700 1200
F 0 "U1" H 3850 1004 60  0000 C CNN
F 1 "78L05" H 3700 1400 60  0000 C CNN
F 2 "Discret:LM78LXX" H 3700 1200 60  0000 C CNN
F 3 "~" H 3700 1200 60  0000 C CNN
	1    3700 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR11
U 1 1 52BB03AA
P 3700 2000
F 0 "#PWR11" H 3700 2000 30  0001 C CNN
F 1 "GND" H 3700 1930 30  0001 C CNN
F 2 "" H 3700 2000 60  0000 C CNN
F 3 "" H 3700 2000 60  0000 C CNN
	1    3700 2000
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 52BB0450
P 2150 3200
F 0 "R1" V 2230 3200 40  0000 C CNN
F 1 "22k" V 2157 3201 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2080 3200 30  0000 C CNN
F 3 "~" H 2150 3200 30  0000 C CNN
	1    2150 3200
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 52BB045F
P 1050 4200
F 0 "C4" H 1050 4300 40  0000 L CNN
F 1 "100n" H 1056 4115 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1088 4050 30  0000 C CNN
F 3 "~" H 1050 4200 60  0000 C CNN
	1    1050 4200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 52BB046E
P 1350 4200
F 0 "C5" H 1350 4300 40  0000 L CNN
F 1 "100n" H 1356 4115 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1388 4050 30  0000 C CNN
F 3 "~" H 1350 4200 60  0000 C CNN
	1    1350 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 52BB0515
P 1350 4650
F 0 "#PWR4" H 1350 4650 30  0001 C CNN
F 1 "GND" H 1350 4580 30  0001 C CNN
F 2 "" H 1350 4650 60  0000 C CNN
F 3 "" H 1350 4650 60  0000 C CNN
	1    1350 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 52BB0524
P 1050 4650
F 0 "#PWR1" H 1050 4650 30  0001 C CNN
F 1 "GND" H 1050 4580 30  0001 C CNN
F 2 "" H 1050 4650 60  0000 C CNN
F 3 "" H 1050 4650 60  0000 C CNN
	1    1050 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 52BB0533
P 1550 4650
F 0 "#PWR5" H 1550 4650 30  0001 C CNN
F 1 "GND" H 1550 4580 30  0001 C CNN
F 2 "" H 1550 4650 60  0000 C CNN
F 3 "" H 1550 4650 60  0000 C CNN
	1    1550 4650
	1    0    0    -1  
$EndComp
$Comp
L FILTER L1
U 1 1 52BB073B
P 1300 3350
F 0 "L1" H 1300 3500 60  0000 C CNN
F 1 "10uH" H 1300 3250 60  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1300 3350 60  0000 C CNN
F 3 "~" H 1300 3350 60  0000 C CNN
	1    1300 3350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR10
U 1 1 52BB0800
P 3150 6450
F 0 "#PWR10" H 3150 6450 30  0001 C CNN
F 1 "GND" H 3150 6380 30  0001 C CNN
F 2 "" H 3150 6450 60  0000 C CNN
F 3 "" H 3150 6450 60  0000 C CNN
	1    3150 6450
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 52BB1715
P 5900 6850
F 0 "SW1" H 6050 6960 50  0000 C CNN
F 1 "SW_edit" H 5900 6770 50  0000 C CNN
F 2 "Discret:SW_PUSH-12mm" H 5900 6850 60  0000 C CNN
F 3 "~" H 5900 6850 60  0000 C CNN
	1    5900 6850
	1    0    0    1   
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 52BB1724
P 5900 7150
F 0 "SW2" H 6050 7260 50  0000 C CNN
F 1 "SW_up" H 5900 7070 50  0000 C CNN
F 2 "Discret:SW_PUSH-12mm" H 5900 7150 60  0000 C CNN
F 3 "~" H 5900 7150 60  0000 C CNN
	1    5900 7150
	1    0    0    1   
$EndComp
$Comp
L SW_PUSH SW3
U 1 1 52BB1733
P 5900 7450
F 0 "SW3" H 6050 7560 50  0000 C CNN
F 1 "SW_down" H 5900 7370 50  0000 C CNN
F 2 "Discret:SW_PUSH-12mm" H 5900 7450 60  0000 C CNN
F 3 "~" H 5900 7450 60  0000 C CNN
	1    5900 7450
	1    0    0    1   
$EndComp
$Comp
L +12V #PWR9
U 1 1 52D0DF48
P 2950 950
F 0 "#PWR9" H 2950 900 20  0001 C CNN
F 1 "+12V" H 2950 1050 30  0000 C CNN
F 2 "" H 2950 950 60  0000 C CNN
F 3 "" H 2950 950 60  0000 C CNN
	1    2950 950 
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 52D0E01F
P 7150 4650
F 0 "D1" H 7150 4750 50  0000 C CNN
F 1 "LED" H 7150 4550 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 7150 4650 60  0000 C CNN
F 3 "~" H 7150 4650 60  0000 C CNN
	1    7150 4650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR15
U 1 1 52D0FAD8
P 6200 3150
F 0 "#PWR15" H 6200 3150 30  0001 C CNN
F 1 "GND" H 6200 3080 30  0001 C CNN
F 2 "" H 6200 3150 60  0000 C CNN
F 3 "" H 6200 3150 60  0000 C CNN
	1    6200 3150
	1    0    0    -1  
$EndComp
NoConn ~ 4150 4500
NoConn ~ 4150 4600
NoConn ~ 4150 4700
NoConn ~ 4150 4800
NoConn ~ 4150 4900
$Comp
L C C1
U 1 1 52D1F8D5
P 3150 1450
F 0 "C1" H 3150 1550 40  0000 L CNN
F 1 "100n" H 3156 1365 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3188 1300 30  0000 C CNN
F 3 "~" H 3150 1450 60  0000 C CNN
	1    3150 1450
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P1
U 1 1 545930E9
P 1650 1000
F 0 "P1" V 1600 1000 40  0000 C CNN
F 1 "PWR_IN" V 1700 1000 40  0000 C CNN
F 2 "Connect:AK300-2" H 1650 1000 60  0000 C CNN
F 3 "~" H 1650 1000 60  0000 C CNN
	1    1650 1000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR6
U 1 1 5459316E
P 1750 1450
F 0 "#PWR6" H 1750 1450 30  0001 C CNN
F 1 "GND" H 1750 1380 30  0001 C CNN
F 2 "" H 1750 1450 60  0000 C CNN
F 3 "" H 1750 1450 60  0000 C CNN
	1    1750 1450
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 545931D0
P 8800 5550
F 0 "P2" V 8750 5550 40  0000 C CNN
F 1 "PWR_OUT" V 8850 5550 40  0000 C CNN
F 2 "Connect:AK300-2" H 8800 5550 60  0000 C CNN
F 3 "~" H 8800 5550 60  0000 C CNN
	1    8800 5550
	1    0    0    -1  
$EndComp
$Comp
L HD44780 U3
U 1 1 54593507
P 9800 1800
F 0 "U3" H 10700 1250 60  0000 C CNN
F 1 "HD44780" H 9700 1500 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x16" H 9800 1800 60  0001 C CNN
F 3 "" H 9800 1800 60  0000 C CNN
	1    9800 1800
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR24
U 1 1 54594AD7
P 9400 2650
F 0 "#PWR24" H 9400 2650 30  0001 C CNN
F 1 "GND" H 9400 2580 30  0001 C CNN
F 2 "" H 9400 2650 60  0000 C CNN
F 3 "" H 9400 2650 60  0000 C CNN
	1    9400 2650
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 52BB0288
P 4450 1450
F 0 "C2" H 4450 1550 40  0000 L CNN
F 1 "100n" H 4456 1365 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4488 1300 30  0000 C CNN
F 3 "~" H 4450 1450 60  0000 C CNN
	1    4450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1450 3700 2000
Connection ~ 4200 1150
Wire Wire Line
	1350 4400 1350 4650
Wire Wire Line
	1050 4400 1050 4650
Wire Wire Line
	2250 4000 1350 4000
Wire Wire Line
	1550 4100 2250 4100
Connection ~ 1300 2850
Wire Wire Line
	1050 3900 2250 3900
Connection ~ 1300 3900
Connection ~ 2150 2850
Wire Wire Line
	1300 2700 1300 3000
Wire Wire Line
	1300 3900 1300 3700
Wire Wire Line
	1050 3900 1050 4000
Wire Wire Line
	1550 4650 1550 4100
Wire Wire Line
	6200 7150 6300 7150
Connection ~ 6300 7150
Wire Wire Line
	6200 6850 6300 6850
Wire Wire Line
	5700 4400 4150 4400
Wire Wire Line
	1750 1450 1750 1350
Wire Wire Line
	4100 1150 4450 1150
Wire Wire Line
	9400 2450 9500 2450
Wire Wire Line
	6200 2850 6100 2850
Wire Wire Line
	5700 3150 5700 4400
Wire Wire Line
	6200 3150 6200 2850
Wire Wire Line
	2150 3700 2250 3700
Wire Wire Line
	3100 6300 3100 6350
Wire Wire Line
	3100 6350 3200 6350
Wire Wire Line
	3200 6350 3200 6300
Wire Wire Line
	3150 6350 3150 6450
Connection ~ 3150 6350
Wire Wire Line
	3100 3400 3100 3300
Wire Wire Line
	3100 3300 3200 3300
Wire Wire Line
	3150 2850 3150 3300
Wire Wire Line
	3200 3300 3200 3400
Connection ~ 3150 3300
Wire Wire Line
	2950 1150 3300 1150
Wire Wire Line
	3150 1150 3150 1250
Connection ~ 3150 1150
Wire Wire Line
	3150 1650 3150 1750
Wire Wire Line
	3150 1750 4450 1750
Connection ~ 3700 1750
Wire Wire Line
	4200 1750 4200 1650
Wire Wire Line
	4200 1250 4200 1150
Wire Wire Line
	4150 3700 4350 3700
Wire Wire Line
	4150 3800 4350 3800
Wire Wire Line
	4150 3900 4350 3900
Wire Wire Line
	2950 950  2950 1150
Wire Wire Line
	4450 950  4450 1250
$Comp
L +5V #PWR12
U 1 1 548A0516
P 4450 950
F 0 "#PWR12" H 4450 1040 20  0001 C CNN
F 1 "+5V" H 4450 1040 30  0000 C CNN
F 2 "" H 4450 950 60  0000 C CNN
F 3 "" H 4450 950 60  0000 C CNN
	1    4450 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1050 8800 1050
Wire Wire Line
	9500 1150 9350 1150
Wire Wire Line
	9500 1250 9350 1250
Wire Wire Line
	9500 1350 9350 1350
Wire Wire Line
	9500 1450 9350 1450
Wire Wire Line
	9500 1950 9350 1950
Wire Wire Line
	9500 2050 9350 2050
Wire Wire Line
	9500 2150 9350 2150
Wire Wire Line
	9250 2250 9500 2250
Text Label 9350 1150 2    60   ~ 0
LCD_Data7
Text Label 9350 1250 2    60   ~ 0
LCD_Data6
Text Label 9350 1350 2    60   ~ 0
LCD_Data5
Text Label 9350 1450 2    60   ~ 0
LCD_Data4
Text Label 9350 1950 2    60   ~ 0
LCD_EN
Text Label 9350 2050 2    60   ~ 0
LCD_RW
Text Label 9350 2150 2    60   ~ 0
LCD_RS
$Comp
L +5V #PWR23
U 1 1 548A0D87
P 9100 2350
F 0 "#PWR23" H 9100 2440 20  0001 C CNN
F 1 "+5V" H 9100 2440 30  0000 C CNN
F 2 "" H 9100 2350 60  0000 C CNN
F 3 "" H 9100 2350 60  0000 C CNN
	1    9100 2350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR3
U 1 1 548A102B
P 1300 2700
F 0 "#PWR3" H 1300 2790 20  0001 C CNN
F 1 "+5V" H 1300 2790 30  0000 C CNN
F 2 "" H 1300 2700 60  0000 C CNN
F 3 "" H 1300 2700 60  0000 C CNN
	1    1300 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 2850 3150 2850
Wire Wire Line
	2150 3450 2150 3700
Wire Wire Line
	2150 2950 2150 2850
Wire Wire Line
	4150 5400 5150 5400
Wire Wire Line
	4150 5500 5150 5500
Wire Wire Line
	4150 5600 5150 5600
Text Label 4350 5600 0    60   ~ 0
K_edit
Text Label 4350 5400 0    60   ~ 0
K_up
Text Label 4350 5500 0    60   ~ 0
K_dn
Wire Wire Line
	5350 7450 5600 7450
Wire Wire Line
	5600 7150 5450 7150
Wire Wire Line
	5200 6850 5600 6850
$Comp
L CONN_01X04 P3
U 1 1 548A1A4E
P 5000 7000
F 0 "P3" H 5000 7250 50  0000 C CNN
F 1 "CONN_01X04" V 5100 7000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04" H 5000 7000 60  0001 C CNN
F 3 "" H 5000 7000 60  0000 C CNN
	1    5000 7000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5450 7150 5450 6950
Wire Wire Line
	5450 6950 5200 6950
Wire Wire Line
	5200 7050 5350 7050
Wire Wire Line
	5350 7050 5350 7450
Wire Wire Line
	6300 6850 6300 7650
Wire Wire Line
	6200 7450 6300 7450
Connection ~ 6300 7450
Wire Wire Line
	5250 7650 5250 7150
Wire Wire Line
	5250 7150 5200 7150
$Comp
L CONN_01X04 P4
U 1 1 548A1EBF
P 5350 5550
F 0 "P4" H 5350 5800 50  0000 C CNN
F 1 "CONN_01X04" V 5450 5550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04" H 5350 5550 60  0001 C CNN
F 3 "" H 5350 5550 60  0000 C CNN
	1    5350 5550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 548A1F87
P 5100 5800
F 0 "#PWR13" H 5100 5800 30  0001 C CNN
F 1 "GND" H 5100 5730 30  0001 C CNN
F 2 "" H 5100 5800 60  0000 C CNN
F 3 "" H 5100 5800 60  0000 C CNN
	1    5100 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 5800 5100 5700
Wire Wire Line
	5100 5700 5150 5700
Wire Wire Line
	6300 7650 5250 7650
$Comp
L +12V #PWR2
U 1 1 548A221E
P 1300 1300
F 0 "#PWR2" H 1300 1250 20  0001 C CNN
F 1 "+12V" H 1300 1400 30  0000 C CNN
F 2 "" H 1300 1300 60  0000 C CNN
F 3 "" H 1300 1300 60  0000 C CNN
	1    1300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1300 1300 1450
Wire Wire Line
	1300 1450 1550 1450
Wire Wire Line
	1550 1450 1550 1350
Wire Wire Line
	8250 5650 8450 5650
$Comp
L GND #PWR19
U 1 1 548A2843
P 8250 5800
F 0 "#PWR19" H 8250 5800 30  0001 C CNN
F 1 "GND" H 8250 5730 30  0001 C CNN
F 2 "" H 8250 5800 60  0000 C CNN
F 3 "" H 8250 5800 60  0000 C CNN
	1    8250 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5800 8250 5650
$Comp
L +5V #PWR14
U 1 1 548A2D1F
P 5150 2750
F 0 "#PWR14" H 5150 2840 20  0001 C CNN
F 1 "+5V" H 5150 2840 30  0000 C CNN
F 2 "" H 5150 2750 60  0000 C CNN
F 3 "" H 5150 2750 60  0000 C CNN
	1    5150 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2750 5150 2850
Wire Wire Line
	5150 2850 5300 2850
$Comp
L MOSFET_N Q1
U 1 1 548A2F93
P 7050 5150
F 0 "Q1" H 7060 5320 60  0000 R CNN
F 1 "2N7002" H 7060 5000 60  0000 R CNN
F 2 "SMD_Packages:SOT-23-GDS" H 7050 5150 60  0001 C CNN
F 3 "" H 7050 5150 60  0000 C CNN
	1    7050 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4500 8100 5450
$Comp
L +12V #PWR18
U 1 1 548A3482
P 8100 3400
F 0 "#PWR18" H 8100 3350 20  0001 C CNN
F 1 "+12V" H 8100 3500 30  0000 C CNN
F 2 "" H 8100 3400 60  0000 C CNN
F 3 "" H 8100 3400 60  0000 C CNN
	1    8100 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3400 8100 4100
$Comp
L R R4
U 1 1 548A373A
P 7150 3950
F 0 "R4" V 7230 3950 40  0000 C CNN
F 1 "1k" V 7157 3951 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7080 3950 30  0001 C CNN
F 3 "" H 7150 3950 30  0000 C CNN
	1    7150 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4200 7150 4450
Wire Wire Line
	7150 3700 7150 3500
Wire Wire Line
	7150 3500 8100 3500
Connection ~ 8100 3500
Connection ~ 7150 4300
Wire Wire Line
	7150 4850 7150 4950
$Comp
L GND #PWR16
U 1 1 548A3B00
P 7150 5550
F 0 "#PWR16" H 7150 5550 30  0001 C CNN
F 1 "GND" H 7150 5480 30  0001 C CNN
F 2 "" H 7150 5550 60  0000 C CNN
F 3 "" H 7150 5550 60  0000 C CNN
	1    7150 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5350 7150 5550
$Comp
L R R6
U 1 1 548A3C2A
P 7500 4300
F 0 "R6" V 7580 4300 40  0000 C CNN
F 1 "2k2" V 7507 4301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7430 4300 30  0001 C CNN
F 3 "" H 7500 4300 30  0000 C CNN
	1    7500 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 4300 7250 4300
Wire Wire Line
	7750 4300 7800 4300
$Comp
L R R3
U 1 1 548A3F9E
P 6500 5450
F 0 "R3" V 6580 5450 40  0000 C CNN
F 1 "10k" V 6507 5451 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6430 5450 30  0001 C CNN
F 3 "" H 6500 5450 30  0000 C CNN
	1    6500 5450
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 5450 7150 5450
Connection ~ 7150 5450
$Comp
L R R2
U 1 1 548A408E
P 6500 5150
F 0 "R2" V 6580 5150 40  0000 C CNN
F 1 "47R" V 6507 5151 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6430 5150 30  0001 C CNN
F 3 "" H 6500 5150 30  0000 C CNN
	1    6500 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 5150 6850 5150
Wire Wire Line
	4750 5150 6250 5150
Wire Wire Line
	6150 5150 6150 5450
Wire Wire Line
	6150 5450 6250 5450
Text GLabel 8450 5200 2    60   Output ~ 0
PWR
Wire Wire Line
	8100 5450 8450 5450
Wire Wire Line
	4150 5300 4750 5300
Wire Wire Line
	4750 5300 4750 5150
Connection ~ 6150 5150
Wire Wire Line
	8450 5200 8100 5200
Connection ~ 8100 5200
Text Label 5200 6850 0    60   ~ 0
K_edit
Text Label 5200 6950 0    60   ~ 0
K_up
Text Label 5200 7050 0    60   ~ 0
K_dn
Wire Wire Line
	2250 4400 2050 4400
Wire Wire Line
	2250 4600 2050 4600
Text Label 2050 4400 2    60   ~ 0
LCD_RS
Text Label 2050 4600 2    60   ~ 0
LCD_RW
Text Label 4350 5800 0    60   ~ 0
LCD_EN
Text Label 4350 5900 0    60   ~ 0
LCD_Data4
Wire Wire Line
	4350 5900 4150 5900
Wire Wire Line
	4150 5800 4350 5800
Text Label 4350 6000 0    60   ~ 0
LCD_Data5
Wire Wire Line
	4350 6000 4150 6000
Text Label 4350 3700 0    60   ~ 0
LCD_Data6
Text Label 4350 3800 0    60   ~ 0
LCD_Data7
Text Label 4350 3900 0    60   ~ 0
LCD_Blight_PWM
NoConn ~ 4150 5000
NoConn ~ 4150 5100
NoConn ~ 4150 5700
$Comp
L MOSFET_N Q2
U 1 1 548A6A6A
P 7850 1250
F 0 "Q2" H 7860 1420 60  0000 R CNN
F 1 "2N7002" H 7860 1100 60  0000 R CNN
F 2 "SMD_Packages:SOT-23-GDS" H 7850 1250 60  0001 C CNN
F 3 "" H 7850 1250 60  0000 C CNN
	1    7850 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 1250 7450 1250
$Comp
L R R7
U 1 1 548A6B9C
P 8350 950
F 0 "R7" V 8430 950 40  0000 C CNN
F 1 "100R" V 8357 951 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8280 950 30  0001 C CNN
F 3 "" H 8350 950 30  0000 C CNN
	1    8350 950 
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 950  7950 950 
Wire Wire Line
	7950 950  7950 1050
$Comp
L GND #PWR17
U 1 1 548A6D90
P 7950 1650
F 0 "#PWR17" H 7950 1650 30  0001 C CNN
F 1 "GND" H 7950 1580 30  0001 C CNN
F 2 "" H 7950 1650 60  0000 C CNN
F 3 "" H 7950 1650 60  0000 C CNN
	1    7950 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 1650 7950 1450
Text Label 6750 1250 2    60   ~ 0
LCD_Blight_PWM
$Comp
L POT RV1
U 1 1 548A705D
P 8450 2200
F 0 "RV1" H 8450 2100 50  0000 C CNN
F 1 "47k" H 8450 2200 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_VishaySpectrol-Econtrim-Type36P" H 8450 2200 60  0001 C CNN
F 3 "" H 8450 2200 60  0000 C CNN
	1    8450 2200
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR20
U 1 1 548A7182
P 8450 1800
F 0 "#PWR20" H 8450 1890 20  0001 C CNN
F 1 "+5V" H 8450 1890 30  0000 C CNN
F 2 "" H 8450 1800 60  0000 C CNN
F 3 "" H 8450 1800 60  0000 C CNN
	1    8450 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2350 9500 2350
Wire Wire Line
	9250 2250 9250 2200
Wire Wire Line
	9250 2200 8600 2200
Wire Wire Line
	8450 1800 8450 1950
$Comp
L GND #PWR21
U 1 1 548A73F6
P 8450 2650
F 0 "#PWR21" H 8450 2650 30  0001 C CNN
F 1 "GND" H 8450 2580 30  0001 C CNN
F 2 "" H 8450 2650 60  0000 C CNN
F 3 "" H 8450 2650 60  0000 C CNN
	1    8450 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 2650 8450 2450
Wire Wire Line
	9400 2650 9400 2450
$Comp
L R R5
U 1 1 548A79C8
P 7200 1250
F 0 "R5" V 7280 1250 40  0000 C CNN
F 1 "47R" V 7207 1251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7130 1250 30  0001 C CNN
F 3 "" H 7200 1250 30  0000 C CNN
	1    7200 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 1250 6750 1250
$Comp
L BC516 Q3
U 1 1 548A7C09
P 8000 4300
F 0 "Q3" H 7993 4151 40  0000 R CNN
F 1 "BC516" H 7997 4455 40  0000 R CNN
F 2 "Housings_TO-92:TO-92-Free-inline-wide" H 7897 4413 29  0000 C CNN
F 3 "" H 7980 4300 60  0000 C CNN
	1    8000 4300
	1    0    0    1   
$EndComp
NoConn ~ 9500 1550
NoConn ~ 9500 1650
NoConn ~ 9500 1750
NoConn ~ 9500 1850
$Comp
L +5V #PWR22
U 1 1 548A85D9
P 8800 1050
F 0 "#PWR22" H 8800 1140 20  0001 C CNN
F 1 "+5V" H 8800 1140 30  0000 C CNN
F 2 "" H 8800 1050 60  0000 C CNN
F 3 "" H 8800 1050 60  0000 C CNN
	1    8800 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 950  8700 950 
Wire Wire Line
	8700 950  8700 850 
Wire Wire Line
	8700 850  9100 850 
Wire Wire Line
	9100 850  9100 950 
Wire Wire Line
	9100 950  9500 950 
$Comp
L ATMEGA8-AI IC1
U 1 1 548A8D1B
P 3150 4700
F 0 "IC1" H 2400 5900 40  0000 L BNN
F 1 "ATMEGA8-AI" H 3650 3150 40  0000 L BNN
F 2 "SMD_Packages:TQFP-32" H 3150 4700 30  0000 C CIN
F 3 "" H 3150 4700 60  0000 C CNN
	1    3150 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4000 4350 4000
Wire Wire Line
	4150 4100 4350 4100
Wire Wire Line
	4150 4200 4350 4200
Text Label 4350 4000 0    60   ~ 0
MOSI
Text Label 4350 4100 0    60   ~ 0
MISO
Text Label 4350 4200 0    60   ~ 0
SCK
Text Label 2150 3700 2    60   ~ 0
RST
$Comp
L CONN_02X05 P5
U 1 1 548A98C5
P 1600 6850
F 0 "P5" H 1600 7150 50  0000 C CNN
F 1 "ISP_IDC" H 1600 6550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05" H 1600 5650 60  0001 C CNN
F 3 "" H 1600 5650 60  0000 C CNN
	1    1600 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6650 1200 6650
Wire Wire Line
	1350 6850 1200 6850
Wire Wire Line
	1200 6950 1350 6950
Wire Wire Line
	1350 7050 1200 7050
Wire Wire Line
	1850 6650 2000 6650
Wire Wire Line
	2000 6650 2000 6550
Wire Wire Line
	1850 6750 2000 6750
Wire Wire Line
	2000 6750 2000 7250
Wire Wire Line
	1850 7050 2000 7050
Connection ~ 2000 7050
Wire Wire Line
	1850 6950 2000 6950
Connection ~ 2000 6950
Wire Wire Line
	1850 6850 2000 6850
Connection ~ 2000 6850
$Comp
L GND #PWR8
U 1 1 548AA223
P 2000 7250
F 0 "#PWR8" H 2000 7250 30  0001 C CNN
F 1 "GND" H 2000 7180 30  0001 C CNN
F 2 "" H 2000 7250 60  0000 C CNN
F 3 "" H 2000 7250 60  0000 C CNN
	1    2000 7250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR7
U 1 1 548AA403
P 2000 6550
F 0 "#PWR7" H 2000 6640 20  0001 C CNN
F 1 "+5V" H 2000 6640 30  0000 C CNN
F 2 "" H 2000 6550 60  0000 C CNN
F 3 "" H 2000 6550 60  0000 C CNN
	1    2000 6550
	1    0    0    -1  
$EndComp
NoConn ~ 1350 6750
Text Label 1200 6650 2    60   ~ 0
MOSI
Text Label 1200 7050 2    60   ~ 0
MISO
Text Label 1200 6850 2    60   ~ 0
RST
Text Label 1200 6950 2    60   ~ 0
SCK
Connection ~ 4450 1150
Wire Wire Line
	4450 1750 4450 1650
Connection ~ 4200 1750
$Comp
L CAPAPOL C3
U 1 1 548A40D8
P 4200 1450
F 0 "C3" H 4250 1550 40  0000 L CNN
F 1 "10u" H 4250 1350 40  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:TantalC_SizeA_EIA-3216_HandSoldering" H 4300 1300 30  0001 C CNN
F 3 "" H 4200 1450 300 0000 C CNN
	1    4200 1450
	1    0    0    -1  
$EndComp
$Comp
L LM35 U2
U 1 1 548BE2A8
P 5700 2850
F 0 "U2" H 5350 2900 60  0000 C CNN
F 1 "LM35" H 5350 2800 60  0000 C CNN
F 2 "" H 5700 3100 60  0000 C CNN
F 3 "" H 5700 3100 60  0000 C CNN
	1    5700 2850
	0    -1   1    0   
$EndComp
$Comp
L R R8
U 1 1 548D650C
P 7200 1550
F 0 "R8" V 7280 1550 40  0000 C CNN
F 1 "10k" V 7207 1551 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7130 1550 30  0001 C CNN
F 3 "" H 7200 1550 30  0000 C CNN
	1    7200 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 1550 7950 1550
Connection ~ 7950 1550
Wire Wire Line
	6950 1550 6850 1550
Wire Wire Line
	6850 1550 6850 1250
Connection ~ 6850 1250
$EndSCHEMATC
