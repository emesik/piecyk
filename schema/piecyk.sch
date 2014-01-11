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
LIBS:display_lcd
LIBS:capteurs
LIBS:noname-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "piecyk"
Date "11 jan 2014"
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
P 1600 1050
F 0 "U1" H 1750 854 60  0000 C CNN
F 1 "78L05" H 1600 1250 60  0000 C CNN
F 2 "~" H 1600 1050 60  0000 C CNN
F 3 "~" H 1600 1050 60  0000 C CNN
	1    1600 1050
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 52BB0288
P 2100 1400
F 0 "C1" H 2100 1500 40  0000 L CNN
F 1 "100n" H 2106 1315 40  0000 L CNN
F 2 "~" H 2138 1250 30  0000 C CNN
F 3 "~" H 2100 1400 60  0000 C CNN
	1    2100 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 52BB03AA
P 1600 1850
F 0 "#PWR01" H 1600 1850 30  0001 C CNN
F 1 "GND" H 1600 1780 30  0001 C CNN
F 2 "" H 1600 1850 60  0000 C CNN
F 3 "" H 1600 1850 60  0000 C CNN
	1    1600 1850
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA8A-P IC1
U 1 1 52BB0430
P 3600 4950
F 0 "IC1" H 2850 6250 40  0000 L BNN
F 1 "ATMEGA8A-P" H 4100 3500 40  0000 L BNN
F 2 "DIL28" H 3600 4950 30  0000 C CIN
F 3 "~" H 3600 4950 60  0000 C CNN
	1    3600 4950
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 52BB0450
P 2600 3600
F 0 "R1" V 2680 3600 40  0000 C CNN
F 1 "10k" V 2607 3601 40  0000 C CNN
F 2 "~" V 2530 3600 30  0000 C CNN
F 3 "~" H 2600 3600 30  0000 C CNN
	1    2600 3600
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 52BB045F
P 1500 4350
F 0 "C3" H 1500 4450 40  0000 L CNN
F 1 "100n" H 1506 4265 40  0000 L CNN
F 2 "~" H 1538 4200 30  0000 C CNN
F 3 "~" H 1500 4350 60  0000 C CNN
	1    1500 4350
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 52BB046E
P 1800 4350
F 0 "C4" H 1800 4450 40  0000 L CNN
F 1 "100n" H 1806 4265 40  0000 L CNN
F 2 "~" H 1838 4200 30  0000 C CNN
F 3 "~" H 1800 4350 60  0000 C CNN
	1    1800 4350
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 52BB04EE
P 3200 3200
F 0 "C2" H 3200 3300 40  0000 L CNN
F 1 "100n" H 3206 3115 40  0000 L CNN
F 2 "~" H 3238 3050 30  0000 C CNN
F 3 "~" H 3200 3200 60  0000 C CNN
	1    3200 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 52BB0506
P 3200 3550
F 0 "#PWR02" H 3200 3550 30  0001 C CNN
F 1 "GND" H 3200 3480 30  0001 C CNN
F 2 "" H 3200 3550 60  0000 C CNN
F 3 "" H 3200 3550 60  0000 C CNN
	1    3200 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 52BB0515
P 1800 4800
F 0 "#PWR03" H 1800 4800 30  0001 C CNN
F 1 "GND" H 1800 4730 30  0001 C CNN
F 2 "" H 1800 4800 60  0000 C CNN
F 3 "" H 1800 4800 60  0000 C CNN
	1    1800 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 52BB0524
P 1500 4800
F 0 "#PWR04" H 1500 4800 30  0001 C CNN
F 1 "GND" H 1500 4730 30  0001 C CNN
F 2 "" H 1500 4800 60  0000 C CNN
F 3 "" H 1500 4800 60  0000 C CNN
	1    1500 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 52BB0533
P 2000 4800
F 0 "#PWR05" H 2000 4800 30  0001 C CNN
F 1 "GND" H 2000 4730 30  0001 C CNN
F 2 "" H 2000 4800 60  0000 C CNN
F 3 "" H 2000 4800 60  0000 C CNN
	1    2000 4800
	1    0    0    -1  
$EndComp
$Comp
L FILTER L1
U 1 1 52BB073B
P 1750 3500
F 0 "L1" H 1750 3650 60  0000 C CNN
F 1 "10uH" H 1750 3400 60  0000 C CNN
F 2 "~" H 1750 3500 60  0000 C CNN
F 3 "~" H 1750 3500 60  0000 C CNN
	1    1750 3500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR06
U 1 1 52BB0800
P 3600 6600
F 0 "#PWR06" H 3600 6600 30  0001 C CNN
F 1 "GND" H 3600 6530 30  0001 C CNN
F 2 "" H 3600 6600 60  0000 C CNN
F 3 "" H 3600 6600 60  0000 C CNN
	1    3600 6600
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 52BB1715
P 6050 5650
F 0 "SW1" H 6200 5760 50  0000 C CNN
F 1 "SW_edit" H 6050 5570 50  0000 C CNN
F 2 "~" H 6050 5650 60  0000 C CNN
F 3 "~" H 6050 5650 60  0000 C CNN
	1    6050 5650
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 52BB1724
P 6050 5350
F 0 "SW2" H 6200 5460 50  0000 C CNN
F 1 "SW_down" H 6050 5270 50  0000 C CNN
F 2 "~" H 6050 5350 60  0000 C CNN
F 3 "~" H 6050 5350 60  0000 C CNN
	1    6050 5350
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW3
U 1 1 52BB1733
P 6050 5050
F 0 "SW3" H 6200 5160 50  0000 C CNN
F 1 "SW_up" H 6050 4970 50  0000 C CNN
F 2 "~" H 6050 5050 60  0000 C CNN
F 3 "~" H 6050 5050 60  0000 C CNN
	1    6050 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 52BB190C
P 6450 5850
F 0 "#PWR07" H 6450 5850 30  0001 C CNN
F 1 "GND" H 6450 5780 30  0001 C CNN
F 2 "" H 6450 5850 60  0000 C CNN
F 3 "" H 6450 5850 60  0000 C CNN
	1    6450 5850
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR08
U 1 1 52D0DF48
P 850 1000
F 0 "#PWR08" H 850 950 20  0001 C CNN
F 1 "+12V" H 850 1100 30  0000 C CNN
F 2 "" H 850 1000 60  0000 C CNN
F 3 "" H 850 1000 60  0000 C CNN
	1    850  1000
	1    0    0    -1  
$EndComp
Text GLabel 2300 1000 2    60   Output ~ 0
Vcc +5V
Text GLabel 1550 3000 0    60   Input ~ 0
Vcc +5V
$Comp
L PNP Q1
U 1 1 52D0DFF4
P 6950 5500
F 0 "Q1" H 6950 5350 60  0000 R CNN
F 1 "PNP" H 6950 5650 60  0000 R CNN
F 2 "~" H 6950 5500 60  0000 C CNN
F 3 "~" H 6950 5500 60  0000 C CNN
	1    6950 5500
	-1   0    0    1   
$EndComp
$Comp
L LED D1
U 1 1 52D0E01F
P 7100 4700
F 0 "D1" H 7100 4800 50  0000 C CNN
F 1 "LED" H 7100 4600 50  0000 C CNN
F 2 "~" H 7100 4700 60  0000 C CNN
F 3 "~" H 7100 4700 60  0000 C CNN
	1    7100 4700
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 52D0E03F
P 7300 5250
F 0 "R2" V 7380 5250 40  0000 C CNN
F 1 "R" V 7307 5251 40  0000 C CNN
F 2 "~" V 7230 5250 30  0000 C CNN
F 3 "~" H 7300 5250 30  0000 C CNN
	1    7300 5250
	-1   0    0    1   
$EndComp
Text GLabel 7200 6100 2    60   Output ~ 0
+12V piec
$Comp
L +12V #PWR09
U 1 1 52D0E282
P 6850 5100
F 0 "#PWR09" H 6850 5050 20  0001 C CNN
F 1 "+12V" H 6850 5200 30  0000 C CNN
F 2 "" H 6850 5100 60  0000 C CNN
F 3 "" H 6850 5100 60  0000 C CNN
	1    6850 5100
	1    0    0    -1  
$EndComp
$Comp
L DISPLAY_HD44780 D_LCD1
U 1 1 52D0E826
P 9600 2600
F 0 "D_LCD1" H 10200 2150 60  0000 C CNN
F 1 "DISPLAY_HD44780" H 9350 3000 60  0000 C CNN
F 2 "" H 9600 2600 60  0000 C CNN
F 3 "" H 9600 2600 60  0000 C CNN
	1    9600 2600
	-1   0    0    1   
$EndComp
Text Label 10150 1700 3    60   ~ 0
LCD_db7
Text Label 10050 1700 3    60   ~ 0
LCD_db6
Text Label 9950 1700 3    60   ~ 0
LCD_db5
Text Label 9850 1700 3    60   ~ 0
LCD_db4
Text Label 9350 1700 3    60   ~ 0
LCD_e
Text Label 9250 1700 3    60   ~ 0
LCD_rw
Text Label 9150 1700 3    60   ~ 0
LCD_rs
Text Label 5000 3850 0    60   ~ 0
LCD_e
Text Label 5000 3950 0    60   ~ 0
LCD_rw
Text Label 5000 4050 0    60   ~ 0
LCD_rs
Text Label 2250 4750 0    60   ~ 0
LCD_db7
Text Label 5050 5750 0    60   ~ 0
LCD_db6
Text Label 5050 5850 0    60   ~ 0
LCD_db5
Text Label 5050 5950 0    60   ~ 0
LCD_db4
Text GLabel 8800 1050 0    60   Input ~ 0
Vcc +5V
$Comp
L R R3
U 1 1 52D0F742
P 9050 1450
F 0 "R3" V 9130 1450 40  0000 C CNN
F 1 "R" V 9057 1451 40  0000 C CNN
F 2 "~" V 8980 1450 30  0000 C CNN
F 3 "~" H 9050 1450 30  0000 C CNN
	1    9050 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 52D0F85C
P 8650 2900
F 0 "#PWR010" H 8650 2900 30  0001 C CNN
F 1 "GND" H 8650 2830 30  0001 C CNN
F 2 "" H 8650 2900 60  0000 C CNN
F 3 "" H 8650 2900 60  0000 C CNN
	1    8650 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 52D0F869
P 10550 2900
F 0 "#PWR011" H 10550 2900 30  0001 C CNN
F 1 "GND" H 10550 2830 30  0001 C CNN
F 2 "" H 10550 2900 60  0000 C CNN
F 3 "" H 10550 2900 60  0000 C CNN
	1    10550 2900
	1    0    0    -1  
$EndComp
$Comp
L LM35 U2
U 1 1 52D0F9F1
P 6200 4000
F 0 "U2" H 5900 4400 60  0000 C CNN
F 1 "LM35" H 5900 4300 60  0000 C CNN
F 2 "" H 6200 4000 60  0000 C CNN
F 3 "" H 6200 4000 60  0000 C CNN
	1    6200 4000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR012
U 1 1 52D0FAD8
P 5750 4350
F 0 "#PWR012" H 5750 4350 30  0001 C CNN
F 1 "GND" H 5750 4280 30  0001 C CNN
F 2 "" H 5750 4350 60  0000 C CNN
F 3 "" H 5750 4350 60  0000 C CNN
	1    5750 4350
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 52D0FB43
P 10250 1450
F 0 "R4" V 10330 1450 40  0000 C CNN
F 1 "R" V 10257 1451 40  0000 C CNN
F 2 "~" V 10180 1450 30  0000 C CNN
F 3 "~" H 10250 1450 30  0000 C CNN
	1    10250 1450
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 52D0FC62
P 5750 3700
F 0 "C5" H 5750 3800 40  0000 L CNN
F 1 "100n" H 5756 3615 40  0000 L CNN
F 2 "~" H 5788 3550 30  0000 C CNN
F 3 "~" H 5750 3700 60  0000 C CNN
	1    5750 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1000 2100 1200
Wire Wire Line
	1600 1300 1600 1850
Connection ~ 1600 1600
Connection ~ 2100 1000
Wire Wire Line
	850  1000 1200 1000
Wire Wire Line
	2000 1000 2300 1000
Wire Wire Line
	3600 3000 3600 3550
Wire Wire Line
	3200 3550 3200 3400
Wire Wire Line
	1800 4550 1800 4800
Wire Wire Line
	1500 4550 1500 4800
Wire Wire Line
	2700 4150 1800 4150
Wire Wire Line
	2000 4250 2700 4250
Wire Wire Line
	2700 3850 2600 3850
Connection ~ 3200 3000
Connection ~ 1750 3000
Wire Wire Line
	1500 4050 2700 4050
Connection ~ 1750 4050
Wire Wire Line
	2600 3000 2600 3350
Connection ~ 2600 3000
Wire Wire Line
	1750 3000 1750 3150
Wire Wire Line
	1750 4050 1750 3850
Wire Wire Line
	3600 6450 3600 6600
Wire Wire Line
	7200 6100 6850 6100
Wire Wire Line
	6850 6100 6850 5700
Wire Wire Line
	6850 5100 6850 5300
Wire Wire Line
	7300 5500 7150 5500
Wire Wire Line
	1550 3000 6700 3000
Wire Wire Line
	4600 3850 5000 3850
Wire Wire Line
	4600 3950 5000 3950
Wire Wire Line
	4600 4050 5000 4050
Wire Wire Line
	1500 4050 1500 4150
Wire Wire Line
	2000 4800 2000 4250
Wire Wire Line
	2250 4750 2700 4750
Wire Wire Line
	4600 5750 5050 5750
Wire Wire Line
	4600 5850 5050 5850
Wire Wire Line
	4600 5950 5050 5950
Wire Wire Line
	4600 5650 5750 5650
Wire Wire Line
	4600 5550 5650 5550
Wire Wire Line
	5650 5550 5650 5350
Wire Wire Line
	5650 5350 5750 5350
Wire Wire Line
	5750 5050 5550 5050
Wire Wire Line
	5550 5050 5550 5450
Wire Wire Line
	5550 5450 4600 5450
Wire Wire Line
	6350 5050 6450 5050
Wire Wire Line
	6450 5050 6450 5850
Wire Wire Line
	6350 5350 6450 5350
Connection ~ 6450 5350
Wire Wire Line
	6350 5650 6450 5650
Connection ~ 6450 5650
Wire Wire Line
	7300 4700 7300 5000
Wire Wire Line
	4600 5250 5250 5250
Wire Wire Line
	5250 5250 5250 4700
Wire Wire Line
	5250 4700 6900 4700
Wire Wire Line
	9350 2250 9350 1700
Wire Wire Line
	9250 1700 9250 2250
Wire Wire Line
	9150 2250 9150 1700
Wire Wire Line
	9850 2250 9850 1700
Wire Wire Line
	9950 1700 9950 2250
Wire Wire Line
	10050 2250 10050 1700
Wire Wire Line
	10150 1700 10150 2250
Wire Wire Line
	9050 1700 9050 2250
Wire Wire Line
	9050 1050 9050 1200
Wire Wire Line
	8800 1050 10250 1050
Wire Wire Line
	8950 2250 8950 1050
Connection ~ 8950 1050
Connection ~ 9050 1050
Wire Wire Line
	10550 2900 10550 2100
Wire Wire Line
	10550 2100 10350 2100
Wire Wire Line
	10350 2100 10350 2250
Wire Wire Line
	8650 2900 8650 2100
Wire Wire Line
	8650 2100 8850 2100
Wire Wire Line
	8850 2100 8850 2250
Wire Wire Line
	6200 4350 6200 4550
Wire Wire Line
	6200 4550 4600 4550
Wire Wire Line
	5900 4000 5750 4000
Wire Wire Line
	5750 3900 5750 4350
Wire Wire Line
	10250 2250 10250 1700
Wire Wire Line
	10250 1050 10250 1200
Wire Wire Line
	6700 4000 6500 4000
Connection ~ 5750 4000
Wire Wire Line
	1600 1600 2100 1600
Wire Wire Line
	6700 3000 6700 4000
Connection ~ 3600 3000
Wire Wire Line
	5750 3500 5750 3000
Connection ~ 5750 3000
NoConn ~ 2700 4550
NoConn ~ 4600 4150
NoConn ~ 4600 4250
NoConn ~ 4600 4350
NoConn ~ 4600 4650
NoConn ~ 4600 4750
NoConn ~ 4600 4850
NoConn ~ 4600 4950
NoConn ~ 4600 5050
NoConn ~ 4600 5350
NoConn ~ 9450 2250
NoConn ~ 9550 2250
NoConn ~ 9650 2250
NoConn ~ 9750 2250
$Comp
L GND #PWR013
U 1 1 52D0FA8D
P 9450 6100
F 0 "#PWR013" H 9450 6100 30  0001 C CNN
F 1 "GND" H 9450 6030 30  0001 C CNN
F 2 "" H 9450 6100 60  0000 C CNN
F 3 "" H 9450 6100 60  0000 C CNN
	1    9450 6100
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR014
U 1 1 52D0FA9C
P 9650 5650
F 0 "#PWR014" H 9650 5600 20  0001 C CNN
F 1 "+12V" H 9650 5750 30  0000 C CNN
F 2 "" H 9650 5650 60  0000 C CNN
F 3 "" H 9650 5650 60  0000 C CNN
	1    9650 5650
	1    0    0    -1  
$EndComp
Text GLabel 9900 6100 2    60   Output ~ 0
+12V piec
$Comp
L PWR_FLAG #FLG015
U 1 1 52D0FAAD
P 9450 5700
F 0 "#FLG015" H 9450 5795 30  0001 C CNN
F 1 "PWR_FLAG" H 9450 5880 30  0000 C CNN
F 2 "" H 9450 5700 60  0000 C CNN
F 3 "" H 9450 5700 60  0000 C CNN
	1    9450 5700
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG016
U 1 1 52D0FAC7
P 9650 6000
F 0 "#FLG016" H 9650 6095 30  0001 C CNN
F 1 "PWR_FLAG" H 9650 6180 30  0000 C CNN
F 2 "" H 9650 6000 60  0000 C CNN
F 3 "" H 9650 6000 60  0000 C CNN
	1    9650 6000
	-1   0    0    1   
$EndComp
Wire Wire Line
	9650 6000 9650 5650
Wire Wire Line
	9450 5700 9450 6100
$Comp
L PWR_FLAG #FLG017
U 1 1 52D0FB94
P 9850 5700
F 0 "#FLG017" H 9850 5795 30  0001 C CNN
F 1 "PWR_FLAG" H 9850 5880 30  0000 C CNN
F 2 "" H 9850 5700 60  0000 C CNN
F 3 "" H 9850 5700 60  0000 C CNN
	1    9850 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 6100 9850 6100
Wire Wire Line
	9850 6100 9850 5700
Text GLabel 10350 5900 0    60   Input ~ 0
Vcc +5v
$Comp
L PWR_FLAG #FLG018
U 1 1 52D0FBFB
P 10500 5700
F 0 "#FLG018" H 10500 5795 30  0001 C CNN
F 1 "PWR_FLAG" H 10500 5880 30  0000 C CNN
F 2 "" H 10500 5700 60  0000 C CNN
F 3 "" H 10500 5700 60  0000 C CNN
	1    10500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 5700 10500 5900
Wire Wire Line
	10500 5900 10350 5900
$EndSCHEMATC
