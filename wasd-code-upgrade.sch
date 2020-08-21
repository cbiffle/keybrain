EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "WASD CODE v2B Replacement Controller"
Date ""
Rev ""
Comp "Cliff L. Biffle / cliffle.com"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_02x12_Odd_Even H201
U 1 1 5F433520
P 8750 1650
F 0 "H201" H 8800 2367 50  0000 C CNN
F 1 "Molex0877582416" H 8800 2276 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_2x12_P2.00mm_Vertical" H 8750 1650 50  0001 C CNN
F 3 "~" H 8750 1650 50  0001 C CNN
	1    8750 1650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x12_Odd_Even H202
U 1 1 5F4350AE
P 10550 1650
F 0 "H202" H 10600 2367 50  0000 C CNN
F 1 "Molex0877582416" H 10600 2276 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_2x12_P2.00mm_Vertical" H 10550 1650 50  0001 C CNN
F 3 "~" H 10550 1650 50  0001 C CNN
	1    10550 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5F43B569
P 10350 1150
F 0 "#PWR0101" H 10350 1000 50  0001 C CNN
F 1 "+5V" V 10365 1278 50  0000 L CNN
F 2 "" H 10350 1150 50  0001 C CNN
F 3 "" H 10350 1150 50  0001 C CNN
	1    10350 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5F43B88A
P 10350 1550
F 0 "#PWR0102" H 10350 1300 50  0001 C CNN
F 1 "GND" V 10355 1422 50  0000 R CNN
F 2 "" H 10350 1550 50  0001 C CNN
F 3 "" H 10350 1550 50  0001 C CNN
	1    10350 1550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F43B96A
P 10350 2250
F 0 "#PWR0103" H 10350 2000 50  0001 C CNN
F 1 "GND" V 10355 2122 50  0000 R CNN
F 2 "" H 10350 2250 50  0001 C CNN
F 3 "" H 10350 2250 50  0001 C CNN
	1    10350 2250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5F43BA10
P 10850 2250
F 0 "#PWR0104" H 10850 2000 50  0001 C CNN
F 1 "GND" V 10855 2122 50  0000 R CNN
F 2 "" H 10850 2250 50  0001 C CNN
F 3 "" H 10850 2250 50  0001 C CNN
	1    10850 2250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5F43BBEB
P 8550 1950
F 0 "#PWR0105" H 8550 1700 50  0001 C CNN
F 1 "GND" V 8555 1822 50  0000 R CNN
F 2 "" H 8550 1950 50  0001 C CNN
F 3 "" H 8550 1950 50  0001 C CNN
	1    8550 1950
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5F43BCDF
P 9500 750
F 0 "#PWR0106" H 9500 600 50  0001 C CNN
F 1 "+5V" H 9515 923 50  0000 C CNN
F 2 "" H 9500 750 50  0001 C CNN
F 3 "" H 9500 750 50  0001 C CNN
	1    9500 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5F43C4E3
P 9050 1150
F 0 "#PWR0107" H 9050 900 50  0001 C CNN
F 1 "GND" V 9055 1022 50  0000 R CNN
F 2 "" H 9050 1150 50  0001 C CNN
F 3 "" H 9050 1150 50  0001 C CNN
	1    9050 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5F43CADA
P 9050 1450
F 0 "#PWR0108" H 9050 1200 50  0001 C CNN
F 1 "GND" V 9055 1322 50  0000 R CNN
F 2 "" H 9050 1450 50  0001 C CNN
F 3 "" H 9050 1450 50  0001 C CNN
	1    9050 1450
	0    -1   -1   0   
$EndComp
Text Label 8250 1250 2    50   ~ 0
LOW1
Text Label 8550 1350 2    50   ~ 0
LOW2
Text Label 8550 1650 2    50   ~ 0
LOW4
Text Label 8550 1750 2    50   ~ 0
LOW5
Text Label 9050 1650 0    50   ~ 0
LOW7
Text Label 10350 1250 2    50   ~ 0
LOW9
Text Label 8550 1850 2    50   ~ 0
LOW6
Text Label 9050 1750 0    50   ~ 0
LOW8
Text Label 9050 1850 0    50   ~ 0
LEDA0
Text Label 9050 1950 0    50   ~ 0
LEDA1
Text Label 9050 2050 0    50   ~ 0
LEDA3
Text Label 9050 2150 0    50   ~ 0
LEDA5
Text Label 9050 2250 0    50   ~ 0
LEDA7
Text Label 8550 2050 2    50   ~ 0
LEDA2
Text Label 8550 2150 2    50   ~ 0
LEDA4
Text Label 8550 2250 2    50   ~ 0
LEDA6
Text Label 10050 1350 2    50   ~ 0
LOW10
Text Label 10350 1950 2    50   ~ 0
HIGH0
Text Label 10350 2050 2    50   ~ 0
HIGH1
Text Label 10350 2150 2    50   ~ 0
HIGH2
Text Label 10850 1750 0    50   ~ 0
HIGH4
Text Label 10850 1950 0    50   ~ 0
HIGH6
Text Label 10850 1650 0    50   ~ 0
HIGH3
Text Label 10850 1850 0    50   ~ 0
HIGH5
Text Label 10850 2050 0    50   ~ 0
HIGH7
Text Label 9050 1250 0    50   ~ 0
USBD+
Text Label 9050 1350 0    50   ~ 0
USBD-
$Comp
L MCU_ST_STM32L4:STM32L433CBTx U101
U 1 1 5F442F3D
P 9700 4750
F 0 "U101" H 9700 4550 50  0000 C CNN
F 1 "STM32L422CBTx" H 9700 4450 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 9200 3350 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00257211.pdf" H 9700 4750 50  0001 C CNN
	1    9700 4750
	1    0    0    -1  
$EndComp
Text Label 9100 6050 2    50   ~ 0
LOW0
Text Label 9100 5850 2    50   ~ 0
LOW1
Text Label 9100 5950 2    50   ~ 0
LOW2
Text Label 9100 5750 2    50   ~ 0
LOW3
Text Label 9100 5650 2    50   ~ 0
LOW4
Text Label 9100 4750 2    50   ~ 0
LOW5
Text Label 9100 4550 2    50   ~ 0
LOW6
Text Label 9100 4650 2    50   ~ 0
LOW8
Text Label 9100 4950 2    50   ~ 0
LOW9
Text Label 9100 4850 2    50   ~ 0
LOW10
Text Label 10300 5750 0    50   ~ 0
USBD+
Text Label 10350 1450 2    50   ~ 0
LOW11
Text Label 10350 1650 2    50   ~ 0
LOW12
Text Label 10850 1350 0    50   ~ 0
LOW13
Text Label 10850 1450 0    50   ~ 0
LOW14
Text Label 10850 1550 0    50   ~ 0
LOW15
$Comp
L power:GND #PWR0109
U 1 1 5F44D418
P 10850 2150
F 0 "#PWR0109" H 10850 1900 50  0001 C CNN
F 1 "GND" V 10855 2022 50  0000 R CNN
F 2 "" H 10850 2150 50  0001 C CNN
F 3 "" H 10850 2150 50  0001 C CNN
	1    10850 2150
	0    -1   -1   0   
$EndComp
Text Notes 8900 3000 0    50   ~ 0
There are 19 physical sense lines, but most\ncolumns are sparse. By unifying two sets\nof physical lines we can reduce the required\npins to 16.\n\nLOW1 includes INS and LSHIF cols.\n\nLOW10 includes LCTL and both SUP cols.
Text Label 10300 5650 0    50   ~ 0
USBD-
Text Label 10300 5850 0    50   ~ 0
SWDIO
Text Label 10300 5950 0    50   ~ 0
SWCLK
Text Label 8550 1550 2    50   ~ 0
LOW3
Text Label 8550 1150 2    50   ~ 0
LOW0
Wire Wire Line
	8550 1250 8300 1250
Wire Wire Line
	8550 1450 8300 1450
Wire Wire Line
	8300 1450 8300 1250
Connection ~ 8300 1250
Wire Wire Line
	8300 1250 8250 1250
Wire Wire Line
	10350 1350 10150 1350
Wire Wire Line
	10150 1350 10150 850 
Wire Wire Line
	10150 850  10850 850 
Wire Wire Line
	10850 850  10850 1150
Connection ~ 10150 1350
Wire Wire Line
	10150 1350 10050 1350
Connection ~ 10850 1150
Wire Wire Line
	10850 1150 10850 1250
Text Label 9100 5150 2    50   ~ 0
LOW14
Text Label 9100 5250 2    50   ~ 0
LOW11
Text Label 9100 5350 2    50   ~ 0
LOW15
Text Label 9100 5450 2    50   ~ 0
LOW12
Text Label 9100 5550 2    50   ~ 0
LOW7
NoConn ~ 9100 3750
NoConn ~ 9100 3850
NoConn ~ 10300 6050
Text Label 9100 5050 2    50   ~ 0
LOW13
Wire Wire Line
	9500 6250 9600 6250
Connection ~ 9600 6250
Wire Wire Line
	9600 6250 9650 6250
Connection ~ 9700 6250
Wire Wire Line
	9700 6250 9800 6250
$Comp
L power:GND #PWR0110
U 1 1 5F468E46
P 9650 6250
F 0 "#PWR0110" H 9650 6000 50  0001 C CNN
F 1 "GND" H 9655 6077 50  0000 C CNN
F 2 "" H 9650 6250 50  0001 C CNN
F 3 "" H 9650 6250 50  0001 C CNN
	1    9650 6250
	1    0    0    -1  
$EndComp
Connection ~ 9650 6250
Wire Wire Line
	9650 6250 9700 6250
Wire Wire Line
	9900 3350 9800 3350
Connection ~ 9600 3350
Wire Wire Line
	9600 3350 9500 3350
Connection ~ 9700 3350
Wire Wire Line
	9700 3350 9600 3350
Connection ~ 9800 3350
Wire Wire Line
	9800 3350 9700 3350
$Comp
L power:+3V3 #PWR0111
U 1 1 5F4697D5
P 9700 3350
F 0 "#PWR0111" H 9700 3200 50  0001 C CNN
F 1 "+3V3" H 9715 3523 50  0000 C CNN
F 2 "" H 9700 3350 50  0001 C CNN
F 3 "" H 9700 3350 50  0001 C CNN
	1    9700 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_ARM_JTAG_SWD_10 J101
U 1 1 5F469C3E
P 7300 5550
F 0 "J101" H 6857 5596 50  0000 R CNN
F 1 "Conn_ARM_JTAG_SWD_10" H 6857 5505 50  0001 R CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD" H 7300 5550 50  0001 C CNN
F 3 "http://infocenter.arm.com/help/topic/com.arm.doc.ddi0314h/DDI0314H_coresight_components_trm.pdf" V 6950 4300 50  0001 C CNN
	1    7300 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0112
U 1 1 5F46A612
P 7300 4950
F 0 "#PWR0112" H 7300 4800 50  0001 C CNN
F 1 "+3V3" H 7315 5123 50  0000 C CNN
F 2 "" H 7300 4950 50  0001 C CNN
F 3 "" H 7300 4950 50  0001 C CNN
	1    7300 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5F46A9A4
P 7300 6150
F 0 "#PWR0113" H 7300 5900 50  0001 C CNN
F 1 "GND" H 7305 5977 50  0000 C CNN
F 2 "" H 7300 6150 50  0001 C CNN
F 3 "" H 7300 6150 50  0001 C CNN
	1    7300 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 6150 7200 6150
Connection ~ 7300 6150
Text Label 7800 5450 0    50   ~ 0
SWCLK
Text Label 7800 5550 0    50   ~ 0
SWDIO
NoConn ~ 7800 5750
NoConn ~ 7800 5650
Text Label 9100 3550 2    50   ~ 0
~RESET
Text Label 10300 5450 0    50   ~ 0
I2C1_SCL
Text Label 10300 5550 0    50   ~ 0
I2C1_SDA
Text Label 10300 6050 0    50   ~ 0
TIM2_CH1
Wire Wire Line
	8550 2050 8300 2050
Wire Wire Line
	8300 2050 8300 2150
Wire Wire Line
	8300 2250 8550 2250
Wire Wire Line
	8550 2150 8300 2150
Connection ~ 8300 2150
Wire Wire Line
	8300 2150 8300 2250
Wire Wire Line
	9050 2050 9300 2050
Wire Wire Line
	9300 2050 9300 2150
Wire Wire Line
	9300 2250 9050 2250
Wire Wire Line
	9050 2150 9300 2150
Connection ~ 9300 2150
Wire Wire Line
	9300 2150 9300 2250
$Comp
L Device:C_Small C102
U 1 1 5F46FB7C
P 5000 2500
F 0 "C102" H 5092 2546 50  0000 L CNN
F 1 "0µ1" H 5092 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5000 2500 50  0001 C CNN
F 3 "~" H 5000 2500 50  0001 C CNN
	1    5000 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C103
U 1 1 5F4700D2
P 5350 2500
F 0 "C103" H 5442 2546 50  0000 L CNN
F 1 "0µ1" H 5442 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5350 2500 50  0001 C CNN
F 3 "~" H 5350 2500 50  0001 C CNN
	1    5350 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C104
U 1 1 5F4702FF
P 5700 2500
F 0 "C104" H 5792 2546 50  0000 L CNN
F 1 "0µ1" H 5792 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5700 2500 50  0001 C CNN
F 3 "~" H 5700 2500 50  0001 C CNN
	1    5700 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C106
U 1 1 5F470A8B
P 6400 2500
F 0 "C106" H 6492 2546 50  0000 L CNN
F 1 "0µ1" H 6492 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6400 2500 50  0001 C CNN
F 3 "~" H 6400 2500 50  0001 C CNN
	1    6400 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C107
U 1 1 5F470E2F
P 6750 2500
F 0 "C107" H 6838 2546 50  0000 L CNN
F 1 "10µ" H 6838 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6750 2500 50  0001 C CNN
F 3 "~" H 6750 2500 50  0001 C CNN
	1    6750 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2400 5350 2400
Connection ~ 5350 2400
Wire Wire Line
	5350 2400 5700 2400
Connection ~ 5700 2400
Wire Wire Line
	5700 2400 5900 2400
Connection ~ 6400 2400
Wire Wire Line
	6400 2400 6750 2400
Wire Wire Line
	6750 2600 6400 2600
Connection ~ 5350 2600
Wire Wire Line
	5350 2600 5000 2600
Connection ~ 5700 2600
Wire Wire Line
	5700 2600 5350 2600
Connection ~ 6400 2600
$Comp
L power:+3V3 #PWR0114
U 1 1 5F472393
P 5900 2400
F 0 "#PWR0114" H 5900 2250 50  0001 C CNN
F 1 "+3V3" H 5915 2573 50  0000 C CNN
F 2 "" H 5900 2400 50  0001 C CNN
F 3 "" H 5900 2400 50  0001 C CNN
	1    5900 2400
	1    0    0    -1  
$EndComp
Connection ~ 5900 2400
$Comp
L power:GND #PWR0115
U 1 1 5F4729EF
P 5900 2600
F 0 "#PWR0115" H 5900 2350 50  0001 C CNN
F 1 "GND" H 5905 2427 50  0000 C CNN
F 2 "" H 5900 2600 50  0001 C CNN
F 3 "" H 5900 2600 50  0001 C CNN
	1    5900 2600
	1    0    0    -1  
$EndComp
Connection ~ 5900 2600
Wire Wire Line
	5900 2600 5700 2600
$Comp
L Device:C_Small C101
U 1 1 5F472F11
P 8200 5350
F 0 "C101" H 8292 5396 50  0000 L CNN
F 1 "0µ1" H 8292 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8200 5350 50  0001 C CNN
F 3 "~" H 8200 5350 50  0001 C CNN
	1    8200 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 5250 8200 5250
Connection ~ 8200 5250
Wire Wire Line
	8200 5250 8400 5250
$Comp
L power:GND #PWR0116
U 1 1 5F474051
P 8200 5450
F 0 "#PWR0116" H 8200 5200 50  0001 C CNN
F 1 "GND" H 8205 5277 50  0000 C CNN
F 2 "" H 8200 5450 50  0001 C CNN
F 3 "" H 8200 5450 50  0001 C CNN
	1    8200 5450
	1    0    0    -1  
$EndComp
Text Notes 9600 1150 0    50   ~ 0
stat LED A>
Wire Wire Line
	8300 2150 8250 2150
Text Label 8250 2150 2    50   ~ 0
BACKLIGHT_ANODES
$Comp
L power:+5V #PWR0117
U 1 1 5F47B514
P 2450 850
F 0 "#PWR0117" H 2450 700 50  0001 C CNN
F 1 "+5V" H 2465 1023 50  0000 C CNN
F 2 "" H 2450 850 50  0001 C CNN
F 3 "" H 2450 850 50  0001 C CNN
	1    2450 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1250 2450 1350
Wire Wire Line
	2450 1350 2500 1350
Text Label 2500 1350 0    50   ~ 0
BACKLIGHT_ANODES
$Comp
L Regulator_Linear:MIC5205-3.3YM5 U102
U 1 1 5F47D4A6
P 5600 1150
F 0 "U102" H 5600 1492 50  0000 C CNN
F 1 "MIC5205-3.3YM5" H 5600 1401 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5600 1475 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005785A.pdf" H 5600 1150 50  0001 C CNN
	1    5600 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1150 5300 1050
$Comp
L power:+5V #PWR0118
U 1 1 5F47FD77
P 5300 1050
F 0 "#PWR0118" H 5300 900 50  0001 C CNN
F 1 "+5V" V 5315 1178 50  0000 L CNN
F 2 "" H 5300 1050 50  0001 C CNN
F 3 "" H 5300 1050 50  0001 C CNN
	1    5300 1050
	0    -1   -1   0   
$EndComp
Connection ~ 5300 1050
$Comp
L Device:C_Small C108
U 1 1 5F47FE65
P 5950 1250
F 0 "C108" H 6042 1296 50  0000 L CNN
F 1 "470p" H 6042 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5950 1250 50  0001 C CNN
F 3 "~" H 5950 1250 50  0001 C CNN
	1    5950 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1150 5950 1150
Wire Wire Line
	5950 1350 5950 1450
Wire Wire Line
	5950 1450 5600 1450
$Comp
L Device:CP_Small C109
U 1 1 5F4826E1
P 6300 1250
F 0 "C109" H 6388 1296 50  0000 L CNN
F 1 "2µ2 tant" H 6388 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6300 1250 50  0001 C CNN
F 3 "~" H 6300 1250 50  0001 C CNN
	1    6300 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 1350 6300 1450
Connection ~ 5950 1450
$Comp
L power:GND #PWR0119
U 1 1 5F48544D
P 5950 1450
F 0 "#PWR0119" H 5950 1200 50  0001 C CNN
F 1 "GND" H 5955 1277 50  0000 C CNN
F 2 "" H 5950 1450 50  0001 C CNN
F 3 "" H 5950 1450 50  0001 C CNN
	1    5950 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 1150 6300 1050
$Comp
L power:+3V3 #PWR0120
U 1 1 5F486C57
P 6300 1050
F 0 "#PWR0120" H 6300 900 50  0001 C CNN
F 1 "+3V3" H 6315 1223 50  0000 C CNN
F 2 "" H 6300 1050 50  0001 C CNN
F 3 "" H 6300 1050 50  0001 C CNN
	1    6300 1050
	1    0    0    -1  
$EndComp
Connection ~ 6300 1050
$Comp
L Device:Q_PNP_BEC Q102
U 1 1 5F488383
P 2350 1050
F 0 "Q102" H 2540 1004 50  0000 L CNN
F 1 "ZXTP25020CFFTA" H 2540 1095 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2550 1150 50  0001 C CNN
F 3 "~" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    1   
$EndComp
$Comp
L Device:R R102
U 1 1 5F48A395
P 2200 850
F 0 "R102" V 1993 850 50  0000 C CNN
F 1 "1k" V 2084 850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2130 850 50  0001 C CNN
F 3 "~" H 2200 850 50  0001 C CNN
	1    2200 850 
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 850  2450 850 
Connection ~ 2450 850 
Wire Wire Line
	2050 850  1900 850 
Wire Wire Line
	1900 850  1900 1050
Wire Wire Line
	1900 1050 2150 1050
$Comp
L Device:Q_NPN_BEC Q101
U 1 1 5F48E1BA
P 1800 1250
F 0 "Q101" H 1991 1296 50  0000 L CNN
F 1 "ZXTN08400BFF" H 1991 1205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2000 1350 50  0001 C CNN
F 3 "~" H 1800 1250 50  0001 C CNN
	1    1800 1250
	1    0    0    -1  
$EndComp
Connection ~ 1900 1050
$Comp
L Device:R R101
U 1 1 5F48E8F4
P 1450 1250
F 0 "R101" V 1243 1250 50  0000 C CNN
F 1 "1k" V 1334 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1380 1250 50  0001 C CNN
F 3 "~" H 1450 1250 50  0001 C CNN
	1    1450 1250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5F48F37F
P 1900 1450
F 0 "#PWR0121" H 1900 1200 50  0001 C CNN
F 1 "GND" H 1905 1277 50  0000 C CNN
F 2 "" H 1900 1450 50  0001 C CNN
F 3 "" H 1900 1450 50  0001 C CNN
	1    1900 1450
	1    0    0    -1  
$EndComp
Text Label 1300 1250 2    50   ~ 0
BACKLIGHT_PWM
$Comp
L Device:R R103
U 1 1 5F491049
P 10000 1850
F 0 "R103" V 10100 1850 50  0000 C CNN
F 1 "180" V 10000 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9930 1850 50  0001 C CNN
F 3 "~" H 10000 1850 50  0001 C CNN
	1    10000 1850
	0    1    1    0   
$EndComp
$Comp
L Device:R R104
U 1 1 5F4910E3
P 10000 1750
F 0 "R104" V 9900 1750 50  0000 C CNN
F 1 "180" V 10000 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9930 1750 50  0001 C CNN
F 3 "~" H 10000 1750 50  0001 C CNN
	1    10000 1750
	0    1    1    0   
$EndComp
Text Label 9850 1850 2    50   ~ 0
STATUS0
Text Label 9850 1750 2    50   ~ 0
STATUS1
Wire Wire Line
	10350 1850 10150 1850
Wire Wire Line
	10150 1750 10350 1750
Wire Wire Line
	9300 2250 9300 2350
Wire Wire Line
	9300 2350 8300 2350
Wire Wire Line
	8300 2350 8300 2250
Connection ~ 9300 2250
Connection ~ 8300 2250
Text Label 9100 3950 2    50   ~ 0
BOOT0
$Comp
L Jumper:SolderJumper_2_Open JP101
U 1 1 5F4B5F79
P 8350 3800
F 0 "JP101" V 8304 3868 50  0000 L CNN
F 1 "SolderJumper_2_Open" V 8395 3868 50  0001 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 8350 3800 50  0001 C CNN
F 3 "~" H 8350 3800 50  0001 C CNN
	1    8350 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	8350 3950 9100 3950
$Comp
L Device:R R105
U 1 1 5F4B8B68
P 8350 4100
F 0 "R105" H 8420 4146 50  0000 L CNN
F 1 "10k" H 8420 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8280 4100 50  0001 C CNN
F 3 "~" H 8350 4100 50  0001 C CNN
	1    8350 4100
	1    0    0    -1  
$EndComp
Connection ~ 8350 3950
$Comp
L power:GND #PWR0122
U 1 1 5F4B9232
P 8350 4250
F 0 "#PWR0122" H 8350 4000 50  0001 C CNN
F 1 "GND" H 8355 4077 50  0000 C CNN
F 2 "" H 8350 4250 50  0001 C CNN
F 3 "" H 8350 4250 50  0001 C CNN
	1    8350 4250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0123
U 1 1 5F4B97B8
P 8350 3650
F 0 "#PWR0123" H 8350 3500 50  0001 C CNN
F 1 "+3V3" H 8365 3823 50  0000 C CNN
F 2 "" H 8350 3650 50  0001 C CNN
F 3 "" H 8350 3650 50  0001 C CNN
	1    8350 3650
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F4C07B6
P 9500 850
F 0 "#FLG0102" H 9500 925 50  0001 C CNN
F 1 "PWR_FLAG" V 9500 978 50  0000 L CNN
F 2 "" H 9500 850 50  0001 C CNN
F 3 "~" H 9500 850 50  0001 C CNN
	1    9500 850 
	0    1    1    0   
$EndComp
Wire Wire Line
	9500 750  9500 850 
Wire Wire Line
	9500 1550 9050 1550
Connection ~ 9500 850 
Wire Wire Line
	9500 850  9500 1550
Wire Wire Line
	5900 1050 6300 1050
Wire Wire Line
	9050 1850 9300 1850
Wire Wire Line
	9300 1850 9300 1950
Connection ~ 9300 2050
Wire Wire Line
	9050 1950 9300 1950
Connection ~ 9300 1950
Wire Wire Line
	9300 1950 9300 2050
Text Label 10300 4550 0    50   ~ 0
STATUS1
Text Label 10300 4650 0    50   ~ 0
STATUS0
Text Label 10300 5250 0    50   ~ 0
BACKLIGHT_PWM
NoConn ~ 9100 4350
Wire Wire Line
	5900 2400 6400 2400
Wire Wire Line
	5900 2600 6400 2600
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J102
U 1 1 5F3772D3
P 8300 6050
F 0 "J102" H 8350 6367 50  0000 C CNN
F 1 "TC2030" H 8350 6276 50  0000 C CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 8300 6050 50  0001 C CNN
F 3 "~" H 8300 6050 50  0001 C CNN
	1    8300 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0124
U 1 1 5F377C4B
P 8100 5950
F 0 "#PWR0124" H 8100 5800 50  0001 C CNN
F 1 "+3V3" H 8115 6123 50  0000 C CNN
F 2 "" H 8100 5950 50  0001 C CNN
F 3 "" H 8100 5950 50  0001 C CNN
	1    8100 5950
	0    -1   -1   0   
$EndComp
Text Label 8600 5950 0    50   ~ 0
SWDIO
Text Label 8400 5250 0    50   ~ 0
~RESET
Text Label 8100 6050 2    50   ~ 0
~RESET
Text Label 8600 6050 0    50   ~ 0
SWCLK
$Comp
L power:GND #PWR0125
U 1 1 5F37878B
P 8100 6150
F 0 "#PWR0125" H 8100 5900 50  0001 C CNN
F 1 "GND" H 8105 5977 50  0000 C CNN
F 2 "" H 8100 6150 50  0001 C CNN
F 3 "" H 8100 6150 50  0001 C CNN
	1    8100 6150
	1    0    0    -1  
$EndComp
NoConn ~ 8600 6150
Wire Notes Line
	11200 3100 6650 3100
Wire Notes Line
	6650 3100 6650 6500
Wire Notes Line
	6650 6500 11200 6500
Text Notes 6700 3250 0    50   ~ 10
CPU, Debug
Wire Notes Line
	7500 3100 7500 500 
Wire Notes Line
	7500 500  11200 500 
Wire Notes Line
	11200 500  11200 6500
Text Notes 7550 650  0    50   ~ 10
Keyboard Connectors
Text Notes 700  6150 2    50   ~ 10
0
Text Notes 700  6300 2    50   ~ 10
1
Text Notes 700  6450 2    50   ~ 10
2
Text Notes 700  6600 2    50   ~ 10
3
Text Notes 700  6750 2    50   ~ 10
4
Text Notes 700  6900 2    50   ~ 10
5
Text Notes 700  7050 2    50   ~ 10
6
Text Notes 700  7200 2    50   ~ 10
7
Text Notes 850  6150 0    50   ~ 0
PGDN
Text Notes 850  7200 0    50   ~ 0
PGUP
Text Notes 1150 6300 0    50   ~ 0
RT
Text Notes 1150 7200 0    50   ~ 0
INS
Text Notes 850  6000 0    50   ~ 10
0
Text Notes 1400 6750 0    50   ~ 0
RSHIF
Text Notes 1400 6900 0    50   ~ 0
LSHIF
Text Notes 1700 6300 0    50   ~ 0
DN
Text Notes 1700 6600 0    50   ~ 0
SPACE
Text Notes 1700 7200 0    50   ~ 0
DEL
Text Notes 2000 6150 0    50   ~ 0
7
Text Notes 2000 6300 0    50   ~ 0
N
Text Notes 2000 6450 0    50   ~ 0
M
Text Notes 2000 6600 0    50   ~ 0
H
Text Notes 2000 6750 0    50   ~ 0
J
Text Notes 2000 6900 0    50   ~ 0
Y
Text Notes 2000 7050 0    50   ~ 0
U
Text Notes 2300 6150 0    50   ~ 0
F10
Text Notes 2300 6300 0    50   ~ 0
F12
Text Notes 2300 6450 0    50   ~ 0
ENT
Text Notes 2300 6600 0    50   ~ 0
F11
Text Notes 2300 6750 0    50   ~ 0
\
Text Notes 2300 6900 0    50   ~ 0
BSP
Text Notes 2300 7200 0    50   ~ 0
F9
Text Notes 2650 6150 0    50   ~ 0
0
Text Notes 2650 6300 0    50   ~ 0
/
Text Notes 2650 6600 0    50   ~ 0
(')
Text Notes 2650 6750 0    50   ~ 0
;
Text Notes 2650 6900 0    50   ~ 0
[
Text Notes 2650 7050 0    50   ~ 0
P
Text Notes 2650 7200 0    50   ~ 0
(-)
Text Notes 2950 6150 0    50   ~ 0
9
Text Notes 2950 6450 0    50   ~ 0
(.)
Text Notes 2950 6750 0    50   ~ 0
L
Text Notes 2950 6900 0    50   ~ 0
F7
Text Notes 2950 7050 0    50   ~ 0
O
Text Notes 2950 7200 0    50   ~ 0
F8
Text Notes 3250 6150 0    50   ~ 0
8
Text Notes 3250 6450 0    50   ~ 0
(,)
Text Notes 3250 6600 0    50   ~ 0
F6
Text Notes 3250 6750 0    50   ~ 0
K
Text Notes 3250 6900 0    50   ~ 0
]
Text Notes 3250 7050 0    50   ~ 0
i
Text Notes 3250 7200 0    50   ~ 0
=
Text Notes 3550 6150 0    50   ~ 0
END
Text Notes 3550 6300 0    50   ~ 0
LT
Text Notes 3550 6600 0    50   ~ 0
UP
Text Notes 3550 7200 0    50   ~ 0
HOM
Text Notes 3900 6150 0    50   ~ 0
4
Text Notes 3900 6300 0    50   ~ 0
B
Text Notes 3900 6450 0    50   ~ 0
V
Text Notes 3900 6600 0    50   ~ 0
G
Text Notes 3900 6750 0    50   ~ 0
F
Text Notes 3900 6900 0    50   ~ 0
T
Text Notes 3900 7050 0    50   ~ 0
R
Text Notes 3900 7200 0    50   ~ 0
5
Text Notes 4200 6150 0    50   ~ 0
F5
Text Notes 4200 6450 0    50   ~ 0
RCTL
Text Notes 4200 7050 0    50   ~ 0
PAU
Text Notes 4200 7200 0    50   ~ 0
LCTL
Text Notes 4500 6750 0    50   ~ 0
RSUP
Text Notes 4800 6900 0    50   ~ 0
LSUP
Text Notes 5100 6150 0    50   ~ 0
PRTS
Text Notes 5100 6300 0    50   ~ 0
RALT
Text Notes 5100 6600 0    50   ~ 0
LALT
Text Notes 5100 6900 0    50   ~ 0
MENU
Text Notes 5100 7050 0    50   ~ 0
SCLK
Text Notes 5500 6150 0    50   ~ 0
DIP1
Text Notes 5500 7200 0    50   ~ 0
DIP2
Text Notes 5500 6300 0    50   ~ 0
DIP3
Text Notes 5500 6450 0    50   ~ 0
DIP4
Text Notes 5500 6600 0    50   ~ 0
DIP5
Text Notes 5500 6750 0    50   ~ 0
DIP6
Text Notes 5900 6150 0    50   ~ 0
3
Text Notes 5900 6450 0    50   ~ 0
C
Text Notes 5900 6600 0    50   ~ 0
F4
Text Notes 5900 6750 0    50   ~ 0
D
Text Notes 5900 6900 0    50   ~ 0
F3
Text Notes 5900 7050 0    50   ~ 0
E
Text Notes 5900 7200 0    50   ~ 0
F2
Text Notes 6200 6150 0    50   ~ 0
2
Text Notes 6200 6450 0    50   ~ 0
X
Text Notes 6200 6600 0    50   ~ 0
K88
Text Notes 6200 6750 0    50   ~ 0
S
Text Notes 6200 6900 0    50   ~ 0
CAPS
Text Notes 6200 7050 0    50   ~ 0
W
Text Notes 6200 7200 0    50   ~ 0
F1
Text Notes 6450 6150 0    50   ~ 0
1
Text Notes 6450 6450 0    50   ~ 0
Z
Text Notes 6450 6600 0    50   ~ 0
ESC
Text Notes 6450 6750 0    50   ~ 0
A
Text Notes 6450 6900 0    50   ~ 0
TAB
Text Notes 6450 7050 0    50   ~ 0
Q
Text Notes 6450 7200 0    50   ~ 0
(`)
Text Notes 1150 6000 0    50   ~ 10
1
Text Notes 1700 6000 0    50   ~ 10
2
Text Notes 2000 6000 0    50   ~ 10
3
Text Notes 2300 6000 0    50   ~ 10
4
Text Notes 2600 6000 0    50   ~ 10
5
Text Notes 2950 6000 0    50   ~ 10
6
Text Notes 3250 6000 0    50   ~ 10
7
Text Notes 3550 6000 0    50   ~ 10
8
Text Notes 3900 6000 0    50   ~ 10
9
Text Notes 4200 6000 0    50   ~ 10
10
Text Notes 5100 6000 0    50   ~ 10
11
Text Notes 5500 6000 0    50   ~ 10
12
Text Notes 5900 6000 0    50   ~ 10
13
Text Notes 6200 6000 0    50   ~ 10
14
Text Notes 6450 6000 0    50   ~ 10
15
Wire Notes Line
	1150 6050 1650 6050
Wire Notes Line
	4200 6050 5050 6050
Text Notes 850  5900 0    50   ~ 10
LOW
Text Notes 600  6250 1    50   ~ 10
HIGH
Text Notes 1700 7350 0    50   ~ 10
Note that LOW1 and LOW10 combine multiple sparse physical columns to save I/Os.
$Bitmap
Pos 1500 4850
Scale 1.000000
Data
89 50 4E 47 0D 0A 1A 0A 00 00 00 0D 49 48 44 52 00 00 02 04 00 00 01 6A 08 02 00 00 00 EA 76 D5 
76 00 00 00 03 73 42 49 54 08 08 08 DB E1 4F E0 00 00 0C 23 49 44 41 54 78 9C ED DD DD B6 A2 BA 
12 06 D0 E2 8C 7E 6B AF 7D 6E CE 85 BD DD 6E 51 48 02 E4 07 E7 BC EA 5E 0B 51 47 8F AE 8F 4A 42 
98 E6 79 0E 00 7E DB FF 5A 7F 00 00 DA 13 06 00 08 03 00 84 01 00 21 0C 00 88 88 3F B9 2F 98 A6 
E9 F9 E7 94 95 48 B9 C7 03 50 5F 5E 67 F0 5A D9 97 7F DD 7F 3C 00 4D EC 1D 26 5A A9 EF 4A 3F C0 
28 B2 87 89 96 1E 45 FF 75 08 48 0C 00 8C E5 B0 09 E4 67 00 48 02 80 E1 4C B9 93 BA 7B 6A BD 09 
64 80 3E 65 87 C1 DF 97 65 46 82 18 00 E8 59 E1 30 51 56 71 97 04 00 9D 2B 9F 33 48 2C F1 92 00 
A0 7F EE 40 06 60 47 18 24 4E 1B 58 5C 04 D0 BF C2 30 C8 2A F1 F2 00 A0 73 96 96 02 B0 6F 6F A2 
37 F3 3C AF 97 7B 2D 02 40 9F 0E 9B 40 7E C6 80 CB 7F 80 E1 1C B0 37 D1 B2 FA 3F 7E A2 0F 00 18 
C5 DE CE 60 A5 0F D0 22 00 8C 22 2F 0C DE EA FB 66 B9 CF 3D 1E 80 26 0A F7 26 02 E0 4A DC 81 0C 
80 30 00 60 65 35 D1 19 6B 81 0C 49 01 F4 E9 73 67 70 D2 AA 50 8B 4D 01 FA 64 98 08 00 61 00 80 
30 00 20 BE 85 C1 49 33 BD 26 90 01 FA 54 78 D3 D9 72 2A 58 A1 07 18 97 61 22 00 84 01 00 C2 00 
80 10 06 00 84 30 00 20 84 01 00 B1 B2 B4 D4 46 75 00 BF C3 46 75 00 18 26 02 40 18 00 10 C2 00 
80 B0 51 1D 00 51 BC 51 1D 00 57 62 98 08 00 61 00 80 30 00 20 22 FE EC 3F C5 E6 AD 64 A6 25 00 
3A B7 31 81 FC 2C F4 3B 77 AD 90 07 00 3D 5B 1B 26 7A 2D F4 3B 77 92 B0 11 05 40 CF BE 86 81 F2 
0D F0 3B EA 4D 20 4B 17 80 6E 55 DD B5 14 80 3E A5 76 06 E9 33 C0 F3 3F 96 BF 92 31 00 7D 72 9F 
01 00 67 86 81 E5 A4 00 A3 D0 19 00 B0 3B 0C DE 2E FF 5F FF 6A 86 00 60 14 A9 DB 51 4C D3 D7 7B 
95 0D 07 01 8C 2E E3 E1 36 59 57 FA 1F 0F 16 1B 00 7D CA 1B 26 4A CC 03 49 00 30 96 EC 5D 4B 1F 
85 7E E7 BE 75 00 74 65 6D D7 D2 03 2B BB B6 00 A0 67 6B C3 44 47 55 70 49 00 D0 B9 8D 39 03 75 
1C E0 17 6C 4F 20 EF CC 03 71 02 D0 BF 8D 27 9D FD E7 D0 CC 29 04 31 00 30 8A 8C 30 F8 CF CB BE 
04 83 00 00 18 51 61 18 00 70 25 36 AA 03 40 18 00 20 0C 00 88 95 ED 28 CE D8 58 C2 FC 04 40 9F 
3E 77 06 27 6D 31 64 E7 22 80 3E 19 26 02 40 18 00 20 0C 00 88 AC 27 9D ED 67 02 19 A0 4F EE 40 
06 C0 30 11 00 C2 00 80 10 06 00 84 30 00 20 6C 47 01 40 D8 8E 02 80 30 4C 04 40 08 03 00 42 18 
00 10 B6 A3 00 20 8A B7 A3 58 4E 05 2B F4 00 E3 32 4C 04 80 30 00 40 18 00 10 C2 00 80 10 06 00 
84 30 00 20 56 96 96 DA A8 0E E0 77 D8 A8 0E 00 C3 44 00 08 03 00 42 18 00 10 36 AA 03 20 8A 37 
AA 03 E0 4A 0C 13 01 20 0C 00 10 06 00 44 C4 9F 26 EF 3A DD FF DE 7D 36 DF CC 58 00 B4 A7 33 00 
40 18 00 20 0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 
00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 40 
08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 40 08 03 00 42 18 
00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 40 44 FC 69 FD 01 86 37 DD EF 1F 
7F 3E DF 6E EB C7 7F 3B 20 E5 BD 3E BE F6 DB 27 29 7B 2F E0 A7 E8 0C 76 59 A9 BF 2B BF AA FC 49 
36 7F 0B 20 0C 0A 4D F7 FB 66 85 4D 39 E6 A8 0F 73 C8 31 C0 CF 32 4C 54 E2 AD B0 2E 07 61 12 AB 
F3 19 A3 37 6F E7 94 01 40 0A 9D C1 5E 1F 0B FA 7C BB 7D 2B F4 C5 01 B0 39 D9 F0 F1 4D 5F 7F 22 
18 80 6F 74 06 D9 5E 4B EA 7A 65 AF 36 6D BB F9 31 C4 00 B0 4E 67 50 CE 12 1D E0 32 84 41 4B CB 
0B F6 C7 9C B3 0B 79 A0 32 61 D0 40 E5 96 42 B4 00 9B 84 41 B9 9A 45 B6 F8 56 B5 F4 19 0E E0 97 
09 83 6C 75 D6 E7 EC 3C B3 E1 26 20 8B 30 D8 6B 67 CD 7D 7D ED 51 B5 FB E3 79 B4 05 C0 0A 4B 4B 
4B 2C 17 6B E6 0E E3 54 5B EE 29 03 80 14 3A 83 42 8F 3B BC 3E DE 7B 7C F8 05 7E F1 09 25 01 90 
48 18 EC 75 76 24 BC BD D7 E6 31 7B 76 45 05 7E 96 30 38 C6 B7 48 48 79 ED E3 30 45 1C 68 48 18 
1C 69 65 4B A2 8F 07 9F FA 61 00 D2 09 83 E3 ED 5F 7B FA 3C 83 B5 A1 40 1D 56 13 9D E2 C0 C5 42 
05 EB 94 0E 79 5F E0 A7 E8 0C BA 60 C2 00 68 4B 18 B4 A4 F4 03 9D 10 06 A7 D8 3F 46 B4 E7 19 38 
36 A2 00 72 09 83 73 B9 F6 07 86 20 0C B2 6D 5E 7A 9F 71 C3 70 62 A8 9C B1 D3 11 F0 0B AC 26 2A 
B7 59 6D 53 2A F8 CA BA 23 8F AB 04 AA D1 19 00 A0 33 C8 97 72 47 58 AB A9 82 D7 66 C2 74 05 90 
6E 9A E7 B9 C1 BB DE A7 C7 1F E6 5B 83 77 07 E0 8D 61 22 00 84 01 00 C2 00 80 10 06 00 84 30 00 
20 84 01 00 21 0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 
0C 00 08 61 00 40 08 03 00 42 18 00 10 9E 81 DC 95 FB F4 F7 69 A0 B7 16 CF 22 05 7E 99 CE 00 00 
61 00 80 30 00 20 84 01 00 21 0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 
20 84 01 00 21 0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 
0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 
40 08 03 00 42 18 00 10 11 7F 5A 7F 80 F6 EE D3 D4 FA 23 BC EB E7 23 DD E6 B9 F5 47 00 6A D0 19 
00 20 0C 00 30 4C 14 46 42 00 74 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 40 08 03 00 42 18 
00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 61 6F A2 8F BE ED 20 7D 8D 5D 8C AE FD ED 80 32 D3 
5C B7 04 4C F7 F7 4A 34 DF 3A AA 41 29 0F 12 18 B7 68 5E FB DB 01 7B D4 0B 83 65 0C BC EA 21 12 
D2 1F 29 33 62 C5 BC F6 B7 03 76 AA 14 06 EB 49 F0 D0 36 0F 5E 6B E5 B7 6A 98 72 4C 9F AE FD ED 
80 FD 6A 4C 20 A7 24 41 FA 61 67 48 AC 83 AF BF EA E7 C9 94 9B AE FD ED 80 43 9C 1E 06 59 25 BE 
61 1E 3C 6C 5E 11 0F 7D C9 7C ED 6F 07 EC 61 69 E9 BF 57 C1 89 A5 F0 79 D8 10 97 CF D7 FE 76 C0 
51 CE 0D 83 82 2B FD E6 CD 01 C0 0F EA F1 3E 03 D7 A4 9D E8 ED 1F C2 28 16 9C C7 30 11 00 C2 00 
80 3E 87 89 2A 8F 06 F4 36 18 D2 8F 4E 86 65 FC 03 41 05 3A 03 00 4E 0E 83 82 9B 8A 7B D8 97 02 
E0 D7 E8 0C B2 57 D6 E7 AE DC 6F EB DA DF 0E 38 CA E9 61 90 75 A5 DF BC 2D D8 AC 98 43 8F 5F 5F 
FB DB 01 7B D4 E8 0C 12 4B 7C C3 24 48 DC 96 67 D0 AD DC AE FD ED 80 43 D8 C2 FA 5F D7 DE E4 79 
DC 6F 67 E4 0A 2A A8 37 67 30 DF E6 8F 15 FF DB CF EB CB DD BD 67 2C D7 FE 76 C0 4E B5 9F 74 F6 
F7 5D FF E9 12 3A 89 81 37 D7 7E 30 E4 70 DF 4E 67 00 15 F4 78 D3 59 73 D7 2E 3A D7 FE 76 40 19 
4B 4B 01 10 06 00 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 
40 08 03 00 42 18 00 10 C2 00 80 10 06 00 84 30 00 20 84 01 00 21 0C 00 08 61 00 40 78 EC 25 D5 
7C 7B F6 F2 D9 67 F0 98 4F 48 21 0C A8 61 7F 12 EC 79 EB 82 3C 98 EE F7 6F BF 9A 6F B7 FA E7 81 
B3 09 03 C8 F3 AC EF EB D5 7C 25 06 5E 0F 10 09 74 C2 9C 01 14 DA 2C F7 30 10 9D 01 35 DC E6 B9 
D5 48 D1 CE 39 83 E5 95 FB 6B 06 4C F7 FB E6 A5 FD C7 03 9E 27 49 39 03 54 20 0C A8 E4 63 1E A4 
54 EA E7 AB B2 0E 3E CF 7C BB A5 F4 04 EB 25 3E F1 24 50 8D 61 22 AE A3 2C 6C 0A B8 96 E7 7A 84 
01 F5 2C 4B F3 81 17 F2 D5 92 E0 E1 99 07 2E F0 B9 06 61 40 55 A7 E6 01 50 4C 18 70 05 95 DB 82 
FD F4 13 F4 46 18 50 DB E1 CD 41 F3 24 C8 AD EC AF C7 9B 7E A0 13 C2 80 06 AE 31 58 54 50 C7 A7 
FB 5D 4F 40 9F 84 01 63 6B DE 16 A4 5B C6 80 B6 80 7E 08 03 DA 38 A4 39 18 28 09 DE CC B7 9B 24 
A0 2B C2 80 66 AE 31 58 94 C8 3C 01 9D 13 06 8C 6A D0 B6 40 12 D0 27 61 40 4B 07 36 07 43 24 01 
74 4B 18 D0 58 59 1E 5C 78 40 09 9A B0 51 1D E3 E9 64 80 28 F1 C1 06 E9 C7 40 43 3A 03 DA FB A9 
99 64 E8 93 30 A0 0B E9 79 D0 49 5B 00 17 23 0C 18 49 3F 49 E0 46 62 2E 46 18 D0 8B 81 06 8B 0A 
6E 1A 78 6C 44 21 42 E8 96 30 A0 23 EB 79 D0 49 5B 50 B0 2D DD DB 93 32 8F FE 44 70 00 AB 89 18 
55 9D 24 58 AF DD D6 08 71 19 3A 03 FA 32 D0 6C B0 24 E0 4A 74 06 74 E7 36 CF 9B B3 05 0D 33 23 
37 03 1E C7 67 DD 94 00 F5 4D 73 9B 51 D7 BF FF D5 E7 DB 30 97 81 D4 B4 1E 06 03 75 0F 30 0A 9D 
01 95 1C B8 34 28 EB 54 92 03 52 98 33 A0 86 86 8B 44 BB 5D 9F 0A 5D 11 06 00 18 26 02 AE 6E 7A 
E9 0E 8B 67 49 A7 EF 2D E6 CA 39 F7 BC F5 21 1F 3B 9D CE 80 1A 1A 0E DC 9B 33 F8 71 6F 45 7C A5 
A6 AF 9C 61 FD 55 DF 0E D8 F3 D6 FB 3F 76 2E 9D 01 95 14 17 E5 E7 A0 BF B2 4E 7D E9 55 78 9A DA 
2C CE 3C 4A A5 30 78 AE 25 5D F9 B9 65 A6 40 57 72 AF C7 87 CE 83 7A 9D C1 6B AD 5F DE 67 F0 2D 
2D 00 9A 28 1B 99 39 24 0F 2A 0C 0A 2D 99 33 00 A8 A4 B8 CA 57 68 38 CC 19 00 A4 7A 2B CA EB C5 
7D 9E E7 CD 99 E7 6F 67 AE 4F 18 00 24 59 D6 EB CD 72 BF 62 B9 5E 68 33 0F 12 C3 A3 2C 63 0C 13 
01 BC 5B 96 F8 6F 55 75 F9 F3 9D 23 FE DF 5E 9E B8 D8 B4 78 4D AA 30 00 E8 5A 9D 11 A4 36 C3 44 
56 91 02 BF 60 39 8E F4 18 0E 3A 64 BD D0 72 64 69 CF 69 75 06 00 5D 28 BB 3B 7A CF CB 5F 09 03 
80 5E 14 EC 80 F4 78 C9 B7 17 9A 40 06 B8 82 D7 6A BE 9E 07 EB AF DD 54 6F CE C0 3D C6 00 B1 6F 
3C 27 7D BE 21 77 DA B9 52 18 98 31 06 7E D3 51 D3 C5 59 27 2C 58 80 64 98 08 00 61 00 D0 AB F4 
3B DD D2 7F FB 8D 30 00 40 18 00 9C EC F0 5B 88 D3 F7 BF 4B 27 0C 00 B6 25 EE 05 34 2E 61 00 F0 
6E CF B5 7C D6 6B 57 0E CE BD AB 20 F7 98 37 C2 00 20 C9 B2 C2 D6 6F 0B B2 9E C9 9C 75 66 CF 33 
00 7E CB E6 13 69 8A 5F 7B B6 95 3B 8D 3F FE 2A EB 19 9C 03 3F BE 99 CB 5B DE B5 EE EE 45 72 15 
94 EF 47 55 3C FC 11 95 1F 4F B8 F2 5E 89 3B 92 3E 0F DB B9 29 85 30 A0 47 EB 9B 97 88 04 D2 15 
87 C1 CE D7 A6 7C 92 95 37 4A 7C 66 4E 4A 60 24 16 79 73 06 74 67 73 1B 2B FB 5C 51 47 EE B5 72 
E5 6B EB 8F 8F E1 2C 3E 9B CE 60 78 29 95 71 A0 4B E9 F4 42 3F D0 97 A2 A1 FD 57 F7 47 6D 0C 77 
78 67 F0 F1 1D D3 1F D8 F9 C6 04 F2 15 AC 97 C5 81 AE A3 B3 3E EA 74 9F E4 01 9B 72 F7 89 FB 76 
B9 5D F0 A4 81 F5 4F F2 B6 37 F5 E6 53 EC 53 8E 49 3F 6C 49 18 00 17 77 C8 F8 C7 D9 27 49 39 7F 
7A EA 64 7C A6 7F 98 33 A0 17 05 1D CC 40 4D 0F 74 4E 67 70 35 1F EB E3 85 8B E6 85 BF 1A C5 8C 
1F 16 D0 19 00 20 0C 00 30 4C 74 3D CB 06 79 94 55 37 65 03 3E 43 7C 35 E8 9F CE 00 00 61 40 37 
0A AE F1 B5 05 70 14 61 00 80 39 83 4B B8 CC F2 CA F9 36 DB 8E 02 9A B0 37 11 DD B9 D8 6E 4B 30 
04 C3 44 74 67 B3 D0 4B 02 38 9C CE 80 7E 79 B8 0D 54 23 0C 00 30 4C 04 80 30 00 20 84 01 00 21 
0C 00 08 61 00 40 08 03 00 42 18 00 10 C2 00 80 10 06 00 44 C4 FF 01 FE B9 AF FC CB 3B 13 9F 00 
00 00 00 49 45 4E 44 AE 42 60 82 
EndData
$EndBitmap
Text Notes 900  5650 0    50   ~ 10
TYPICAL KEY CIRCUIT\nFOR ILLUSTRATION ONLY
Wire Notes Line
	600  4200 2400 4200
Wire Notes Line
	2400 4200 2400 5700
Wire Notes Line
	2400 5700 600  5700
Wire Notes Line
	600  5700 600  4200
Text Notes 2700 5800 0    50   ~ 10
WASD CODE V2B KEY MATRIX WIRING
Wire Notes Line
	4950 700  6850 700 
Wire Notes Line
	6850 700  6850 1850
Wire Notes Line
	6850 1850 4950 1850
Wire Notes Line
	4950 1850 4950 700 
Text Notes 5000 1800 0    50   ~ 10
3.3V Power
Wire Notes Line
	3300 550  3300 1750
Wire Notes Line
	600  550  3300 550 
Text Notes 650  1700 0    50   ~ 10
Backlight LED Array Driver
Wire Notes Line
	4850 2150 7100 2150
Wire Notes Line
	7100 2150 7100 2900
Wire Notes Line
	7100 2900 4850 2900
Wire Notes Line
	4850 2900 4850 2150
Text Notes 4900 2850 0    50   ~ 10
Decoupling
Text Notes 1700 7500 0    50   ~ 10
Keys named by PC104 convention, e.g. CAPS next to A, ALTs by SPACE, MENU by RSUP.
Text Notes 1700 7650 0    50   ~ 10
DIP1-6 are the configuration switches, which act like keys.
$Comp
L power:+3V3 #PWR0126
U 1 1 5F3B29C9
P 3100 2050
F 0 "#PWR0126" H 3100 1900 50  0001 C CNN
F 1 "+3V3" H 3115 2223 50  0000 C CNN
F 2 "" H 3100 2050 50  0001 C CNN
F 3 "" H 3100 2050 50  0001 C CNN
	1    3100 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5F3B302B
P 3100 2950
F 0 "#PWR0127" H 3100 2700 50  0001 C CNN
F 1 "GND" H 3105 2777 50  0000 C CNN
F 2 "" H 3100 2950 50  0001 C CNN
F 3 "" H 3100 2950 50  0001 C CNN
	1    3100 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2450 2400 2450
Wire Wire Line
	2400 2450 2400 2050
$Comp
L power:+3V3 #PWR0128
U 1 1 5F3B62DA
P 2400 2050
F 0 "#PWR0128" H 2400 1900 50  0001 C CNN
F 1 "+3V3" H 2415 2223 50  0000 C CNN
F 2 "" H 2400 2050 50  0001 C CNN
F 3 "" H 2400 2050 50  0001 C CNN
	1    2400 2050
	1    0    0    -1  
$EndComp
$Comp
L 74xx_IEEE:74HC238 U103
U 1 1 5F3A148E
P 3100 2350
F 0 "U103" H 3050 2150 50  0000 C CNN
F 1 "TC74VHC238F" H 3450 1650 50  0000 C CNN
F 2 "vssop:VSSOP-16_3.0x4.25mm_P0.5mm" H 3100 2350 50  0001 C CNN
F 3 "" H 3100 2350 50  0001 C CNN
	1    3100 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5F3BB24A
P 2500 2500
F 0 "#PWR0129" H 2500 2250 50  0001 C CNN
F 1 "GND" H 2505 2327 50  0000 C CNN
F 2 "" H 2500 2500 50  0001 C CNN
F 3 "" H 2500 2500 50  0001 C CNN
	1    2500 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2700 2400 2700
Wire Wire Line
	2550 2800 2400 2800
Wire Wire Line
	2550 2900 2400 2900
Text Label 2400 2700 2    50   ~ 0
EH0
Text Label 2400 2800 2    50   ~ 0
EH1
Text Label 2400 2900 2    50   ~ 0
EH2
Text Label 3650 2850 0    50   ~ 0
HIGH3
Text Label 3650 2750 0    50   ~ 0
HIGH4
Text Label 3650 2650 0    50   ~ 0
HIGH5
Text Label 3650 2550 0    50   ~ 0
HIGH6
Text Label 3650 2450 0    50   ~ 0
HIGH0
Text Label 3650 2350 0    50   ~ 0
HIGH7
Text Label 3650 2250 0    50   ~ 0
HIGH1
Text Label 3650 2150 0    50   ~ 0
HIGH2
Text Label 10300 5150 0    50   ~ 0
EH0
Text Label 10300 5050 0    50   ~ 0
EH1
Text Label 10300 4950 0    50   ~ 0
EH2
$Comp
L Connector_Generic:Conn_01x04 J103
U 1 1 5F3EFF40
P 1700 2250
F 0 "J103" H 1780 2242 50  0000 L CNN
F 1 "OLED" H 1780 2151 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical_SMD_Pin1Left" H 1700 2250 50  0001 C CNN
F 3 "~" H 1700 2250 50  0001 C CNN
	1    1700 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 5F3F060A
P 1500 2450
F 0 "#PWR0130" H 1500 2200 50  0001 C CNN
F 1 "GND" H 1505 2277 50  0000 C CNN
F 2 "" H 1500 2450 50  0001 C CNN
F 3 "" H 1500 2450 50  0001 C CNN
	1    1500 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0131
U 1 1 5F3F136D
P 1500 2350
F 0 "#PWR0131" H 1500 2200 50  0001 C CNN
F 1 "+3V3" V 1515 2478 50  0000 L CNN
F 2 "" H 1500 2350 50  0001 C CNN
F 3 "" H 1500 2350 50  0001 C CNN
	1    1500 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1500 2150 1050 2150
Text Label 1050 2150 2    50   ~ 0
I2C1_SDA
Text Label 1050 2250 2    50   ~ 0
I2C1_SCL
Wire Wire Line
	1050 2250 1500 2250
Text Notes 850  2000 0    50   ~ 0
MakerFocus OLED module\nhas 4k7 pullups onboard
Wire Notes Line
	600  2700 2100 2700
Wire Notes Line
	600  550  600  2700
Text Notes 650  2650 0    50   ~ 10
Display
Wire Notes Line
	2100 3200 3950 3200
Wire Notes Line
	3950 3200 3950 1750
Wire Notes Line
	600  1750 3950 1750
Wire Notes Line
	2100 1750 2100 3200
Text Notes 2150 3150 0    50   ~ 10
3:8 Decoder
NoConn ~ 10300 5350
Wire Wire Line
	5950 1450 6150 1450
Wire Wire Line
	2550 2150 2500 2150
Wire Wire Line
	2500 2150 2500 2300
Wire Wire Line
	2550 2300 2500 2300
Connection ~ 2500 2300
Wire Wire Line
	2500 2300 2500 2500
NoConn ~ 10300 4750
NoConn ~ 10300 4850
NoConn ~ 9100 4250
NoConn ~ 9100 4150
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F423B07
P 6150 1450
F 0 "#FLG0101" H 6150 1525 50  0001 C CNN
F 1 "PWR_FLAG" V 6150 1578 50  0000 L CNN
F 2 "" H 6150 1450 50  0001 C CNN
F 3 "~" H 6150 1450 50  0001 C CNN
	1    6150 1450
	-1   0    0    1   
$EndComp
Connection ~ 6150 1450
Wire Wire Line
	6150 1450 6300 1450
Text Notes 2000 7200 0    50   ~ 0
6
$EndSCHEMATC
