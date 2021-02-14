EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L All-in-one-rescue:RTC_DS3231-megasaturnv_custom_components RTC1
U 1 1 6011CB32
P 1550 3700
F 0 "RTC1" H 1633 4187 60  0000 C CNN
F 1 "RTC_DS3231" H 1633 4081 60  0000 C CNN
F 2 "megasaturnv_custom_components:RTC_DS3231" H 1350 3700 60  0001 C CNN
F 3 "" H 1350 3700 60  0001 C CNN
	1    1550 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 601225CC
P 1200 4900
F 0 "J1" H 1308 5181 50  0000 C CNN
F 1 "Conn_01x04_Male" H 1308 5090 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1200 4900 50  0001 C CNN
F 3 "~" H 1200 4900 50  0001 C CNN
	1    1200 4900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J2
U 1 1 60124552
P 1250 1400
F 0 "J2" H 1307 1725 50  0000 C CNN
F 1 "Barrel_Jack" H 1307 1634 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 1300 1360 50  0001 C CNN
F 3 "~" H 1300 1360 50  0001 C CNN
	1    1250 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60125900
P 5650 6450
F 0 "R1" H 5720 6496 50  0000 L CNN
F 1 "10k" H 5720 6405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 5580 6450 50  0001 C CNN
F 3 "~" H 5650 6450 50  0001 C CNN
	1    5650 6450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 60126085
P 5250 6450
F 0 "R2" H 5320 6496 50  0000 L CNN
F 1 "10k" H 5320 6405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 5180 6450 50  0001 C CNN
F 3 "~" H 5250 6450 50  0001 C CNN
	1    5250 6450
	-1   0    0    1   
$EndComp
$Comp
L megasaturnv_custom_components:BMP280_flipped U2
U 1 1 6026F1A7
P 2950 5850
F 0 "U2" H 3378 5753 60  0000 L CNN
F 1 "BMP280_flipped" H 3378 5647 60  0000 L CNN
F 2 "megasaturnv_custom_components:BMP280_flipped" H 3050 5750 60  0001 C CNN
F 3 "" H 3050 5750 60  0001 C CNN
	1    2950 5850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 6027126D
P 3400 4550
F 0 "#PWR0101" H 3400 4400 50  0001 C CNN
F 1 "+5V" H 3415 4723 50  0000 C CNN
F 2 "" H 3400 4550 50  0001 C CNN
F 3 "" H 3400 4550 50  0001 C CNN
	1    3400 4550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 602719B5
P 3650 4950
F 0 "#PWR0102" H 3650 4700 50  0001 C CNN
F 1 "GND" H 3655 4777 50  0000 C CNN
F 2 "" H 3650 4950 50  0001 C CNN
F 3 "" H 3650 4950 50  0001 C CNN
	1    3650 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4650 3650 4650
Wire Wire Line
	3650 4650 3650 4950
Text GLabel 3450 4150 0    50   Input ~ 0
I2C1_SCL
Text GLabel 3450 4250 0    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	3400 4550 4150 4550
Wire Wire Line
	3450 4150 4150 4150
Wire Wire Line
	4150 4250 3450 4250
Text GLabel 1750 6050 0    50   Input ~ 0
I2C1_SCL
Text GLabel 1750 5950 0    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	1750 6050 2450 6050
Wire Wire Line
	2450 5950 1750 5950
$Comp
L power:GND #PWR0103
U 1 1 60277BA8
P 1300 6150
F 0 "#PWR0103" H 1300 5900 50  0001 C CNN
F 1 "GND" H 1305 5977 50  0000 C CNN
F 2 "" H 1300 6150 50  0001 C CNN
F 3 "" H 1300 6150 50  0001 C CNN
	1    1300 6150
	0    1    1    0   
$EndComp
Text GLabel 4150 4750 0    50   Input ~ 0
3V3
Text GLabel 5250 5600 1    50   Input ~ 0
3V3
Wire Wire Line
	2450 6250 2100 6250
Wire Wire Line
	2450 5850 2100 5850
Wire Wire Line
	2100 5850 2100 6250
Connection ~ 2100 6250
Wire Wire Line
	2100 6250 1550 6250
Wire Wire Line
	2450 5750 1950 5750
Wire Wire Line
	1950 5750 1950 6150
Wire Wire Line
	1950 6150 2450 6150
Text GLabel 2850 3800 2    50   Input ~ 0
I2C1_SCL
Text GLabel 2850 3700 2    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	2850 3800 2150 3800
Wire Wire Line
	2150 3700 2850 3700
$Comp
L power:GND #PWR0104
U 1 1 6027C7C2
P 3200 3500
F 0 "#PWR0104" H 3200 3250 50  0001 C CNN
F 1 "GND" H 3205 3327 50  0000 C CNN
F 2 "" H 3200 3500 50  0001 C CNN
F 3 "" H 3200 3500 50  0001 C CNN
	1    3200 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2150 3500 3200 3500
Wire Wire Line
	2150 3600 2650 3600
Wire Wire Line
	2650 3600 2650 3350
$Comp
L Connector:Conn_01x02_Male J4
U 1 1 60280B92
P 3000 1300
F 0 "J4" H 3108 1481 50  0000 C CNN
F 1 "Conn_01x02_Male" H 3108 1390 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3000 1300 50  0001 C CNN
F 3 "~" H 3000 1300 50  0001 C CNN
	1    3000 1300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 60282546
P 2350 1050
F 0 "#PWR0106" H 2350 900 50  0001 C CNN
F 1 "+5V" H 2365 1223 50  0000 C CNN
F 2 "" H 2350 1050 50  0001 C CNN
F 3 "" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 60283698
P 2350 1650
F 0 "#PWR0107" H 2350 1400 50  0001 C CNN
F 1 "GND" H 2355 1477 50  0000 C CNN
F 2 "" H 2350 1650 50  0001 C CNN
F 3 "" H 2350 1650 50  0001 C CNN
	1    2350 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1200 2350 1200
Wire Wire Line
	2350 1200 2350 1050
Wire Wire Line
	2800 1300 2350 1300
Wire Wire Line
	5750 3350 6250 3350
Wire Wire Line
	6250 3250 5750 3250
$Comp
L power:+5V #PWR0108
U 1 1 6028AC92
P 2000 1050
F 0 "#PWR0108" H 2000 900 50  0001 C CNN
F 1 "+5V" H 2015 1223 50  0000 C CNN
F 2 "" H 2000 1050 50  0001 C CNN
F 3 "" H 2000 1050 50  0001 C CNN
	1    2000 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 6028B69A
P 2000 1650
F 0 "#PWR0109" H 2000 1400 50  0001 C CNN
F 1 "GND" H 2005 1477 50  0000 C CNN
F 2 "" H 2000 1650 50  0001 C CNN
F 3 "" H 2000 1650 50  0001 C CNN
	1    2000 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1500 2000 1500
Wire Wire Line
	2000 1500 2000 1650
Wire Wire Line
	1550 1300 2000 1300
Wire Wire Line
	2000 1300 2000 1050
Wire Wire Line
	2350 1300 2350 1650
$Comp
L Connector:Conn_01x02_Male J5
U 1 1 6029617F
P 3950 1300
F 0 "J5" H 4058 1481 50  0000 C CNN
F 1 "Conn_01x02_Male" H 4058 1390 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3950 1300 50  0001 C CNN
F 3 "~" H 3950 1300 50  0001 C CNN
	1    3950 1300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0110
U 1 1 60296185
P 3300 1050
F 0 "#PWR0110" H 3300 900 50  0001 C CNN
F 1 "+5V" H 3315 1223 50  0000 C CNN
F 2 "" H 3300 1050 50  0001 C CNN
F 3 "" H 3300 1050 50  0001 C CNN
	1    3300 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 6029618B
P 3300 1650
F 0 "#PWR0111" H 3300 1400 50  0001 C CNN
F 1 "GND" H 3305 1477 50  0000 C CNN
F 2 "" H 3300 1650 50  0001 C CNN
F 3 "" H 3300 1650 50  0001 C CNN
	1    3300 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1200 3300 1200
Wire Wire Line
	3300 1200 3300 1050
Wire Wire Line
	3750 1300 3300 1300
Wire Wire Line
	3300 1300 3300 1650
Text GLabel 1950 4900 2    50   Input ~ 0
I2C1_SCL
Text GLabel 1950 4800 2    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	1950 4800 1400 4800
Wire Wire Line
	1400 4900 1950 4900
$Comp
L power:+5V #PWR0112
U 1 1 6029F856
P 2450 5000
F 0 "#PWR0112" H 2450 4850 50  0001 C CNN
F 1 "+5V" H 2465 5173 50  0000 C CNN
F 2 "" H 2450 5000 50  0001 C CNN
F 3 "" H 2450 5000 50  0001 C CNN
	1    2450 5000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 602A09B2
P 2450 5200
F 0 "#PWR0113" H 2450 4950 50  0001 C CNN
F 1 "GND" H 2455 5027 50  0000 C CNN
F 2 "" H 2450 5200 50  0001 C CNN
F 3 "" H 2450 5200 50  0001 C CNN
	1    2450 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5000 1400 5000
Wire Wire Line
	2450 5200 2450 5100
Wire Wire Line
	2450 5100 1400 5100
$Comp
L Connector:Conn_01x02_Male J7
U 1 1 602AD6BF
P 5850 6050
F 0 "J7" H 5958 6231 50  0000 C CNN
F 1 "Conn_01x02_Male" H 5958 6140 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5850 6050 50  0001 C CNN
F 3 "~" H 5850 6050 50  0001 C CNN
	1    5850 6050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Male J8
U 1 1 602AE4B2
P 5050 5950
F 0 "J8" H 5158 6131 50  0000 C CNN
F 1 "Conn_01x02_Male" H 5158 6040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5050 5950 50  0001 C CNN
F 3 "~" H 5050 5950 50  0001 C CNN
	1    5050 5950
	1    0    0    -1  
$EndComp
Text GLabel 5650 5600 1    50   Input ~ 0
3V3
Wire Wire Line
	5250 6950 5250 6600
Wire Wire Line
	5650 6600 5650 6950
Wire Wire Line
	5250 6300 5250 6250
$Comp
L power:GND #PWR0114
U 1 1 602B89B5
P 5250 6950
F 0 "#PWR0114" H 5250 6700 50  0001 C CNN
F 1 "GND" H 5255 6777 50  0000 C CNN
F 2 "" H 5250 6950 50  0001 C CNN
F 3 "" H 5250 6950 50  0001 C CNN
	1    5250 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 602B96EA
P 5650 6950
F 0 "#PWR0115" H 5650 6700 50  0001 C CNN
F 1 "GND" H 5655 6777 50  0000 C CNN
F 2 "" H 5650 6950 50  0001 C CNN
F 3 "" H 5650 6950 50  0001 C CNN
	1    5650 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5950 5650 5600
Wire Wire Line
	5250 5950 5250 5600
Text GLabel 6000 4250 2    50   Input ~ 0
A1
Text GLabel 6000 4150 2    50   Input ~ 0
A2
Wire Wire Line
	6000 4150 5750 4150
Wire Wire Line
	5750 4250 6000 4250
Text GLabel 6200 6250 2    50   Input ~ 0
A1
Text GLabel 4800 6250 0    50   Input ~ 0
A2
Wire Wire Line
	5250 6250 4800 6250
Connection ~ 5250 6250
Wire Wire Line
	5250 6250 5250 6050
Wire Wire Line
	5650 6300 5650 6250
Wire Wire Line
	6200 6250 5650 6250
Connection ~ 5650 6250
Wire Wire Line
	5650 6250 5650 6050
$Comp
L megasaturnv_custom_components:microSD_flipped U4
U 1 1 602D1613
P 8100 3600
F 0 "U4" H 8528 3503 60  0000 L CNN
F 1 "microSD_flipped" H 8528 3397 60  0000 L CNN
F 2 "megasaturnv_custom_components:microSD_Card_3V3" H 8200 3500 60  0001 C CNN
F 3 "" H 8200 3500 60  0001 C CNN
	1    8100 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 602D6319
P 7150 3500
F 0 "#PWR0116" H 7150 3250 50  0001 C CNN
F 1 "GND" H 7155 3327 50  0000 C CNN
F 2 "" H 7150 3500 50  0001 C CNN
F 3 "" H 7150 3500 50  0001 C CNN
	1    7150 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 3500 7150 3500
Wire Wire Line
	1950 6150 1300 6150
Connection ~ 1950 6150
Text GLabel 1550 6250 0    50   Input ~ 0
3V3
Text GLabel 7550 4000 0    50   Input ~ 0
3V3
Wire Wire Line
	7550 4000 7600 4000
Wire Wire Line
	5750 3750 7300 3750
Wire Wire Line
	7300 3750 7300 3600
Wire Wire Line
	7300 3600 7600 3600
Wire Wire Line
	7600 3800 6200 3800
Wire Wire Line
	6200 3800 6200 3650
Wire Wire Line
	6200 3650 5750 3650
Wire Wire Line
	5750 3850 7400 3850
Wire Wire Line
	7400 3850 7400 3700
Wire Wire Line
	7400 3700 7600 3700
Wire Wire Line
	5750 3950 7100 3950
Wire Wire Line
	7100 3950 7100 3900
Wire Wire Line
	7100 3900 7600 3900
Text Notes 1200 4550 0    118  ~ 0
К LCD 16х2
Text GLabel 5900 3050 2    50   Input ~ 0
3V3
Wire Wire Line
	5750 3050 5900 3050
$Comp
L power:GND #PWR0117
U 1 1 602AE654
P 6350 2850
F 0 "#PWR0117" H 6350 2600 50  0001 C CNN
F 1 "GND" H 6355 2677 50  0000 C CNN
F 2 "" H 6350 2850 50  0001 C CNN
F 3 "" H 6350 2850 50  0001 C CNN
	1    6350 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5750 2850 5850 2850
Wire Wire Line
	5750 2950 5850 2950
Wire Wire Line
	5850 2950 5850 2850
Connection ~ 5850 2850
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 602BF410
P 6450 3250
F 0 "J3" H 6530 3292 50  0000 L CNN
F 1 "Conn_01x03" H 6530 3201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6450 3250 50  0001 C CNN
F 3 "~" H 6450 3250 50  0001 C CNN
	1    6450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2850 6200 2850
Wire Wire Line
	6250 3150 6200 3150
Wire Wire Line
	6200 3150 6200 2850
Connection ~ 6200 2850
Wire Wire Line
	6200 2850 6350 2850
$Comp
L YAAJ_BluePill_Part_Like:YAAJ_BluePill_Part_Like U3
U 1 1 602D2333
P 4950 3750
F 0 "U3" H 4950 4915 50  0000 C CNN
F 1 "YAAJ_BluePill_Part_Like" H 4950 4824 50  0000 C CNN
F 2 "Footprints:YAAJ_BluePill_1" H 5650 2750 50  0001 C CNN
F 3 "" H 5650 2750 50  0001 C CNN
	1    4950 3750
	1    0    0    -1  
$EndComp
Text GLabel 2650 3350 1    50   Input ~ 0
3V3
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 602FFCC0
P 2850 2500
F 0 "J6" H 2930 2542 50  0000 L CNN
F 1 "Conn_01x03" H 2930 2451 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2850 2500 50  0001 C CNN
F 3 "~" H 2850 2500 50  0001 C CNN
	1    2850 2500
	-1   0    0    1   
$EndComp
Text GLabel 3450 4350 0    50   Input ~ 0
DHT_11
Wire Wire Line
	3450 4350 4150 4350
Text GLabel 3200 2500 2    50   Input ~ 0
DHT_11
Text GLabel 3200 2400 2    50   Input ~ 0
3V3
$Comp
L power:GND #PWR0105
U 1 1 6030D641
P 3200 2700
F 0 "#PWR0105" H 3200 2450 50  0001 C CNN
F 1 "GND" H 3205 2527 50  0000 C CNN
F 2 "" H 3200 2700 50  0001 C CNN
F 3 "" H 3200 2700 50  0001 C CNN
	1    3200 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2400 3200 2400
Wire Wire Line
	3050 2500 3200 2500
Wire Wire Line
	3050 2600 3200 2600
Wire Wire Line
	3200 2600 3200 2700
Text Notes 2350 2300 0    118  ~ 0
DHT_11
$EndSCHEMATC
