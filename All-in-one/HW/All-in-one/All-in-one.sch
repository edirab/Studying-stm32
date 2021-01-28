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
L YAAJ_BluePill_Part_Like_SWD_Breakout:YAAJ_BluePill_Part_Like_SWD_Breakout U3
U 1 1 60118BDE
P 4850 3750
F 0 "U3" H 4850 4915 50  0000 C CNN
F 1 "YAAJ_BluePill_Part_Like_SWD_Breakout" H 4850 4824 50  0000 C CNN
F 2 "" H 5650 2750 50  0001 C CNN
F 3 "" H 5650 2750 50  0001 C CNN
	1    4850 3750
	1    0    0    -1  
$EndComp
$Comp
L megasaturnv_custom_components:BME280_module U2
U 1 1 6011B28D
P 4550 5650
F 0 "U2" H 4978 5653 60  0000 L CNN
F 1 "BME280_module" H 4978 5547 60  0000 L CNN
F 2 "" H 4650 5550 60  0001 C CNN
F 3 "" H 4650 5550 60  0001 C CNN
	1    4550 5650
	1    0    0    -1  
$EndComp
$Comp
L megasaturnv_custom_components:RTC_DS3231 RTC1
U 1 1 6011CB32
P 2750 5800
F 0 "RTC1" H 2833 6287 60  0000 C CNN
F 1 "RTC_DS3231" H 2833 6181 60  0000 C CNN
F 2 "" H 2550 5800 60  0001 C CNN
F 3 "" H 2550 5800 60  0001 C CNN
	1    2750 5800
	1    0    0    -1  
$EndComp
$Comp
L Sensor:DHT11 U1
U 1 1 6011E39A
P 2100 3300
F 0 "U1" H 1856 3346 50  0000 R CNN
F 1 "DHT11" H 1856 3255 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 2100 2900 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 2250 3550 50  0001 C CNN
	1    2100 3300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Micro_SD_Card J3
U 1 1 6011EFAA
P 8000 3100
F 0 "J3" H 7950 3817 50  0000 C CNN
F 1 "Micro_SD_Card" H 7950 3726 50  0000 C CNN
F 2 "" H 9150 3400 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 8000 3100 50  0001 C CNN
	1    8000 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 601225CC
P 3150 6650
F 0 "J1" H 3258 6931 50  0000 C CNN
F 1 "Conn_01x04_Male" H 3258 6840 50  0000 C CNN
F 2 "" H 3150 6650 50  0001 C CNN
F 3 "~" H 3150 6650 50  0001 C CNN
	1    3150 6650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J2
U 1 1 60124552
P 6300 1800
F 0 "J2" H 6357 2125 50  0000 C CNN
F 1 "Barrel_Jack" H 6357 2034 50  0000 C CNN
F 2 "" H 6350 1760 50  0001 C CNN
F 3 "~" H 6350 1760 50  0001 C CNN
	1    6300 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60125900
P 7100 5250
F 0 "R1" H 7170 5296 50  0000 L CNN
F 1 "R" H 7170 5205 50  0000 L CNN
F 2 "" V 7030 5250 50  0001 C CNN
F 3 "~" H 7100 5250 50  0001 C CNN
	1    7100 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60126085
P 7450 5250
F 0 "R2" H 7520 5296 50  0000 L CNN
F 1 "R" H 7520 5205 50  0000 L CNN
F 2 "" V 7380 5250 50  0001 C CNN
F 3 "~" H 7450 5250 50  0001 C CNN
	1    7450 5250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 60127039
P 7050 6050
F 0 "SW1" H 7050 6335 50  0000 C CNN
F 1 "SW_Push" H 7050 6244 50  0000 C CNN
F 2 "" H 7050 6250 50  0001 C CNN
F 3 "~" H 7050 6250 50  0001 C CNN
	1    7050 6050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
