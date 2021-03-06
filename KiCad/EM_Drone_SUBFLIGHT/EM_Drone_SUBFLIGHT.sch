EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 4050 2800 1950 2500
U 6059AC3E
F0 "Arduino_MEGA" 50
F1 "Arduino_MEGA.sch" 50
F2 "D4" I R 6000 3500 50 
F3 "D5" O R 6000 3400 50 
F4 "D6" O R 6000 3200 50 
F5 "TXD2" O R 6000 3050 50 
F6 "RXD2" I R 6000 2950 50 
F7 "TXD1" O L 4050 3100 50 
F8 "RXD1" I L 4050 3000 50 
F9 "TXD3" O L 4050 4050 50 
F10 "RXD3" I L 4050 3950 50 
F11 "D8" B L 4050 5100 50 
F12 "D9" B L 4050 5200 50 
$EndSheet
Wire Wire Line
	6000 3500 6300 3500
Wire Wire Line
	6300 3400 6000 3400
Wire Wire Line
	6000 3200 6300 3200
Wire Wire Line
	6300 3050 6000 3050
Wire Wire Line
	6000 2950 6300 2950
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 608C4407
P 3350 2900
F 0 "J1" H 3268 3217 50  0000 C CNN
F 1 "Telemetry_Port" H 3268 3126 50  0000 C CNN
F 2 "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal" H 3350 2900 50  0001 C CNN
F 3 "~" H 3350 2900 50  0001 C CNN
	1    3350 2900
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 608C5F11
P 3700 3700
F 0 "#PWR010" H 3700 3550 50  0001 C CNN
F 1 "+5V" H 3715 3873 50  0000 C CNN
F 2 "" H 3700 3700 50  0001 C CNN
F 3 "" H 3700 3700 50  0001 C CNN
	1    3700 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2900 3550 2900
Wire Wire Line
	3550 3000 4050 3000
Wire Wire Line
	4050 3100 3550 3100
$Comp
L power:GND #PWR09
U 1 1 608C9787
P 3600 3250
F 0 "#PWR09" H 3600 3000 50  0001 C CNN
F 1 "GND" H 3605 3077 50  0000 C CNN
F 2 "" H 3600 3250 50  0001 C CNN
F 3 "" H 3600 3250 50  0001 C CNN
	1    3600 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2800 3550 2800
Wire Wire Line
	3600 2800 3600 3250
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 6062B48B
P 3300 3850
F 0 "J4" H 3218 4167 50  0000 C CNN
F 1 "Serial3" H 3218 4076 50  0000 C CNN
F 2 "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal" H 3300 3850 50  0001 C CNN
F 3 "~" H 3300 3850 50  0001 C CNN
	1    3300 3850
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 6062DAA9
P 3600 4150
F 0 "#PWR0101" H 3600 3900 50  0001 C CNN
F 1 "GND" H 3605 3977 50  0000 C CNN
F 2 "" H 3600 4150 50  0001 C CNN
F 3 "" H 3600 4150 50  0001 C CNN
	1    3600 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3950 3500 3950
Wire Wire Line
	3500 4050 4050 4050
Wire Wire Line
	3500 3750 3600 3750
Wire Wire Line
	3600 3750 3600 4150
Wire Wire Line
	3500 3850 3700 3850
Wire Wire Line
	3700 3850 3700 3700
$Comp
L power:+5V #PWR0102
U 1 1 6062EA99
P 3750 2650
F 0 "#PWR0102" H 3750 2500 50  0001 C CNN
F 1 "+5V" H 3765 2823 50  0000 C CNN
F 2 "" H 3750 2650 50  0001 C CNN
F 3 "" H 3750 2650 50  0001 C CNN
	1    3750 2650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 6062F748
P 3300 5000
F 0 "J6" H 3218 5317 50  0000 C CNN
F 1 "IO" H 3218 5226 50  0000 C CNN
F 2 "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal" H 3300 5000 50  0001 C CNN
F 3 "~" H 3300 5000 50  0001 C CNN
	1    3300 5000
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 606301F5
P 3700 4750
F 0 "#PWR0103" H 3700 4600 50  0001 C CNN
F 1 "+5V" H 3715 4923 50  0000 C CNN
F 2 "" H 3700 4750 50  0001 C CNN
F 3 "" H 3700 4750 50  0001 C CNN
	1    3700 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 606306CD
P 3600 5300
F 0 "#PWR0104" H 3600 5050 50  0001 C CNN
F 1 "GND" H 3605 5127 50  0000 C CNN
F 2 "" H 3600 5300 50  0001 C CNN
F 3 "" H 3600 5300 50  0001 C CNN
	1    3600 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5000 3700 5000
Wire Wire Line
	3700 5000 3700 4750
Wire Wire Line
	3500 4900 3600 4900
Wire Wire Line
	3600 4900 3600 5300
Wire Wire Line
	3500 5100 4050 5100
Wire Wire Line
	3500 5200 4050 5200
Wire Wire Line
	3750 2650 3750 2900
$Sheet
S 6300 2800 2150 1650
U 605D1685
F0 "SIM868" 50
F1 "SIM7020.sch" 50
F2 "TXD" O L 6300 2950 50 
F3 "RXD" I L 6300 3050 50 
F4 "DTR" I L 6300 3200 50 
F5 "PKEY" I L 6300 3400 50 
F6 "STATUS_" O L 6300 3500 50 
$EndSheet
$EndSCHEMATC
