EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title "Emergency_Drone"
Date ""
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATmega:ATmega2560-16AU U1
U 1 1 604F16FD
P 5550 4150
F 0 "U1" H 4850 1350 50  0000 C CNN
F 1 "ATmega2560-16AU" H 5150 1200 50  0000 C CNN
F 2 "Package_QFP:TQFP-100_14x14mm_P0.5mm" H 5550 4150 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf" H 5550 4150 50  0001 C CNN
	1    5550 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 604F6D5A
P 5550 7150
F 0 "#PWR02" H 5550 6900 50  0001 C CNN
F 1 "GND" H 5555 6977 50  0000 C CNN
F 2 "" H 5550 7150 50  0001 C CNN
F 3 "" H 5550 7150 50  0001 C CNN
	1    5550 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 7150 5550 7050
$Comp
L power:+5V #PWR01
U 1 1 604F752B
P 5550 1150
F 0 "#PWR01" H 5550 1000 50  0001 C CNN
F 1 "+5V" H 5565 1323 50  0000 C CNN
F 2 "" H 5550 1150 50  0001 C CNN
F 3 "" H 5550 1150 50  0001 C CNN
	1    5550 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1150 5550 1200
Wire Wire Line
	5550 1200 5650 1200
Wire Wire Line
	5650 1200 5650 1250
Connection ~ 5550 1200
Wire Wire Line
	5550 1200 5550 1250
Text Label 4500 1550 0    50   ~ 0
RESET
Wire Wire Line
	4500 1550 4750 1550
Text Label 4500 1750 0    50   ~ 0
XTAL1
Text Label 4500 1950 0    50   ~ 0
XTAL2
Wire Wire Line
	4500 1950 4750 1950
Wire Wire Line
	4750 1750 4500 1750
Text Label 7050 1700 0    50   ~ 0
XTAL1
Text Label 7850 1700 2    50   ~ 0
XTAL2
$Comp
L Device:Crystal_GND24 Y1
U 1 1 604F9635
P 7450 1700
F 0 "Y1" H 7600 1950 50  0000 L CNN
F 1 "Crystal_GND24" H 7600 1850 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 7450 1700 50  0001 C CNN
F 3 "~" H 7450 1700 50  0001 C CNN
	1    7450 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1700 7300 1700
Wire Wire Line
	7850 1700 7600 1700
$Comp
L power:GND #PWR05
U 1 1 604FA1B0
P 7450 1950
F 0 "#PWR05" H 7450 1700 50  0001 C CNN
F 1 "GND" H 7455 1777 50  0000 C CNN
F 2 "" H 7450 1950 50  0001 C CNN
F 3 "" H 7450 1950 50  0001 C CNN
	1    7450 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 604FA3F1
P 7200 1400
F 0 "#PWR03" H 7200 1150 50  0001 C CNN
F 1 "GND" H 7205 1227 50  0000 C CNN
F 2 "" H 7200 1400 50  0001 C CNN
F 3 "" H 7200 1400 50  0001 C CNN
	1    7200 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 1400 7450 1400
Wire Wire Line
	7450 1400 7450 1500
Wire Wire Line
	7450 1900 7450 1950
$Comp
L Device:C C1
U 1 1 604FAC35
P 7250 2550
F 0 "C1" H 7365 2596 50  0000 L CNN
F 1 "22p" H 7365 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7288 2400 50  0001 C CNN
F 3 "~" H 7250 2550 50  0001 C CNN
	1    7250 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 604FB048
P 7700 2550
F 0 "C2" H 7815 2596 50  0000 L CNN
F 1 "22p" H 7815 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7738 2400 50  0001 C CNN
F 3 "~" H 7700 2550 50  0001 C CNN
	1    7700 2550
	1    0    0    -1  
$EndComp
Text Label 7050 2350 0    50   ~ 0
XTAL1
Text Label 7500 2350 0    50   ~ 0
XTAL2
Wire Wire Line
	7050 2350 7250 2350
Wire Wire Line
	7250 2350 7250 2400
Wire Wire Line
	7500 2350 7700 2350
Wire Wire Line
	7700 2350 7700 2400
$Comp
L power:GND #PWR04
U 1 1 604FC2B8
P 7250 2750
F 0 "#PWR04" H 7250 2500 50  0001 C CNN
F 1 "GND" H 7255 2577 50  0000 C CNN
F 2 "" H 7250 2750 50  0001 C CNN
F 3 "" H 7250 2750 50  0001 C CNN
	1    7250 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 604FC5EB
P 7700 2750
F 0 "#PWR06" H 7700 2500 50  0001 C CNN
F 1 "GND" H 7705 2577 50  0000 C CNN
F 2 "" H 7700 2750 50  0001 C CNN
F 3 "" H 7700 2750 50  0001 C CNN
	1    7700 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2750 7700 2700
Wire Wire Line
	7250 2750 7250 2700
$Comp
L Device:R R6
U 1 1 604FD7BD
P 8150 2550
F 0 "R6" H 8220 2596 50  0000 L CNN
F 1 "1M" H 8220 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8080 2550 50  0001 C CNN
F 3 "~" H 8150 2550 50  0001 C CNN
	1    8150 2550
	1    0    0    -1  
$EndComp
Text Label 7950 2350 0    50   ~ 0
XTAL1
Text Label 8400 2800 2    50   ~ 0
XTAL2
Wire Wire Line
	8400 2800 8150 2800
Wire Wire Line
	8150 2800 8150 2700
Wire Wire Line
	7950 2350 8150 2350
Wire Wire Line
	8150 2350 8150 2400
$Comp
L Interface_USB:CP2102N-A01-GQFN28 U4
U 1 1 604FEAC3
P 1600 5700
F 0 "U4" H 1250 4450 50  0000 C CNN
F 1 "CP2102N-A01-GQFN28" V 1100 4950 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-28-1EP_5x5mm_P0.5mm_EP3.35x3.35mm" H 2050 4500 50  0001 L CNN
F 3 "https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf" H 1650 4950 50  0001 C CNN
	1    1600 5700
	-1   0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J5
U 1 1 60500F3B
P 3150 5100
F 0 "J5" H 2920 5089 50  0000 R CNN
F 1 "USB_B_Micro" V 3000 5900 50  0000 R CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 3300 5050 50  0001 C CNN
F 3 "~" H 3300 5050 50  0001 C CNN
	1    3150 5100
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 605029A2
P 2400 5100
F 0 "R19" V 2300 5050 50  0000 C CNN
F 1 "22" V 2300 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2330 5100 50  0001 C CNN
F 3 "~" H 2400 5100 50  0001 C CNN
	1    2400 5100
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 60503555
P 2400 5200
F 0 "R20" V 2300 5150 50  0000 C CNN
F 1 "22" V 2300 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2330 5200 50  0001 C CNN
F 3 "~" H 2400 5200 50  0001 C CNN
	1    2400 5200
	0    1    -1   0   
$EndComp
Wire Wire Line
	2250 5100 2100 5100
Wire Wire Line
	2100 5200 2250 5200
$Comp
L power:GND #PWR045
U 1 1 605055DF
P 1600 7050
F 0 "#PWR045" H 1600 6800 50  0001 C CNN
F 1 "GND" H 1605 6877 50  0000 C CNN
F 2 "" H 1600 7050 50  0001 C CNN
F 3 "" H 1600 7050 50  0001 C CNN
	1    1600 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 7050 1600 7000
$Comp
L power:GND #PWR060
U 1 1 60505EA2
P 3250 5600
F 0 "#PWR060" H 3250 5350 50  0001 C CNN
F 1 "GND" H 3255 5427 50  0000 C CNN
F 2 "" H 3250 5600 50  0001 C CNN
F 3 "" H 3250 5600 50  0001 C CNN
	1    3250 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 5600 3250 5550
Wire Wire Line
	3150 5500 3150 5550
Wire Wire Line
	3150 5550 3250 5550
Connection ~ 3250 5550
Wire Wire Line
	3250 5550 3250 5500
NoConn ~ 2850 5300
$Comp
L power:VBUS #PWR057
U 1 1 60507746
P 2800 4450
F 0 "#PWR057" H 2800 4300 50  0001 C CNN
F 1 "VBUS" H 2815 4623 50  0000 C CNN
F 2 "" H 2800 4450 50  0001 C CNN
F 3 "" H 2800 4450 50  0001 C CNN
	1    2800 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4900 2850 4900
$Comp
L Device:Fuse F2
U 1 1 6050852D
P 2800 4650
F 0 "F2" H 2860 4696 50  0000 L CNN
F 1 "0.5A" H 2860 4605 50  0000 L CNN
F 2 "Fuse:Fuse_2512_6332Metric" V 2730 4650 50  0001 C CNN
F 3 "~" H 2800 4650 50  0001 C CNN
	1    2800 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4800 2800 4900
Wire Wire Line
	2800 4450 2800 4500
$Comp
L power:VBUS #PWR051
U 1 1 6050A466
P 2250 4850
F 0 "#PWR051" H 2250 4700 50  0001 C CNN
F 1 "VBUS" H 2265 5023 50  0000 C CNN
F 2 "" H 2250 4850 50  0001 C CNN
F 3 "" H 2250 4850 50  0001 C CNN
	1    2250 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4850 2250 5000
Wire Wire Line
	2250 5000 2100 5000
NoConn ~ 2100 4800
$Comp
L Device:C C20
U 1 1 6050B833
P 3350 6850
F 0 "C20" H 3465 6896 50  0000 L CNN
F 1 "100n" H 3465 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3388 6700 50  0001 C CNN
F 3 "~" H 3350 6850 50  0001 C CNN
	1    3350 6850
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR061
U 1 1 6050BE61
P 3350 6650
F 0 "#PWR061" H 3350 6500 50  0001 C CNN
F 1 "VBUS" H 3365 6823 50  0000 C CNN
F 2 "" H 3350 6650 50  0001 C CNN
F 3 "" H 3350 6650 50  0001 C CNN
	1    3350 6650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR062
U 1 1 6050C15A
P 3350 7050
F 0 "#PWR062" H 3350 6800 50  0001 C CNN
F 1 "GND" H 3355 6877 50  0000 C CNN
F 2 "" H 3350 7050 50  0001 C CNN
F 3 "" H 3350 7050 50  0001 C CNN
	1    3350 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 7050 3350 7000
Wire Wire Line
	3350 6700 3350 6650
$Comp
L Device:C C14
U 1 1 6050F148
P 1850 4050
F 0 "C14" H 1965 4096 50  0000 L CNN
F 1 "10u" H 1965 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1888 3900 50  0001 C CNN
F 3 "~" H 1850 4050 50  0001 C CNN
	1    1850 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 6050F8A9
P 2250 4050
F 0 "C15" H 2365 4096 50  0000 L CNN
F 1 "100n" H 2365 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2288 3900 50  0001 C CNN
F 3 "~" H 2250 4050 50  0001 C CNN
	1    2250 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR050
U 1 1 6050FD1D
P 2250 4250
F 0 "#PWR050" H 2250 4000 50  0001 C CNN
F 1 "GND" H 2255 4077 50  0000 C CNN
F 2 "" H 2250 4250 50  0001 C CNN
F 3 "" H 2250 4250 50  0001 C CNN
	1    2250 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR048
U 1 1 60510D52
P 1850 4250
F 0 "#PWR048" H 1850 4000 50  0001 C CNN
F 1 "GND" H 1855 4077 50  0000 C CNN
F 2 "" H 1850 4250 50  0001 C CNN
F 3 "" H 1850 4250 50  0001 C CNN
	1    1850 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4250 1850 4200
Wire Wire Line
	2250 4250 2250 4200
Wire Wire Line
	1700 4400 1700 3850
Wire Wire Line
	1700 3850 1850 3850
Wire Wire Line
	2250 3850 2250 3900
Wire Wire Line
	1850 3900 1850 3850
Connection ~ 1850 3850
Wire Wire Line
	1850 3850 2250 3850
$Comp
L power:+5V #PWR046
U 1 1 605134E2
P 1700 3800
F 0 "#PWR046" H 1700 3650 50  0001 C CNN
F 1 "+5V" H 1715 3973 50  0000 C CNN
F 2 "" H 1700 3800 50  0001 C CNN
F 3 "" H 1700 3800 50  0001 C CNN
	1    1700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 3800 1700 3850
Connection ~ 1700 3850
$Comp
L Device:C C13
U 1 1 60514784
P 1250 4050
F 0 "C13" H 1365 4096 50  0000 L CNN
F 1 "100n" H 1365 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1288 3900 50  0001 C CNN
F 3 "~" H 1250 4050 50  0001 C CNN
	1    1250 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 4400 1600 3850
Wire Wire Line
	1600 3850 1250 3850
Wire Wire Line
	1250 3850 1250 3900
$Comp
L power:+3.3V #PWR043
U 1 1 60515CEE
P 1250 3800
F 0 "#PWR043" H 1250 3650 50  0001 C CNN
F 1 "+3.3V" H 1265 3973 50  0000 C CNN
F 2 "" H 1250 3800 50  0001 C CNN
F 3 "" H 1250 3800 50  0001 C CNN
	1    1250 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3800 1250 3850
Connection ~ 1250 3850
$Comp
L power:GND #PWR044
U 1 1 605174EF
P 1250 4250
F 0 "#PWR044" H 1250 4000 50  0001 C CNN
F 1 "GND" H 1255 4077 50  0000 C CNN
F 2 "" H 1250 4250 50  0001 C CNN
F 3 "" H 1250 4250 50  0001 C CNN
	1    1250 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4250 1250 4200
$Comp
L Device:C C16
U 1 1 605184F2
P 2500 6850
F 0 "C16" H 2615 6896 50  0000 L CNN
F 1 "10u" H 2615 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2538 6700 50  0001 C CNN
F 3 "~" H 2500 6850 50  0001 C CNN
	1    2500 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C18
U 1 1 6051898A
P 2900 6850
F 0 "C18" H 3015 6896 50  0000 L CNN
F 1 "100n" H 3015 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2938 6700 50  0001 C CNN
F 3 "~" H 2900 6850 50  0001 C CNN
	1    2900 6850
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR053
U 1 1 60518F8B
P 2500 6600
F 0 "#PWR053" H 2500 6450 50  0001 C CNN
F 1 "VBUS" H 2515 6773 50  0000 C CNN
F 2 "" H 2500 6600 50  0001 C CNN
F 3 "" H 2500 6600 50  0001 C CNN
	1    2500 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 6600 2500 6650
Wire Wire Line
	2500 6650 2900 6650
Wire Wire Line
	2900 6650 2900 6700
Connection ~ 2500 6650
Wire Wire Line
	2500 6650 2500 6700
$Comp
L power:GND #PWR058
U 1 1 6051B1EC
P 2900 7050
F 0 "#PWR058" H 2900 6800 50  0001 C CNN
F 1 "GND" H 2905 6877 50  0000 C CNN
F 2 "" H 2900 7050 50  0001 C CNN
F 3 "" H 2900 7050 50  0001 C CNN
	1    2900 7050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR054
U 1 1 6051B50B
P 2500 7050
F 0 "#PWR054" H 2500 6800 50  0001 C CNN
F 1 "GND" H 2505 6877 50  0000 C CNN
F 2 "" H 2500 7050 50  0001 C CNN
F 3 "" H 2500 7050 50  0001 C CNN
	1    2500 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 7050 2500 7000
Wire Wire Line
	2900 7050 2900 7000
$Comp
L Diode:ESD9B3.3ST5G D8
U 1 1 60521AAB
P 2450 5750
F 0 "D8" V 2404 5829 50  0000 L CNN
F 1 "ESD" V 2495 5829 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 2450 5750 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/ESD9B-D.PDF" H 2450 5750 50  0001 C CNN
	1    2450 5750
	0    1    1    0   
$EndComp
$Comp
L Diode:ESD9B3.3ST5G D9
U 1 1 60527A4A
P 2750 5750
F 0 "D9" V 2704 5829 50  0000 L CNN
F 1 "ESD" V 2795 5829 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 2750 5750 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/ESD9B-D.PDF" H 2750 5750 50  0001 C CNN
	1    2750 5750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR056
U 1 1 60528F1B
P 2750 5950
F 0 "#PWR056" H 2750 5700 50  0001 C CNN
F 1 "GND" H 2755 5777 50  0000 C CNN
F 2 "" H 2750 5950 50  0001 C CNN
F 3 "" H 2750 5950 50  0001 C CNN
	1    2750 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR052
U 1 1 6052922F
P 2450 5950
F 0 "#PWR052" H 2450 5700 50  0001 C CNN
F 1 "GND" H 2455 5777 50  0000 C CNN
F 2 "" H 2450 5950 50  0001 C CNN
F 3 "" H 2450 5950 50  0001 C CNN
	1    2450 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5950 2450 5900
Wire Wire Line
	2750 5950 2750 5900
Wire Wire Line
	2650 5200 2650 5500
Connection ~ 2650 5200
Wire Wire Line
	2650 5200 2850 5200
Wire Wire Line
	2550 5200 2650 5200
Wire Wire Line
	2550 5100 2750 5100
Wire Wire Line
	2450 5500 2450 5600
Wire Wire Line
	2450 5500 2650 5500
Wire Wire Line
	2750 5600 2750 5100
Connection ~ 2750 5100
Wire Wire Line
	2750 5100 2850 5100
NoConn ~ 1100 6800
NoConn ~ 1100 6700
NoConn ~ 1100 6600
NoConn ~ 1100 6500
NoConn ~ 1100 6400
NoConn ~ 1100 6300
NoConn ~ 1100 6200
NoConn ~ 1100 6000
NoConn ~ 1100 5900
NoConn ~ 1100 5800
NoConn ~ 1100 5600
NoConn ~ 1100 5500
NoConn ~ 1100 5300
NoConn ~ 1100 5200
NoConn ~ 1100 5000
NoConn ~ 1100 4700
NoConn ~ 1100 4600
Text Label 800  4800 0    50   ~ 0
U_RXD
Text Label 800  4900 0    50   ~ 0
U_TXD
Wire Wire Line
	800  4900 1100 4900
Wire Wire Line
	1100 4800 800  4800
$Comp
L Device:C C12
U 1 1 6054B921
P 700 5300
F 0 "C12" H 815 5346 50  0000 L CNN
F 1 "100n" H 815 5255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 738 5150 50  0001 C CNN
F 3 "~" H 700 5300 50  0001 C CNN
	1    700  5300
	1    0    0    -1  
$EndComp
Text Label 700  5700 1    50   ~ 0
RESET
Wire Wire Line
	700  5700 700  5450
Wire Wire Line
	700  5150 700  5100
Wire Wire Line
	700  5100 1100 5100
$Comp
L Amplifier_Operational:LMV358 U2
U 1 1 6055BF2A
P 8800 6250
F 0 "U2" H 8800 6617 50  0000 C CNN
F 1 "LMV358" H 8800 6526 50  0000 C CNN
F 2 "Package_SO:VSSOP-8_3.0x3.0mm_P0.65mm" H 8800 6250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 8800 6250 50  0001 C CNN
	1    8800 6250
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LMV358 U2
U 2 1 6055D4DE
P 2800 2650
F 0 "U2" H 2800 3017 50  0000 C CNN
F 1 "LMV358" H 2800 2926 50  0000 C CNN
F 2 "Package_SO:VSSOP-8_3.0x3.0mm_P0.65mm" H 2800 2650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 2800 2650 50  0001 C CNN
	2    2800 2650
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LMV358 U2
U 3 1 6055EA60
P 700 2450
F 0 "U2" H 658 2496 50  0000 L CNN
F 1 "LMV358" H 658 2405 50  0000 L CNN
F 2 "Package_SO:VSSOP-8_3.0x3.0mm_P0.65mm" H 700 2450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 700 2450 50  0001 C CNN
	3    700  2450
	1    0    0    -1  
$EndComp
Text Label 8300 6150 0    50   ~ 0
PB7
$Comp
L Device:R R18
U 1 1 605647D3
P 9350 6250
F 0 "R18" V 9143 6250 50  0000 C CNN
F 1 "1k" V 9234 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9280 6250 50  0001 C CNN
F 3 "~" H 9350 6250 50  0001 C CNN
	1    9350 6250
	0    1    1    0   
$EndComp
Wire Wire Line
	8300 6150 8500 6150
Wire Wire Line
	8500 6350 8450 6350
Wire Wire Line
	8450 6350 8450 6500
Wire Wire Line
	8450 6500 9150 6500
Wire Wire Line
	9150 6500 9150 6250
Wire Wire Line
	9150 6250 9200 6250
Wire Wire Line
	9150 6250 9100 6250
Connection ~ 9150 6250
$Comp
L Device:LED D5
U 1 1 6056AE4B
P 9700 6250
F 0 "D5" H 9693 5995 50  0000 C CNN
F 1 "L" H 9693 6086 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9700 6250 50  0001 C CNN
F 3 "~" H 9700 6250 50  0001 C CNN
	1    9700 6250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR038
U 1 1 6056B91D
P 9900 6300
F 0 "#PWR038" H 9900 6050 50  0001 C CNN
F 1 "GND" H 9905 6127 50  0000 C CNN
F 2 "" H 9900 6300 50  0001 C CNN
F 3 "" H 9900 6300 50  0001 C CNN
	1    9900 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 6250 9900 6250
Wire Wire Line
	9900 6250 9900 6300
Wire Wire Line
	9550 6250 9500 6250
$Comp
L FDN340P:FDN340P Q3
U 1 1 60573482
P 3400 2550
F 0 "Q3" H 3498 2596 50  0000 L CNN
F 1 "FDN340P" H 3498 2505 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3400 2550 50  0001 L BNN
F 3 "ON Semiconductor" H 3400 2550 50  0001 L BNN
F 4 "Die Assymertrical For Fdc6329l" H 3400 2550 50  0001 L BNN "Field4"
F 5 "9.90 USD" H 3400 2550 50  0001 L BNN "Field5"
F 6 "FDN340P" H 3400 2550 50  0001 L BNN "Field6"
F 7 "TO-236-3 Micross" H 3400 2550 50  0001 L BNN "Field7"
F 8 "Unavailable" H 3400 2550 50  0001 L BNN "Field8"
	1    3400 2550
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR041
U 1 1 60574C76
P 3400 2300
F 0 "#PWR041" H 3400 2150 50  0001 C CNN
F 1 "VBUS" H 3415 2473 50  0000 C CNN
F 2 "" H 3400 2300 50  0001 C CNN
F 3 "" H 3400 2300 50  0001 C CNN
	1    3400 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR042
U 1 1 60574EAA
P 3550 2900
F 0 "#PWR042" H 3550 2750 50  0001 C CNN
F 1 "+5V" H 3565 3073 50  0000 C CNN
F 2 "" H 3550 2900 50  0001 C CNN
F 3 "" H 3550 2900 50  0001 C CNN
	1    3550 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2750 3400 2900
Wire Wire Line
	3400 2900 3550 2900
Wire Wire Line
	3400 2300 3400 2350
Wire Wire Line
	3200 2650 3100 2650
$Comp
L Device:C C8
U 1 1 605885A8
P 1900 2600
F 0 "C8" H 2015 2646 50  0000 L CNN
F 1 "100n" H 2015 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1938 2450 50  0001 C CNN
F 3 "~" H 1900 2600 50  0001 C CNN
	1    1900 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR034
U 1 1 60589894
P 2350 2700
F 0 "#PWR034" H 2350 2550 50  0001 C CNN
F 1 "+3.3V" H 2365 2873 50  0000 C CNN
F 2 "" H 2350 2700 50  0001 C CNN
F 3 "" H 2350 2700 50  0001 C CNN
	1    2350 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2750 2350 2750
Wire Wire Line
	2500 2400 2500 2550
Wire Wire Line
	2350 2700 2350 2750
$Comp
L Device:R R16
U 1 1 60592084
P 1550 2600
F 0 "R16" H 1620 2646 50  0000 L CNN
F 1 "10k" H 1620 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1480 2600 50  0001 C CNN
F 3 "~" H 1550 2600 50  0001 C CNN
	1    1550 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 60592D8F
P 1550 2200
F 0 "R15" H 1620 2246 50  0000 L CNN
F 1 "10k" H 1620 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1480 2200 50  0001 C CNN
F 3 "~" H 1550 2200 50  0001 C CNN
	1    1550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2350 1550 2400
Wire Wire Line
	1550 2400 1900 2400
Connection ~ 1550 2400
Wire Wire Line
	1550 2400 1550 2450
Wire Wire Line
	1900 2400 1900 2450
Connection ~ 1900 2400
Wire Wire Line
	1900 2400 2500 2400
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY01
U 1 1 6059AC15
P 1550 2000
F 0 "#SUPPLY01" H 1600 2000 45  0001 L BNN
F 1 "VIN" H 1550 2170 45  0000 C CNN
F 2 "XXX-00000" H 1550 2181 60  0001 C CNN
F 3 "" H 1550 2000 60  0001 C CNN
	1    1550 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2000 1550 2050
$Comp
L power:GND #PWR033
U 1 1 605B1F3F
P 1900 2800
F 0 "#PWR033" H 1900 2550 50  0001 C CNN
F 1 "GND" H 1905 2627 50  0000 C CNN
F 2 "" H 1900 2800 50  0001 C CNN
F 3 "" H 1900 2800 50  0001 C CNN
	1    1900 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 605B2349
P 1550 2800
F 0 "#PWR032" H 1550 2550 50  0001 C CNN
F 1 "GND" H 1555 2627 50  0000 C CNN
F 2 "" H 1550 2800 50  0001 C CNN
F 3 "" H 1550 2800 50  0001 C CNN
	1    1550 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2750 1550 2800
Wire Wire Line
	1900 2750 1900 2800
$Comp
L power:GND #PWR020
U 1 1 605C0662
P 600 2800
F 0 "#PWR020" H 600 2550 50  0001 C CNN
F 1 "GND" H 605 2627 50  0000 C CNN
F 2 "" H 600 2800 50  0001 C CNN
F 3 "" H 600 2800 50  0001 C CNN
	1    600  2800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR019
U 1 1 605C07C0
P 600 2050
F 0 "#PWR019" H 600 1900 50  0001 C CNN
F 1 "+5V" H 615 2223 50  0000 C CNN
F 2 "" H 600 2050 50  0001 C CNN
F 3 "" H 600 2050 50  0001 C CNN
	1    600  2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	600  2050 600  2100
Wire Wire Line
	600  2800 600  2750
$Comp
L Device:C C7
U 1 1 605D1FFF
P 1050 2450
F 0 "C7" H 1165 2496 50  0000 L CNN
F 1 "100n" H 1165 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1088 2300 50  0001 C CNN
F 3 "~" H 1050 2450 50  0001 C CNN
	1    1050 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	600  2100 1050 2100
Wire Wire Line
	1050 2100 1050 2300
Connection ~ 600  2100
Wire Wire Line
	600  2100 600  2150
$Comp
L power:GND #PWR025
U 1 1 605D5D19
P 1050 2800
F 0 "#PWR025" H 1050 2550 50  0001 C CNN
F 1 "GND" H 1055 2627 50  0000 C CNN
F 2 "" H 1050 2800 50  0001 C CNN
F 3 "" H 1050 2800 50  0001 C CNN
	1    1050 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 2800 1050 2600
$Comp
L Device:R R17
U 1 1 605D942B
P 9350 5550
F 0 "R17" V 9143 5550 50  0000 C CNN
F 1 "1k" V 9234 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9280 5550 50  0001 C CNN
F 3 "~" H 9350 5550 50  0001 C CNN
	1    9350 5550
	0    1    1    0   
$EndComp
$Comp
L Device:LED D4
U 1 1 605D95E2
P 9700 5550
F 0 "D4" H 9693 5295 50  0000 C CNN
F 1 "ON" H 9693 5386 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9700 5550 50  0001 C CNN
F 3 "~" H 9700 5550 50  0001 C CNN
	1    9700 5550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 605DAC89
P 9900 5600
F 0 "#PWR037" H 9900 5350 50  0001 C CNN
F 1 "GND" H 9905 5427 50  0000 C CNN
F 2 "" H 9900 5600 50  0001 C CNN
F 3 "" H 9900 5600 50  0001 C CNN
	1    9900 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 5550 9900 5550
Wire Wire Line
	9900 5550 9900 5600
Wire Wire Line
	9550 5550 9500 5550
$Comp
L power:+5V #PWR035
U 1 1 605E1EF0
P 9150 5500
F 0 "#PWR035" H 9150 5350 50  0001 C CNN
F 1 "+5V" H 9165 5673 50  0000 C CNN
F 2 "" H 9150 5500 50  0001 C CNN
F 3 "" H 9150 5500 50  0001 C CNN
	1    9150 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 5500 9150 5550
Wire Wire Line
	9150 5550 9200 5550
Text Label 9200 3000 0    50   ~ 0
RESET
$Comp
L Switch:SW_Push SW1
U 1 1 605E63EB
P 9400 3250
F 0 "SW1" V 9354 3398 50  0000 L CNN
F 1 "SW_Push" V 9445 3398 50  0000 L CNN
F 2 "Kicad_My_Symbol:Button_Switch_3.2x4.2" H 9400 3450 50  0001 C CNN
F 3 "~" H 9400 3450 50  0001 C CNN
	1    9400 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	9200 3000 9400 3000
Wire Wire Line
	9400 3000 9400 3050
$Comp
L power:GND #PWR022
U 1 1 605EB1A9
P 9400 3500
F 0 "#PWR022" H 9400 3250 50  0001 C CNN
F 1 "GND" H 9405 3327 50  0000 C CNN
F 2 "" H 9400 3500 50  0001 C CNN
F 3 "" H 9400 3500 50  0001 C CNN
	1    9400 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3450 9400 3500
$Comp
L Device:C C4
U 1 1 605F2CB9
P 9050 3200
F 0 "C4" H 9165 3246 50  0000 L CNN
F 1 "22p" H 9165 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9088 3050 50  0001 C CNN
F 3 "~" H 9050 3200 50  0001 C CNN
	1    9050 3200
	1    0    0    -1  
$EndComp
Text Label 8850 3000 0    50   ~ 0
RESET
Wire Wire Line
	8850 3000 9050 3000
Wire Wire Line
	9050 3000 9050 3050
$Comp
L power:GND #PWR016
U 1 1 605FBA50
P 9050 3400
F 0 "#PWR016" H 9050 3150 50  0001 C CNN
F 1 "GND" H 9055 3227 50  0000 C CNN
F 2 "" H 9050 3400 50  0001 C CNN
F 3 "" H 9050 3400 50  0001 C CNN
	1    9050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3350 9050 3400
$Comp
L Device:D D3
U 1 1 605FFD5D
P 9200 2600
F 0 "D3" V 9154 2680 50  0000 L CNN
F 1 "D" V 9245 2680 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 9200 2600 50  0001 C CNN
F 3 "~" H 9200 2600 50  0001 C CNN
	1    9200 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 60600978
P 8850 2600
F 0 "R14" H 8920 2646 50  0000 L CNN
F 1 "10k" H 8920 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8780 2600 50  0001 C CNN
F 3 "~" H 8850 2600 50  0001 C CNN
	1    8850 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 60601DF5
P 8850 2400
F 0 "#PWR015" H 8850 2250 50  0001 C CNN
F 1 "+5V" H 8865 2573 50  0000 C CNN
F 2 "" H 8850 2400 50  0001 C CNN
F 3 "" H 8850 2400 50  0001 C CNN
	1    8850 2400
	1    0    0    -1  
$EndComp
Text Label 9100 2850 2    50   ~ 0
RESET
Text Label 9450 2850 2    50   ~ 0
RESET
$Comp
L power:+5V #PWR021
U 1 1 60602C44
P 9200 2400
F 0 "#PWR021" H 9200 2250 50  0001 C CNN
F 1 "+5V" H 9215 2573 50  0000 C CNN
F 2 "" H 9200 2400 50  0001 C CNN
F 3 "" H 9200 2400 50  0001 C CNN
	1    9200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2400 9200 2450
Wire Wire Line
	8850 2400 8850 2450
Wire Wire Line
	8850 2750 8850 2850
Wire Wire Line
	8850 2850 9100 2850
Wire Wire Line
	9200 2750 9200 2850
Wire Wire Line
	9200 2850 9450 2850
$Comp
L Device:R R1
U 1 1 60631B5D
P 10450 5350
F 0 "R1" H 10380 5304 50  0000 R CNN
F 1 "1k" H 10380 5395 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10380 5350 50  0001 C CNN
F 3 "~" H 10450 5350 50  0001 C CNN
	1    10450 5350
	-1   0    0    1   
$EndComp
Text Label 10200 5150 0    50   ~ 0
U_RXD
Text Label 10650 5150 0    50   ~ 0
U_TXD
Text Label 10650 5550 2    50   ~ 0
TX0
Wire Wire Line
	10200 5150 10450 5150
Wire Wire Line
	10450 5150 10450 5200
Wire Wire Line
	10650 5550 10450 5550
Wire Wire Line
	10450 5550 10450 5500
$Comp
L Device:R R2
U 1 1 6063D12E
P 10450 5750
F 0 "R2" H 10380 5704 50  0000 R CNN
F 1 "1k" H 10380 5795 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10380 5750 50  0001 C CNN
F 3 "~" H 10450 5750 50  0001 C CNN
	1    10450 5750
	-1   0    0    1   
$EndComp
Wire Wire Line
	10450 5550 10450 5600
Connection ~ 10450 5550
$Comp
L Device:LED D1
U 1 1 60642956
P 10450 6100
F 0 "D1" V 10489 5982 50  0000 R CNN
F 1 "TXD" V 10398 5982 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 10450 6100 50  0001 C CNN
F 3 "~" H 10450 6100 50  0001 C CNN
	1    10450 6100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 60643717
P 10450 6300
F 0 "#PWR07" H 10450 6050 50  0001 C CNN
F 1 "GND" H 10455 6127 50  0000 C CNN
F 2 "" H 10450 6300 50  0001 C CNN
F 3 "" H 10450 6300 50  0001 C CNN
	1    10450 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 6300 10450 6250
Wire Wire Line
	10450 5950 10450 5900
$Comp
L Device:R R8
U 1 1 6064DF52
P 10900 5350
F 0 "R8" H 10830 5304 50  0000 R CNN
F 1 "1k" H 10830 5395 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10830 5350 50  0001 C CNN
F 3 "~" H 10900 5350 50  0001 C CNN
	1    10900 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	10650 5150 10900 5150
Wire Wire Line
	10900 5150 10900 5200
Wire Wire Line
	11100 5550 10900 5550
Wire Wire Line
	10900 5550 10900 5500
$Comp
L Device:R R9
U 1 1 6064DF62
P 10900 5750
F 0 "R9" H 10830 5704 50  0000 R CNN
F 1 "1k" H 10830 5795 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10830 5750 50  0001 C CNN
F 3 "~" H 10900 5750 50  0001 C CNN
	1    10900 5750
	-1   0    0    1   
$EndComp
Wire Wire Line
	10900 5550 10900 5600
Connection ~ 10900 5550
$Comp
L Device:LED D2
U 1 1 6064DF6E
P 10900 6100
F 0 "D2" V 10939 5982 50  0000 R CNN
F 1 "RXD" V 10848 5982 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 10900 6100 50  0001 C CNN
F 3 "~" H 10900 6100 50  0001 C CNN
	1    10900 6100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 6064DF78
P 10900 6300
F 0 "#PWR012" H 10900 6050 50  0001 C CNN
F 1 "GND" H 10905 6127 50  0000 C CNN
F 2 "" H 10900 6300 50  0001 C CNN
F 3 "" H 10900 6300 50  0001 C CNN
	1    10900 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 6300 10900 6250
Wire Wire Line
	10900 5950 10900 5900
Text Label 6650 5250 2    50   ~ 0
TX0
Text Label 11100 5550 2    50   ~ 0
RX0
Text Label 6650 5150 2    50   ~ 0
RX0
Wire Wire Line
	6650 5150 6350 5150
Wire Wire Line
	6350 5250 6650 5250
$Comp
L Device:Fuse F1
U 1 1 6076B09D
P 1200 850
F 0 "F1" V 1003 850 50  0000 C CNN
F 1 "Fuse" V 1094 850 50  0000 C CNN
F 2 "Fuse:Fuse_2512_6332Metric" V 1130 850 50  0001 C CNN
F 3 "~" H 1200 850 50  0001 C CNN
	1    1200 850 
	0    1    1    0   
$EndComp
$Comp
L Device:D D6
U 1 1 6076BB4F
P 1600 850
F 0 "D6" H 1600 633 50  0000 C CNN
F 1 "D" H 1600 724 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 1600 850 50  0001 C CNN
F 3 "~" H 1600 850 50  0001 C CNN
	1    1600 850 
	-1   0    0    1   
$EndComp
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY03
U 1 1 6076C430
P 1800 800
F 0 "#SUPPLY03" H 1850 800 45  0001 L BNN
F 1 "VIN" H 1800 970 45  0000 C CNN
F 2 "XXX-00000" H 1800 981 60  0001 C CNN
F 3 "" H 1800 800 60  0001 C CNN
	1    1800 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR040
U 1 1 6076D2C5
P 950 1100
F 0 "#PWR040" H 950 850 50  0001 C CNN
F 1 "GND" H 955 927 50  0000 C CNN
F 2 "" H 950 1100 50  0001 C CNN
F 3 "" H 950 1100 50  0001 C CNN
	1    950  1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  950  950  950 
Wire Wire Line
	900  850  1050 850 
Wire Wire Line
	1350 850  1450 850 
Wire Wire Line
	1750 850  1800 850 
Wire Wire Line
	1800 850  1800 800 
Text Label 6650 3150 2    50   ~ 0
PB7
Wire Wire Line
	6650 3150 6350 3150
$Comp
L Device:C C3
U 1 1 607B5B7F
P 7250 3650
F 0 "C3" H 7365 3696 50  0000 L CNN
F 1 "100n" H 7365 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7288 3500 50  0001 C CNN
F 3 "~" H 7250 3650 50  0001 C CNN
	1    7250 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 607B6476
P 7250 3450
F 0 "#PWR013" H 7250 3300 50  0001 C CNN
F 1 "+5V" H 7265 3623 50  0000 C CNN
F 2 "" H 7250 3450 50  0001 C CNN
F 3 "" H 7250 3450 50  0001 C CNN
	1    7250 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 607B6B0E
P 7250 3850
F 0 "#PWR014" H 7250 3600 50  0001 C CNN
F 1 "GND" H 7255 3677 50  0000 C CNN
F 2 "" H 7250 3850 50  0001 C CNN
F 3 "" H 7250 3850 50  0001 C CNN
	1    7250 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3850 7250 3800
Wire Wire Line
	7250 3450 7250 3500
$Comp
L Device:C C5
U 1 1 607C9F35
P 7700 3650
F 0 "C5" H 7815 3696 50  0000 L CNN
F 1 "100n" H 7815 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7738 3500 50  0001 C CNN
F 3 "~" H 7700 3650 50  0001 C CNN
	1    7700 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 607C9F3F
P 7700 3450
F 0 "#PWR017" H 7700 3300 50  0001 C CNN
F 1 "+5V" H 7715 3623 50  0000 C CNN
F 2 "" H 7700 3450 50  0001 C CNN
F 3 "" H 7700 3450 50  0001 C CNN
	1    7700 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 607C9F49
P 7700 3850
F 0 "#PWR018" H 7700 3600 50  0001 C CNN
F 1 "GND" H 7705 3677 50  0000 C CNN
F 2 "" H 7700 3850 50  0001 C CNN
F 3 "" H 7700 3850 50  0001 C CNN
	1    7700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 3850 7700 3800
Wire Wire Line
	7700 3450 7700 3500
$Comp
L Device:C C6
U 1 1 607D448F
P 8150 3650
F 0 "C6" H 8265 3696 50  0000 L CNN
F 1 "100n" H 8265 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8188 3500 50  0001 C CNN
F 3 "~" H 8150 3650 50  0001 C CNN
	1    8150 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 607D4499
P 8150 3450
F 0 "#PWR023" H 8150 3300 50  0001 C CNN
F 1 "+5V" H 8165 3623 50  0000 C CNN
F 2 "" H 8150 3450 50  0001 C CNN
F 3 "" H 8150 3450 50  0001 C CNN
	1    8150 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 607D44A3
P 8150 3850
F 0 "#PWR024" H 8150 3600 50  0001 C CNN
F 1 "GND" H 8155 3677 50  0000 C CNN
F 2 "" H 8150 3850 50  0001 C CNN
F 3 "" H 8150 3850 50  0001 C CNN
	1    8150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 3850 8150 3800
Wire Wire Line
	8150 3450 8150 3500
Text Label 6650 4350 2    50   ~ 0
SDA
Text Label 6650 4250 2    50   ~ 0
SCL
Wire Wire Line
	6650 4250 6350 4250
Wire Wire Line
	6350 4350 6650 4350
Text Label 9850 2750 0    50   ~ 0
SCL
Text Label 9850 2850 0    50   ~ 0
SDA
$Comp
L Device:R R3
U 1 1 607F7A6A
P 10100 2550
F 0 "R3" H 10170 2596 50  0000 L CNN
F 1 "4.7k" H 10170 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10030 2550 50  0001 C CNN
F 3 "~" H 10100 2550 50  0001 C CNN
	1    10100 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 6080BF8F
P 10450 2550
F 0 "R7" H 10520 2596 50  0000 L CNN
F 1 "4.7k" H 10520 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10380 2550 50  0001 C CNN
F 3 "~" H 10450 2550 50  0001 C CNN
	1    10450 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2750 10100 2750
Wire Wire Line
	10100 2750 10100 2700
Wire Wire Line
	9850 2850 10450 2850
Wire Wire Line
	10450 2850 10450 2700
$Comp
L power:+5V #PWR08
U 1 1 608218A2
P 10100 2350
F 0 "#PWR08" H 10100 2200 50  0001 C CNN
F 1 "+5V" H 10115 2523 50  0000 C CNN
F 2 "" H 10100 2350 50  0001 C CNN
F 3 "" H 10100 2350 50  0001 C CNN
	1    10100 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 60821E73
P 10450 2350
F 0 "#PWR011" H 10450 2200 50  0001 C CNN
F 1 "+5V" H 10465 2523 50  0000 C CNN
F 2 "" H 10450 2350 50  0001 C CNN
F 3 "" H 10450 2350 50  0001 C CNN
	1    10450 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 2350 10100 2400
Wire Wire Line
	10450 2350 10450 2400
Wire Wire Line
	4500 5150 4750 5150
Wire Wire Line
	4750 5250 4500 5250
Text Label 6600 5550 2    50   ~ 0
D2
Wire Wire Line
	6600 5550 6350 5550
Text Label 6600 5650 2    50   ~ 0
D3
Wire Wire Line
	6350 5650 6600 5650
$Comp
L Mechanical:MountingHole H1
U 1 1 60564D19
P 10500 600
F 0 "H1" H 10600 646 50  0000 L CNN
F 1 "MountingHole" H 10600 555 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad_Via" H 10500 600 50  0001 C CNN
F 3 "~" H 10500 600 50  0001 C CNN
	1    10500 600 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 605656E6
P 10500 800
F 0 "H2" H 10600 846 50  0000 L CNN
F 1 "MountingHole" H 10600 755 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad_Via" H 10500 800 50  0001 C CNN
F 3 "~" H 10500 800 50  0001 C CNN
	1    10500 800 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 60565A16
P 10500 1000
F 0 "H3" H 10600 1046 50  0000 L CNN
F 1 "MountingHole" H 10600 955 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad_Via" H 10500 1000 50  0001 C CNN
F 3 "~" H 10500 1000 50  0001 C CNN
	1    10500 1000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 60565BA0
P 10500 1200
F 0 "H4" H 10600 1246 50  0000 L CNN
F 1 "MountingHole" H 10600 1155 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad_Via" H 10500 1200 50  0001 C CNN
F 3 "~" H 10500 1200 50  0001 C CNN
	1    10500 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 60790905
P 700 850
F 0 "J2" H 618 1067 50  0000 C CNN
F 1 "POWER_IN" H 618 976 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical" H 700 850 50  0001 C CNN
F 3 "~" H 700 850 50  0001 C CNN
	1    700  850 
	-1   0    0    -1  
$EndComp
Wire Wire Line
	950  950  950  1100
$Comp
L Regulator_Linear:LM1084-5.0 U3
U 1 1 60792EF5
P 2600 900
F 0 "U3" H 2600 1142 50  0000 C CNN
F 1 "LM1084-5.0" H 2600 1051 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 2600 1150 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm1084.pdf" H 2600 900 50  0001 C CNN
	1    2600 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C9
U 1 1 607942A1
P 2050 1100
F 0 "C9" H 2165 1146 50  0000 L CNN
F 1 "470u" H 2165 1055 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 2050 1100 50  0001 C CNN
F 3 "~" H 2050 1100 50  0001 C CNN
	1    2050 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C10
U 1 1 60795525
P 3000 1100
F 0 "C10" H 3115 1146 50  0000 L CNN
F 1 "470u" H 3115 1055 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 3000 1100 50  0001 C CNN
F 3 "~" H 3000 1100 50  0001 C CNN
	1    3000 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 900  2050 900 
Wire Wire Line
	2050 900  2050 950 
Wire Wire Line
	2900 900  3000 900 
Wire Wire Line
	3000 900  3000 950 
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY02
U 1 1 607A84D0
P 2050 800
F 0 "#SUPPLY02" H 2100 800 45  0001 L BNN
F 1 "VIN" H 2050 970 45  0000 C CNN
F 2 "XXX-00000" H 2050 981 60  0001 C CNN
F 3 "" H 2050 800 60  0001 C CNN
	1    2050 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 800  2050 900 
Connection ~ 2050 900 
$Comp
L power:+5V #PWR028
U 1 1 607B1F70
P 3000 800
F 0 "#PWR028" H 3000 650 50  0001 C CNN
F 1 "+5V" H 3015 973 50  0000 C CNN
F 2 "" H 3000 800 50  0001 C CNN
F 3 "" H 3000 800 50  0001 C CNN
	1    3000 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 800  3000 900 
Connection ~ 3000 900 
$Comp
L power:GND #PWR026
U 1 1 607BC270
P 2050 1300
F 0 "#PWR026" H 2050 1050 50  0001 C CNN
F 1 "GND" H 2055 1127 50  0000 C CNN
F 2 "" H 2050 1300 50  0001 C CNN
F 3 "" H 2050 1300 50  0001 C CNN
	1    2050 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 607BC581
P 2600 1300
F 0 "#PWR027" H 2600 1050 50  0001 C CNN
F 1 "GND" H 2605 1127 50  0000 C CNN
F 2 "" H 2600 1300 50  0001 C CNN
F 3 "" H 2600 1300 50  0001 C CNN
	1    2600 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 607BC878
P 3000 1300
F 0 "#PWR029" H 3000 1050 50  0001 C CNN
F 1 "GND" H 3005 1127 50  0000 C CNN
F 2 "" H 3000 1300 50  0001 C CNN
F 3 "" H 3000 1300 50  0001 C CNN
	1    3000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1250 3000 1300
Wire Wire Line
	2600 1200 2600 1300
Wire Wire Line
	2050 1250 2050 1300
Text HLabel 4450 6550 0    50   Input ~ 0
D4
Text HLabel 6650 5450 2    50   Output ~ 0
D5
Text HLabel 4450 5450 0    50   Output ~ 0
D6
Wire Wire Line
	6650 5450 6350 5450
Wire Wire Line
	4450 6550 4750 6550
Wire Wire Line
	4450 5450 4750 5450
Text HLabel 4500 5250 0    50   Output ~ 0
TXD2
Text HLabel 4500 5150 0    50   Input ~ 0
RXD2
Text HLabel 6650 4550 2    50   Output ~ 0
TXD1
Text HLabel 6650 4450 2    50   Input ~ 0
RXD1
Wire Wire Line
	6650 4450 6350 4450
Wire Wire Line
	6350 4550 6650 4550
Wire Wire Line
	4500 4250 4750 4250
Wire Wire Line
	4750 4350 4500 4350
Text HLabel 4500 4350 0    50   Output ~ 0
TXD3
Text HLabel 4500 4250 0    50   Input ~ 0
RXD3
Text HLabel 4450 5650 0    50   BiDi ~ 0
D8
Text HLabel 4450 5750 0    50   BiDi ~ 0
D9
Wire Wire Line
	4450 5750 4750 5750
Wire Wire Line
	4750 5650 4450 5650
Text Notes 650  1650 0    118  ~ 24
Power In
Text Notes 650  3300 0    118  ~ 24
Power Switching VIN/VBUS
Wire Bus Line
	500  3400 3900 3400
Wire Bus Line
	3900 3400 3900 1700
Wire Bus Line
	500  1700 3900 1700
Connection ~ 3900 1700
Wire Bus Line
	3900 1700 3900 500 
Text Notes 700  7600 0    118  ~ 24
USB
Wire Bus Line
	3900 3400 3900 7750
Connection ~ 3900 3400
Text Notes 8300 5300 0    118  ~ 24
LED
Wire Bus Line
	8200 6500 8200 5050
Wire Bus Line
	8200 5050 11200 5050
Wire Notes Line
	10100 5050 10100 6500
Wire Notes Line
	10350 500  10350 1350
Wire Notes Line
	10350 1350 11200 1350
Text Notes 4050 750  0    118  ~ 24
ATMEGA2560-16AU
$EndSCHEMATC
