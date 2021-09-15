EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L My_Symbol:microSIM_Holder_0181AAAA06A U6
U 1 1 605E648E
P 4100 6250
F 0 "U6" H 4100 6715 50  0000 C CNN
F 1 "microSIM_Holder_0181AAAA06A" H 4100 6624 50  0000 C CNN
F 2 "Kicad_My_Symbol:microSIM_Holder_0181AAAA06A" H 4100 6300 50  0001 C CNN
F 3 "" H 4100 6300 50  0001 C CNN
	1    4100 6250
	1    0    0    -1  
$EndComp
$Comp
L Power_Protection:ESDA6V1-5SC6 D10
U 1 1 605E6CC1
P 3250 6850
F 0 "D10" H 3580 6896 50  0000 L CNN
F 1 "ESDA6V1-5SC6" H 3580 6805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 3950 6600 50  0001 C CNN
F 3 "www.st.com/resource/en/datasheet/esda6v1-5sc6.pdf" V 3250 6850 50  0001 C CNN
	1    3250 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 605E8252
P 2550 6200
F 0 "R10" V 2500 6000 50  0000 C CNN
F 1 "510" V 2500 6400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 6200 50  0001 C CNN
F 3 "~" H 2550 6200 50  0001 C CNN
	1    2550 6200
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 605E9544
P 2550 6350
F 0 "R11" V 2500 6150 50  0000 C CNN
F 1 "510" V 2500 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 6350 50  0001 C CNN
F 3 "~" H 2550 6350 50  0001 C CNN
	1    2550 6350
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 605E9B00
P 2550 6500
F 0 "R12" V 2500 6300 50  0000 C CNN
F 1 "510" V 2500 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 6500 50  0001 C CNN
F 3 "~" H 2550 6500 50  0001 C CNN
	1    2550 6500
	0    1    1    0   
$EndComp
$Comp
L Device:C C26
U 1 1 605EC49F
P 2100 6750
F 0 "C26" H 2215 6796 50  0000 L CNN
F 1 "100n" H 2215 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2138 6600 50  0001 C CNN
F 3 "~" H 2100 6750 50  0001 C CNN
	1    2100 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 605ECFA6
P 1700 6750
F 0 "C23" H 1815 6796 50  0000 L CNN
F 1 "22p" H 1815 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1738 6600 50  0001 C CNN
F 3 "~" H 1700 6750 50  0001 C CNN
	1    1700 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 605EDA67
P 1350 6750
F 0 "C22" H 1465 6796 50  0000 L CNN
F 1 "22p" H 1465 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1388 6600 50  0001 C CNN
F 3 "~" H 1350 6750 50  0001 C CNN
	1    1350 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 605EDD5D
P 1000 6750
F 0 "C19" H 1115 6796 50  0000 L CNN
F 1 "22p" H 1115 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1038 6600 50  0001 C CNN
F 3 "~" H 1000 6750 50  0001 C CNN
	1    1000 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 6600 2100 6050
Wire Wire Line
	2400 6200 1700 6200
Wire Wire Line
	1700 6200 1700 6600
Wire Wire Line
	2400 6350 1350 6350
Wire Wire Line
	1350 6350 1350 6600
Wire Wire Line
	2400 6500 1000 6500
Wire Wire Line
	1000 6500 1000 6600
Text Label 650  6050 0    50   ~ 0
SIM_VDD
Text Label 650  6200 0    50   ~ 0
SIM_RST
Text Label 650  6350 0    50   ~ 0
SIM_CLK
Text Label 650  6500 0    50   ~ 0
SIM_DATA
Wire Wire Line
	650  6500 1000 6500
Connection ~ 1000 6500
Wire Wire Line
	650  6350 1350 6350
Connection ~ 1350 6350
Wire Wire Line
	650  6200 1700 6200
Connection ~ 1700 6200
Wire Wire Line
	650  6050 2100 6050
Connection ~ 2100 6050
$Comp
L power:GND #PWR036
U 1 1 605F04C5
P 1000 6950
F 0 "#PWR036" H 1000 6700 50  0001 C CNN
F 1 "GND" H 1005 6777 50  0000 C CNN
F 2 "" H 1000 6950 50  0001 C CNN
F 3 "" H 1000 6950 50  0001 C CNN
	1    1000 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 605F0E6E
P 1350 6950
F 0 "#PWR047" H 1350 6700 50  0001 C CNN
F 1 "GND" H 1355 6777 50  0000 C CNN
F 2 "" H 1350 6950 50  0001 C CNN
F 3 "" H 1350 6950 50  0001 C CNN
	1    1350 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR059
U 1 1 605F1092
P 1700 6950
F 0 "#PWR059" H 1700 6700 50  0001 C CNN
F 1 "GND" H 1705 6777 50  0000 C CNN
F 2 "" H 1700 6950 50  0001 C CNN
F 3 "" H 1700 6950 50  0001 C CNN
	1    1700 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR065
U 1 1 605F13B2
P 2100 6950
F 0 "#PWR065" H 2100 6700 50  0001 C CNN
F 1 "GND" H 2105 6777 50  0000 C CNN
F 2 "" H 2100 6950 50  0001 C CNN
F 3 "" H 2100 6950 50  0001 C CNN
	1    2100 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR068
U 1 1 605F16AA
P 3250 7100
F 0 "#PWR068" H 3250 6850 50  0001 C CNN
F 1 "GND" H 3255 6927 50  0000 C CNN
F 2 "" H 3250 7100 50  0001 C CNN
F 3 "" H 3250 7100 50  0001 C CNN
	1    3250 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR069
U 1 1 605F1C29
P 4550 6600
F 0 "#PWR069" H 4550 6350 50  0001 C CNN
F 1 "GND" H 4555 6427 50  0000 C CNN
F 2 "" H 4550 6600 50  0001 C CNN
F 3 "" H 4550 6600 50  0001 C CNN
	1    4550 6600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR070
U 1 1 605F2070
P 4750 6600
F 0 "#PWR070" H 4750 6350 50  0001 C CNN
F 1 "GND" H 4755 6427 50  0000 C CNN
F 2 "" H 4750 6600 50  0001 C CNN
F 3 "" H 4750 6600 50  0001 C CNN
	1    4750 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 6600 4750 6200
Wire Wire Line
	4750 6200 4500 6200
Wire Wire Line
	4500 6550 4550 6550
Wire Wire Line
	4550 6550 4550 6600
Wire Wire Line
	3250 7100 3250 7050
Wire Wire Line
	2100 6950 2100 6900
Wire Wire Line
	1700 6950 1700 6900
Wire Wire Line
	1350 6950 1350 6900
Wire Wire Line
	1000 6950 1000 6900
Text Label 7150 1950 2    50   ~ 0
SIM_VDD
Text Label 7150 2250 2    50   ~ 0
SIM_RST
Text Label 7150 2150 2    50   ~ 0
SIM_CLK
Text Label 7150 2050 2    50   ~ 0
SIM_DATA
Wire Wire Line
	7150 2250 6750 2250
Wire Wire Line
	6750 2150 7150 2150
Wire Wire Line
	6750 2050 7150 2050
Wire Wire Line
	6750 1950 7150 1950
$Comp
L power:GND #PWR072
U 1 1 605F9DF7
P 5950 3600
F 0 "#PWR072" H 5950 3350 50  0001 C CNN
F 1 "GND" H 5955 3427 50  0000 C CNN
F 2 "" H 5950 3600 50  0001 C CNN
F 3 "" H 5950 3600 50  0001 C CNN
	1    5950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3600 5950 3550
$Comp
L Transistor_BJT:BC848 Q5
U 1 1 605FC063
P 10250 1450
F 0 "Q5" V 10485 1450 50  0000 C CNN
F 1 "1P" V 10576 1450 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10450 1375 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 10250 1450 50  0001 L CNN
	1    10250 1450
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR071
U 1 1 605FD1EA
P 5750 1250
F 0 "#PWR071" H 5750 1100 50  0001 C CNN
F 1 "VDD" H 5765 1423 50  0000 C CNN
F 2 "" H 5750 1250 50  0001 C CNN
F 3 "" H 5750 1250 50  0001 C CNN
	1    5750 1250
	1    0    0    -1  
$EndComp
Text Label 6300 1150 2    50   ~ 0
VBAT
Wire Wire Line
	6300 1150 6150 1150
Wire Wire Line
	6150 1150 6150 1350
Wire Wire Line
	5750 1350 5750 1250
$Comp
L Device:R R36
U 1 1 606006BF
P 10600 1350
F 0 "R36" H 10670 1396 50  0000 L CNN
F 1 "4.7k" H 10670 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10530 1350 50  0001 C CNN
F 3 "~" H 10600 1350 50  0001 C CNN
	1    10600 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R31
U 1 1 60600F90
P 10250 1000
F 0 "R31" H 10320 1046 50  0000 L CNN
F 1 "4.7k" H 10320 955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10180 1000 50  0001 C CNN
F 3 "~" H 10250 1000 50  0001 C CNN
	1    10250 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R27
U 1 1 60601230
P 9900 1350
F 0 "R27" H 9970 1396 50  0000 L CNN
F 1 "47k" H 9970 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9830 1350 50  0001 C CNN
F 3 "~" H 9900 1350 50  0001 C CNN
	1    9900 1350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR081
U 1 1 606015AA
P 10600 1150
F 0 "#PWR081" H 10600 1000 50  0001 C CNN
F 1 "+5V" H 10615 1323 50  0000 C CNN
F 2 "" H 10600 1150 50  0001 C CNN
F 3 "" H 10600 1150 50  0001 C CNN
	1    10600 1150
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR075
U 1 1 60601D9C
P 10250 800
F 0 "#PWR075" H 10250 650 50  0001 C CNN
F 1 "VDD" H 10265 973 50  0000 C CNN
F 2 "" H 10250 800 50  0001 C CNN
F 3 "" H 10250 800 50  0001 C CNN
	1    10250 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 800  10250 850 
Wire Wire Line
	10250 1150 10250 1200
Wire Wire Line
	9900 1200 10250 1200
Connection ~ 10250 1200
Wire Wire Line
	10250 1200 10250 1250
Wire Wire Line
	10600 1150 10600 1200
Wire Wire Line
	9900 1500 9900 1550
Wire Wire Line
	9900 1550 10050 1550
Wire Wire Line
	10450 1550 10600 1550
Wire Wire Line
	10600 1550 10600 1500
Text Label 9600 1550 0    50   ~ 0
U1_TXD
Wire Wire Line
	9600 1550 9900 1550
Connection ~ 9900 1550
Text HLabel 10850 1550 2    50   Output ~ 0
TXD
Wire Wire Line
	10850 1550 10600 1550
Connection ~ 10600 1550
$Comp
L Transistor_BJT:BC848 Q6
U 1 1 60607CBB
P 10250 2800
F 0 "Q6" V 10485 2800 50  0000 C CNN
F 1 "1P" V 10576 2800 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10450 2725 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 10250 2800 50  0001 L CNN
	1    10250 2800
	0    -1   1    0   
$EndComp
$Comp
L Device:R R34
U 1 1 606096EA
P 10550 2700
F 0 "R34" H 10620 2746 50  0000 L CNN
F 1 "47k" H 10620 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10480 2700 50  0001 C CNN
F 3 "~" H 10550 2700 50  0001 C CNN
	1    10550 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R32
U 1 1 60609C51
P 10250 2400
F 0 "R32" H 10320 2446 50  0000 L CNN
F 1 "4.7k" H 10320 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10180 2400 50  0001 C CNN
F 3 "~" H 10250 2400 50  0001 C CNN
	1    10250 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 6060A458
P 9900 2700
F 0 "R28" H 9970 2746 50  0000 L CNN
F 1 "4.7k" H 9970 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9830 2700 50  0001 C CNN
F 3 "~" H 9900 2700 50  0001 C CNN
	1    9900 2700
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR076
U 1 1 6060D034
P 10250 2150
F 0 "#PWR076" H 10250 2000 50  0001 C CNN
F 1 "VDD" H 10265 2323 50  0000 C CNN
F 2 "" H 10250 2150 50  0001 C CNN
F 3 "" H 10250 2150 50  0001 C CNN
	1    10250 2150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR079
U 1 1 6060D4CC
P 10550 2500
F 0 "#PWR079" H 10550 2350 50  0001 C CNN
F 1 "+5V" H 10565 2673 50  0000 C CNN
F 2 "" H 10550 2500 50  0001 C CNN
F 3 "" H 10550 2500 50  0001 C CNN
	1    10550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 2550 9900 2200
Wire Wire Line
	9900 2200 10250 2200
Wire Wire Line
	10250 2200 10250 2250
Wire Wire Line
	10250 2200 10250 2150
Connection ~ 10250 2200
Wire Wire Line
	10250 2550 10250 2600
Wire Wire Line
	9900 2850 9900 2900
Wire Wire Line
	9900 2900 10050 2900
Wire Wire Line
	10450 2900 10550 2900
Wire Wire Line
	10550 2900 10550 2850
Wire Wire Line
	10550 2550 10550 2500
Text Label 9600 2900 0    50   ~ 0
U1_RXD
Wire Wire Line
	9600 2900 9900 2900
Connection ~ 9900 2900
Text HLabel 10850 2900 2    50   Input ~ 0
RXD
Wire Wire Line
	10850 2900 10550 2900
Connection ~ 10550 2900
Text Label 4850 2050 0    50   ~ 0
U1_TXD
Text Label 4850 2150 0    50   ~ 0
U1_RXD
Wire Wire Line
	4850 2150 5150 2150
Wire Wire Line
	5150 2050 4850 2050
$Comp
L Transistor_BJT:BC848 Q7
U 1 1 6062876E
P 10250 4150
F 0 "Q7" V 10485 4150 50  0000 C CNN
F 1 "1P" V 10576 4150 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10450 4075 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 10250 4150 50  0001 L CNN
	1    10250 4150
	0    -1   1    0   
$EndComp
$Comp
L Device:R R35
U 1 1 60628778
P 10550 4050
F 0 "R35" H 10620 4096 50  0000 L CNN
F 1 "47k" H 10620 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10480 4050 50  0001 C CNN
F 3 "~" H 10550 4050 50  0001 C CNN
	1    10550 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R33
U 1 1 60628782
P 10250 3750
F 0 "R33" H 10320 3796 50  0000 L CNN
F 1 "4.7k" H 10320 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10180 3750 50  0001 C CNN
F 3 "~" H 10250 3750 50  0001 C CNN
	1    10250 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R29
U 1 1 6062878C
P 9900 4050
F 0 "R29" H 9970 4096 50  0000 L CNN
F 1 "4.7k" H 9970 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9830 4050 50  0001 C CNN
F 3 "~" H 9900 4050 50  0001 C CNN
	1    9900 4050
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR077
U 1 1 60628796
P 10250 3500
F 0 "#PWR077" H 10250 3350 50  0001 C CNN
F 1 "VDD" H 10265 3673 50  0000 C CNN
F 2 "" H 10250 3500 50  0001 C CNN
F 3 "" H 10250 3500 50  0001 C CNN
	1    10250 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR080
U 1 1 606287A0
P 10550 3850
F 0 "#PWR080" H 10550 3700 50  0001 C CNN
F 1 "+5V" H 10565 4023 50  0000 C CNN
F 2 "" H 10550 3850 50  0001 C CNN
F 3 "" H 10550 3850 50  0001 C CNN
	1    10550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3900 9900 3550
Wire Wire Line
	9900 3550 10250 3550
Wire Wire Line
	10250 3550 10250 3600
Wire Wire Line
	10250 3550 10250 3500
Connection ~ 10250 3550
Wire Wire Line
	10250 3900 10250 3950
Wire Wire Line
	9900 4200 9900 4250
Wire Wire Line
	9900 4250 10050 4250
Wire Wire Line
	10450 4250 10550 4250
Wire Wire Line
	10550 4250 10550 4200
Wire Wire Line
	10550 3900 10550 3850
Text Label 9600 4250 0    50   ~ 0
U1_DTR
Wire Wire Line
	9600 4250 9900 4250
Connection ~ 9900 4250
Text HLabel 10850 4250 2    50   Input ~ 0
DTR
Wire Wire Line
	10850 4250 10550 4250
Connection ~ 10550 4250
Text Label 4850 1550 0    50   ~ 0
U1_DTR
Wire Wire Line
	4850 1550 5150 1550
Wire Wire Line
	4800 2650 5150 2650
Text Label 4800 2650 0    50   ~ 0
PWRKEY
Text Label 7150 2950 2    50   ~ 0
STATUS
Text Label 7150 3050 2    50   ~ 0
NETLIGHT
Wire Wire Line
	7150 2950 6750 2950
Wire Wire Line
	6750 3050 7150 3050
Text Label 7150 1550 2    50   ~ 0
GSM_ANT
Wire Wire Line
	6750 1550 7150 1550
Text Label 7650 5800 0    50   ~ 0
STATUS
Text Label 5200 6550 0    50   ~ 0
NETLIGHT
Text Label 10600 5500 2    50   ~ 0
PWRKEY
$Comp
L Transistor_BJT:BC848 Q4
U 1 1 6066A7A9
P 10200 5800
F 0 "Q4" H 10390 5846 50  0000 L CNN
F 1 "1P" H 10390 5755 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10400 5725 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 10200 5800 50  0001 L CNN
	1    10200 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 6066AC09
P 9750 5800
F 0 "R26" V 9543 5800 50  0000 C CNN
F 1 "4.7k" V 9634 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9680 5800 50  0001 C CNN
F 3 "~" H 9750 5800 50  0001 C CNN
	1    9750 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R R30
U 1 1 6066AE0E
P 9950 6050
F 0 "R30" H 10020 6096 50  0000 L CNN
F 1 "47k" H 10020 6005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9880 6050 50  0001 C CNN
F 3 "~" H 9950 6050 50  0001 C CNN
	1    9950 6050
	1    0    0    -1  
$EndComp
Text HLabel 9450 5800 0    50   Input ~ 0
PKEY
Wire Wire Line
	9450 5800 9600 5800
Wire Wire Line
	9900 5800 9950 5800
Wire Wire Line
	9950 5900 9950 5800
Connection ~ 9950 5800
Wire Wire Line
	9950 5800 10000 5800
Wire Wire Line
	9950 6200 10300 6200
Wire Wire Line
	10300 6200 10300 6000
$Comp
L power:GND #PWR078
U 1 1 606768FF
P 10300 6250
F 0 "#PWR078" H 10300 6000 50  0001 C CNN
F 1 "GND" H 10305 6077 50  0000 C CNN
F 2 "" H 10300 6250 50  0001 C CNN
F 3 "" H 10300 6250 50  0001 C CNN
	1    10300 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 6250 10300 6200
Connection ~ 10300 6200
Wire Wire Line
	10600 5500 10300 5500
Wire Wire Line
	10300 5500 10300 5600
$Comp
L Transistor_BJT:BC848 Q2
U 1 1 6067EEE6
P 8550 5800
F 0 "Q2" H 8740 5846 50  0000 L CNN
F 1 "1P" H 8740 5755 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8750 5725 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 8550 5800 50  0001 L CNN
	1    8550 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 6067EEF0
P 8100 5800
F 0 "R23" V 7893 5800 50  0000 C CNN
F 1 "4.7k" V 7984 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8030 5800 50  0001 C CNN
F 3 "~" H 8100 5800 50  0001 C CNN
	1    8100 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R R24
U 1 1 6067EEFA
P 8300 6050
F 0 "R24" H 8370 6096 50  0000 L CNN
F 1 "47k" H 8370 6005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8230 6050 50  0001 C CNN
F 3 "~" H 8300 6050 50  0001 C CNN
	1    8300 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5800 8300 5800
Wire Wire Line
	8300 5900 8300 5800
Connection ~ 8300 5800
Wire Wire Line
	8300 5800 8350 5800
Wire Wire Line
	8300 6200 8650 6200
Wire Wire Line
	8650 6200 8650 6000
$Comp
L power:GND #PWR074
U 1 1 6067EF0A
P 8650 6250
F 0 "#PWR074" H 8650 6000 50  0001 C CNN
F 1 "GND" H 8655 6077 50  0000 C CNN
F 2 "" H 8650 6250 50  0001 C CNN
F 3 "" H 8650 6250 50  0001 C CNN
	1    8650 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 6250 8650 6200
Connection ~ 8650 6200
$Comp
L Device:R R25
U 1 1 6068594B
P 8650 5350
F 0 "R25" H 8720 5396 50  0000 L CNN
F 1 "470" H 8720 5305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8580 5350 50  0001 C CNN
F 3 "~" H 8650 5350 50  0001 C CNN
	1    8650 5350
	1    0    0    -1  
$EndComp
Text Label 8850 5100 2    50   ~ 0
VBAT
Wire Wire Line
	8850 5100 8650 5100
Wire Wire Line
	8650 5100 8650 5200
Wire Wire Line
	7650 5800 7950 5800
Wire Wire Line
	8650 5500 8650 5550
Text HLabel 8800 5550 2    50   Output ~ 0
STATUS_
Wire Wire Line
	8800 5550 8650 5550
Connection ~ 8650 5550
Wire Wire Line
	8650 5550 8650 5600
$Comp
L Transistor_BJT:BC848 Q1
U 1 1 606ADFCB
P 6100 6550
F 0 "Q1" H 6290 6596 50  0000 L CNN
F 1 "1P" H 6290 6505 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6300 6475 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 6100 6550 50  0001 L CNN
	1    6100 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 606ADFD5
P 5650 6550
F 0 "R13" V 5443 6550 50  0000 C CNN
F 1 "4.7k" V 5534 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5580 6550 50  0001 C CNN
F 3 "~" H 5650 6550 50  0001 C CNN
	1    5650 6550
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 606ADFDF
P 5850 6800
F 0 "R21" H 5920 6846 50  0000 L CNN
F 1 "47k" H 5920 6755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5780 6800 50  0001 C CNN
F 3 "~" H 5850 6800 50  0001 C CNN
	1    5850 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 6550 5850 6550
Wire Wire Line
	5850 6650 5850 6550
Connection ~ 5850 6550
Wire Wire Line
	5850 6550 5900 6550
Wire Wire Line
	5850 6950 6200 6950
Wire Wire Line
	6200 6950 6200 6750
$Comp
L power:GND #PWR073
U 1 1 606ADFEF
P 6200 7000
F 0 "#PWR073" H 6200 6750 50  0001 C CNN
F 1 "GND" H 6205 6827 50  0000 C CNN
F 2 "" H 6200 7000 50  0001 C CNN
F 3 "" H 6200 7000 50  0001 C CNN
	1    6200 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 7000 6200 6950
Connection ~ 6200 6950
$Comp
L Device:R R22
U 1 1 606ADFFB
P 6200 6150
F 0 "R22" H 6270 6196 50  0000 L CNN
F 1 "470" H 6270 6105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6130 6150 50  0001 C CNN
F 3 "~" H 6200 6150 50  0001 C CNN
	1    6200 6150
	1    0    0    -1  
$EndComp
Text Label 6400 5550 2    50   ~ 0
VBAT
Wire Wire Line
	6400 5550 6200 5550
Wire Wire Line
	6200 5550 6200 5650
Wire Wire Line
	5200 6550 5500 6550
$Comp
L Device:LED D11
U 1 1 606B8DAF
P 6200 5800
F 0 "D11" V 6239 5682 50  0000 R CNN
F 1 "SIM" V 6148 5682 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6200 5800 50  0001 C CNN
F 3 "~" H 6200 5800 50  0001 C CNN
	1    6200 5800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6200 5950 6200 6000
Wire Wire Line
	6200 6300 6200 6350
$Comp
L Device:CP1 C17
U 1 1 606E3477
P 750 4150
F 0 "C17" H 865 4196 50  0000 L CNN
F 1 "CA_470u" H 865 4105 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 750 4150 50  0001 C CNN
F 3 "~" H 750 4150 50  0001 C CNN
	1    750  4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C21
U 1 1 606E423E
P 1350 4150
F 0 "C21" H 1465 4196 50  0000 L CNN
F 1 "CB_10u" H 1465 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1388 4000 50  0001 C CNN
F 3 "~" H 1350 4150 50  0001 C CNN
	1    1350 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C24
U 1 1 606E4C2E
P 1900 4150
F 0 "C24" H 2015 4196 50  0000 L CNN
F 1 "33p" H 2015 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1938 4000 50  0001 C CNN
F 3 "~" H 1900 4150 50  0001 C CNN
	1    1900 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C27
U 1 1 606E4E6C
P 2350 4150
F 0 "C27" H 2465 4196 50  0000 L CNN
F 1 "10p" H 2465 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2388 4000 50  0001 C CNN
F 3 "~" H 2350 4150 50  0001 C CNN
	1    2350 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D7
U 1 1 606E55D1
P 2850 4150
F 0 "D7" V 2804 4230 50  0000 L CNN
F 1 "D_Zener" V 2895 4230 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-323F" H 2850 4150 50  0001 C CNN
F 3 "~" H 2850 4150 50  0001 C CNN
	1    2850 4150
	0    1    1    0   
$EndComp
Text Label 550  3850 0    50   ~ 0
VBAT
Wire Wire Line
	2350 3850 2350 4000
Wire Wire Line
	550  3850 750  3850
Wire Wire Line
	750  4000 750  3850
Connection ~ 750  3850
Wire Wire Line
	750  3850 1350 3850
Wire Wire Line
	1350 4000 1350 3850
Connection ~ 1350 3850
Wire Wire Line
	1350 3850 1900 3850
Wire Wire Line
	1900 4000 1900 3850
Connection ~ 1900 3850
Wire Wire Line
	1900 3850 2350 3850
Wire Wire Line
	2350 3850 2850 3850
Wire Wire Line
	2850 3850 2850 4000
Connection ~ 2350 3850
$Comp
L power:GND #PWR031
U 1 1 606FE411
P 750 4350
F 0 "#PWR031" H 750 4100 50  0001 C CNN
F 1 "GND" H 755 4177 50  0000 C CNN
F 2 "" H 750 4350 50  0001 C CNN
F 3 "" H 750 4350 50  0001 C CNN
	1    750  4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR039
U 1 1 606FEAAB
P 1350 4350
F 0 "#PWR039" H 1350 4100 50  0001 C CNN
F 1 "GND" H 1355 4177 50  0000 C CNN
F 2 "" H 1350 4350 50  0001 C CNN
F 3 "" H 1350 4350 50  0001 C CNN
	1    1350 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR063
U 1 1 606FED1A
P 1900 4350
F 0 "#PWR063" H 1900 4100 50  0001 C CNN
F 1 "GND" H 1905 4177 50  0000 C CNN
F 2 "" H 1900 4350 50  0001 C CNN
F 3 "" H 1900 4350 50  0001 C CNN
	1    1900 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR066
U 1 1 606FEF9B
P 2350 4350
F 0 "#PWR066" H 2350 4100 50  0001 C CNN
F 1 "GND" H 2355 4177 50  0000 C CNN
F 2 "" H 2350 4350 50  0001 C CNN
F 3 "" H 2350 4350 50  0001 C CNN
	1    2350 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR067
U 1 1 606FF153
P 2850 4350
F 0 "#PWR067" H 2850 4100 50  0001 C CNN
F 1 "GND" H 2855 4177 50  0000 C CNN
F 2 "" H 2850 4350 50  0001 C CNN
F 3 "" H 2850 4350 50  0001 C CNN
	1    2850 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 4300 2850 4350
Wire Wire Line
	2350 4300 2350 4350
Wire Wire Line
	1900 4300 1900 4350
Wire Wire Line
	1350 4300 1350 4350
Wire Wire Line
	750  4300 750  4350
$Comp
L SparkFun-Connectors:U.FL2PIN J3
U 1 1 6071E87C
P 1600 850
F 0 "J3" H 1658 1104 45  0000 C CNN
F 1 "U.FL2PIN" H 1658 1020 45  0000 C CNN
F 2 "Connectors:U.FL" H 1600 1100 20  0001 C CNN
F 3 "" H 1600 850 50  0001 C CNN
F 4 "CONN-09193" H 1658 1031 60  0001 C CNN "Field4"
	1    1600 850 
	1    0    0    -1  
$EndComp
Text Label 2150 850  2    50   ~ 0
GSM_ANT
Wire Wire Line
	2150 850  1800 850 
$Comp
L power:GND #PWR049
U 1 1 60723F67
P 1600 1100
F 0 "#PWR049" H 1600 850 50  0001 C CNN
F 1 "GND" H 1605 927 50  0000 C CNN
F 2 "" H 1600 1100 50  0001 C CNN
F 3 "" H 1600 1100 50  0001 C CNN
	1    1600 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1100 1600 1050
$Comp
L Regulator_Linear:LM1084-ADJ U5
U 1 1 6073D64F
P 1300 2600
F 0 "U5" H 1300 2842 50  0000 C CNN
F 1 "LM1084-ADJ" H 1300 2751 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 1300 2850 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm1084.pdf" H 1300 2600 50  0001 C CNN
	1    1300 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 6073E3FD
P 1700 2800
F 0 "R4" H 1770 2846 50  0000 L CNN
F 1 "2k" H 1770 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 2800 50  0001 C CNN
F 3 "~" H 1700 2800 50  0001 C CNN
	1    1700 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 6073EDFA
P 1700 3200
F 0 "R5" H 1770 3246 50  0000 L CNN
F 1 "4.7k" H 1770 3155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 3200 50  0001 C CNN
F 3 "~" H 1700 3200 50  0001 C CNN
	1    1700 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C25
U 1 1 6073F174
P 2100 2800
F 0 "C25" H 2215 2846 50  0000 L CNN
F 1 "470u" H 2215 2755 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 2100 2800 50  0001 C CNN
F 3 "~" H 2100 2800 50  0001 C CNN
	1    2100 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C11
U 1 1 6073FF09
P 750 2800
F 0 "C11" H 865 2846 50  0000 L CNN
F 1 "470u" H 865 2755 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 750 2800 50  0001 C CNN
F 3 "~" H 750 2800 50  0001 C CNN
	1    750  2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  2650 750  2600
Wire Wire Line
	750  2600 1000 2600
Wire Wire Line
	1600 2600 1700 2600
Wire Wire Line
	2100 2600 2100 2650
Wire Wire Line
	1700 2650 1700 2600
Connection ~ 1700 2600
Wire Wire Line
	1700 2600 2100 2600
Wire Wire Line
	1700 2950 1700 3000
Wire Wire Line
	1700 3000 1300 3000
Wire Wire Line
	1300 3000 1300 2900
Connection ~ 1700 3000
Wire Wire Line
	1700 3000 1700 3050
$Comp
L power:GND #PWR064
U 1 1 6075CC21
P 2100 3000
F 0 "#PWR064" H 2100 2750 50  0001 C CNN
F 1 "GND" H 2105 2827 50  0000 C CNN
F 2 "" H 2100 3000 50  0001 C CNN
F 3 "" H 2100 3000 50  0001 C CNN
	1    2100 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR055
U 1 1 6075D4D0
P 1700 3400
F 0 "#PWR055" H 1700 3150 50  0001 C CNN
F 1 "GND" H 1705 3227 50  0000 C CNN
F 2 "" H 1700 3400 50  0001 C CNN
F 3 "" H 1700 3400 50  0001 C CNN
	1    1700 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 6075D898
P 750 3000
F 0 "#PWR030" H 750 2750 50  0001 C CNN
F 1 "GND" H 755 2827 50  0000 C CNN
F 2 "" H 750 3000 50  0001 C CNN
F 3 "" H 750 3000 50  0001 C CNN
	1    750  3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  2950 750  3000
Wire Wire Line
	1700 3350 1700 3400
Wire Wire Line
	2100 2950 2100 3000
Text Label 2300 2600 2    50   ~ 0
VBAT
Wire Wire Line
	2300 2600 2100 2600
Connection ~ 2100 2600
Wire Wire Line
	750  2500 750  2600
Connection ~ 750  2600
Wire Wire Line
	2100 6050 3050 6050
Wire Wire Line
	2700 6200 3450 6200
Wire Wire Line
	3050 6650 3050 6050
Connection ~ 3050 6050
Wire Wire Line
	3050 6050 3700 6050
Wire Wire Line
	3450 6650 3450 6200
Connection ~ 3450 6200
Wire Wire Line
	3450 6200 3700 6200
Wire Wire Line
	3150 6350 3700 6350
Wire Wire Line
	2700 6350 3150 6350
Connection ~ 3150 6350
Wire Wire Line
	3150 6650 3150 6350
Wire Wire Line
	3250 6500 3700 6500
Wire Wire Line
	2700 6500 3250 6500
Connection ~ 3250 6500
Wire Wire Line
	3250 6650 3250 6500
$Comp
L RF_GSM:SIM7020E U7
U 1 1 607C139B
P 5950 2450
F 0 "U7" H 5300 1250 50  0000 C CNN
F 1 "SIM868" H 5400 1350 50  0000 C CNN
F 2 "RF_GSM:SIMCom_SIM800C" H 6500 1400 50  0001 C CNN
F 3 "https://simcom.ee/documents/SIM7020/SIM7020%20Hardware%20Design_V1.02.pdf" H 1300 100 50  0001 C CNN
	1    5950 2450
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY04
U 1 1 60736975
P 750 2500
F 0 "#SUPPLY04" H 800 2500 45  0001 L BNN
F 1 "VIN" H 750 2670 45  0000 C CNN
F 2 "XXX-00000" H 750 2681 60  0001 C CNN
F 3 "" H 750 2500 60  0001 C CNN
	1    750  2500
	1    0    0    -1  
$EndComp
Text Notes 650  750  0    118  ~ 24
Antenna
Text Notes 650  2100 0    118  ~ 24
Power
Text Notes 650  5300 0    118  ~ 24
SIM-Card Holder
Wire Bus Line
	3350 450  3350 1600
Wire Bus Line
	3350 1600 500  1600
Text Notes 5100 5200 0    118  ~ 24
LED
Wire Bus Line
	11200 4850 8250 4850
Wire Bus Line
	5000 4850 5000 7750
Connection ~ 5000 4850
Wire Bus Line
	5000 4850 3350 4850
Wire Bus Line
	6950 4850 6950 6500
Connection ~ 6950 4850
Wire Bus Line
	6950 4850 5000 4850
Wire Bus Line
	3350 1600 3350 4850
Connection ~ 3350 1600
Connection ~ 3350 4850
Wire Bus Line
	3350 4850 550  4850
Wire Bus Line
	8250 500  8250 4850
Connection ~ 8250 4850
Wire Bus Line
	8250 4850 6950 4850
Text Notes 9350 5100 0    118  ~ 24
Power_Key & Status
Text Notes 8300 700  0    118  ~ 24
UART
$EndSCHEMATC
