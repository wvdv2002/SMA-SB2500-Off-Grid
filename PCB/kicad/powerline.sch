EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L pspice:CAP C?
U 1 1 61D9E6A1
P 3700 2450
AR Path="/61D9E6A1" Ref="C?"  Part="1" 
AR Path="/61D74676/61D9E6A1" Ref="C?"  Part="1" 
F 0 "C?" H 3878 2496 50  0000 L CNN
F 1 "4.7u" H 3878 2405 50  0000 L CNN
F 2 "" H 3700 2450 50  0001 C CNN
F 3 "~" H 3700 2450 50  0001 C CNN
	1    3700 2450
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 61D9E6A7
P 5800 2450
AR Path="/61D9E6A7" Ref="C?"  Part="1" 
AR Path="/61D74676/61D9E6A7" Ref="C?"  Part="1" 
F 0 "C?" H 5978 2496 50  0000 L CNN
F 1 "CAP" H 5978 2405 50  0000 L CNN
F 2 "" H 5800 2450 50  0001 C CNN
F 3 "~" H 5800 2450 50  0001 C CNN
	1    5800 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:L L?
U 1 1 61D9E6AD
P 3250 2200
AR Path="/61D9E6AD" Ref="L?"  Part="1" 
AR Path="/61D74676/61D9E6AD" Ref="L?"  Part="1" 
F 0 "L?" V 3440 2200 50  0000 C CNN
F 1 "2mH 15A" V 3349 2200 50  0000 C CNN
F 2 "" H 3250 2200 50  0001 C CNN
F 3 "~" H 3250 2200 50  0001 C CNN
	1    3250 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5600 2650 5600 2700
Wire Wire Line
	5600 2700 5800 2700
Wire Wire Line
	5350 2100 5350 2200
Wire Wire Line
	5350 2200 5800 2200
Wire Wire Line
	5150 2100 5150 2250
$Comp
L power:+15V #PWR?
U 1 1 61D9E6B8
P 4750 1700
AR Path="/61D9E6B8" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E6B8" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4750 1550 50  0001 C CNN
F 1 "+15V" H 4765 1873 50  0000 C CNN
F 2 "" H 4750 1700 50  0001 C CNN
F 3 "" H 4750 1700 50  0001 C CNN
	1    4750 1700
	1    0    0    -1  
$EndComp
Text GLabel 5250 1000 1    50   Input ~ 0
IAC
Wire Wire Line
	5250 1300 5250 1000
Wire Wire Line
	4750 1700 4850 1700
$Comp
L Relay:RM50-xx21 K?
U 1 1 61D9E6C1
P 6500 2000
AR Path="/61D9E6C1" Ref="K?"  Part="1" 
AR Path="/61D74676/61D9E6C1" Ref="K?"  Part="1" 
F 0 "K?" V 5933 2000 50  0000 C CNN
F 1 "RM50-xx21" V 6024 2000 50  0000 C CNN
F 2 "Relay_THT:Relay_SPST_Finder_32.21-x300" H 7770 1970 50  0001 C CNN
F 3 "http://www.relpol.pl/en/content/download/13683/165953/file/e_RM50.pdf" H 6500 2000 50  0001 C CNN
	1    6500 2000
	0    1    1    0   
$EndComp
$Comp
L Relay:RM50-xx21 K?
U 1 1 61D9E6C7
P 6500 3050
AR Path="/61D9E6C7" Ref="K?"  Part="1" 
AR Path="/61D74676/61D9E6C7" Ref="K?"  Part="1" 
F 0 "K?" V 7067 3050 50  0000 C CNN
F 1 "RM50-xx21" V 6976 3050 50  0000 C CNN
F 2 "Relay_THT:Relay_SPST_Finder_32.21-x300" H 7770 3020 50  0001 C CNN
F 3 "http://www.relpol.pl/en/content/download/13683/165953/file/e_RM50.pdf" H 6500 3050 50  0001 C CNN
	1    6500 3050
	0    -1   -1   0   
$EndComp
$Comp
L Relay:RM50-xx21 K?
U 1 1 61D9E6CD
P 7400 3050
AR Path="/61D9E6CD" Ref="K?"  Part="1" 
AR Path="/61D74676/61D9E6CD" Ref="K?"  Part="1" 
F 0 "K?" V 7967 3050 50  0000 C CNN
F 1 "RM50-xx21" V 7876 3050 50  0000 C CNN
F 2 "Relay_THT:Relay_SPST_Finder_32.21-x300" H 8670 3020 50  0001 C CNN
F 3 "http://www.relpol.pl/en/content/download/13683/165953/file/e_RM50.pdf" H 7400 3050 50  0001 C CNN
	1    7400 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6200 2700 6200 2750
Wire Wire Line
	6800 2850 7000 2850
Wire Wire Line
	7100 2850 7100 2750
Wire Wire Line
	7700 2850 7700 2500
$Comp
L pspice:CAP C?
U 1 1 61D9E6D7
P 8950 2400
AR Path="/61D9E6D7" Ref="C?"  Part="1" 
AR Path="/61D74676/61D9E6D7" Ref="C?"  Part="1" 
F 0 "C?" H 9128 2446 50  0000 L CNN
F 1 "0.01u" H 9128 2355 50  0000 L CNN
F 2 "" H 8950 2400 50  0001 C CNN
F 3 "~" H 8950 2400 50  0001 C CNN
	1    8950 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 2300 7700 2300
Connection ~ 5800 2200
Wire Wire Line
	5800 2200 6200 2200
Connection ~ 5800 2700
Wire Wire Line
	5800 2700 6200 2700
Wire Wire Line
	5000 2250 5150 2250
Wire Wire Line
	5000 2650 5600 2650
$Comp
L Device:Transformer_1P_1S T?
U 1 1 61D9E6E4
P 4600 2450
AR Path="/61D9E6E4" Ref="T?"  Part="1" 
AR Path="/61D74676/61D9E6E4" Ref="T?"  Part="1" 
F 0 "T?" H 4600 2831 50  0000 C CNN
F 1 "153/250v 9.6A, RK2000" H 4600 2740 50  0000 C CNN
F 2 "" H 4600 2450 50  0001 C CNN
F 3 "~" H 4600 2450 50  0001 C CNN
	1    4600 2450
	1    0    0    -1  
$EndComp
$Comp
L Filter:1FP41-4R FL?
U 1 1 61D9E6EA
P 8200 2400
AR Path="/61D9E6EA" Ref="FL?"  Part="1" 
AR Path="/61D74676/61D9E6EA" Ref="FL?"  Part="1" 
F 0 "FL?" H 8200 2725 50  0000 C CNN
F 1 "1FP41-4R" H 8200 2634 50  0000 C CNN
F 2 "Filter:Filter_FILTERCON_1FPxx" H 8200 2400 50  0001 C CNN
F 3 "https://filtercon.com.pl/wp-content/uploads/2019/07/Karta-katalogowa-FP-12-1.pdf" H 8200 2400 50  0001 C CNN
	1    8200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1700 5850 1700
$Comp
L Sensor_Current:HX15-P-SP2 U?
U 1 1 61D9E6F1
P 5250 1700
AR Path="/61D9E6F1" Ref="U?"  Part="1" 
AR Path="/61D74676/61D9E6F1" Ref="U?"  Part="1" 
F 0 "U?" V 5831 1700 50  0000 C CNN
F 1 "HX15-P-SP2" V 5650 1300 50  0000 C CNN
F 2 "Sensor_Current:LEM_HX15-P-SP2" H 5250 1700 50  0001 C CNN
F 3 "https://www.lem.com/sites/default/files/products_datasheets/hx%203_50-p_sp2_e%20v07.pdf" H 5250 1700 50  0001 C CNN
	1    5250 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:-15V #PWR?
U 1 1 61D9E6F7
P 5850 1700
AR Path="/61D9E6F7" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E6F7" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5850 1800 50  0001 C CNN
F 1 "-15V" H 5865 1873 50  0000 C CNN
F 2 "" H 5850 1700 50  0001 C CNN
F 3 "" H 5850 1700 50  0001 C CNN
	1    5850 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 2300 8700 2150
Wire Wire Line
	8700 2150 8950 2150
Wire Wire Line
	8700 2500 8700 2650
Wire Wire Line
	8700 2650 8950 2650
$Comp
L Device:EMI_Filter_CommonMode FL?
U 1 1 61D9E7A1
P 9500 2800
AR Path="/61D9E7A1" Ref="FL?"  Part="1" 
AR Path="/61D74676/61D9E7A1" Ref="FL?"  Part="1" 
F 0 "FL?" H 9500 3081 50  0000 C CNN
F 1 "EMI_Filter_CommonMode" H 9500 2990 50  0000 C CNN
F 2 "" H 9500 2840 50  0001 C CNN
F 3 "~" H 9500 2840 50  0001 C CNN
	1    9500 2800
	1    0    0    -1  
$EndComp
Text GLabel 10350 2150 2    50   Input ~ 0
L
Text GLabel 10350 2600 2    50   Input ~ 0
N
Wire Wire Line
	8950 2650 9300 2650
Wire Wire Line
	9300 2650 9300 2700
Connection ~ 8950 2650
Wire Wire Line
	9300 2700 9300 2900
Connection ~ 9300 2700
Wire Wire Line
	9700 2900 9700 2700
Wire Wire Line
	9700 2600 9700 2700
Connection ~ 9700 2700
Wire Wire Line
	8950 2150 10100 2150
Connection ~ 8950 2150
$Comp
L pspice:CAP C?
U 1 1 61D9E7B3
P 9550 3250
AR Path="/61D9E7B3" Ref="C?"  Part="1" 
AR Path="/61D74676/61D9E7B3" Ref="C?"  Part="1" 
F 0 "C?" V 9235 3250 50  0000 C CNN
F 1 "0.015u" V 9326 3250 50  0000 C CNN
F 2 "" H 9550 3250 50  0001 C CNN
F 3 "~" H 9550 3250 50  0001 C CNN
	1    9550 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	9300 2900 9300 3250
Connection ~ 9300 2900
Wire Wire Line
	9800 3250 9800 2900
Wire Wire Line
	9800 2900 9700 2900
Connection ~ 9700 2900
$Comp
L pspice:CAP C?
U 1 1 61D9E7BE
P 9800 3500
AR Path="/61D9E7BE" Ref="C?"  Part="1" 
AR Path="/61D74676/61D9E7BE" Ref="C?"  Part="1" 
F 0 "C?" H 9978 3546 50  0000 L CNN
F 1 "150n" H 9978 3455 50  0000 L CNN
F 2 "" H 9800 3500 50  0001 C CNN
F 3 "~" H 9800 3500 50  0001 C CNN
	1    9800 3500
	1    0    0    -1  
$EndComp
Connection ~ 9800 3250
Wire Wire Line
	9800 3750 9800 4100
Wire Wire Line
	9800 4500 9800 4850
Wire Wire Line
	9800 4850 10250 4850
$Comp
L Device:Transformer_1P_SS T?
U 1 1 61D9E7C8
P 9400 4300
AR Path="/61D9E7C8" Ref="T?"  Part="1" 
AR Path="/61D74676/61D9E7C8" Ref="T?"  Part="1" 
F 0 "T?" H 9400 4681 50  0000 C CNN
F 1 "Transformer_1P_SS" H 9400 4590 50  0000 C CNN
F 2 "" H 9400 4300 50  0001 C CNN
F 3 "~" H 9400 4300 50  0001 C CNN
	1    9400 4300
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61D9E7CE
P 9000 4500
AR Path="/61D9E7CE" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E7CE" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 9000 4250 50  0001 C CNN
F 1 "GND" H 9005 4327 50  0000 C CNN
F 2 "" H 9000 4500 50  0001 C CNN
F 3 "" H 9000 4500 50  0001 C CNN
	1    9000 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:L L?
U 1 1 61D9E7D4
P 8600 4300
AR Path="/61D9E7D4" Ref="L?"  Part="1" 
AR Path="/61D74676/61D9E7D4" Ref="L?"  Part="1" 
F 0 "L?" H 8653 4346 50  0000 L CNN
F 1 "680uh" H 8653 4255 50  0000 L CNN
F 2 "" H 8600 4300 50  0001 C CNN
F 3 "~" H 8600 4300 50  0001 C CNN
	1    8600 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4500 8600 4500
Wire Wire Line
	8600 4500 8600 4450
Connection ~ 9000 4500
Wire Wire Line
	8600 4100 8600 4150
Wire Wire Line
	8600 4100 9000 4100
Wire Wire Line
	9700 2600 10350 2600
Wire Wire Line
	10250 4850 10250 3050
Wire Wire Line
	10250 3050 10100 3050
Wire Wire Line
	10100 3050 10100 2150
Connection ~ 10100 2150
Wire Wire Line
	10100 2150 10350 2150
$Comp
L Device:Q_NIGBT_GCE Q?
U 1 1 61D9E7E5
P 2350 1700
AR Path="/61D9E7E5" Ref="Q?"  Part="1" 
AR Path="/61D74676/61D9E7E5" Ref="Q?"  Part="1" 
F 0 "Q?" H 2540 1746 50  0000 L CNN
F 1 "Q_NIGBT_GCE" H 2540 1655 50  0000 L CNN
F 2 "" H 2550 1800 50  0001 C CNN
F 3 "~" H 2350 1700 50  0001 C CNN
	1    2350 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NIGBT_GCE Q?
U 1 1 61D9E7EB
P 2350 2350
AR Path="/61D9E7EB" Ref="Q?"  Part="1" 
AR Path="/61D74676/61D9E7EB" Ref="Q?"  Part="1" 
F 0 "Q?" H 2540 2396 50  0000 L CNN
F 1 "Q_NIGBT_GCE" H 2540 2305 50  0000 L CNN
F 2 "" H 2550 2450 50  0001 C CNN
F 3 "~" H 2350 2350 50  0001 C CNN
	1    2350 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NIGBT_GCE Q?
U 1 1 61D9E7F1
P 4050 3750
AR Path="/61D9E7F1" Ref="Q?"  Part="1" 
AR Path="/61D74676/61D9E7F1" Ref="Q?"  Part="1" 
F 0 "Q?" H 4241 3796 50  0000 L CNN
F 1 "Q_NIGBT_GCE" H 4241 3705 50  0000 L CNN
F 2 "" H 4250 3850 50  0001 C CNN
F 3 "~" H 4050 3750 50  0001 C CNN
	1    4050 3750
	-1   0    0    -1  
$EndComp
$Comp
L Device:Q_NIGBT_GCE Q?
U 1 1 61D9E7F7
P 4050 3100
AR Path="/61D9E7F7" Ref="Q?"  Part="1" 
AR Path="/61D74676/61D9E7F7" Ref="Q?"  Part="1" 
F 0 "Q?" H 4241 3146 50  0000 L CNN
F 1 "Q_NIGBT_GCE" H 4241 3055 50  0000 L CNN
F 2 "" H 4250 3200 50  0001 C CNN
F 3 "~" H 4050 3100 50  0001 C CNN
	1    4050 3100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3700 2700 4200 2700
Wire Wire Line
	4200 2700 4200 2650
Wire Wire Line
	3700 2700 3700 3400
Wire Wire Line
	3700 3400 3950 3400
Wire Wire Line
	3950 3400 3950 3300
Connection ~ 3700 2700
Wire Wire Line
	3950 3550 3950 3400
Connection ~ 3950 3400
Wire Wire Line
	2450 1900 2450 2100
Wire Wire Line
	2450 2100 3100 2100
Wire Wire Line
	3100 2100 3100 2200
Connection ~ 2450 2100
Wire Wire Line
	2450 2100 2450 2150
Wire Wire Line
	3400 2200 3700 2200
Wire Wire Line
	4200 2200 4200 2250
Connection ~ 3700 2200
Wire Wire Line
	3700 2200 4200 2200
$Comp
L power:-VDC #PWR?
U 1 1 61D9E80E
P 2450 2550
AR Path="/61D9E80E" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E80E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2450 2450 50  0001 C CNN
F 1 "-VDC" H 2450 2825 50  0000 C CNN
F 2 "" H 2450 2550 50  0001 C CNN
F 3 "" H 2450 2550 50  0001 C CNN
	1    2450 2550
	-1   0    0    1   
$EndComp
$Comp
L power:-VDC #PWR?
U 1 1 61D9E814
P 3950 3950
AR Path="/61D9E814" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E814" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3950 3850 50  0001 C CNN
F 1 "-VDC" H 3950 4225 50  0000 C CNN
F 2 "" H 3950 3950 50  0001 C CNN
F 3 "" H 3950 3950 50  0001 C CNN
	1    3950 3950
	-1   0    0    1   
$EndComp
$Comp
L power:+VDC #PWR?
U 1 1 61D9E81A
P 2450 1500
AR Path="/61D9E81A" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E81A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2450 1400 50  0001 C CNN
F 1 "+VDC" H 2450 1775 50  0000 C CNN
F 2 "" H 2450 1500 50  0001 C CNN
F 3 "" H 2450 1500 50  0001 C CNN
	1    2450 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+VDC #PWR?
U 1 1 61D9E820
P 3950 2900
AR Path="/61D9E820" Ref="#PWR?"  Part="1" 
AR Path="/61D74676/61D9E820" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3950 2800 50  0001 C CNN
F 1 "+VDC" H 3950 3175 50  0000 C CNN
F 2 "" H 3950 2900 50  0001 C CNN
F 3 "" H 3950 2900 50  0001 C CNN
	1    3950 2900
	1    0    0    -1  
$EndComp
Text Notes 6400 1350 0    50   ~ 0
RELAY1
Text Notes 7300 3550 0    50   ~ 0
RELAY2
$Comp
L Device:R R?
U 1 1 61D10E09
P 7400 2450
F 0 "R?" V 7193 2450 50  0000 C CNN
F 1 "33R" V 7284 2450 50  0000 C CNN
F 2 "" V 7330 2450 50  0001 C CNN
F 3 "~" H 7400 2450 50  0001 C CNN
	1    7400 2450
	0    1    1    0   
$EndComp
$Comp
L Device:Thermistor_PTC TH?
U 1 1 61D11913
P 7000 2700
F 0 "TH?" H 7097 2746 50  0000 L CNN
F 1 "145Degr" H 7097 2655 50  0000 L CNN
F 2 "" H 7050 2500 50  0001 L CNN
F 3 "~" H 7000 2700 50  0001 C CNN
	1    7000 2700
	1    0    0    -1  
$EndComp
Connection ~ 7000 2850
Wire Wire Line
	7000 2850 7100 2850
Wire Wire Line
	7000 2550 7000 2450
Wire Wire Line
	7000 2450 7250 2450
Wire Wire Line
	7550 2450 7700 2450
Wire Wire Line
	7700 2450 7700 2500
Connection ~ 7700 2500
$Comp
L Device:Transformer_1P_1S T?
U 1 1 61D15D9B
P 5950 4350
F 0 "T?" H 5950 4731 50  0000 C CNN
F 1 "3.13V/230V" H 5950 4640 50  0000 C CNN
F 2 "" H 5950 4350 50  0001 C CNN
F 3 "~" H 5950 4350 50  0001 C CNN
	1    5950 4350
	1    0    0    -1  
$EndComp
Text GLabel 7150 2300 1    50   Input ~ 0
EMC_L
Text GLabel 7700 2850 2    50   Input ~ 0
EMC_N
Text GLabel 6350 4150 2    50   Input ~ 0
EMC_N
Text GLabel 6350 4550 2    50   Input ~ 0
EMC_L
$EndSCHEMATC
