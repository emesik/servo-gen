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
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
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
L ATTINY85-P IC1
U 1 1 5487148F
P 3250 2150
F 0 "IC1" H 2100 2550 40  0000 C CNN
F 1 "ATTINY85-P" H 4250 1750 40  0000 C CNN
F 2 "DIP8" H 4250 2150 35  0000 C CIN
F 3 "" H 3250 2150 60  0000 C CNN
	1    3250 2150
	-1   0    0    -1  
$EndComp
$Comp
L POT RV1
U 1 1 54871515
P 5600 2200
F 0 "RV1" H 5600 2100 50  0000 C CNN
F 1 "POT" H 5600 2200 50  0000 C CNN
F 2 "" H 5600 2200 60  0000 C CNN
F 3 "" H 5600 2200 60  0000 C CNN
	1    5600 2200
	0    -1   1    0   
$EndComp
Wire Wire Line
	5450 2200 4600 2200
$Comp
L +5V #PWR1
U 1 1 54871548
P 1750 1250
F 0 "#PWR1" H 1750 1340 20  0001 C CNN
F 1 "+5V" H 1750 1340 30  0000 C CNN
F 2 "" H 1750 1250 60  0000 C CNN
F 3 "" H 1750 1250 60  0000 C CNN
	1    1750 1250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR5
U 1 1 5487155C
P 5600 1250
F 0 "#PWR5" H 5600 1340 20  0001 C CNN
F 1 "+5V" H 5600 1340 30  0000 C CNN
F 2 "" H 5600 1250 60  0000 C CNN
F 3 "" H 5600 1250 60  0000 C CNN
	1    5600 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 54871569
P 1750 2700
F 0 "#PWR2" H 1750 2700 30  0001 C CNN
F 1 "GND" H 1750 2630 30  0001 C CNN
F 2 "" H 1750 2700 60  0000 C CNN
F 3 "" H 1750 2700 60  0000 C CNN
	1    1750 2700
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5487157D
P 1750 2150
F 0 "C1" H 1750 2250 40  0000 L CNN
F 1 "100n" H 1756 2065 40  0000 L CNN
F 2 "" H 1788 2000 30  0000 C CNN
F 3 "" H 1750 2150 60  0000 C CNN
	1    1750 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1250 1750 1950
Wire Wire Line
	1900 1900 1750 1900
Connection ~ 1750 1900
Wire Wire Line
	1750 2350 1750 2700
Wire Wire Line
	1600 2400 1900 2400
Connection ~ 1750 2400
$Comp
L GND #PWR6
U 1 1 548715C3
P 5600 2700
F 0 "#PWR6" H 5600 2700 30  0001 C CNN
F 1 "GND" H 5600 2630 30  0001 C CNN
F 2 "" H 5600 2700 60  0000 C CNN
F 3 "" H 5600 2700 60  0000 C CNN
	1    5600 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2700 5600 2450
Wire Wire Line
	5600 1950 5600 1250
$Comp
L R R1
U 1 1 54871642
P 2150 1600
F 0 "R1" V 2230 1600 40  0000 C CNN
F 1 "22k" V 2157 1601 40  0000 C CNN
F 2 "" V 2080 1600 30  0000 C CNN
F 3 "" H 2150 1600 30  0000 C CNN
	1    2150 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 1600 1900 1600
Connection ~ 1750 1600
Wire Wire Line
	2400 1600 2600 1600
Text GLabel 2600 1600 2    60   Output ~ 0
RST
Text GLabel 4850 2400 2    60   Input ~ 0
RST
Wire Wire Line
	4850 2400 4600 2400
$Comp
L CONN_01X03 P2
U 1 1 5487185B
P 5150 1750
F 0 "P2" H 5150 1950 50  0000 C CNN
F 1 "CONN_01X03" V 5250 1750 50  0000 C CNN
F 2 "" H 5150 1750 60  0000 C CNN
F 3 "" H 5150 1750 60  0000 C CNN
	1    5150 1750
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR3
U 1 1 5487191A
P 5150 1250
F 0 "#PWR3" H 5150 1340 20  0001 C CNN
F 1 "+5V" H 5150 1340 30  0000 C CNN
F 2 "" H 5150 1250 60  0000 C CNN
F 3 "" H 5150 1250 60  0000 C CNN
	1    5150 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 54871941
P 5400 2000
F 0 "#PWR4" H 5400 2000 30  0001 C CNN
F 1 "GND" H 5400 1930 30  0001 C CNN
F 2 "" H 5400 2000 60  0000 C CNN
F 3 "" H 5400 2000 60  0000 C CNN
	1    5400 2000
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 54871A73
P 4750 1700
F 0 "R2" V 4830 1700 40  0000 C CNN
F 1 "470R" V 4757 1701 40  0000 C CNN
F 2 "" V 4680 1700 30  0000 C CNN
F 3 "" H 4750 1700 30  0000 C CNN
	1    4750 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 2000 4750 2000
$Comp
L CONN_01X02 P1
U 1 1 54871B3B
P 1150 1650
F 0 "P1" H 1150 1800 50  0000 C CNN
F 1 "CONN_01X02" V 1250 1650 50  0000 C CNN
F 2 "" H 1150 1650 60  0000 C CNN
F 3 "" H 1150 1650 60  0000 C CNN
	1    1150 1650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1350 1700 1600 1700
Wire Wire Line
	1600 1700 1600 2400
Wire Wire Line
	5400 2000 5400 1450
Wire Wire Line
	5400 1450 5250 1450
Wire Wire Line
	5250 1450 5250 1550
Wire Wire Line
	5150 1250 5150 1550
Wire Wire Line
	4750 2000 4750 1950
Wire Wire Line
	4750 1450 4750 1400
Wire Wire Line
	4750 1400 5050 1400
Wire Wire Line
	5050 1400 5050 1550
$EndSCHEMATC
