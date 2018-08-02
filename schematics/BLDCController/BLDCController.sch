EESchema Schematic File Version 4
LIBS:BLDCController-cache
EELAYER 26 0
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
L teensy:Teensy3.5 U1
U 1 1 5B60938B
P 5700 3500
F 0 "U1" V 5753 1272 60  0000 R CNN
F 1 "Teensy3.5" V 5647 1272 60  0000 R CNN
F 2 "Teensy:Teensy35_36" H 5700 3500 60  0001 C CNN
F 3 "" H 5700 3500 60  0000 C CNN
	1    5700 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6200 4500 6200 5250
Wire Wire Line
	6200 5250 1050 5250
$Comp
L power:GND #PWR0101
U 1 1 5B609468
P 1050 5250
F 0 "#PWR0101" H 1050 5000 50  0001 C CNN
F 1 "GND" H 1055 5077 50  0000 C CNN
F 2 "" H 1050 5250 50  0001 C CNN
F 3 "" H 1050 5250 50  0001 C CNN
	1    1050 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 2500 6900 1050
Wire Wire Line
	6900 1050 1100 1050
$Comp
L power:+5V #PWR0102
U 1 1 5B60957A
P 1100 1050
F 0 "#PWR0102" H 1100 900 50  0001 C CNN
F 1 "+5V" H 1115 1223 50  0000 C CNN
F 2 "" H 1100 1050 50  0001 C CNN
F 3 "" H 1100 1050 50  0001 C CNN
	1    1100 1050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
