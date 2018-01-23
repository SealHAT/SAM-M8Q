EESchema Schematic File Version 2
LIBS:MF_Aesthetics
LIBS:MF_Connectors
LIBS:MF_Discrete_Semiconductor
LIBS:MF_Displays
LIBS:MF_Frequency_Control
LIBS:MF_IC_Analog
LIBS:MF_IC_Digital
LIBS:MF_IC_Power
LIBS:MF_LEDs
LIBS:MF_Passives
LIBS:MF_Sensors
LIBS:MF_Switches
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
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
LIBS:MAX44009EDT
LIBS:UBLOX_CAM-M8
LIBS:cam-m8-breakout-cache
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
L CAM-M8 U1
U 1 1 5A666307
P 5150 3750
F 0 "U1" H 4495 4475 50  0000 L BNN
F 1 "CAM-M8" H 5535 4470 50  0000 L BNN
F 2 "GNSS:UBLOX_CAM-M8" H 4500 3000 50  0001 L BNN
F 3 "https://www.u-blox.com/sites/default/files/CAM-M8-FW3_DataSheet_%28UBX-15031574%29.pdf" H 4495 3095 50  0001 L BNN
F 4 "https://www.digikey.com/products/en?FV=ffecd577" H 4500 2900 50  0001 L BNN "Digikey"
	1    5150 3750
	1    0    0    -1  
$EndComp
$Comp
L CON_01X06_PTH_2.54MM J2
U 1 1 5A6663C9
P 7700 3700
F 0 "J2" H 7700 3900 45  0000 L BNN
F 1 "CON_01X06_PTH_2.54MM" H 7700 3800 45  0000 L BNN
F 2 "MF_Connectors:MF_Connectors-PTH_2.54MM_01X06" H 8025 3115 20  0001 C CNN
F 3 "" H 7700 3700 60  0000 C CNN
F 4 ">LABEL01" H 8325 3700 32  0000 R CNN "LABEL01"
F 5 ">LABEL02" H 8325 3600 32  0000 R CNN "LABEL02"
F 6 ">LABEL03" H 8325 3500 32  0000 R CNN "LABEL03"
F 7 ">LABEL04" H 8325 3400 32  0000 R CNN "LABEL04"
F 8 ">LABEL05" H 8325 3300 32  0000 R CNN "LABEL05"
F 9 ">LABEL06" H 8325 3200 32  0000 R CNN "LABEL06"
	1    7700 3700
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR6
U 1 1 5A666510
P 7400 2400
F 0 "#PWR6" H 7400 2250 50  0001 C CNN
F 1 "VCC" H 7400 2550 50  0000 C CNN
F 2 "" H 7400 2400 50  0001 C CNN
F 3 "" H 7400 2400 50  0001 C CNN
	1    7400 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 5A66652A
P 7050 2400
F 0 "#PWR5" H 7050 2150 50  0001 C CNN
F 1 "GND" H 7050 2250 50  0000 C CNN
F 2 "" H 7050 2400 50  0001 C CNN
F 3 "" H 7050 2400 50  0001 C CNN
	1    7050 2400
	-1   0    0    1   
$EndComp
Text Label 7050 3700 0    60   ~ 0
~RESET
Text Label 7050 3800 0    60   ~ 0
SAFEBOOT
Text Label 7050 4200 0    60   ~ 0
LNA_EN
Text Label 7200 2800 0    60   ~ 0
EXT_INT
Text Label 7050 4100 0    60   ~ 0
TIMEPULSE
Text Label 7200 3100 0    60   ~ 0
VCCIO
Text Label 7200 3000 0    60   ~ 0
SDA
Text Label 7200 2900 0    60   ~ 0
SCL
Text Label 7050 4000 0    60   ~ 0
RXD
Text Label 7050 3900 0    60   ~ 0
TXD
$Comp
L CAPACITOR_NP_0805 C2
U 1 1 5A666872
P 3450 4150
F 0 "C2" H 3550 4210 45  0000 L BNN
F 1 "100nF" H 3550 4090 45  0000 L BNN
F 2 "MF_Passives:MF_Passives-C0805" H 3700 4060 20  0001 C CNN
F 3 "" H 3450 4150 60  0001 C CNN
	1    3450 4150
	1    0    0    -1  
$EndComp
$Comp
L CAPACITOR_NP_0805 C1
U 1 1 5A66690D
P 2950 4150
F 0 "C1" H 3050 4210 45  0000 L BNN
F 1 "4.7uF" H 3050 4090 45  0000 L BNN
F 2 "MF_Passives:MF_Passives-C0805" H 3200 4060 20  0001 C CNN
F 3 "" H 2950 4150 60  0001 C CNN
	1    2950 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5A6669B3
P 3450 4550
F 0 "#PWR2" H 3450 4300 50  0001 C CNN
F 1 "GND" H 3450 4400 50  0000 C CNN
F 2 "" H 3450 4550 50  0001 C CNN
F 3 "" H 3450 4550 50  0001 C CNN
	1    3450 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 5A6669DB
P 2950 4550
F 0 "#PWR1" H 2950 4300 50  0001 C CNN
F 1 "GND" H 2950 4400 50  0000 C CNN
F 2 "" H 2950 4550 50  0001 C CNN
F 3 "" H 2950 4550 50  0001 C CNN
	1    2950 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 5A666A3B
P 4150 4550
F 0 "#PWR3" H 4150 4300 50  0001 C CNN
F 1 "GND" H 4150 4400 50  0000 C CNN
F 2 "" H 4150 4550 50  0001 C CNN
F 3 "" H 4150 4550 50  0001 C CNN
	1    4150 4550
	1    0    0    -1  
$EndComp
Text Label 3900 3900 0    60   ~ 0
VCC
Text Label 3900 4000 0    60   ~ 0
VCCIO
$Comp
L RESISTOR_0805 R1
U 1 1 5A666C13
P 6250 2800
F 0 "R1" H 6350 2860 45  0000 L BNN
F 1 "0" H 6350 2740 45  0000 L BNN
F 2 "MF_Passives:MF_Passives-R0805" H 6500 2710 20  0001 C CNN
F 3 "" H 6250 2800 60  0001 C CNN
	1    6250 2800
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR4
U 1 1 5A666CB0
P 6250 2500
F 0 "#PWR4" H 6250 2250 50  0001 C CNN
F 1 "GND" H 6250 2350 50  0000 C CNN
F 2 "" H 6250 2500 50  0001 C CNN
F 3 "" H 6250 2500 50  0001 C CNN
	1    6250 2500
	-1   0    0    1   
$EndComp
Text Label 6550 3350 2    60   ~ 0
TXD
Text Label 6550 3450 2    60   ~ 0
RXD
Text Label 6550 3600 2    60   ~ 0
SCL
Text Label 6550 3700 2    60   ~ 0
SDA
Text Label 6550 3900 2    60   ~ 0
EXT_INT
Text Label 6550 4000 2    60   ~ 0
TIMEPULSE
Text Label 6550 4100 2    60   ~ 0
LNA_EN
Text Label 6550 4200 2    60   ~ 0
SAFEBOOT
Text Label 3950 3150 0    60   ~ 0
~RESET
Text Label 3900 4100 0    60   ~ 0
VBAK
Text Label 7200 2700 0    60   ~ 0
VBAK
$Comp
L CON_01X08_PTH_2.54MM J1
U 1 1 5A667C90
P 7700 2500
F 0 "J1" H 7700 2700 45  0000 L BNN
F 1 "CON_01X08_PTH_2.54MM" H 7700 2600 45  0000 L BNN
F 2 "MF_Connectors:MF_Connectors-PTH_2.54MM_01X08" H 8025 1715 20  0001 C CNN
F 3 "" H 7700 2500 60  0000 C CNN
F 4 ">LABEL01" H 8325 2500 32  0000 R CNN "LABEL01"
F 5 ">LABEL02" H 8325 2400 32  0000 R CNN "LABEL02"
F 6 ">LABEL03" H 8325 2300 32  0000 R CNN "LABEL03"
F 7 ">LABEL04" H 8325 2200 32  0000 R CNN "LABEL04"
F 8 ">LABEL05" H 8325 2100 32  0000 R CNN "LABEL05"
F 9 ">LABEL06" H 8325 2000 32  0000 R CNN "LABEL06"
F 10 ">LABEL07" H 8325 1900 32  0000 R CNN "LABEL07"
F 11 ">LABEL08" H 8325 1800 32  0000 R CNN "LABEL08"
	1    7700 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3700 7600 3700
Wire Wire Line
	7050 3800 7600 3800
Wire Wire Line
	7050 3900 7600 3900
Wire Wire Line
	7050 4000 7600 4000
Wire Wire Line
	7050 4100 7600 4100
Wire Wire Line
	7050 4200 7600 4200
Wire Wire Line
	7400 2400 7400 2600
Wire Wire Line
	7200 2900 7600 2900
Wire Wire Line
	7200 2700 7600 2700
Wire Wire Line
	4300 4000 3450 4000
Wire Wire Line
	3450 4000 3450 4050
Wire Wire Line
	3450 4250 3450 4550
Wire Wire Line
	4300 4200 4150 4200
Wire Wire Line
	4150 4200 4150 4550
Wire Wire Line
	2950 4250 2950 4550
Wire Wire Line
	4300 3900 2950 3900
Wire Wire Line
	2950 3900 2950 4050
Wire Wire Line
	6050 3150 6250 3150
Wire Wire Line
	6250 3150 6250 3000
Wire Wire Line
	6250 2600 6250 2500
Wire Wire Line
	4300 3150 3950 3150
Wire Wire Line
	6050 3350 6550 3350
Wire Wire Line
	6550 3450 6050 3450
Wire Wire Line
	6050 3600 6550 3600
Wire Wire Line
	6550 3700 6050 3700
Wire Wire Line
	6050 3900 6550 3900
Wire Wire Line
	6550 4000 6050 4000
Wire Wire Line
	6050 4100 6550 4100
Wire Wire Line
	6550 4200 6050 4200
Wire Wire Line
	4300 4100 3900 4100
Wire Wire Line
	7050 2400 7050 3200
Wire Wire Line
	7600 3100 7200 3100
Wire Wire Line
	7050 3200 7600 3200
Wire Wire Line
	7600 3000 7200 3000
Wire Wire Line
	7200 2800 7600 2800
Wire Wire Line
	7400 2600 7600 2600
Wire Wire Line
	7600 2500 7050 2500
Connection ~ 7050 2500
$EndSCHEMATC
