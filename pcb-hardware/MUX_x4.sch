EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L power:GND #PWR?
U 1 1 6152F9D4
P 2500 6375
AR Path="/6152F9D4" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6152F9D4" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6152F9D4" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6152F9D4" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6152F9D4" Ref="#PWR030"  Part="1" 
F 0 "#PWR030" H 2500 6125 50  0001 C CNN
F 1 "GND" H 2505 6202 50  0000 C CNN
F 2 "" H 2500 6375 50  0001 C CNN
F 3 "" H 2500 6375 50  0001 C CNN
	1    2500 6375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6152F9EA
P 8650 3125
AR Path="/6152F9EA" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6152F9EA" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6152F9EA" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6152F9EA" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6152F9EA" Ref="#PWR036"  Part="1" 
F 0 "#PWR036" H 8650 2875 50  0001 C CNN
F 1 "GND" H 8655 2952 50  0000 C CNN
F 2 "" H 8650 3125 50  0001 C CNN
F 3 "" H 8650 3125 50  0001 C CNN
	1    8650 3125
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 6152F9F0
P 8650 1725
AR Path="/6152F9F0" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6152F9F0" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6152F9F0" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6152F9F0" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6152F9F0" Ref="#PWR035"  Part="1" 
F 0 "#PWR035" H 8650 1575 50  0001 C CNN
F 1 "+3V3" H 8665 1898 50  0000 C CNN
F 2 "" H 8650 1725 50  0001 C CNN
F 3 "" H 8650 1725 50  0001 C CNN
	1    8650 1725
	1    0    0    -1  
$EndComp
Text Label 8250 2225 2    50   ~ 0
S3_A
Text Label 8250 2125 2    50   ~ 0
S2_A
Text Label 8250 2025 2    50   ~ 0
S1_A
Text Label 8250 1925 2    50   ~ 0
S0_A
Text Label 3000 2925 0    50   ~ 0
S3_A
$Comp
L 74xx:74HC595 U?
U 1 1 6152FA07
P 8650 2325
AR Path="/6152FA07" Ref="U?"  Part="1" 
AR Path="/6125DC69/6152FA07" Ref="U?"  Part="1" 
AR Path="/614F71F8/6152FA07" Ref="U?"  Part="1" 
AR Path="/6167F126/6152FA07" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6152FA07" Ref="U10"  Part="1" 
F 0 "U10" H 9025 2950 50  0000 C CNN
F 1 "SN74HC595" H 8900 2875 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 8650 2325 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8650 2325 50  0001 C CNN
	1    8650 2325
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6152FA59
P 2500 3825
AR Path="/6152FA59" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6152FA59" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6152FA59" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6152FA59" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6152FA59" Ref="#PWR028"  Part="1" 
F 0 "#PWR028" H 2500 3575 50  0001 C CNN
F 1 "GND" H 2505 3652 50  0000 C CNN
F 2 "" H 2500 3825 50  0001 C CNN
F 3 "" H 2500 3825 50  0001 C CNN
	1    2500 3825
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 2125 9350 2125
Wire Wire Line
	9050 2425 9350 2425
Wire Wire Line
	9050 1925 9350 1925
Text Label 3000 5475 0    50   ~ 0
S3_B
Text Label 3000 5375 0    50   ~ 0
S2_B
Text Label 3000 5275 0    50   ~ 0
S1_B
Text Label 3000 5175 0    50   ~ 0
S0_B
Text HLabel 9350 1925 2    50   Input ~ 0
SER
Text HLabel 9350 2125 2    50   Input ~ 0
SRCLK
Text HLabel 9350 2425 2    50   Input ~ 0
RCLK
Text HLabel 975  2025 0    50   Input ~ 0
E0
Text HLabel 975  2125 0    50   Input ~ 0
E1
Text HLabel 975  2225 0    50   Input ~ 0
E2
Text HLabel 975  2325 0    50   Input ~ 0
E3
Text HLabel 975  2425 0    50   Input ~ 0
E4
Text HLabel 975  2525 0    50   Input ~ 0
E5
Text HLabel 975  2625 0    50   Input ~ 0
E6
Text HLabel 975  2725 0    50   Input ~ 0
E7
Text HLabel 975  2825 0    50   Input ~ 0
E8
Text HLabel 975  2925 0    50   Input ~ 0
E9
Text HLabel 975  3025 0    50   Input ~ 0
E10
Text HLabel 975  3125 0    50   Input ~ 0
E11
Text HLabel 975  3225 0    50   Input ~ 0
E12
Text HLabel 975  3325 0    50   Input ~ 0
E13
Text HLabel 975  3425 0    50   Input ~ 0
E14
Text HLabel 975  3525 0    50   Input ~ 0
E15
Text HLabel 3000 2225 2    50   Input ~ 0
RawA1
Text HLabel 3000 3325 2    50   Input ~ 0
EN_MUX_A1
Text HLabel 3000 4775 2    50   Input ~ 0
RawB1
Text HLabel 3000 5875 2    50   Input ~ 0
EN_MUX_B1
$Comp
L 74xx:CD74HC4067M U?
U 1 1 6152FA5F
P 2500 5275
AR Path="/6152FA5F" Ref="U?"  Part="1" 
AR Path="/6125DC69/6152FA5F" Ref="U?"  Part="1" 
AR Path="/614F71F8/6152FA5F" Ref="U?"  Part="1" 
AR Path="/6167F126/6152FA5F" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6152FA5F" Ref="U7"  Part="1" 
F 0 "U7" H 2400 6250 50  0000 C CNN
F 1 "CD74HC4067M" H 2175 6175 50  0000 C CNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 3400 4275 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4067.pdf" H 2150 6125 50  0001 C CNN
	1    2500 5275
	-1   0    0    -1  
$EndComp
Text Label 1725 2025 2    50   ~ 0
E0x
Text Label 1725 2125 2    50   ~ 0
E1x
Text Label 1725 2225 2    50   ~ 0
E2x
Text Label 1725 2325 2    50   ~ 0
E3x
Text Label 1725 2425 2    50   ~ 0
E4x
Text Label 1725 2525 2    50   ~ 0
E5x
Text Label 1725 2625 2    50   ~ 0
E6x
Text Label 1725 2725 2    50   ~ 0
E7x
Text Label 1725 2825 2    50   ~ 0
E8x
Text Label 1725 2925 2    50   ~ 0
E9x
Text Label 1725 3025 2    50   ~ 0
E10x
Text Label 1725 3125 2    50   ~ 0
E11x
Text Label 1725 3225 2    50   ~ 0
E12x
Text Label 1725 3325 2    50   ~ 0
E13x
Text Label 1725 3425 2    50   ~ 0
E14x
Text Label 1725 3525 2    50   ~ 0
E15x
Wire Wire Line
	975  2025 2000 2025
Wire Wire Line
	975  3525 2000 3525
Wire Wire Line
	975  3425 2000 3425
Wire Wire Line
	975  3325 2000 3325
Wire Wire Line
	975  3225 2000 3225
Wire Wire Line
	975  3125 2000 3125
Wire Wire Line
	975  3025 2000 3025
Wire Wire Line
	975  2925 2000 2925
Wire Wire Line
	975  2825 2000 2825
Wire Wire Line
	975  2725 2000 2725
Wire Wire Line
	975  2625 2000 2625
Wire Wire Line
	975  2125 2000 2125
Wire Wire Line
	975  2225 2000 2225
Wire Wire Line
	975  2325 2000 2325
Wire Wire Line
	975  2425 2000 2425
Wire Wire Line
	975  2525 2000 2525
Wire Wire Line
	1550 6075 2000 6075
Wire Wire Line
	1550 4575 2000 4575
Wire Wire Line
	2000 4675 1550 4675
Wire Wire Line
	2000 4775 1550 4775
Wire Wire Line
	2000 4875 1550 4875
Wire Wire Line
	2000 4975 1550 4975
Wire Wire Line
	2000 5075 1550 5075
Wire Wire Line
	2000 5175 1550 5175
Wire Wire Line
	2000 5275 1550 5275
Wire Wire Line
	2000 5375 1550 5375
Wire Wire Line
	2000 5475 1550 5475
Wire Wire Line
	2000 5575 1550 5575
Wire Wire Line
	2000 5675 1550 5675
Wire Wire Line
	2000 5775 1550 5775
Wire Wire Line
	2000 5875 1550 5875
Wire Wire Line
	2000 5975 1550 5975
Text Label 1750 4575 2    50   ~ 0
E0x
Text Label 1750 4675 2    50   ~ 0
E1x
Text Label 1750 4775 2    50   ~ 0
E2x
Text Label 1750 4875 2    50   ~ 0
E3x
Text Label 1750 4975 2    50   ~ 0
E4x
Text Label 1750 5075 2    50   ~ 0
E5x
Text Label 1750 5175 2    50   ~ 0
E6x
Text Label 1750 5275 2    50   ~ 0
E7x
Text Label 1750 5375 2    50   ~ 0
E8x
Text Label 1750 5475 2    50   ~ 0
E9x
Text Label 1750 5575 2    50   ~ 0
E10x
Text Label 1750 5675 2    50   ~ 0
E11x
Text Label 1750 5775 2    50   ~ 0
E12x
Text Label 1750 5875 2    50   ~ 0
E13x
Text Label 1750 5975 2    50   ~ 0
E14x
Text Label 1750 6075 2    50   ~ 0
E15x
$Comp
L power:GND #PWR?
U 1 1 6159F338
P 4800 6375
AR Path="/6159F338" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6159F338" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6159F338" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6159F338" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6159F338" Ref="#PWR034"  Part="1" 
F 0 "#PWR034" H 4800 6125 50  0001 C CNN
F 1 "GND" H 4805 6202 50  0000 C CNN
F 2 "" H 4800 6375 50  0001 C CNN
F 3 "" H 4800 6375 50  0001 C CNN
	1    4800 6375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6159F33E
P 8650 5000
AR Path="/6159F33E" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6159F33E" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6159F33E" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6159F33E" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6159F33E" Ref="#PWR038"  Part="1" 
F 0 "#PWR038" H 8650 4750 50  0001 C CNN
F 1 "GND" H 8655 4827 50  0000 C CNN
F 2 "" H 8650 5000 50  0001 C CNN
F 3 "" H 8650 5000 50  0001 C CNN
	1    8650 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 6159F344
P 8650 3625
AR Path="/6159F344" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6159F344" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6159F344" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6159F344" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6159F344" Ref="#PWR037"  Part="1" 
F 0 "#PWR037" H 8650 3475 50  0001 C CNN
F 1 "+3V3" H 8665 3798 50  0000 C CNN
F 2 "" H 8650 3625 50  0001 C CNN
F 3 "" H 8650 3625 50  0001 C CNN
	1    8650 3625
	1    0    0    -1  
$EndComp
Text Label 8250 2625 2    50   ~ 0
S3_B
Text Label 8250 2525 2    50   ~ 0
S2_B
Text Label 8250 2425 2    50   ~ 0
S1_B
Text Label 8250 2325 2    50   ~ 0
S0_B
Text Label 5300 2925 0    50   ~ 0
S7_A
Text Label 5300 2825 0    50   ~ 0
S6_A
Text Label 5300 2725 0    50   ~ 0
S5_A
$Comp
L 74xx:74HC595 U?
U 1 1 6159F355
P 8650 4225
AR Path="/6159F355" Ref="U?"  Part="1" 
AR Path="/6125DC69/6159F355" Ref="U?"  Part="1" 
AR Path="/614F71F8/6159F355" Ref="U?"  Part="1" 
AR Path="/6167F126/6159F355" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6159F355" Ref="U11"  Part="1" 
F 0 "U11" H 9025 4850 50  0000 C CNN
F 1 "SN74HC595" H 8900 4775 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 8650 4225 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8650 4225 50  0001 C CNN
	1    8650 4225
	-1   0    0    -1  
$EndComp
$Comp
L 74xx:CD74HC4067M U?
U 1 1 6159F35B
P 4800 2725
AR Path="/6159F35B" Ref="U?"  Part="1" 
AR Path="/6125DC69/6159F35B" Ref="U?"  Part="1" 
AR Path="/614F71F8/6159F35B" Ref="U?"  Part="1" 
AR Path="/6167F126/6159F35B" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6159F35B" Ref="U8"  Part="1" 
F 0 "U8" H 4700 3700 50  0000 C CNN
F 1 "CD74HC4067M" H 4475 3625 50  0000 C CNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 5700 1725 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4067.pdf" H 4450 3575 50  0001 C CNN
	1    4800 2725
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6159F361
P 4800 3825
AR Path="/6159F361" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6159F361" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6159F361" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6159F361" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6159F361" Ref="#PWR032"  Part="1" 
F 0 "#PWR032" H 4800 3575 50  0001 C CNN
F 1 "GND" H 4805 3652 50  0000 C CNN
F 2 "" H 4800 3825 50  0001 C CNN
F 3 "" H 4800 3825 50  0001 C CNN
	1    4800 3825
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 4025 9275 4025
Wire Wire Line
	9050 4325 9275 4325
Text Label 5300 2625 0    50   ~ 0
S4_A
NoConn ~ 4850 9425
Text HLabel 9275 4025 2    50   Input ~ 0
SRCLK
Text HLabel 9275 4325 2    50   Input ~ 0
RCLK
Text HLabel 5300 2225 2    50   Input ~ 0
RawA2
Text HLabel 5300 3325 2    50   Input ~ 0
EN_MUX_A2
Text HLabel 5300 4775 2    50   Input ~ 0
RawB2
Text HLabel 5300 5875 2    50   Input ~ 0
EN_MUX_B2
$Comp
L 74xx:CD74HC4067M U?
U 1 1 6159F38B
P 4800 5275
AR Path="/6159F38B" Ref="U?"  Part="1" 
AR Path="/6125DC69/6159F38B" Ref="U?"  Part="1" 
AR Path="/614F71F8/6159F38B" Ref="U?"  Part="1" 
AR Path="/6167F126/6159F38B" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6159F38B" Ref="U9"  Part="1" 
F 0 "U9" H 4700 6250 50  0000 C CNN
F 1 "CD74HC4067M" H 4475 6175 50  0000 C CNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 5700 4275 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4067.pdf" H 4450 6125 50  0001 C CNN
	1    4800 5275
	-1   0    0    -1  
$EndComp
Text Label 4025 2025 2    50   ~ 0
E0x
Text Label 4025 2125 2    50   ~ 0
E1x
Text Label 4025 2225 2    50   ~ 0
E2x
Text Label 4025 2325 2    50   ~ 0
E3x
Text Label 4025 2425 2    50   ~ 0
E4x
Text Label 4025 2525 2    50   ~ 0
E5x
Text Label 4025 2625 2    50   ~ 0
E6x
Text Label 4025 2725 2    50   ~ 0
E7x
Text Label 4025 2825 2    50   ~ 0
E8x
Text Label 4025 2925 2    50   ~ 0
E9x
Text Label 4025 3025 2    50   ~ 0
E10x
Text Label 4025 3125 2    50   ~ 0
E11x
Text Label 4025 3225 2    50   ~ 0
E12x
Text Label 4025 3325 2    50   ~ 0
E13x
Text Label 4025 3425 2    50   ~ 0
E14x
Text Label 4025 3525 2    50   ~ 0
E15x
Wire Wire Line
	3800 2025 4300 2025
Wire Wire Line
	3800 3525 4300 3525
Wire Wire Line
	3800 3425 4300 3425
Wire Wire Line
	3800 3325 4300 3325
Wire Wire Line
	3800 3225 4300 3225
Wire Wire Line
	3800 3125 4300 3125
Wire Wire Line
	3800 3025 4300 3025
Wire Wire Line
	3800 2925 4300 2925
Wire Wire Line
	3800 2825 4300 2825
Wire Wire Line
	3800 2725 4300 2725
Wire Wire Line
	3800 2625 4300 2625
Wire Wire Line
	3800 2125 4300 2125
Wire Wire Line
	3800 2225 4300 2225
Wire Wire Line
	3800 2325 4300 2325
Wire Wire Line
	3800 2425 4300 2425
Wire Wire Line
	3800 2525 4300 2525
Wire Wire Line
	3850 6075 4300 6075
Wire Wire Line
	3850 4575 4300 4575
Wire Wire Line
	4300 4675 3850 4675
Wire Wire Line
	4300 4775 3850 4775
Wire Wire Line
	4300 4875 3850 4875
Wire Wire Line
	4300 4975 3850 4975
Wire Wire Line
	4300 5075 3850 5075
Wire Wire Line
	4300 5175 3850 5175
Wire Wire Line
	4300 5275 3850 5275
Wire Wire Line
	4300 5375 3850 5375
Wire Wire Line
	4300 5475 3850 5475
Wire Wire Line
	4300 5575 3850 5575
Wire Wire Line
	4300 5675 3850 5675
Wire Wire Line
	4300 5775 3850 5775
Wire Wire Line
	4300 5875 3850 5875
Wire Wire Line
	4300 5975 3850 5975
Text Label 4050 4575 2    50   ~ 0
E0x
Text Label 4050 4675 2    50   ~ 0
E1x
Text Label 4050 4775 2    50   ~ 0
E2x
Text Label 4050 4875 2    50   ~ 0
E3x
Text Label 4050 4975 2    50   ~ 0
E4x
Text Label 4050 5075 2    50   ~ 0
E5x
Text Label 4050 5175 2    50   ~ 0
E6x
Text Label 4050 5275 2    50   ~ 0
E7x
Text Label 4050 5375 2    50   ~ 0
E8x
Text Label 4050 5475 2    50   ~ 0
E9x
Text Label 4050 5575 2    50   ~ 0
E10x
Text Label 4050 5675 2    50   ~ 0
E11x
Text Label 4050 5775 2    50   ~ 0
E12x
Text Label 4050 5875 2    50   ~ 0
E13x
Text Label 4050 5975 2    50   ~ 0
E14x
Text Label 4050 6075 2    50   ~ 0
E15x
Text Label 3000 2825 0    50   ~ 0
S2_A
Text Label 3000 2725 0    50   ~ 0
S1_A
Text Label 3000 2625 0    50   ~ 0
S0_A
Text Label 8250 4525 2    50   ~ 0
S7_B
Text Label 8250 4425 2    50   ~ 0
S6_B
Text Label 8250 4325 2    50   ~ 0
S5_B
Text Label 8250 4225 2    50   ~ 0
S4_B
Text Label 8250 4125 2    50   ~ 0
S7_A
Text Label 8250 4025 2    50   ~ 0
S6_A
Text Label 8250 3925 2    50   ~ 0
S5_A
Text Label 8250 3825 2    50   ~ 0
S4_A
Wire Wire Line
	8250 2825 8150 2825
Wire Wire Line
	8150 2825 8150 3350
Wire Wire Line
	8150 3350 9150 3350
Wire Wire Line
	9150 3350 9150 3825
Wire Wire Line
	9150 3825 9050 3825
$Comp
L power:+3V3 #PWR?
U 1 1 6160815C
P 9725 4075
AR Path="/6160815C" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6160815C" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6160815C" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6160815C" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6160815C" Ref="#PWR040"  Part="1" 
F 0 "#PWR040" H 9725 3925 50  0001 C CNN
F 1 "+3V3" H 9740 4248 50  0000 C CNN
F 2 "" H 9725 4075 50  0001 C CNN
F 3 "" H 9725 4075 50  0001 C CNN
	1    9725 4075
	1    0    0    -1  
$EndComp
Wire Wire Line
	9725 4075 9725 4125
Wire Wire Line
	9725 4125 9050 4125
$Comp
L power:+3V3 #PWR?
U 1 1 6160D865
P 9725 2175
AR Path="/6160D865" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/6160D865" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/6160D865" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/6160D865" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/6160D865" Ref="#PWR039"  Part="1" 
F 0 "#PWR039" H 9725 2025 50  0001 C CNN
F 1 "+3V3" H 9740 2348 50  0000 C CNN
F 2 "" H 9725 2175 50  0001 C CNN
F 3 "" H 9725 2175 50  0001 C CNN
	1    9725 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	9725 2175 9725 2225
Wire Wire Line
	9725 2225 9050 2225
Wire Wire Line
	8650 5000 8650 4950
Wire Wire Line
	8650 4950 9050 4950
Wire Wire Line
	9050 4950 9050 4425
Connection ~ 8650 4950
Wire Wire Line
	8650 4950 8650 4925
Wire Wire Line
	8650 3125 8650 3075
Wire Wire Line
	8650 3075 9050 3075
Wire Wire Line
	9050 3075 9050 2525
Connection ~ 8650 3075
Wire Wire Line
	8650 3075 8650 3025
Text Label 5300 5475 0    50   ~ 0
S7_B
Text Label 5300 5375 0    50   ~ 0
S6_B
Text Label 5300 5275 0    50   ~ 0
S5_B
Text Label 5300 5175 0    50   ~ 0
S4_B
Text Label 9100 2425 0    50   ~ 0
RCLK
Text Label 9100 2125 0    50   ~ 0
SRCLK
Text Label 9100 1925 0    50   ~ 0
SER
NoConn ~ 8250 4725
$Comp
L 74xx:CD74HC4067M U?
U 1 1 6152FA53
P 2500 2725
AR Path="/6152FA53" Ref="U?"  Part="1" 
AR Path="/6125DC69/6152FA53" Ref="U?"  Part="1" 
AR Path="/614F71F8/6152FA53" Ref="U?"  Part="1" 
AR Path="/6167F126/6152FA53" Ref="U?"  Part="1" 
AR Path="/61C54C0F/6152FA53" Ref="U6"  Part="1" 
F 0 "U6" H 2400 3700 50  0000 C CNN
F 1 "CD74HC4067M" H 2175 3625 50  0000 C CNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 3400 1725 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4067.pdf" H 2150 3575 50  0001 C CNN
	1    2500 2725
	-1   0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 61C1F3BC
P 4800 1725
AR Path="/61C1F3BC" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/61C1F3BC" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/61C1F3BC" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/61C1F3BC" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/61C1F3BC" Ref="#PWR0105"  Part="1" 
F 0 "#PWR0105" H 4800 1575 50  0001 C CNN
F 1 "+3V3" H 4815 1898 50  0000 C CNN
F 2 "" H 4800 1725 50  0001 C CNN
F 3 "" H 4800 1725 50  0001 C CNN
	1    4800 1725
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 61C26A48
P 2500 1725
AR Path="/61C26A48" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/61C26A48" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/61C26A48" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/61C26A48" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/61C26A48" Ref="#PWR0106"  Part="1" 
F 0 "#PWR0106" H 2500 1575 50  0001 C CNN
F 1 "+3V3" H 2515 1898 50  0000 C CNN
F 2 "" H 2500 1725 50  0001 C CNN
F 3 "" H 2500 1725 50  0001 C CNN
	1    2500 1725
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 61C298DA
P 2500 4275
AR Path="/61C298DA" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/61C298DA" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/61C298DA" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/61C298DA" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/61C298DA" Ref="#PWR0107"  Part="1" 
F 0 "#PWR0107" H 2500 4125 50  0001 C CNN
F 1 "+3V3" H 2515 4448 50  0000 C CNN
F 2 "" H 2500 4275 50  0001 C CNN
F 3 "" H 2500 4275 50  0001 C CNN
	1    2500 4275
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 61C2C208
P 4800 4275
AR Path="/61C2C208" Ref="#PWR?"  Part="1" 
AR Path="/6125DC69/61C2C208" Ref="#PWR?"  Part="1" 
AR Path="/614F71F8/61C2C208" Ref="#PWR?"  Part="1" 
AR Path="/6167F126/61C2C208" Ref="#PWR?"  Part="1" 
AR Path="/61C54C0F/61C2C208" Ref="#PWR0108"  Part="1" 
F 0 "#PWR0108" H 4800 4125 50  0001 C CNN
F 1 "+3V3" H 4815 4448 50  0000 C CNN
F 2 "" H 4800 4275 50  0001 C CNN
F 3 "" H 4800 4275 50  0001 C CNN
	1    4800 4275
	1    0    0    -1  
$EndComp
$EndSCHEMATC
