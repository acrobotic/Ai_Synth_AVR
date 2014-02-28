/**************************************************************
*
AVR Synthesizer
Author:
Sam Stratter @humanHardDrive

***
Please consider buying products from Acrobotic to help fund future Open-Source projects
like this! We'll always put our best effort in every project, and release all our design
files and code for you to use.
***

Description:
Firmware for the AVR Synthesizer Kit (http://acrobotic.com/catalog/product/view/id/51)
written for the ATMega328p microcontroller using the Arduino IDE v1.0.5.

Using the principle of DDS, the code features:
- 2 Oscillators
- 6 Wave forms (Sine, Triangle, Left Saw, Right Saw, Square, Noise)
- 3 Detune options for second oscillator (Cents, Semitone, Octave)
- Adjustable mixing between two oscillators
- 5 banks

- LFO with 5 wave forms (Sine, Triangle, Left Saw Right Saw, Square)
- LFO routes to control second oscillator detuning (Cents, Semitone, Octave)
- LFO ramps from 0 to 100 hertz

- 20 note arpegiator, relative key control
- Arpegiator ramps from 0 to 50 hertz
- XOR logic modulation
- MIDI in

License:
The contents of this file are licensed under a Creative Commons Attribution-ShareAlike 3.0
Unported License http://creativecommons.org/licenses/by-sa/3.0/.
*
****************************************************************/

#include "avr/pgmspace.h"

//Wave form defines
#define WAVE_SINE  0
#define WAVE_TRI   1
#define WAVE_LSAW  2
#define WAVE_RSAW  3
#define WAVE_SQU   4
#define WAVE_FLAT  5
#define WAVE_NOISE 6

//LFO route options
#define ROUTE_NONE  0
#define ROUTE_SEMI  1
#define ROUTE_CENT  2
#define ROUTE_OCT   3
#define ROUTE_PHASE	4

//Menu defines
#define MENU_NORM	0
#define MENU_LFO	1
#define MENU_EXTRA	2

//Pin Definitions
#define OSC1_BTN_PIN	6
#define OSC2_BTN_PIN	7
#define LOCK_PIN		10
#define BANK_PIN		12
#define SAVE_PIN		8
#define OSC1_PIN		11
#define EXTRA_PIN		9

#define CENTS_PIN   A3
#define SEMIS_PIN   A2
#define OCTAVE_PIN  A1
#define WEIGHT_PIN  A0

#define LFOFREQ_PIN  A3
#define LFOAMNT_PIN  A2
#define ARPSPD_PIN   A1

//MIDI Commands
#define NOTE_ON		9
#define NOTE_OFF	8
#define AFTERTOUCH	13
#define CC			11
#define PC			12
#define PITCH_WHEEL	14

//MIDI CC
#define CC_osc1WaveForm		0x77
#define CC_osc2WaveForm		0x76
#define CC_cents			0x0E
#define CC_semis			0x0F
#define CC_octave			0x10
#define CC_weight			0x11

#define CC_lfoWaveForm	0x72
#define CC_lfoRoute		0x73
#define CC_lfoSpeed		0x12


#define PIN_SCE   A4 //Pin 3 on LCD
#define PIN_RESET A5 //Pin 4 on LCD
#define PIN_DC    13 //Pin 5 on LCD
#define PIN_SDIN  5 //Pin 6 on LCD
#define PIN_SCLK  4 //Pin 7 on LCD

#define BOUNCE_THRESH 100

//ADC extremes
#define ADC_MIN		5
#define ADC_MAX		1015

//Detune max
#define CENTS_MAX	50
#define OCT_MAX		2
#define SEMIS_MAX	12
#define PHASE_MAX	128

#define WEIGHT_MAX    16
#define NUM_BANKS     5
#define MAX_arp_NOTES 20
#define MIDI_OFFSET   20

//Menu Options
#define OSC1_MAX	WAVE_FLAT
#define OSC2_MAX	WAVE_NOISE
#define LFO_MAX		WAVE_FLAT
#define ROUTE_MAX	ROUTE_PHASE

//The DC pin tells the LCD if we are sending a command or data
#define LCD_COMMAND 0
#define LCD_DATA  1

//You may find a different size screen, but this one is 84 by 48 pixels
#define LCD_X     84
#define LCD_Y     48

//This table contains the hex values that represent pixels
//for a font that is 5 pixels wide and 8 pixels high
static const byte ASCII[][5] =
{
	{0x00, 0x00, 0x00, 0x00, 0x00} // 20
	,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
	,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
	,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
	,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
	,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
	,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
	,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
	,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
	,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
	,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
	,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
	,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
	,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
	,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
	,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
	,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
	,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
	,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
	,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
	,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
	,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
	,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
	,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
	,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
	,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
	,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
	,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
	,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
	,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
	,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
	,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
	,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
	,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
	,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
	,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
	,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
	,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
	,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
	,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
	,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
	,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
	,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
	,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
	,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
	,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
	,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
	,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
	,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
	,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
	,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
	,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
	,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
	,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
	,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
	,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
	,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
	,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
	,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
	,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
	,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
	,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
	,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
	,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
	,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
	,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
	,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
	,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
	,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
	,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
	,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
	,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
	,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
	,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
	,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
	,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
	,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
	,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
	,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
	,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
	,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
	,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
	,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
	,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
	,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
	,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
	,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
	,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
	,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
	,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
	,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
	,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
	,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
	,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
	,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
	,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

//Waveform definitions
PROGMEM  prog_uchar waveTable[]  = {
	//sine wave
	0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,
	70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124,127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,
	221,223,225,227,229,231,233,234,236,238,239,240,242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,
	253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,221,219,217,215,212,210,208,205,
	203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,
	102,99,96,93,90,87,84,81,78,76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,
	3,2,2,1,1,1,0,0,0,

	//triangle wave
	1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85,
	87,89,91,93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,
	197,199,201,203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,237,239,241,243,245,247,249,251,253,255,255,253,251,249,247,
	245,243,241,239,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,189,187,185,183,181,179,177,
	175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,145,143,141,139,137,135,133,131,129,127,125,123,121,119,117,115,113,111,109,107,
	105,103,101,99,97,95,93,91,89,87,85,83,81,79,77,75,73,71,69,67,65,63,61,59,57,55,53,51,49,47,45,43,41,39,37,35,33,31,29,27,25,23,21,19,17,15,
	13,11,9,7,5,3,1,

	//left saw (negative slope)
	255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,
	212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,
	176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,
	140,139,138,137,136,135,134,133,132,131,130,129,128127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,
	89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,
	41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,

	//right saw (positive slope)
	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,
	78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,
	120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,
	162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,
	204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,
	240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,

	//50% duty cycle square wave
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,

	//flat wave
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

//Note frequencies
double keyFreq[] = {
	27.5, 29.1352, 30.8677,     //Octave 0
	32.7032, 34.6478, 36.7081, 38.8909, 41.2034, 43.6535, 46.2493, 48.9994, 51.9131, 55, 58.2075, 61.7354,     //Octave 1
	65.4064, 69.2957, 73.4162, 77.7817, 82.4069, 87.3071, 92.4986, 97.9989, 103.826, 110, 116.541, 123.471,    //Octave 2
	130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220, 233.082, 246.942,    //Octave 3
	261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 394.995, 415.305, 440, 466.164, 493.883,    //Octave 4
	523.251, 554.365, 587.330, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880, 932.328, 987.767,    //Octave 5
	1406.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53,   //Octave 6
	2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520, 3729.31, 3951.07,   //Octave 7
	4186.01                                                                                                    //Octave 8
};

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// const double refclk=31372.549;  // =16MHz / 510
const double refclk=31376.6;      // measured 16MHz

//Phase Accumulators
volatile unsigned long phaccu1 = 0;   // phase accumulator
volatile unsigned long phaccu2 = 0;   // phase accumulator
volatile unsigned long phaccu3 = 0;
volatile unsigned long tword_mOsc1 = 0;  // dds tuning word m
volatile unsigned long tword_mOsc2 = 0;
volatile unsigned long tword_mLFO = 0;

volatile byte smallTimer = 0;
volatile unsigned long millisecs = 0; //because timer0 is disabled

//Detune settings
double centMultiplier = 0; //the number multiplied to the frequency to create cent shift
double cents = 0; //number of cents
int octaveShift = 0;
int semis = 0;
int phase = 0;

//LFO Settings
volatile int centsRange = 0;
volatile int dCents = 0;
volatile int octaveRange = 0;
volatile int dOctave = 0;
volatile int semisRange = 0;
volatile int dSemis = 0;
volatile int phaseRange = 0;
volatile int dPhase = 0;

double lfoFreq = 0;
char osc1WaveForm = WAVE_LSAW;
char osc2WaveForm = WAVE_SINE;
char lfoWaveForm = WAVE_FLAT;
char lfoRoute = ROUTE_NONE;
boolean noteSync = false;

//Mixing Settings
boolean xorAddition = false;
volatile byte weight1 = 8;
volatile byte weight2 = 8;

//Button debouncing
boolean osc1BtnPressed = false;
boolean osc2BtnPressed = false;
boolean btnLockPressed = false;
boolean btnSavePressed = false;
boolean btnBankPressed = false;
boolean btnExtraPressed = false;

unsigned long osc1BtnLastPressed = 0;
unsigned long osc2BtnLastPressed = 0;
unsigned long btnLockLastPressed = 0;
unsigned long btnSaveLastPressed = 0;
unsigned long btnBankLastPressed = 0;
unsigned long btnarpLastPressed = 0;
unsigned long btnExtraLastPressed = 0;

//Control Settings
int noteSelect = 44;
boolean notePlaying = false;
byte currentCommand = 0;

//Arpeggio Settings
boolean arpMode = false;
int rootKey = 0;
volatile char arp[MAX_arp_NOTES];
volatile byte arpMaxCount = 0;
volatile byte arpCount = 0;
volatile byte arpSpeed = 0;
volatile unsigned long arpTimer = 0;

boolean legato = false; //Not used, yet

//Settings settings
int settingsMenu = 0;
boolean settingsLocked[3];

//Bank Settings
int bankSelect = 1;
int bank[NUM_BANKS][35];

void setup()
{
	Serial.begin(31250); //The appropriate MIDI communication speed
	//See the SerialEvent() function for more info
	LCDInit();
	clearLCD();
	
	delay(1500);
	
	normalSettingsLCD();

	pinMode(OSC1_PIN, OUTPUT); //PWM pin is an output

	pinMode(OSC1_BTN_PIN, INPUT); //Switch Osc1 waveform button
	digitalWrite(OSC1_BTN_PIN, HIGH); //pull-up, because I can't get used to the "appropriate" way to handle it

	pinMode(OSC2_BTN_PIN, INPUT);  //Switch Osc2 waveform button
	digitalWrite(OSC2_BTN_PIN, HIGH);

	pinMode(LOCK_PIN, INPUT); //Lock preset button
	digitalWrite(LOCK_PIN, HIGH);

	pinMode(BANK_PIN, INPUT); //Bank switch button
	digitalWrite(BANK_PIN, HIGH);

	pinMode(SAVE_PIN, INPUT); //Save preset button
	digitalWrite(SAVE_PIN, HIGH);

	pinMode(EXTRA_PIN, INPUT); //Arpeggio button
	digitalWrite(EXTRA_PIN, HIGH);

	Setup_timer2();

	//disable interrupts to avoid timing distortion
	cbi (TIMSK0,TOIE0);              // disable Timer0 !!! delay() is now not available
	sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt
}

void loop()
{
	osc1BtnCheck();
	osc2BtnCheck();
	
	bankBtnCheck();
	
	saveBtnCheck();
	lockBtnCheck();
	extraBtnCheck();

	if((millisecs - arpTimer > arpSpeed) && notePlaying && arpSpeed > 0 && !arpMode) //handle the arpeggio
	{
		noteSelect = rootKey + arp[arpCount];
		arpTimer = millisecs;

		arpCount++; //move through the array

		if(arpCount >= arpMaxCount)
		{
			arpCount = 0;
		}
		
		if(noteSync)
		{
			phaccu3 = 0;
		}
	}

	if(millisecs % 5 < 5)
	{
		switch(settingsMenu)
		{
			case MENU_NORM:
			if(!settingsLocked[MENU_NORM])
			{
				cents = map(analogRead(CENTS_PIN),ADC_MIN,ADC_MAX,-CENTS_MAX,CENTS_MAX);  //from -100 to 100 cents
				semis = map(analogRead(SEMIS_PIN),ADC_MIN,ADC_MAX,-12,12);  //from -12 to 12 semitone
				octaveShift = map(analogRead(OCTAVE_PIN),ADC_MIN,ADC_MAX,-OCT_MAX, OCT_MAX);  //from -2 to 2 octaves
				weight2 = map(analogRead(WEIGHT_PIN),ADC_MAX,ADC_MIN,0,WEIGHT_MAX);  //weight of osc2 in summation
			}
			break;
			
			case MENU_LFO:
			if(!settingsLocked[MENU_LFO])
			{
				lfoFreq = analogRead(LFOFREQ_PIN)/100.0; //lfo frequency from 0 to 100 hertz
				uint16_t lfoAmnt = analogRead(LFOAMNT_PIN);
				
				centsRange = map(lfoAmnt,ADC_MIN,ADC_MAX,0,CENTS_MAX);
				semisRange = map(lfoAmnt,ADC_MIN,ADC_MAX,0,SEMIS_MAX);
				octaveRange = map(lfoAmnt,ADC_MIN,ADC_MAX,0,3);
				phaseRange = map(lfoAmnt,ADC_MIN,ADC_MAX,0,128);
			}
			break;
			
			case MENU_EXTRA:
			if(!settingsLocked[MENU_EXTRA])
			{
				phase = map(analogRead(OCTAVE_PIN), ADC_MIN, ADC_MAX, -128, 128);	
				arpSpeed = 1000.0/map(analogRead(WEIGHT_PIN),ADC_MIN,ADC_MAX,1,50); //arpSpeed is measured in milliseconds not hertz
			}
			break;
		}
	}

	noteUpdate(); //update the notes every 2 milliseconds
}

void osc1BtnCheck()
{
	if(!digitalRead(OSC1_BTN_PIN))
	{
		if(!osc1BtnPressed && abs(millisecs - osc1BtnLastPressed) > BOUNCE_THRESH)  //debouncing
		{
			osc1BtnPressed = true;
			
			switch(settingsMenu)
			{
				case MENU_NORM:
				if(!settingsLocked[MENU_NORM])
				{
					osc1WaveForm++;
					
					if(osc1WaveForm > OSC1_MAX)
					{
						osc1WaveForm = WAVE_SINE;
					}
					
					phaccu1 = 0;
					osc1Update();
				}
				break;
				
				case MENU_LFO:
				if(!settingsLocked[MENU_LFO])
				{
					lfoWaveForm++;
					
					if(lfoWaveForm > WAVE_FLAT)
					{
						lfoWaveForm = WAVE_SINE;
					}
					
					lfoUpdate();
				}
				break;
				
				case MENU_EXTRA:
				if(!settingsLocked[MENU_EXTRA])
				{
					xorAddition = !xorAddition;
					
					xorUpdate();
				}
				break;
			}
		}
	}
	else
	{
		if(osc1BtnPressed)
		{
			osc1BtnLastPressed = millisecs;
			osc1BtnPressed = false;
		}
	}
}

void osc2BtnCheck()
{
	if(!digitalRead(OSC2_BTN_PIN))
	{
		if(!osc2BtnPressed && abs(millisecs - osc2BtnLastPressed) > BOUNCE_THRESH)
		{
			osc2BtnPressed = true;

			switch(settingsMenu)
			{
				case MENU_NORM:
				if(!settingsLocked[MENU_NORM])
				{
					osc2WaveForm++;
					
					if(osc2WaveForm > WAVE_NOISE)
					{
						osc2WaveForm = WAVE_SINE;
					}
					
					phaccu2 = 0;
					osc2Update();
				}
				break;
				
				case MENU_LFO:
				if(!settingsLocked[MENU_LFO])
				{
					lfoRoute++;
					
					if(lfoRoute > ROUTE_PHASE)
					{
						lfoRoute = ROUTE_NONE;
					}
					
					routeUpdate();
				}
				break;
				
				case MENU_EXTRA:
				if(!settingsLocked[MENU_EXTRA])
				{
					legato = !legato;
					
					legatoUpdate();
				}
				break;
			}
		}
	}
	else
	{
		if(osc2BtnPressed)
		{
			osc2BtnLastPressed = millisecs;
			osc2BtnPressed = false;
		}
	}
}

void lockBtnCheck()
{
	if(!digitalRead(LOCK_PIN))
	{
		if(!btnLockPressed && abs(millisecs - btnLockLastPressed) > BOUNCE_THRESH)
		{
			btnLockPressed = true;

			settingsLocked[settingsMenu] = !settingsLocked[settingsMenu];

			lockUpdate();
		}
	}
	else
	{
		if(btnLockPressed)
		{
			btnLockLastPressed = millisecs;
			btnLockPressed = false;
		}
	}
}

void bankBtnCheck()
{
	if(!digitalRead(BANK_PIN))
	{
		if(!btnBankPressed && abs(millisecs - btnBankLastPressed) > BOUNCE_THRESH)
		{
			btnBankPressed = true;

			if(settingsMenu == MENU_NORM)
			{
				bankSelect++;
				if(bankSelect > NUM_BANKS)
				{
					bankSelect = 1;
				}

				loadPreset(bankSelect);
				normalSettingsLCD();
			}
		}
	}
	else
	{
		if(btnBankPressed)
		{
			btnBankLastPressed = millisecs;
			btnBankPressed = false;
		}
	}
}

void saveBtnCheck()
{
	if(!digitalRead(SAVE_PIN))
	{
		if(!btnSavePressed && abs(millisecs - btnSaveLastPressed) > BOUNCE_THRESH)
		{
			btnSavePressed = true;

			switch(settingsMenu)
			{
				case MENU_NORM:
				if(!settingsLocked[MENU_NORM])
				{
					savePreset(bankSelect);
				}
				break;
				
				case MENU_LFO:
				if(!settingsLocked[MENU_LFO])
				{
					noteSync = !noteSync;
					
					noteSyncUpdate();
				}
				break;
				
				case MENU_EXTRA:
				if(!settingsLocked[MENU_EXTRA])
				{
					arpMode = !arpMode;

					arpUpdate();

					if(arpMode) //reset the previous
					{
						arpMaxCount = 0; //reset the number of notes
						for(int i = 0;i < MAX_arp_NOTES;i++)
						{
							arp[i] = 0; //reset each note
						}
					}
				}
				break;
			}
		}
	}
	else
	{
		if(btnSavePressed)
		{
			btnSaveLastPressed = millisecs;
			btnSavePressed = false;
		}
	}
}

void extraBtnCheck()
{
	if(!digitalRead(EXTRA_PIN))
	{
		if(!btnExtraPressed && abs(millisecs - btnExtraLastPressed) > BOUNCE_THRESH)
		{
			btnExtraPressed = true;

			settingsMenu++;
			
			if(settingsMenu > MENU_EXTRA)
			{
				settingsMenu = MENU_NORM;
			}
			
			switch(settingsMenu)
			{
				case MENU_NORM:
				normalSettingsLCD();
				break;
				
				case MENU_LFO:
				lfoSettingsLCD();
				break;
				
				case MENU_EXTRA:
				extraSettingsLCD();
				break;
			}

		}
	}
	else
	{
		if(btnExtraPressed)
		{
			btnExtraLastPressed = millisecs;
			btnExtraPressed = false;
		}
	}
}

//Check my included software
void serialEvent()
{
	byte data = Serial.read();
	byte data1 = 0;
	byte data2 = 0;
	
	//Is this a new command?
	if(data >= 128)
	{
		byte command = (data >> 4);
		byte channel = data - (command << 4) + 1; //I ignore the channel data
		
		currentCommand = command;
		
		while(!Serial.available());
		
		data1 = Serial.read();
	}
	else //The old command, what we got was new data
	{
		data1 = data;
	}
	
	//Avoid polychannel aftertouch
	if(currentCommand != PC)
	{
		while(!Serial.available());
		
		data2 = Serial.read();
	}
	else
	{
		return;
	}
	
	
	switch(currentCommand)
	{
		case NOTE_ON:
		noteSelect = data1 - MIDI_OFFSET;
		
		if(!notePlaying && noteSync)
		{
			phaccu3 = 0;
		}

		notePlaying = true;

		if(arpMode) //add notes to the arp array
		{
			if(arpMaxCount == 0) //if just starting arp mode
			{
				rootKey = noteSelect; //get new root key, all notes in array are relative to this value
			}
			else
			{
				arp[arpMaxCount] = noteSelect - rootKey; //calculate relative note
			}
			
			arpMaxCount++; //increment number of notes in arp array

			if(arpMaxCount > MAX_arp_NOTES)
			{
				arpMode = false;
				arpUpdate();
			}
		}
		else
		{
			rootKey = noteSelect; //find the new root key to play the arpeggio
		}

		arpTimer = millisecs;
		//arpCount = 0; //reset arp count
		break;
		
		case NOTE_OFF:
		if((rootKey == data1 - MIDI_OFFSET || noteSelect == data1 - MIDI_OFFSET))
		{
			notePlaying = false;
		}
		break;
		
		case CC:
		switch(data1)
		{
			case CC_osc1WaveForm:
			if(data2 > 64)
			{
				osc1WaveForm++;
				
				if(osc1WaveForm > WAVE_FLAT)
				{
					osc1WaveForm = WAVE_SINE;
				}
				
				if(settingsMenu == MENU_NORM)
				{
					osc1Update();
				}
			}
			break;
			
			case CC_osc2WaveForm:
			if(data2 > 64)
			{
				osc2WaveForm++;
				
				if(osc2WaveForm > WAVE_NOISE)
				{
					osc2WaveForm = WAVE_SINE;
				}
				
				if(settingsMenu == MENU_NORM)
				{
					osc2Update();
				}
			}
			break;
			
			case CC_lfoWaveForm:
			break;
			
			case CC_cents:
			cents = map(data2, 0, 127, -CENTS_MAX, CENTS_MAX);
			break;
			
			case CC_semis:
			semis = map(data2, 0,127, -SEMIS_MAX, SEMIS_MAX);
			break;
			
			case CC_octave:
			octaveShift = map(data2, 0, 127, -2, 2);
			break;
			
			case CC_weight:
			weight2 = map(data2,0,127,0,WEIGHT_MAX);  //weight of osc2 in summation
			break;
		}
		break;
		
		case PC:
		break;
		
		case PITCH_WHEEL:
		cents = map(data2, 0, 127, -100, 100);
		break;
	}
}


//******************************************************************
// timer2 setup
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer2() {

	// Timer2 Clock Prescaler to : 1
	sbi (TCCR2B, CS20);
	cbi (TCCR2B, CS21);
	cbi (TCCR2B, CS22);

	// Timer2 PWM Mode set to Phase Correct PWM
	cbi (TCCR2A, COM2A0);  // clear Compare Match
	sbi (TCCR2A, COM2A1);

	sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
	cbi (TCCR2A, WGM21);
	cbi (TCCR2B, WGM22);
}


ISR(TIMER2_OVF_vect) {
	if(notePlaying)
	{
		phaccu1=phaccu1+tword_mOsc1; // soft DDS, phase accu with 32 bits
		byte icnt1=phaccu1 >> 24;     // use upper 8 bits for phase accu as frequency information

		phaccu2=phaccu2+tword_mOsc2;
		byte icnt2=phaccu2 >> 24;
		
		if(!xorAddition)
		{
			byte osc2;
			
			if(osc2WaveForm != WAVE_NOISE)
			{
				osc2 = ((pgm_read_byte(waveTable + icnt2 + (phase + dPhase) + (osc2WaveForm<<8))*weight2)>>4);
			}
			else
			{
				osc2 += ((pgm_read_byte(waveTable + icnt2 + (osc2WaveForm<<8)))*weight2)>>4;
			}
			
			byte osc1 = ((pgm_read_byte(waveTable + icnt1 + (osc1WaveForm<<8))*weight1)>>4); //second oscillator
			
			OCR2A = ((osc1 + osc2)); //sum the two with oscillators
		}
		else
		{
			OCR2A = pgm_read_byte(waveTable + icnt1 + (osc1WaveForm<<8)) ^ pgm_read_byte(waveTable + icnt2 + (phase + dPhase) + (osc2WaveForm<<8));
		}
		
	}

	phaccu3 = phaccu3 + tword_mLFO;
	byte lfo = ((pgm_read_byte(waveTable + (phaccu3 >> 24) + (lfoWaveForm<<8))));

	dSemis = 0;
	dCents = 0;
	dOctave = 0;
	dPhase = 0;
	
	if(lfoWaveForm  != WAVE_FLAT)
	{
		switch(lfoRoute)
		{
			case ROUTE_NONE:
			break;

			case ROUTE_SEMI:
			dSemis = lfo - 128;
			break;

			case ROUTE_CENT:
			dCents = lfo - 128;
			break;

			case ROUTE_OCT:
			dOctave = lfo - 128;
			break;
			
			case ROUTE_PHASE:
			dPhase = lfo - 128;
			break;
		}
	}

	smallTimer++; //increment small timer because we don't have access to timer0

	if(smallTimer >= 31)
	{
		smallTimer = 0;
		millisecs++;
	}
}

void noteUpdate()
{	
	int dC = map(dCents,-128,128,-centsRange,centsRange);
	int dS = map(dSemis,-128,128,-semisRange,semisRange);
	int dO = map(dOctave,-128,128,-octaveRange,octaveRange);

	centMultiplier = pow(2.0,(cents + dC)/1200.0); //calculate the cents, multiplied to note frequency

	if(noteSelect < 0) //negative note values are bad
	{
		noteSelect = 0;
	}

	if(osc2WaveForm == WAVE_FLAT) //ignore the second osc
	{
		weight2 = 0;
	}

	weight1 = WEIGHT_MAX - weight2;

	tword_mOsc1=pow(2,32)*keyFreq[noteSelect]/refclk;  // calculate DDS new tuning word
	tword_mOsc2=pow(2,32)*(keyFreq[noteSelect + semis + dS]*pow(2,octaveShift + dO)*centMultiplier)/refclk; //cents affect osc2
	tword_mLFO=pow(2,32)*lfoFreq/refclk;
}

void savePreset(int preset)
{
	//values loaded to bank
	bank[preset - 1][0] = osc1WaveForm;
	bank[preset - 1][1] = osc2WaveForm;
	bank[preset - 1][2] = octaveShift;
	bank[preset - 1][3] = cents;
	bank[preset - 1][4] = weight2;
	bank[preset - 1][5] = int(lfoFreq*100);
	bank[preset - 1][6] = lfoWaveForm;
	bank[preset - 1][7] = lfoRoute;
	bank[preset - 1][8] = octaveRange;
	bank[preset - 1][9] = centsRange;
	bank[preset - 1][10] = semisRange;

	//all the arp values
	for(int i = 0;i < MAX_arp_NOTES;i++)
	{
		bank[preset - 1][11 + i] = arp[i];
	}

	bank[preset - 1][31] = arpMaxCount;
	bank[preset - 1][32] = arpSpeed;
	bank[preset - 1][33] = xorAddition;
	bank[preset - 1][34] = noteSync;
}

void loadPreset(int preset)
{
	//values pulled from bank
	osc1WaveForm = bank[preset - 1][0];
	osc2WaveForm = bank[preset - 1][1];
	octaveShift = bank[preset - 1][2];
	cents = bank[preset - 1][3];
	weight2 = bank[preset - 1][4];
	lfoFreq = bank[preset - 1][5]/100.0;
	lfoWaveForm = bank[preset - 1][6];
	lfoRoute = bank[preset - 1][7];
	octaveRange = bank[preset - 1][8];
	centsRange = bank[preset - 1][9];
	semisRange = bank[preset - 1][10];

	for(int i = 0;i < MAX_arp_NOTES;i++)
	{
		arp[i] = bank[preset - 1][11 + i];
	}

	arpMaxCount = bank[preset - 1][31];
	arpSpeed = bank[preset - 1][32];

	xorAddition = bank[preset - 1][33];
	noteSync = bank[preset - 1][34];

	settingsLocked[0] = true;
	settingsLocked[1] = true;
	settingsLocked[2] = true;
}

void normalSettingsLCD()
{
	osc1Update();
	osc2Update();
	bankUpdate();
	lockUpdate();
}

void lfoSettingsLCD()
{
	lfoUpdate();
	routeUpdate();
	noteSyncUpdate();
	lockUpdate();
}

void extraSettingsLCD()
{
	xorUpdate();
	legatoUpdate();
	arpUpdate();
	lockUpdate();
}

void osc1Update()
{
	String printStr = "OSC1: ";
	char buffer[13];
	
	switch(osc1WaveForm)
	{
		case WAVE_SINE:
		printStr += "SINE  ";
		break;
		
		case WAVE_TRI:
		printStr += "TRI   ";
		break;
		
		case WAVE_LSAW:
		printStr += "LSAW  ";
		break;
		
		case WAVE_RSAW:
		printStr += "RSAW  ";
		break;
		
		case WAVE_SQU:
		printStr += "SQUARE";
		break;
		
		case WAVE_FLAT:
		printStr += "FLAT  ";
		break;
		
		case WAVE_NOISE:
		printStr += "NOISE ";
		break;
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,0);
	LCDString(buffer);
}

void lfoUpdate()
{
	String printStr = "LFO: ";
	char buffer[13];
	
	switch(lfoWaveForm)
	{
		case WAVE_SINE:
		printStr += "SINE   ";
		break;
		
		case WAVE_TRI:
		printStr += "TRI    ";
		break;
		
		case WAVE_LSAW:
		printStr += "LSAW   ";
		break;
		
		case WAVE_RSAW:
		printStr += "RSAW   ";
		break;
		
		case WAVE_SQU:
		printStr += "SQUARE ";
		break;
		
		case WAVE_FLAT:
		printStr += "FLAT   ";
		break;
		
		case WAVE_NOISE:
		printStr += "NOISE  ";
		break;
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,0);
	LCDString(buffer);
}

void midiLearnUpdate()
{
	String printStr = "LFO: ";
	char buffer[13];
	
	switch(lfoWaveForm)
	{
		case WAVE_SINE:
		printStr += "SINE   ";
		break;
		
		case WAVE_TRI:
		printStr += "TRI    ";
		break;
		
		case WAVE_LSAW:
		printStr += "LSAW   ";
		break;
		
		case WAVE_RSAW:
		printStr += "RSAW   ";
		break;
		
		case WAVE_SQU:
		printStr += "SQUARE ";
		break;
		
		case WAVE_FLAT:
		printStr += "FLAT   ";
		break;
		
		case WAVE_NOISE:
		printStr += "NOISE  ";
		break;
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,0);
	LCDString(buffer);
}

void xorUpdate()
{
	String printStr = "XOR MOD: ";
	char buffer[13];
	
	if(!xorAddition)
	{
		printStr += "OFF";
	}
	else
	{
		printStr += "ON ";
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,0);
	LCDString(buffer);
}

void osc2Update()
{
	String printStr = "OSC2: ";
	char buffer[13];
	
	switch(osc2WaveForm)
	{
		case WAVE_SINE:
		printStr += "SINE  ";
		break;
		
		case WAVE_TRI:
		printStr += "TRI   ";
		break;
		
		case WAVE_LSAW:
		printStr += "LSAW  ";
		break;
		
		case WAVE_RSAW:
		printStr += "RSAW  ";
		break;
		
		case WAVE_SQU:
		printStr += "SQUARE";
		break;
		
		case WAVE_FLAT:
		printStr += "FLAT  ";
		break;
		
		case WAVE_NOISE:
		printStr += "NOISE ";
		break;
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,1);
	LCDString(buffer);
}

void routeUpdate()
{
	String printStr = "ROUTE: ";
	char buffer[13];
	
	switch(lfoRoute)
	{
		case ROUTE_CENT:
		printStr += "CENT ";
		break;
		
		case ROUTE_SEMI:
		printStr += "SEMI ";
		break;
		
		case ROUTE_OCT:
		printStr += "OCT  ";
		break;
		
		case ROUTE_PHASE:
		printStr += "PHASE";
		break;
		
		case ROUTE_NONE:
		printStr += "NONE ";
		break;
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,1);
	LCDString(buffer);
}

void legatoUpdate()
{
	String printStr = "LEGATO: ";
	char buffer[13];
	
	if(legato)
	{
		printStr += "ON ";
	}
	else
	{
		printStr += "OFF";
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,1);
	LCDString(buffer);
}

void bankUpdate()
{
	String printStr = "BANK: ";
	char buffer[13];
		
	printStr += bankSelect;
	printStr += "     ";
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,2);
	LCDString(buffer);
}

void arpUpdate()
{
	String printStr = "ARP: ";
	char buffer[13];
	
	if(arpMode)
	{
		printStr += "SAMPLE ";
	}
	else
	{
		printStr += "PLAYING";
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,2);
	LCDString(buffer);
}

void noteSyncUpdate()
{
	String printStr = "SYNC: ";
	char buffer[13];
	
	if(noteSync)
	{
		printStr += "ON    ";
	}
	else
	{
		printStr += "OFF   ";
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,2);
	LCDString(buffer);
}

void lockUpdate()
{
	String printStr = "LOCK: ";
	char buffer[13];
	
	if(settingsLocked[settingsMenu])
	{
		printStr += "ON    ";
	}
	else
	{
		printStr += "OFF   ";
	}
	
	printStr.toCharArray(buffer, 13);
	
	gotoXY(0,3);
	LCDString(buffer);
}

void clearLCD()
{
	LCDClear();
}

void gotoXY(int x, int y) {
	LCDWrite(0, 0x80 | x);  // Column.
	LCDWrite(0, 0x40 | y);  // Row.  ?
}

//This takes a large array of bits and sends them to the LCD
void LCDBitmap(char my_array[]){
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	LCDWrite(LCD_DATA, my_array[index]);
}

//This function takes in a character, looks it up in the font table/array
//And writes it to the screen
//Each character is 8 bits tall and 5 bits wide. We pad one blank column of
//pixels on each side of the character for readability.
void LCDCharacter(char character) {
	LCDWrite(LCD_DATA, 0x00); //Blank vertical line padding

	for (int index = 0 ; index < 5 ; index++)
	LCDWrite(LCD_DATA, ASCII[character - 0x20][index]);
	//0x20 is the ASCII character for Space (' '). The font table starts with this character

	LCDWrite(LCD_DATA, 0x00); //Blank vertical line padding
}

//Given a string of characters, one by one is passed to the LCD
void LCDString(char *characters) {
	while (*characters)
	LCDCharacter(*characters++);
}

//Clears the LCD by writing zeros to the entire screen
void LCDClear(void) {
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	LCDWrite(LCD_DATA, 0x00);
	
	gotoXY(0, 0); //After we clear the display, return to the home position
}

//This sends the magical commands to the PCD8544
void LCDInit(void) {

	//Configure control pins
	pinMode(PIN_SCE, OUTPUT);
	pinMode(PIN_RESET, OUTPUT);
	pinMode(PIN_DC, OUTPUT);
	pinMode(PIN_SDIN, OUTPUT);
	pinMode(PIN_SCLK, OUTPUT);

	//Reset the LCD to a known state
	digitalWrite(PIN_RESET, LOW);
	digitalWrite(PIN_RESET, HIGH);

	LCDWrite(LCD_COMMAND, 0x21); //Tell LCD that extended commands follow
	LCDWrite(LCD_COMMAND, 0xA0); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
	LCDWrite(LCD_COMMAND, 0x04); //Set Temp coefficient
	LCDWrite(LCD_COMMAND, 0x14); //LCD bias mode 1:48: Try 0x13 or 0x14

	LCDWrite(LCD_COMMAND, 0x20); //We must send 0x20 before modifying the display control mode
	LCDWrite(LCD_COMMAND, 0x0C); //Set display control, normal mode. 0x0D for inverse
}

//There are two memory banks in the LCD, data/RAM and commands. This
//function sets the DC pin high or low depending, and then sends
//the data byte
void LCDWrite(byte data_or_command, byte data) {
	digitalWrite(PIN_DC, data_or_command); //Tell the LCD that we are writing either to data or a command

	//Send the data
	digitalWrite(PIN_SCE, LOW);
	shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
	digitalWrite(PIN_SCE, HIGH);
}
