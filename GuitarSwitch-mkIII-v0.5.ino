/*  Programmable Arduino based stompbox looper / switching system with Midi output
 *  Programmed by Patrick Klaassen (patrick.klaassen@gmail.com)
 *  Photo's on Google Photos : https://goo.gl/photos/JYGqpnPPz51WmjSG7
 *  Short introduction video on Youtube: https://youtu.be/dVaR2lphJOY
 *  
 *  Project was based on the projects by CarraN and Pascal Paquay
 *  
 *  I removed about 70% of the original code and added a lot of new stuff
 *  
 * 
 * Version 0.1  uses a 3 way switch to determine the mode
 * Version 0.2  Removed 3 way switch mode selection and replaced it with momentary mode switch to switch between looper and preset mode
 * Version 0.3  uses a single output for the looper leds LED and relays
 * Version 0.5  cleaned up code and restored memory 8
 * Version 0.6  removal of relay leds, moved indicators to bottom row of LCD, saves drilling holes in enclosure
 * Version 0.7  introduction of amp channel switching TRS tip ring sleeve for controlling Bugera V20 tube amp
 * Version 0.8  adding bank up and down buttons
 * Version 0.9  exchanged row and col in the keymap
 * Version 0.10 Improved bank handling
 * Version 0.11 DJ - added support for i2c looper and preset led control
 *              Added support for 4x20 i2c lcd display
 *              Added support for dedicated loop buttons
 * Version 0.12 Removed mode indicator LED's
 *              Added midi support for EVH 5150III, Flashback X4 and Strymon BigSky, using a configurable mechanism to easily support 
 *              new pedals when added to the pedal board
 *              Tried to use semi configurable mechanism for midi support using array's to store all the midi information for each device
 * Version 0.12 Added SwitchOrder to solve double audio boost issue with boost pedal and crunch channel activated during switch from crunch mode to clean mode
 * Version 0.13 Improved switch order mode
 * Version 0.14 Cleaned up the code
 * 
 * mkIII starts here
 * Version 0.1  Introducing the new OLED display, it uses the spi interface 
 * Version 0.2  Fixed the order of preset and loop keys
 * Version 0.3  Inverted loop relays (new relay are inverted)
 * Version 0.4  Improved bank handling 
 *              Improved switching times by modded display handling
 * Version 0.5  Cleaned up old loop LED based code
 * 
 * Functions and subroutines
 * setup()
 * initOLED()
 * showLogo()
 * writeLine(int intRow, String strText, int intFont)
 * clearRow(int pixRow)
 * writeStatus()
 * memoryDump()
 * initLEDs()
 * midiProg(byte status, int data)
 * setLCDChannel()
 * setLCDAmpSettings()
 * setSavePresetState(int led)
 * memory(int addr, int led)
 * checkSaveState(int led, char key)
 * writeOut(int relay)
 * mute()
 * readPreset(int addr, int pcNum, int led)
 * switchLoops(int memValue)
 * writeMidi(int addr) 
 * getAddress(int channel)
 * getAmpSetting(int ampChannel)
 * handleLoopKeyEvent(int channel)
 * handlePresetKeyEvent(int channel)
 * changeDeviceMode(int mode)
 * showLCDBankMode()
 * handleAmpBankEvent()
 * keypadEvent()
 * showTimer()
 * loop()
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Keypad.h>

// Set OLED properties
U8G2_SSD1327_MIDAS_128X128_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 11, /* data=*/ 12, /* cs=*/ 13, /* dc=*/ 49, /* reset=*/ 48);

#define Switch8_logo_128_width 128
#define Switch8_logo_128_height 28
static unsigned char Switch8_logo_128_bits[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xf8, 0x03, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xc0,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0x01, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xfe, 0x01, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x03, 0x00, 0xe0,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xff, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00, 0x80, 0xed, 0x5d, 0xdc,
   0xd8, 0xe3, 0xc6, 0x9e, 0x7f, 0xee, 0xbb, 0xf9, 0xdf, 0x8d, 0xfb, 0x6e,
   0xc0, 0xff, 0xff, 0xfe, 0xfc, 0xf3, 0xef, 0x9f, 0x3f, 0xff, 0xff, 0xf9,
   0xff, 0xcf, 0xff, 0x7f, 0xc0, 0x9f, 0x7f, 0xff, 0xfc, 0xf9, 0xe7, 0xcf,
   0xbf, 0x7f, 0xfe, 0xfd, 0xf3, 0xef, 0x9f, 0x7f, 0xe0, 0x9f, 0x3f, 0x7f,
   0xfe, 0xf9, 0xf3, 0xef, 0x9f, 0x3f, 0xff, 0xfc, 0xf9, 0xe7, 0xcf, 0x3f,
   0xf0, 0x8f, 0x9a, 0x7f, 0xff, 0xfc, 0xfb, 0xe7, 0xdf, 0x3f, 0x6a, 0xfe,
   0xf9, 0xe3, 0xdf, 0x0f, 0xf0, 0x2f, 0x81, 0x3f, 0xff, 0xfe, 0xf9, 0xf7,
   0xcf, 0x9f, 0x04, 0xff, 0xfc, 0xe3, 0xff, 0x0f, 0x50, 0xfd, 0x9f, 0x9f,
   0x7f, 0xfe, 0xfc, 0xf3, 0xe7, 0x9f, 0x3f, 0xff, 0xfe, 0xf9, 0xe7, 0x0f,
   0x08, 0xf0, 0x8f, 0x9f, 0x3f, 0xff, 0xfc, 0xf9, 0xf7, 0xcf, 0xbf, 0x7f,
   0xfe, 0xfd, 0xf3, 0x0f, 0xfc, 0xf3, 0x07, 0xcf, 0x3f, 0x7f, 0xfe, 0xf9,
   0xf3, 0xef, 0x9f, 0x3f, 0xff, 0xfc, 0xf9, 0x07, 0xfc, 0xf9, 0x07, 0xee,
   0x9f, 0x7f, 0xff, 0xfc, 0xfb, 0xe7, 0xcf, 0x3f, 0x7f, 0xfe, 0xf9, 0x03,
   0xfe, 0xfd, 0x03, 0xf6, 0xdf, 0x3f, 0xff, 0xfe, 0xf9, 0xf7, 0xef, 0x9f,
   0x7f, 0xfe, 0xfd, 0x03, 0xfe, 0xff, 0x01, 0xfc, 0xff, 0x1f, 0x7f, 0xfe,
   0xfc, 0xff, 0xe7, 0x9f, 0x3f, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };

const int intRowH = 14;
const int lFont = 1;
const int mFont = 2;
int i=0;
String strLine1 = "";
String strLine2 = "";
String strLine3 = "";
String strLine4 = "";
String strLine5 = "";
String strLine6 = "";
String strLine7 = "";
String strLine8 = "";
String strLine9 = "";

// Set LCD defaults
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Set I2C IO addresses
#define IO_ADDR_Presets 0x20
#define IO_ADDR_Loops 0x39

const String strVersion="0.1";

/* LiquidCrystal_I2C  lcd(0x27,16,2); 
 * set the LCD address to 0x27 for a 16 chars and 2 line display. 
any other display can be used,just change parameters. */
// Define matrix rows and cols
const byte rows = 2; 
const byte cols = 12; /*change it the same value as numberOfPedal variable */
char keys[rows][cols] = {
{'a','b','c','d','e','f','g','h','i','j','k','l'}, // first row contains presets followed by functions
{'m','n','o','p','q','r','s','t','u','v','w','x'}  // second row contains loops u v w x are not used
// Mapping of matrix values
// a - preset/channel 1
// b - preset/channel 2
// c - preset/channel 3
// d - preset/channel 4
// e - preset/channel 5
// f - preset/channel 6
// g - preset/channel 7
// h - preset/channel 8
// i - Mode switch
// j - Mute on/off
// k - Bank up/midi (In older version also used for amp channel swicthing)
// l - Bank down (in older version also used for amp reverb switching)
// m - loop 1
// n - loop 2
// o - loop 3
// p - loop 4
// q - loop 5
// r - loop 6
// s - loop 7
// t - loop 8
};

int intPrevMem = 0; // Stores previous preset pickup, used to check boost pedal for determining the order of switching Midi first or pedals first
// Mode constants, used for switching modes
const int PRESETMODE = 0;
const int PROGRAMMODE = 1;
const int STOREMODE = 2;
const int BANKMODE = 3;
const int MIDIMODE = 4;
const int ORDERMODE = 5; // determines the order in with the pedals and midi devices are switched
const int AMPMODE = 6; // for switching channel and reverb setting

// Var for storing mute mode, 0 for not active, 1 for active and 2 for tilt ;-) 
int muteMode = 0;

// midi settings
const int MIDIAMP = 0; // Position of amp in midi arrays
const int MIDIFB = 1;  // Position of Flashback in midi arrays
const int MIDIBS = 2;  // Position of Bigsky in midi arrays
const int MIDIMB = 3;  // Position og Mobius in midi arrays
String strMidiDevices[4] = {"EVH Amp             ","Flashback           ","BigSky              ","Mobius              "}; // Name tage used when programming midi settings for the devices
String strMidiDevicesShort[4] = {"EVH","FB","BS","MB"};  // Short names used in preset mode to display the presets midi device value
const int intMidiMaxValue[4] = {2,2,127,127};  // Maximum value per midi device
const int intMidiDeviceChannels[4] = {2,0,1,3};  // Midi address (channel) for each device
int intMidiValues[4]= {0,0,0,0};  // Current midi values per device
int intCurMidi = 0;
int intCurMidiValue = 0;
// Determines the order for switching midi or loops first
int intCurSwitchOrderValue = 0; // 0 is loop then midi, 1 is midi then loop

// Arduino I/O pins used for the button marix
byte rowPins[rows] = {30,31}; //  Org mega 22, 23 22 preset+controls, 23 loop
byte colPins[cols] = {41,40,37,36,35,34,33,32,44,45,46,47}; //   32,33,34,35,36,37,40,41,   44,45,46,47 old Mega numbers {2,3,4,5,6,7,8,9,10,11,12,13};  //1,2,3,4,5,6,7,8,mode,mute,up/reveb,down/channel
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
// I/O ports used for the loop relays
int relayPin[8] = {29,39,27,28,25,26,23,24}; //{24,23,26,25,28,27,39,29}; // Original Mega {32,31,30,29,28,27,26,25};
char* relayName[8] = {"NEO Lesley", "EQ", "Black Star", "RC Booster", "Corona", "Flashback", "FX 7" , "FX 8"}; // names of the loop pedals, no longer used in LCD
// Names of the presets, presented in the LCD when activated
char* presetTextA[8] = {"1 Clean           ", "2 Clean FX       ", "3 Crunch Rhythm    ", "4 Crunch lead      ", "5 Lead delay       ", "6 Lead dry         ", "7                    " , "8                    "};
char* presetTextB[8] = {"Preset 1", "Preset 2", "Preset 3", "Preset 4", "Preset 5", "Preset 6", "Preset 7" , "Preset 8"};
char* presetTextC[8] = {"Preset 1", "Preset 2", "Preset 3", "Preset 4", "Preset 5", "Preset 6", "Preset 7" , "Preset 8"};

// Variables for storing temp values of LED's 
int intPresetLEDs=0;
int intPresetBlinkingLEDs=0;
int intLoopLEDs=0;
int intPresetLEDsPrev=0;
int intLoopLEDsPrev=0;

// I/O ports used for addiotional relays
int muteRelay = 52;  // Mute relay for switching the input to tuner and the relay for output to ground
int mmLED = 9; // Mute Mode LED, Mute flashing, preset mode solid

int ampReverbRelay = 51;
int ampGainRelay = 50;
// Not implemented reading of current values from the amp controls
int ampReverbPin = A0;
int ampGainPin = A1;

int ampReverbValue = HIGH;
int ampGainValue = LOW;

int numberOfPedal = 8; /*adapt this number to your needs = number of loop pedals */
int saveState = 0; // 0 - no save active, 1 - waiting for save, 2 - processed save
int deviceMode = PRESETMODE; // 0 = preset mode, 1 = looper mode, 2 = store preset mode, 3 = change bank mode
int previousDeviceMode = PRESETMODE;
int holdProcessed=0; // used for detecting hold state of mode switch (for switching to store mode)
int currentPreset = -1; // Store the current preset number, used when switching back to preset mode
int currentBank = 0; // allowed values 0, 100 or 200 for bank a, b and c, depending on the Arduino you use you can extend the numer of banks 
// The address space is as follows
// 10 contains the settings for the first preset in bank a
// 20 contains the settings for the second pedal up to 80 for the eighth pedal in bank a
// 110 contains the settings for the first preset in bank b up to 180 for the eighth in bank b
// 210 contains the settings for the first preset in bank c up to 180 for the eighth in bank c
// You can extend the banks
int newBank = 0; //Used for changing banks till setting is confirmed
unsigned long previousMillis = 0; // timer is used for blinking led's when saving preset
unsigned long previousReverbMillis = 0; // timer is used for disabling Reverb relay (the Bugera amp uses a mementary swicth to switch reverd)
unsigned long previousGainMillis = 0; // timer is used for disabling Gain relay (the Bugera amp uses a mementary swicth to switch between channels)
unsigned long bankModeMillis = 0; // timer is used for disabling bank mode after max duration
unsigned long bankModeDurationMillis = 2000; // after this duration the bankmode is automatically disabled

const long interval = 250; // interval at which the leds will blink
int previousRelayState[8]={LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW}; 
int previousButtonLEDState[8]={LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};

unsigned long startMillis = 0; // timer is used for calculating event length, used for perfomance testing 

boolean debug = true; // When true the serial monitor is used to communicate various values and states

/******************************************************/

char* presetText()
{
    if (currentBank==0) { return presetTextA[currentPreset]; }
    if (currentBank==100) { return presetTextB[currentPreset]; }
    if (currentBank==200) { return presetTextC[currentPreset]; }
    return "Empty";
}

void setup()
{
  u8g2.begin();
  Wire.begin();
  showLogo();
  // initOLED();  

  Serial.begin(9600);
  pinMode(13, INPUT);
  //  Set MIDI baud rate:
  Serial1.begin(31250);
  
  pinMode(muteRelay, OUTPUT);
  digitalWrite(muteRelay,LOW); // set Mute mode all sound is muted till setuyp is finished
  // Set mode/mute led output
  pinMode(mmLED,OUTPUT);
  digitalWrite(mmLED,HIGH);

  //pinMode(tuneRelay, OUTPUT);
  //digitalWrite(tuneRelay,LOW); // Removed from last build of the Switch8

//  //Serial.begin(9600); /* for midi communication - pin 1 TX  WAS  Serial.begin(31250);*/
//  lcd.begin (20,4); // for 20 x 4 LCD module
//  lcd.setBacklightPin(3,POSITIVE); 
//  lcd.setBacklight(HIGH);
//  lcd.setCursor(0,0);
//  lcd.print("DJ's - Switch8  DJ-8");
//  lcd.setCursor(0,1);
//  lcd.print("Guitar pedal looper ");

  // Set amp in default mode
  pinMode(ampReverbRelay, OUTPUT);
  digitalWrite(ampReverbRelay,HIGH);
  pinMode(ampGainRelay, OUTPUT);
  digitalWrite(ampGainRelay,HIGH);

  // Set pin modes for relays and set default value to bypass all pedals
  for(i=0; i<numberOfPedal; i++)
  {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i],HIGH); //pullup all relay outputs in case off low level relayboard
  }
  initLEDs();
  delay(30);
  keypad.addEventListener(keypadEvent); //add an event listener for this keypad, meaning all the buttons on the Switch8
  keypad.setHoldTime(2000);
  readPreset(10, 1, 0); // Load default preset number 1 from bank a
  mute();
//  ampReverbValue = getAmpSetting(ampReverbPin);
//  ampGainValue = getAmpSetting(ampGainPin);
  
  if (debug) Serial.println("Init done");
  writeLine(3,"Mode: Preset 8", lFont);
  u8g2.sendBuffer();
  strLine3 = "Mode: Mute";
  setLCDChannel();
  refreshOled();
  
}

void initOLED(){
  if (debug) Serial.println("Clear Row");
//  u8g2.clear();
//  // u8g2.setFont(u8g2_font_6x13_tf);
//  writeLine(1,"Switch8 XL", lFont);
//  delay(250);
//  writeLine(3,"Initilizing", mFont);  
//  delay(250);
//  writeLine(4,"Checking relays", mFont); 
//  delay(250);
//  writeLine(5,"Checking switch matrix", mFont); 
//  delay(250);
//  writeLine(6,"Checking big hair", mFont); 
//  delay(250);
//  writeLine(7,"Guitar in tune?!", mFont); 
//  delay(1000);
//  writeLine(8,"Sure......", mFont); 
//  delay(1000);
//  for(i=1; i<8; i++)
//  {
//    clearRow(i);  
//  }
//  writeLine(3,"Mode: Preset 8", lFont); 
//  writeLine(5,"Loops: 1 34 6", lFont); 
//  writeLine(6,"Amp settings", lFont); 
//  writeLine(7,"Reverb off", lFont); 
//  writeLine(8,"Channel Clean", lFont); 
//  writeLine(9,"No midi devices", lFont); 
}

void showLogo(){
  u8g2.clear();
  u8g2.drawXBM( 0, (64-14), Switch8_logo_128_width, Switch8_logo_128_height, Switch8_logo_128_bits);
  u8g2.sendBuffer();
}

void writeLine(int intRow, String strText, int intFont){
  clearRow(intRow);
  switch (intFont)
  {
    case mFont:
      u8g2.setFont(u8g2_font_helvR08_tf);
      break;
    case lFont:
      u8g2.setFont(u8g2_font_t0_17_tf); //u8g2_font_7x14_tf); //u8g2_font_6x13_tf);
      break;
  }
  u8g2.setCursor(0,(intRowH*intRow));
  u8g2.print(strText); 
}

void writeLineFullDisplay(int intRow, String strText, int intFont){
  switch (intFont)
  {
    case mFont:
      u8g2.setFont(u8g2_font_helvR08_tf);
      break;
    case lFont:
      u8g2.setFont(u8g2_font_t0_17_tf); //u8g2_font_7x14_tf); //u8g2_font_6x13_tf);
      break;
  }
  u8g2.setCursor(0,(intRowH*intRow));
  u8g2.print(strText); 
}



void clearRow(int pixRow) {
  if (debug) Serial.println("Clear Row");
//  u8g2.setDrawColor(0);
//  u8g2.drawBox(0,(pixRow*intRowH),128,intRowH);
//  u8g2.sendBuffer();
//  u8g2.setDrawColor(1);
//  u8g2.sendBuffer();
}

void writeStatus(){
  u8g2.clearBuffer();
  // u8g2.setFont(u8g2_font_6x13_tf);
  writeLine(1,"Switch8 XL", lFont);
  for(i=1; i<8; i++)
  {
    clearRow(i);  
  }
  writeLine(3,"Mode: Preset 8", lFont);
  writeLine(4,"Bank A", lFont);
  writeLine(5,"Loops: 1 34 6", lFont); 
  writeLine(6,"Amp settings", lFont); 
  writeLine(7,"Reverb off", lFont); 
  writeLine(8,"Channel Clean", lFont); 
  writeLine(9,"No midi devices", lFont); 
  u8g2.sendBuffer();
}

void refreshOled() {
  u8g2.clearBuffer();
  // u8g2.setFont(u8g2_font_6x13_tf);
  writeLineFullDisplay(1,strLine1, lFont);
  writeLineFullDisplay(2,strLine2, mFont);
  writeLineFullDisplay(3,strLine3, mFont);
  writeLineFullDisplay(4,strLine4, mFont);
  writeLineFullDisplay(5,strLine5, lFont); 
  writeLineFullDisplay(6,strLine6, lFont); 
  writeLineFullDisplay(7,strLine7, lFont); 
  writeLineFullDisplay(8,strLine8, lFont); 
  writeLineFullDisplay(9,strLine9, lFont); 
  u8g2.sendBuffer();

}

/*********************************************************/

void memoryDump()
{
  // Create dump of memory and write this to the console
  // Create looper to loop through the memory space from address 0 to 300
  int intMemory;
  if (debug) Serial.println("Bank A");
  for(i=0; i<99; i++)
  {
    // Read value as int from memory, value can be between 0-255
    intMemory = EEPROM.read(i);
    // Write memory to console
    if (debug) Serial.print("addr: ");
    if (debug) Serial.print(i);
    if (debug) Serial.print(" - ");
    if (debug) Serial.println(intMemory);
  }
  if (debug) Serial.println("Bank B");
  for(i=100; i<199; i++)
  {
    // Read value as int from memory, value can be between 0-255
    intMemory = EEPROM.read(i);
    // Write memory to console
    if (debug) Serial.print("addr: ");
    if (debug) Serial.print(i);
    if (debug) Serial.print(" - ");
    if (debug) Serial.println(intMemory);
  }
  if (debug) Serial.println("Bank C");
  for(i=200; i<299; i++)
  {
    // Read value as int from memory, value can be between 0-255
    intMemory = EEPROM.read(i);
    // Write memory to console
    if (debug) Serial.print("addr: ");
    if (debug) Serial.print(i);
    if (debug) Serial.print(" - ");
    if (debug) Serial.println(intMemory);
  }
  // done
}


void initLEDs()
{
  // initLEDs tests all LED's so the user can see if all is working (and it looks nice)
  static unsigned char data = 0x01;  // data to display on LEDs
  static unsigned char direc = 1;    // direction of knight rider display
  int x = 0;
  for (x=0; x<2;x++)
  {
    for(i=0; i<16; i++)
    {
      // send the data to the LEDs
      Wire.beginTransmission(IO_ADDR_Presets);
      Wire.write(~data);
      Wire.endTransmission();
      Wire.beginTransmission(IO_ADDR_Loops);
      Wire.write(~data);
      Wire.endTransmission();
      delay(40);  // speed of display
      // shift the on LED in the specified direction
      if (direc) {
        data <<= 1;
      }
      else {
        data >>= 1;
      }
      // see if a direction change is needed
      if (data == 0x80) {
        direc = 0;
      }
      if (data == 0x01) {
        direc = 1;
      }
    }
  }
  // LEDs of
  Wire.beginTransmission(IO_ADDR_Presets);
  Wire.write(~0);
  Wire.endTransmission();
  Wire.beginTransmission(IO_ADDR_Loops);
  Wire.write(~0);
  Wire.endTransmission();
}

/*********************************************************/

//  Send a two byte midi message  
void midiProg(char status, int data ) {
  // This routine is used to send the midi program change commands to the midi devices
  if (debug) Serial.println("Write midi data");
  Serial1.write(status);
  Serial1.write(data);
}

/*********************************************************/

/********************************************************/

void setLCDChannel()
{
  String loop1 = "_";
  String loop2 = "_";
  String loop3 = "_";
  String loop4 = "_";
  String loop5 = "_";
  String loop6 = "_";
  String loop7 = "_";
  String loop8 = "_";
  // Dislays the channel status on the LCD, channels are indicated by their numbers, function now controls I2C led's
  // Example __3_567_
  if (debug) Serial.println("setLCDChannel");
  int intVal = LOW;
  int intPos = 2;
  intLoopLEDs =0;
  for(i=0; i<numberOfPedal; i++)
  {
    intPos = intPos+1;
    //if (debug) {Serial.print("Pos ");Serial.println(intPos);} 
    intVal = !digitalRead(relayPin[i]);
    if (i==0&&intVal==1) {loop8="1";}
    if (i==1&&intVal==1) {loop7="2";}
    if (i==2&&intVal==1) {loop6="3";}
    if (i==3&&intVal==1) {loop5="4";}
    if (i==4&&intVal==1) {loop4="5";}
    if (i==5&&intVal==1) {loop3="6";}
    if (i==6&&intVal==1) {loop2="7";}
    if (i==7&&intVal==1) {loop1="8";}
    int intAdd = intVal<<(i);
    intLoopLEDs = intLoopLEDs + intAdd;
  }
  strLine2 = "Loops " + loop1 + loop2 + loop3 + loop4 + loop5 + loop6 + loop7 + loop8;
  // if (debug) {Serial.print("Write value to lcd "); Serial.println(intLoopLEDs);}
  // Write value to loop I2C extension board
  Wire.beginTransmission(IO_ADDR_Loops);
  Wire.write(~intLoopLEDs);
  Wire.endTransmission();

//  setLCDAmpSettings();
  showLCDBankMode();
}

void setLCDAmpSettings()
{
  // Dislays the Amp reverb and channel status on the LCD, in last version no longer used
  // Excemple __3_567_ RC
  clearRow(7); 
  if (ampReverbValue == HIGH)
  {
    strLine7 = "Reverb on";
    // writeLine(7,"Reverb on", lFont); 
  }
  else
  {
    strLine7 = "Reverb off";
    //writeLine(7,"Reverb off", lFont); 
  }
  
  clearRow(8); 
  if (ampGainValue == HIGH)
  {
    strLine8 = "Channel Clean";
    // writeLine(8,"Channel Clean", lFont);
  }
  else
  {
    strLine8 = "Channel Dist";
    //writeLine(8,"Channel Dist.", lFont);
  }
  
}

void setSavePresetState(int led)
{
  // sub is launched when holding the mode button to store a preset
  if (debug) Serial.println("Store preset key detected");
  // Write value to presets IO
  intPresetLEDs = 1<<led;
  Wire.beginTransmission(IO_ADDR_Presets);
  Wire.write(~intPresetLEDs);
  Wire.endTransmission();
  
  saveState = 1;
  if (debug) Serial.println("Set save state = 1");
}

void memory(int addr, int led)
{
  // Memory layout for each preset is the same and as follows:
  // 0 is loop settings 0-256 0x00000000 to 0x11111111 each bit represents a guitar pedal loop
  // For the midi devices only the program change code is stored. In the current memory model we have space for 8 midi devices
  // At the moment I only use three
  // 1 midi amp 0-2
  // 2 midi fb 0-2
  // 3 midi bs 0-127
  // 4 midi mb 0-127
  // 5-8 midi reserved
  // 9 switch order  value, this makes it possible to control the order of the switching of the midi or loops
  // 0-90 contain the preset info for the eight presets in bank A/0 (0 preset 1, 10 preset 2, 20 preset 3 etc.)
  // 100-190 contain the preset info for the eight presets in bank B/1 (100 preset 1, 110 preset 2 etc.)
  // 200-290 contain the preset info for the eight presets in bank C/2
  // Depending on the Arduino used you can add banks till you run out of memory
  
  if (debug) Serial.println("Store setting");
  EEPROM.write((addr), intLoopLEDs);

  // Store Midi settings, get the values from the midi values array for each of the devices
  EEPROM.write((addr) + 1 + MIDIAMP, intMidiValues[MIDIAMP]);
  EEPROM.write((addr) + 1 + MIDIFB, intMidiValues[MIDIFB]);
  EEPROM.write((addr) + 1 + MIDIBS, intMidiValues[MIDIBS]);
  EEPROM.write((addr) + 1 + MIDIMB, intMidiValues[MIDIMB]);
  // Reserve room for additional midi devices
  
  // Store switch order value
  EEPROM.write((addr) + 9, intCurSwitchOrderValue);

  // store amp settings, in the last version of the hardware this is not incorporated
//  EEPROM.write((addr) + 8, ampReverbValue);
//  EEPROM.write((addr) + 9, ampGainValue);

  // Update oled
  clearRow(3);
  String strText = "Preset" + String(led + 1) +" stored";
  strLine3 = strText;
  //writeLine(3,strText , mFont);
  // Stop blinking preset leds
  intPresetLEDs = 0;  
  Wire.beginTransmission(IO_ADDR_Presets);
  Wire.write(~intPresetLEDs);
  Wire.endTransmission();
  // Show selected preset led
  intPresetLEDs = 1<<led;
  Wire.beginTransmission(IO_ADDR_Presets);
  Wire.write(~intPresetLEDs);
  Wire.endTransmission();
  
  if (debug) Serial.println("Preset stored");
  saveState = 2;  
  delay(200);
  if (previousDeviceMode==PRESETMODE) {
    changeDeviceMode(PRESETMODE);
  } else {
    changeDeviceMode(PROGRAMMODE);
  }
}

void checkSaveState(int led, char key)
{
  // Check if save state should be cancelled
  if (debug) Serial.println("Check save state");
  if (saveState==1)
  {
    if (debug) Serial.println("Canceling save state");
    // Disable blinking preset LED's
    intPresetLEDs = 0;
    for(i=0; i<numberOfPedal; i++)
    {
      intPresetLEDs  = intPresetLEDs  + !digitalRead(relayPin[i])* 1<<(i+1); 
    }
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
  }
  saveState = 0;  
}

/*********************************************************/

void writeOut(int relay) 
{
  // Toggle single loop value
  int intChannelVal = digitalRead(relayPin[relay]);
  digitalWrite(relayPin[relay], !digitalRead(relayPin[relay]));
  if (debug) Serial.println("Old loop value " + String(relay) + ": " + String(intChannelVal));
}

void mute() 
{
  // Set mute for tuning and pauze
  if (muteMode==0)
  {
    // Switch Mute relay
    digitalWrite(muteRelay,LOW); 
    // digitalWrite(tuneRelay,LOW);
    // Get current state of Relays and button LEDs
    for(i=0; i<numberOfPedal; i++)
    {
      previousRelayState[i] = !digitalRead(relayPin[i]);
      previousButtonLEDState[i] = !previousRelayState[i]; //digitalRead(ledPin[i]);
    }
//    for(i=0; i<numberOfPedal; i++)
//    {
//      digitalWrite(relayPin[i], HIGH);
//    }
    muteMode = 1;
    clearRow(3); 
    strLine3 = "Mode: Mute        ";
    //writeLine(3,"Mode: Mute        ", mFont);
    digitalWrite(mmLED,LOW); 
  }
  else
  {
    if (muteMode==2)
    {
      intPresetLEDs = intPresetLEDsPrev;
      intLoopLEDs = intLoopLEDsPrev;      
    }
    // reset previous state of Relays and button LEDs
    for(i=0; i<numberOfPedal; i++)
    {
      digitalWrite(relayPin[i], !previousRelayState[i]);
    }
    delay(10);
    // Disable mute relay
    // digitalWrite(tuneRelay,HIGH);
    digitalWrite(muteRelay,HIGH); 
    if (deviceMode==PRESETMODE) // Preset mode
    {
      clearRow(3); 
      strLine3 = "Mode: Preset";
      //writeLine(3,"Mode: Preset", mFont); 
      clearRow(4); 
      strLine4 = presetText();
      //writeLine(4,presetText(), mFont);
      // Set PresetMode LED
      digitalWrite(mmLED,HIGH);
    }
    else // looper mode
    {
      clearRow(3); 
      strLine3 = "Mode: Program";
      digitalWrite(mmLED,LOW);
      //writeLine(3,"Mode: Program", mFont); 
    }
    muteMode = 0;
    setLCDChannel();
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Loops);
    Wire.write(~intLoopLEDs);
    Wire.endTransmission();
  }
  refreshOled();
}

/*********************************************************/
void readPreset(int addr, int pcNum, int led)
// Reads preset from memory and changes the used loops and midi devices
{
  if (currentPreset == led and muteMode==0) 
  {
    //return;
  }
  if (debug) Serial.println("readPreset");

  // Get switch order value first
  intCurSwitchOrderValue = EEPROM.read(addr+9);
  // Read value as int from memory, value can be between 0-255
  int intMemory = EEPROM.read(addr);

  switchLoops(intMemory);

  // Depending on this value the order of switching the loops and midi devices is reversed
//  if (intCurSwitchOrderValue==0)
//  {
//    // Switch Loops, then midi
//    switchLoops(intMemory);
//    writeMidi(addr);
//  } else
//  {
//    // Switch midi then loops
//    writeMidi(addr);
//    switchLoops(intMemory);
//  }
  unsigned long currentMillis = 0;
  // Write midi values to lcd
  if (debug) Serial.println("write midi");
  // Write midi settings to LCD
//  lcd.setCursor(0,3);
//  lcd.print("                    ");
//  lcd.setCursor(0,3);
//  lcd.print(strMidiDevicesShort[MIDIAMP]);
//  lcd.print(intMidiValues[MIDIAMP]);
//  lcd.setCursor(5,3);
//  lcd.print(strMidiDevicesShort[MIDIFB]);
//  lcd.print(intMidiValues[MIDIFB]);
//  lcd.setCursor(9,3);
//  lcd.print(strMidiDevicesShort[MIDIBS]);
//  lcd.print(intMidiValues[MIDIBS]);
//  lcd.setCursor(15,3);
//  lcd.print(strMidiDevicesShort[MIDIMB]);
//  lcd.print(intMidiValues[MIDIMB]);

  // Amp switching is no longer supported  TODO
//  intRelayVal = EEPROM.read((addr)+8); // Get Reverb value
//  if (ampReverbValue!=intRelayVal)
//  {
//    digitalWrite(ampReverbRelay, LOW);
//    currentMillis = millis();
//    ampReverbValue = !ampReverbValue;
//    previousReverbMillis = currentMillis;
//  }
//  intRelayVal = EEPROM.read((addr)+9); // Get Reverb value
//  if (ampGainValue!=intRelayVal)
//  {
//    digitalWrite(ampGainRelay, LOW);
//    currentMillis = millis();
//    ampGainValue = !ampGainValue;
//    previousGainMillis = currentMillis;
//  }
  if (debug) Serial.println("Write preset LED");
  if (debug) Serial.println(led);
  intPresetLEDs = 1<<led;
  if (debug) Serial.println(intPresetLEDs);
  // Write value to presets IO
  Wire.beginTransmission(IO_ADDR_Presets);
  Wire.write(~intPresetLEDs);
  Wire.endTransmission();

  setLCDChannel();
  currentPreset = led;
  clearRow(4); 
  strLine4 = presetText();
  //writeLine(4,presetText(), mFont);
  intPrevMem = intMemory;
}

void switchLoops(int memValue) 
//Switches the loops using the memValue
{
  // Determine value for each loop
  for(i=0; i<numberOfPedal; i++)
  {
    if ((memValue & 1<<(i))!=LOW){
      digitalWrite(relayPin[i], LOW);
    } else {
      digitalWrite(relayPin[i], HIGH);
    }
  }
}

void writeMidi(int addr) 
// Get midi settings from memory and write to midi out
{
  if (debug) Serial.println("read midi");
  // Read midi values
  intMidiValues[MIDIAMP] = EEPROM.read((addr)+1+MIDIAMP);
  intMidiValues[MIDIFB] = EEPROM.read((addr)+1+MIDIFB);
  intMidiValues[MIDIBS] = EEPROM.read((addr)+1+MIDIBS);
  intMidiValues[MIDIMB] = EEPROM.read((addr)+1+MIDIMB);
  // Write Midi signals (0xC0 is the midi base program change address, add the device channel and write the device preset value)
  midiProg( 0xC0 | intMidiDeviceChannels[MIDIAMP], intMidiValues[MIDIAMP]);
  midiProg( 0xC0 | intMidiDeviceChannels[MIDIFB], intMidiValues[MIDIFB]);
  midiProg( 0xC0 | intMidiDeviceChannels[MIDIBS], intMidiValues[MIDIBS]);
  midiProg( 0xC0 | intMidiDeviceChannels[MIDIMB], intMidiValues[MIDIMB]);
}

int getAddress(int channel)
{
  // Determines the memory address for the selected channel
  // Usage of the memory space is explained in the memory function 
  int localAddr = 0;
  switch(channel)
  {
    case 0: // address for preset 1
      localAddr = currentBank + 10;
      return localAddr;
      break;
    case 1: // address for preset 2
      localAddr = currentBank + 20;
      return localAddr;
      break;
    case 2:
      localAddr = currentBank + 30;
      return localAddr;
      break;
    case 3:
      localAddr = currentBank + 40;
      return localAddr;
      break;
    case 4:
      localAddr = currentBank + 50;
      return localAddr;
      break;
    case 5:
      localAddr = currentBank + 60;
      return localAddr;
      break;
    case 6:
      localAddr = currentBank + 70;
      return localAddr;
      break;
    case 7:
      localAddr = currentBank + 80;
      return localAddr;
      break;
  }
}


int getAmpSetting(int ampChannel)
{
  // Read the channel value from the analog port, not implemented
  int inputValue = analogRead(ampChannel);
  if (inputValue > 512)
  {
    if (debug) Serial.println("Channel on");
    return HIGH;
  }
  else
  {
    if (debug) Serial.println("Channel off");
    return LOW;
  }
}


void handleLoopKeyEvent(int channel)
{
  // Handles what to do when a loop buttun is pressed
  if (debug) Serial.println("presetKeyEvent");
  writeOut(channel);
  if (muteMode!=0)
  {
    mute();
  }
  // Update loop led's
  setLCDChannel();
  refreshOled();
}


void handlePresetKeyEvent(int channel)
{
  // main function for processing preset key event
  if (debug) Serial.println("presetKeyEvent");
  if (debug) Serial.println(channel);

  if (deviceMode==PRESETMODE) // Preset mode, get channel settings from memory and activate loops and swicth midi devices
  {
    // Get preset loops
    readPreset(getAddress(channel), channel+1, channel);
  }
  if (deviceMode==BANKMODE) // Bank mode, when switching to different bank. Get channel settings from memory and stop LED's blinking
  {
    currentBank=newBank;
    deviceMode=PRESETMODE;
    readPreset(getAddress(channel), channel+1, channel);
    clearRow(i);
    strLine3 = "Mode: Preset " + String(channel+1);
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
    //writeLine(3,"Mode: Preset 8", mFont); 
  }
  if (deviceMode==MIDIMODE) // Midi mode, in this mode only button/channel 6 and 7 are handled for up and down of the current midi device value
  {
    // set new value
    if (channel==6) // up
    {
      // Handle up event
      intCurMidiValue++;
      if (intCurMidiValue > intMidiMaxValue[intCurMidi])
      {
        intCurMidiValue=0;
      }
    }
    if (channel==7) // down
    {
      // Handle up event
      intCurMidiValue--;
      if (intCurMidiValue < 0 )
      {
        intCurMidiValue=intMidiMaxValue[intCurMidi];
      }
    }
    // Update LCD to show new value and communicate new value with the midi device
    if (channel==6 || channel==7)
    {
      intMidiValues[intCurMidi] = intCurMidiValue;
//      lcd.setCursor(17,2);   TODO midi settings
//      lcd.print("   ");
//      lcd.setCursor(17,2); // print midi channel to lcd
//      lcd.print(intCurMidiValue);
      // Send midi signal TODO
      midiProg( 0xC0 | intMidiDeviceChannels[intCurMidi], intMidiValues[intCurMidi]);
//      if (debug) Serial.println("Write midi");
//      if (debug) Serial.println(intCurMidi);
//      if (debug) Serial.println(intMidiDeviceChannels[intCurMidi]);
//      if (debug) Serial.println(intMidiValues[intCurMidi]);
    }
  }
  if (deviceMode==ORDERMODE) // Order mode, in this mode you can control the switch order, Loops versus midi devices. Only 6 and 7 are handled for reversing the order
  {
    // Change the order value
    if (channel==6) // up
    {
      // Handle up event
      intCurSwitchOrderValue++;
      if (intCurSwitchOrderValue > 1)
      {
        intCurSwitchOrderValue=0;
      }
    }
    if (channel==7) // down
    {
      // Handle up event
      intCurSwitchOrderValue--;
      if (intCurSwitchOrderValue < 0 )
      {
        intCurSwitchOrderValue=1;
      }
    }
    // Update the LCD display to show the new value
    if (channel==6 || channel==7)
    {
      intMidiValues[intCurMidi] = intCurMidiValue;
      clearRow(9);      
      if (intCurSwitchOrderValue==0)
      {
        strLine9 = "Loops -> Midi";
        //writeLine(9,"Loops -> Midi", mFont); 
      }
      if (intCurSwitchOrderValue==1)
      {
        strLine9 = "Midi -> Loops";
        //writeLine(9,"Midi -> Loops", mFont); 
      }
    }
  }

  if (muteMode!=0) // When a preset is pressed in mute mode, disable mute and active preset
  {
    // Disable mute mode
    digitalWrite(muteRelay,HIGH);
    digitalWrite(mmLED,HIGH);
    muteMode=0;
    clearRow(3);
    strLine3 = "Mode: Preset";
    //writeLine(3,"Mode: Preset", mFont); 
  }
  if (deviceMode==STOREMODE) // Store preset mode, store preset in the activated channel (channel variable)
  {
    int localAddr = 0;
    switch (channel)
    {
      case 0:
        localAddr = currentBank + 10;
        memory(localAddr,0); /* (EEPROM address, led) */
        break;
      case 1:
        localAddr = currentBank + 20;
        memory(localAddr,1); /* (EEPROM address, led) */
        break;
      case 2:
        localAddr = currentBank + 30;
        memory(localAddr,2); /* (EEPROM address, led) */
        break;
      case 3:
        localAddr = currentBank + 40;
        memory(localAddr,3); /* (EEPROM address, led) */
        break;
      case 4:
        localAddr = currentBank + 50;
        memory(localAddr,4); /* (EEPROM address, led) */
        break;
      case 5:
        localAddr = currentBank + 60;
        memory(localAddr,5); /* (EEPROM address, led) */
        break;
      case 6:
        localAddr = currentBank + 70;
        memory(localAddr,6); /* (EEPROM address, led) */
        break;
      case 7:
        localAddr = currentBank + 80;
        memory(localAddr,7); /* (EEPROM address, led) */
        break;
    }
  }
  refreshOled();
}

void changeDeviceMode(int mode)
{
// When mode is 1 we switch between looper and preset, 
// when mode is 2 we either go to store preset mode or we disable store preset mode
  if (debug) Serial.println("changeDeviceMode");
  if (mode==PROGRAMMODE && deviceMode==PRESETMODE) {deviceMode=PROGRAMMODE;}
  else if (mode==PROGRAMMODE && deviceMode==PROGRAMMODE) {deviceMode=MIDIMODE;}
  else if (mode==PROGRAMMODE && deviceMode==STOREMODE) {deviceMode=PROGRAMMODE;}
  else if (mode==PRESETMODE && deviceMode==STOREMODE) {deviceMode=PRESETMODE;}  
  else if (mode==PROGRAMMODE && deviceMode==MIDIMODE) {deviceMode=ORDERMODE;}
  else if (mode==PROGRAMMODE && deviceMode==ORDERMODE) {deviceMode=PRESETMODE;}
  else if (mode==STOREMODE && deviceMode==PRESETMODE) {deviceMode=STOREMODE;previousDeviceMode=PRESETMODE;}
  else if (mode==STOREMODE && deviceMode==PROGRAMMODE) {deviceMode=STOREMODE;previousDeviceMode=PROGRAMMODE;}
  else if (mode==STOREMODE && deviceMode==MIDIMODE) {deviceMode=STOREMODE;previousDeviceMode=PRESETMODE;}
  else if (mode==STOREMODE && deviceMode==ORDERMODE) {deviceMode=STOREMODE;previousDeviceMode=PRESETMODE;}
  else if (mode==STOREMODE && deviceMode==STOREMODE) {deviceMode=PRESETMODE;previousDeviceMode=PRESETMODE;}
  if  (debug) Serial.println("New deviceMode");
  if  (debug) Serial.println(deviceMode);
  if  (debug) Serial.println(deviceMode==PRESETMODE);
  if (deviceMode!=PRESETMODE) {
    digitalWrite(mmLED,LOW);
  } else {
    digitalWrite(mmLED,HIGH);
  }
  if (deviceMode==PROGRAMMODE) // set looper mode
  {
    //Write presets 
    clearRow(3);
    strLine3 = "Mode: Program";
    //writeLine(3,"Mode: Program", mFont); 
    setLCDChannel();
  }
  if (deviceMode==MIDIMODE) // set midi mode
  {
    //Write presets 
    clearRow(3);
    strLine3 = "Mode: Midi";
    //writeLine(3,"Mode: Midi", mFont); 
    //lcd.print(strVersion);
    clearRow(8);
    strLine8 = "Midi 2 select source";
    //writeLine(8,"Midi 2 select source", mFont); 
    clearRow(9);
  }
  if (deviceMode==ORDERMODE) // set order mode
  {
    //Write presets 
    clearRow(4);
    strLine4 = "Set switch order";
    //writeLine(4,"Set switch order", mFont); 
    clearRow(8);
    if (intCurSwitchOrderValue==0)
    {
      strLine8 = "Loops -> Midi";
      //writeLine(8,"Loops -> Midi", mFont); 
    }
    if (intCurSwitchOrderValue==1)
    {
      strLine8 = "Midi -> Loops";
      //writeLine(8,"Midi -> Loops", mFont); 
    }
    clearRow(9);
    strLine8 = "_up(7)    (8)down_";
    //writeLine(9,"_up(7)    (8)down_", mFont); 
  }
  if (deviceMode==AMPMODE) // Mode for settng reverb and channel settings
  {
    
  }
  
  if (deviceMode==PRESETMODE) // return to preset mode
  {
    // Reset currentPreset
    int preset = currentPreset;
    currentPreset=-1;
    readPreset(getAddress(preset), 1, preset);
    clearRow(3);
    strLine3 = "Mode: Preset";
    strLine8 = "";
    //writeLine(3,"Mode: Preset", lFont); 
  }
  if (deviceMode==STOREMODE)
  {
    clearRow(3);
    strLine3 = "Mode: Store";
    //writeLine(3,"Mode: Store", lFont); 
    // lcd.clear();
    clearRow(4);
    strLine4 = "Select preset 2store";
    // writeLine(4,"Select preset 2store", lFont); 
    // Show all Preset LED's
    intPresetLEDs = 255;
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
  }
  refreshOled();
}

void showLCDBankMode()
{
// Shows the current mode in the LCD display
    if (debug) Serial.println("currentBank = " + String(currentBank));
    // check currentBank
    clearRow(1);
    if (currentBank==0)
    {
      strLine1 = "Switch8 XL  A";
      //writeLine(1,"Switch8 XL  A", lFont);
    }
    if (currentBank==100)
    {
      strLine1 = "Switch8 XL  B";
      //writeLine(1,"Switch8 XL  B", lFont);
    }
    if (currentBank==200)
    {
      strLine1 = "Switch8 XL  C";
      //writeLine(1,"Switch8 XL  C", lFont);
    }
}

void handleAmpBankEvent(int intButton)
{
  if (deviceMode==BANKMODE)
  {
    String strBank = "A";
    if (intButton==1) // UP
    {
      if (newBank==0) { newBank=100; strBank="B"; }
      else if (newBank==100) { newBank=200; strBank="C"; }
      else if (newBank==200) { newBank=0; strBank="A"; }
    }
    // check currentBank
    if (intButton==2) // Down
    {
      if (newBank==0) { newBank=200; strBank="C"; }
      else if (newBank==100) { newBank=0; strBank="A"; }
      else if (newBank==200) { newBank=100; strBank="B";}
    }
    showLCDBankMode();
    // Update LCD
    clearRow(4);
    strLine4 = "Select bank " + strBank + " preset";
    refreshOled();
    bankModeMillis = millis();
  }
  // Handle amp setting or bank switch event, input either 1 (reverb or down) or 2 (gain or up)
  if (deviceMode==PRESETMODE) // Preset mode => bank up or down
  {
    String strBank = "A";
    // check currentBank
    if (intButton==1) // UP
    {
      if (currentBank==0) { newBank = 100; strBank="B"; }
      if (currentBank==100) { newBank = 200; strBank="C"; }
      if (currentBank==200) { newBank = 0; strBank="A"; }
    }
    // check currentBank
    if (intButton==2) // Down
    {
      if (currentBank==0) { newBank = 200; strBank="C"; }
      if (currentBank==100) { newBank = 0; strBank="A"; }
      if (currentBank==200) { newBank = 100; strBank="B";}
    }
    showLCDBankMode();
    // Update LCD
    clearRow(4);
    strLine4 = "Select bank " + strBank + " preset";
    refreshOled();
    // Show blinking pedal LED's TODO
    intPresetLEDs = 255;
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
    // Set BANKMODE
    deviceMode=BANKMODE;
    bankModeMillis = millis();
  }
  if (deviceMode==PROGRAMMODE) // In program mode switch amp reverb setting (no longer used)
  {
    unsigned long currentMillis = millis();
    if (intButton==1)
    {
      digitalWrite(ampReverbRelay, LOW);
      ampReverbValue = !ampReverbValue;
      previousReverbMillis = currentMillis;
    }
    if (intButton==2)
    {
      digitalWrite(ampGainRelay, LOW);
      ampGainValue = !ampGainValue;
      previousGainMillis = currentMillis;
    }
    // Update LCD
    setLCDAmpSettings();
  }
  if (deviceMode==MIDIMODE) // Show midi mode settings in LCD display cycle through available midi devices and show their settings
  {
    if (intButton==1)
    {
      String strMidi = "                    ";
      intCurMidi = intCurMidi + 1;
      if (intCurMidi > 3) 
      {
        intCurMidi = 0;
      } 
      intCurMidiValue = intMidiValues[intCurMidi];
      clearRow(8);
      strLine8 = strMidiDevices[intCurMidi] + "  " + intCurMidiValue;
      //writeLine(8,strMidiDevices[intCurMidi] + "  " + intCurMidiValue , lFont); 
      clearRow(9);
      strLine9 = "  _up(7)    (8)down_";
      //writeLine(9,"  _up(7)    (8)down_", lFont); 
    }
  }
}

/******************************************************/
//take care of some special events
void keypadEvent(KeypadEvent key){
  // Main function for figuring out which button was pressed by the guitarist
  if (debug) Serial.println("key event");
  if (debug) Serial.println(key);
  switch (keypad.getState())
  {
    case PRESSED:  // In the down event most time critical events are passed, being mute, the preset buttons, loop buttons and the bank buttons
      startMillis = millis();
      if (debug) Serial.println("key pressed event");
      if (debug) Serial.println(key);
      if (key=='j') // 'j' is mute mode
      {
        // Bypass all pedals
        mute();
      }
      else 
      {
        switch (key)
        {
           case 'a': // 'a' to 'h' represent a preset button pressed
            handlePresetKeyEvent(0);
            break;
           case 'b':
            handlePresetKeyEvent(1); 
            break;
           case 'c':
            handlePresetKeyEvent(2);
            break;
           case 'd':
            handlePresetKeyEvent(3);
            break;
           case 'e':
            handlePresetKeyEvent(4);
            break;
           case 'f':
            handlePresetKeyEvent(5);
            break;
           case 'g':
            handlePresetKeyEvent(6);
            break;
           case 'h':
            handlePresetKeyEvent(7);
            break;
           case 'm': // 'm' to 't' represent a loop key pressed
            handleLoopKeyEvent(0); 
            break;
           case 'n':
            handleLoopKeyEvent(1); 
            break;
           case 'o':
            handleLoopKeyEvent(2); 
            break;
           case 'p':
            handleLoopKeyEvent(3); 
            break;
           case 'q':
            handleLoopKeyEvent(4); 
            break;
           case 'r':
            handleLoopKeyEvent(5); 
            break;
           case 's':
            handleLoopKeyEvent(6); 
            break;
           case 't':
            handleLoopKeyEvent(7); 
            break;
           // TODO - Insert handler for moving through the midi settings
           case 'k': // depending on mode either amp Reverb switch or bank down, the midi handling in program mode is also handled
            handleAmpBankEvent(1);
            break;
           case 'l': // depending on mode either amp Channel switch or bank up
            handleAmpBankEvent(2);
            break;
        }
      }
    break;
    case RELEASED: // processes the buttons in the event they are released
      if (debug) Serial.println("key released event");
      if (debug) Serial.println(key);
      // saveState check savestate
      switch (key)
      {
         case 'i': // 'i' represents the mode button
          if (holdProcessed == 0)
          {
            changeDeviceMode(1);
          }
          else holdProcessed = 0;
          break;
//         case 'k': // depending on mode either amp Reverb switch or bank down, the midi handling in program mode is also handled
//          memoryDump();
//          break;
      }
    break;
    case HOLD: // processes the buttons in the event they are hold a longer time (3 seconds)
      if (debug) Serial.println("key hold event");
      if (debug) Serial.println(key);
      switch (key)
      {
        /****************************** STORE PRESET MODE */
         case 'i': // 'i' represents the mode button, when hold it triggers the store preset mode
          holdProcessed = 1;
          changeDeviceMode(STOREMODE);
          break;
         case 'j': // Enable Mute + mode (blinking LEDs)
          if (muteMode!=0)
          {
            muteMode = 2;
            // store value in Prev variable
            intPresetLEDsPrev = intPresetLEDs;
            // Show all Preset LED's
            intPresetLEDs = 255;
            // Write value to presets IO
            Wire.beginTransmission(IO_ADDR_Presets);
            Wire.write(~intPresetLEDs);
            Wire.endTransmission();
            intLoopLEDsPrev = intLoopLEDs;
            intLoopLEDs = 255;
            // Write value to presets IO
            Wire.beginTransmission(IO_ADDR_Loops);
            Wire.write(~intLoopLEDs);
            Wire.endTransmission();
            clearRow(3);
            writeLine(3,"TILT TILT TILT TILT ", lFont); 
          }
          break;
      }
    break;
  }
}

void showTimer()
{
  // Shows duration from keyPressed event till you insert the showTimer call, used when performace testing during development of the hardware and software
  unsigned long currentMillis = millis();
  unsigned long duration = currentMillis - startMillis; 
  // Display duration
  clearRow(4);
  writeLine(4,String(duration), lFont); 
}

/******************************************************/

void loop()
{
  // Trigger keypad events
  char key = keypad.getKey(); // When one of the buttons is pressed it will trigger the keypadEvent() function
  
  // check for store preset mode, if found blink leds
  unsigned long currentMillis = millis();
  if ((deviceMode==STOREMODE) and currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // reverse the preset LEDs
    intPresetLEDs = ~intPresetLEDs;
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
  }
  
  if ((deviceMode==BANKMODE) and currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    if (intPresetBlinkingLEDs==0)
    {
      intPresetBlinkingLEDs = intPresetLEDs;
    }
    else
    {
      intPresetBlinkingLEDs = 0;
    }
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetBlinkingLEDs);
    Wire.endTransmission();
  }

  if (muteMode==2 and currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // reverse the preset LEDs
    intPresetLEDs = ~intPresetLEDs;
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Presets);
    Wire.write(~intPresetLEDs);
    Wire.endTransmission();
    intLoopLEDs = ~intLoopLEDs;
    // Write value to presets IO
    Wire.beginTransmission(IO_ADDR_Loops);
    Wire.write(~intLoopLEDs);
    Wire.endTransmission();
  }

  if (muteMode==1 and currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    digitalWrite(mmLED,!digitalRead(mmLED));
  }

  if (previousReverbMillis!=0 and currentMillis - previousReverbMillis >= 50 )
  {
    digitalWrite(ampReverbRelay, HIGH);
    previousReverbMillis = 0;
  }
  if (previousGainMillis!=0 and currentMillis - previousGainMillis >= 50 )
  {
    digitalWrite(ampGainRelay, HIGH);
    previousGainMillis = 0;
  }
  // Check if bankMode duration is passed
  if (deviceMode==BANKMODE and (currentMillis > (bankModeMillis + bankModeDurationMillis)))
  {
     // undo Bankmode and return to presetmode
    digitalWrite(mmLED,HIGH);
    // Reset currentPreset
    int preset = currentPreset;
    readPreset(getAddress(preset), 1, preset);
    clearRow(3);
    strLine3 = "Mode: Preset";
    strLine8 = "";
    refreshOled();
    deviceMode=PRESETMODE;
  }
}
