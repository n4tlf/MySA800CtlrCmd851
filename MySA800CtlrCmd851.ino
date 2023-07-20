/*  Arduino Shugart Floppy Disk Test Controller
     Current version is on an Arduino Nano, but looks like Duemilinova or Decimila

     INSTANT COMMANDS ARE:
        I moves head IN one track
        O moves hear OUT one track
        Z moves heat to TRACK 00 (first track)
        L moved head to TRACK 77 (last track)
        T Toggles head LOAD on/off
     REGULAR COMMANDS ARE:
        D [x] Select Drive [X].  If no X, current drive
              number is displayed
        E     sidE sElEct  Toggle between side 0 and 1
   NOT NANO     C     Compare current track number read from disk
              with expected track number
   NOT NANO     F X [H] Format Track.  If optional H is a valid HEX
              value, that value is written in the data fields
        G     Display current Drive, Track, Sector information
        H     Display Help information on screen
        N     Toggles NO WRITE protection. Default: NOWRITE ON
   NOT NANO     R x   Read current sector X data
        S X [y] Step to Track X.  If optional Y is specified,
              the drive will alternate between the two tracks
              with each press of the space bar.  Press S to exit.
   NOT NANO     W X HH Write Sector X with HH Hex characters
        0 X   Step to track X, then erase all data on that trackt
        1 X   Step to track X, then write clock-only (data 00) (Shugart 1F)
        2 X   Step to track X, then write all ones (data FF) (Shugart 2F)
        X     Exit & reboot Drive Exerciser


*/

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
//#include <TimerOne.h>
#include <string.h>
#include <FastPwmPin.h>

//#define SERTERM 1
#define KYBDI2C 1

#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1

#define CARDKB_ADDR 0x5F

//  First we define the output signal pins to the SA-800 drive
#define STEP  4         // SA800 #36  step the head one track active low
#define DIR   5         // SA800 #34  step direction (out=high, in=low)
#define DS0   A0         // SA800 #26 Drive Select 0  (port PC0)
#define DS1   A1         // SA800 #28 Drive Select 1  (port PC1)
#define DS2   A2         // SA800 #30 Drive Select 2  (port PC2)
#define DS3   A3         // SA800 #32 Drive Select 3  (port PC3)
#define HEADLD  8        // SA800 #18 head load  (LOW = HEAD LOAD)
#define WRDATA  10       // SA800 #38 Write Data (invert this with 7406)
#define WRGATE  7       // SA800 #40  Write Gate
#define TG43    0       // SA800 #2   Track Greater than 43 (SA-800 not used)
#define SIDESEL 9       // SA851 #14  Side Select (SA851) High=side 0, low=side 1
#define INUSE   0       // SA800 #16  in use signal (not used on SA-800)

// Next comes the input signal pins from the SA-800 drive
#define TRK00   2     // SA800 #42  Head at Track 00 (00=low, not=high) generates INT
#define INDEX   3     // SA800 #20  Index hole pulses  generates INT
#define DREADY  12     // SA800 #22 Drive ready
#define READDTA 6     // SA800 #46  Raw Read Data
#define WRPROT  11    // SA800 #44  Write protect detection
#define CHDISK  00    // SA800 #12  Disk changed (not used on SA-800)
#define SECTOR  00    // SA800 #24  Sector pulses (not used on SA-800)
//#define NOTUSED 9     // Nano pin available ATM

#define LED     13    // Activity LED on Arduino

// NOTE:  A4 is I2C SDA
//        A5 is I2C SCL


//  Its time to define eight inch Single Density parameters
#define MAXDRIVE  4     // Maximum number of disk drives
#define MAXTRACK   77   // Maximum number of tracks
#define MAXSEC    26    // Maximum sectors per track
#define MAXSIDES  2     // Maximum sides on drive
#define SECDSIZE  128   // Sector data size in bytes
#define SECBITS   1024  // Sector data size in bits
#define STEPON    6     // step ON in usec (min. 4 usec)
#define STEPOFF   10     // step OFF in msec (min 8 msec)
#define STEPDIRCH 30    // Step Direction change delay (min 24 usec)

//  Now we define the Drive Modes of current drive
#define MIDLE  0     // No reading or writing
#define MREAD  1     // Drive in READ mode
#define MWRITE 2     // Drive in WRITE MODE
#define MTRANS 3     // Drive in transition mode


#define STEPOUT HIGH  // Step out (HI) will move head toward track 00
#define STEPIN  LOW   // Step in (LOW) will move heard toward center

int currentTrack = 0;
int newTrack;
int reqTrack;
int myTrack;
int testTrack1 = 0;
int testTrack2 = 0;
int curDrive = 0; //DRIVEx;
int newDrive = 0; //DRIVEx;
int newSec = 0;
int driveMode = MIDLE;
int curSec = 0;
int hexData;

//unsigned long startMillis;
//unsigned long currMillis = 0;
unsigned long prevMillis = 0;   // NOTE:  millis() uses TIMER0
unsigned long currMillis;      //set the current millisecond count
//const unsigned long step_on = 10;
//unsigned long stepOff = 8;     // time delay (in msec) between head steps
//unsigned long stepOn = 4;
int StepState = HIGH;

bool  stepDir;
bool  curDir;
bool  headLoaded;
bool  noWrite;
bool  twoTrack;
bool  writeProtect;
bool  driveReady;
bool  track00;
bool  driveIndex;
bool  curSide;
bool  readData;

String cmdStr = "";
//char cmdStr = "";
//char ary[15] = "";
char *ptr = NULL;
char  inByte = 0;
char  cmdChar = 0;
volatile int pWidth = 0;
int   pwCount = 4;


#define MAXARGS 5
#define ARGSIZE 6

char  cmd;
char cmdBfr[31];
int cmdLen = 30;
byte index = 0;

char args[MAXARGS][ARGSIZE];

char varDelimiter = ":";
char endDelimiter = "\n";

int oledLine = 0;

SSD1306AsciiWire oled;

///////////////////////////////////////////////////////////////////
/************************************************************
   Direction-to-step timing 1us minimum, steps on trailing edge
   step trailing edge to trailing edge 10ms minimum
   step pulse width 10us minimum  (active low)
   330 ohm terminating resistor creates 15ma load
*/

void oneStep()
{
  driveMode = MTRANS;

  if (curDir != stepDir)        // if direction has changed
  {
    if (stepDir == STEPIN)       // if direction was low/stepIN
      digitalWrite(DIR, STEPOUT);    // change it to High/STEPOUT
    else
      digitalWrite(DIR, STEPIN);
    curDir = stepDir;               // save new step direction
    delayMicroseconds(STEPDIRCH); // and delay for drive change
  }
  digitalWrite(STEP, LOW);
  delayMicroseconds(STEPON);
  digitalWrite(STEP, HIGH);
  delay(STEPOFF);
  updStat();
  driveMode = MIDLE;
}

///////////////////////////////////////////////////////////////////
void updStat()
{
  driveReady = !digitalRead(DREADY);
  writeProtect = !digitalRead(WRPROT);
  track00 = !digitalRead(TRK00);
  driveIndex = !digitalRead(INDEX);
  readData = digitalRead(READDTA);
}

///////////////////////////////////////////////////////////////////
void gotoTrack(int reqTrack)
{
  int tracksToMove;

  if (currentTrack != reqTrack)
  {
    if (currentTrack > reqTrack)  // if we are too far out
    {
      stepDir = LOW;
      //updateDir();
      tracksToMove = currentTrack - reqTrack;
      //        Serial.print("cur>req ");
    }
    else
    {
      stepDir = HIGH;
      //updateDir();
      tracksToMove = reqTrack - currentTrack;
      //        Serial.print("req>cur ");
    }
    while (tracksToMove != 0)
    {
      oneStep();
      //        Serial.print("Tracks to Move: ");
      //        Serial.print(tracksToMove);
      //        Serial.print(" Cur Track: ");
      //        Serial.print(currentTrack);
      //        Serial.print(" cur Dir: ");
      //        Serial.println(curDir);
      //        Serial.print(" ");
      tracksToMove--;
      if (curDir)
        currentTrack++;
      else
        currentTrack--;
    }
    //currentTrack = reqTrack;
    //      Serial.print("Current Track: ");
    //      Serial.print(currentTrack);
    //      Serial.println(" Track Loop Done");
    updOled();
  }
}

////////////////////////////////////////////////////////////////////////////
///   NOTE:  THis needs to be fixed cuz it only stops on Track 00 sensor
void Track00()
{
  int maxTracks = 78;

  //    stepDir = LOW;
  while ((maxTracks != 0) && !track00)   //(!TRK00))
  {
    oneStep();
    maxTracks--;
  }
  currentTrack = 0;
  Serial.print("HOME SWEET HOME");
}

////////////////////////////////////////////////////////////////////////////
void  loadHead()
{
  if (!headLoaded)
  {
    digitalWrite(HEADLD, LOW);
    digitalWrite(LED, HIGH);
    Serial.println("Head is now LOADED");
    headLoaded = !headLoaded;
  }
  updOled();
}

void  unloadHead()
{
  if (headLoaded)
  {
    digitalWrite(HEADLD, HIGH);
    digitalWrite(LED, LOW);
    Serial.println("Head is now UNLOADED");
    headLoaded = !headLoaded;
  }
  updOled();
}

//////////////////////////////////////////////////////////////////////////
void  driveSelect()
{
  digitalWrite(DS0, HIGH);    // first, reset all Drive Select line OFF
  digitalWrite(DS1, HIGH);    //   /
  digitalWrite(DS2, HIGH);    //  /
  digitalWrite(DS3, HIGH);    // /
  curDrive = newDrive;
  switch (newDrive)
  {
    case 0:
      digitalWrite(DS0, LOW);    // Now turn ON Drive Select 0
      break;
    case 1:
      digitalWrite(DS1, LOW);    // Now turn ON Drive Select 1
      break;
    case 2:
      digitalWrite(DS2, LOW);    // Now turn ON Drive Select 2
      break;
    case 3:
      digitalWrite(DS3, LOW);    // Now turn ON Drive Select 3
      break;
    default:
      break;
  }
  updOled();
}

/////////////////////////////////////////////////////////////////////////
void updOled()
{
  updStat();
  oled.setRow(1);
  oled.setCol(0);
  oled.print("DR:");
  oled.print(curDrive);
  oled.setCol(32);
  oled.print("Trk:  ");
  oled.setCol(56);
  oled.print(currentTrack);
  oled.setCol(76);
  oled.print("Sec:   ");
  oled.setCol(100);
  oled.print(curSec);
  oled.setCol(113);
  if (noWrite && writeProtect)
    oled.print("B");
  else if (noWrite)
    oled.print("N");
  else if (writeProtect)
    oled.print("P");
  else if (!noWrite && !writeProtect)
    oled.print("W");
  oled.setCol(120);
  if (headLoaded)
    oled.print("H");
  else
    oled.print("U");
  oled.setRow(2);
  oled.setCol(0);
  oled.print("Side:");
  oled.setCol(30);
  oled.print(!curSide);
  oled.setCol(44);
  oled.print("Head:");
  oled.setCol(74);
  if (headLoaded)
    oled.println("LOADED!  ");
  else
    oled.println("UNLOADED");


  oled.setRow(3);
  oled.setCol(0);
}

////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  //  pinMode(tmrPin, OUTPUT);

  pinMode(STEP, OUTPUT);
  digitalWrite(STEP, HIGH);

  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH);

  pinMode(DS0, OUTPUT);
  digitalWrite(DS0, HIGH);

  pinMode(DS1, OUTPUT);
  digitalWrite(DS1, HIGH);

  pinMode(DS2, OUTPUT);
  digitalWrite(DS2, HIGH);

  pinMode(DS3, OUTPUT);
  digitalWrite(DS3, HIGH);

  pinMode(HEADLD, OUTPUT);
  digitalWrite(HEADLD, HIGH);

  pinMode(WRDATA, OUTPUT);
  digitalWrite(WRDATA, HIGH);

  pinMode(WRGATE, OUTPUT);
  digitalWrite(WRGATE, HIGH);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(SIDESEL, OUTPUT);
  digitalWrite(SIDESEL, HIGH);

  pinMode(TRK00, INPUT_PULLUP);      // pin
  pinMode(INDEX, INPUT_PULLUP);      // pin
  pinMode(DREADY, INPUT_PULLUP);     // pin
  pinMode(READDTA, INPUT_PULLUP);    // pin
  pinMode(WRPROT, INPUT_PULLUP);     // pin

  noWrite = true;
  //  stepOff =  10;
  //  stepDir = STEPOUT;
  //  curDir = STEPOUT;
  driveMode = MIDLE;
  //  currentTrack = 0;
  curDrive = 0;
  twoTrack = false;
  curSide = 1;          // 1 = side 0
  headLoaded = false;

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  oled.clear();
  oled.println(" SA800 FOXerciser");

  newDrive = 0;
  driveSelect();
  unloadHead();
  Track00();

  updOled();

}   //  END OF setup()

////////////////////////////////////////////////////////////////////////
void loop() {

  updStat();
#ifdef  SERTERM
  if (Serial.available() > 0)
#endif
#ifdef  KYBDI2C
    Wire.requestFrom(CARDKB_ADDR, 1);
  if (Wire.available())
#endif
  {
#ifdef  SERTERM
    inByte = Serial.read();
#endif
#ifdef  KYBDI2C
    inByte = Wire.read();
    if (inByte == 0);
    else {
#endif
      switch (inByte)
      {
        case 'T':
        case 't':
          if (headLoaded)
          {
            oled.println("Unloading Head");
            unloadHead();
          }
          else
          {
            oled.println(F("Loading Head  "));
            loadHead();
          }
          cmdStr = "";
          break;

        case 'I':
        case 'i':
          if (currentTrack < 77)
          {
            gotoTrack(currentTrack + 1);
            Serial.print(F("One track in to Track: "));
            Serial.println(currentTrack);
          }
          else
          {
            Serial.print(F("ERROR: ALready at Track: "));
            Serial.println(currentTrack);
          }
          cmdStr = "";
          break;
        case 'O':
        case 'o':
          if (currentTrack > 0)
          {
            gotoTrack(currentTrack - 1);
            Serial.print(F("One Track Out to Track: "));
            Serial.println(currentTrack);
          }
          else
          {
            Serial.print(F("ERROR: ALready at Track: "));
            Serial.println(currentTrack);
          }
          cmdStr = "";
          break;
        case 'Z':
        case 'z':
          Serial.println(F("go to track 0"));
          gotoTrack(0);
          cmdStr = "";
          break;
        case 'L':
        case 'l':
          Serial.println(F("go to track 77"));
          gotoTrack(77);
          cmdStr = "";
          break;

        case '\b':        //0x08:
          if (index != 0)
          {
            index--;
            cmdBfr[index] = '\0';
            //index--;
            Serial.print('\b');
            cmdStr = "";
          }
          break;
        case 0x0D:    //'\r':              //0x0d:
        case 0x0A:    //'\n':              //0x0a:
          cmdBfr[index] = '\0';
          cmdStr = "";
          Serial.print("rcvd: ");
          Serial.println(cmdBfr);
          //////////  here to process line
          for (index == 0; index < cmdLen; index++)
          {
            cmdBfr[index] = '\0';
          }
          index = 0;
          cmdExec();
          break;
        default:
          cmdBfr[index] = inByte;
          index++;
          cmdBfr[index] = '\0';
          //Serial.print(cmdBfr);
          //         Serial.println("HIT DEFAULT");
          break;
      }
#ifdef KYBDI2C
    }
#endif
  }
}

void cmdParse()
{
  char *argument;
  int counter = 0;
  char parseBfr[31];

  strcpy(parseBfr, cmdBfr);
  Serial.print(F("Parsing Line: "));
  Serial.println(parseBfr);

  for (counter = 0; counter < MAXARGS; counter++)
    args[counter][0] = '\0';

  counter = 0;
  argument = strtok(parseBfr, " ");

  while ((argument != NULL)) {
    if (counter < MAXARGS) {
      if (strlen(argument) < ARGSIZE) {
        strcpy(args[counter], argument);
        argument = strtok(NULL, " ");
        counter++;
      }
      else {
        Serial.println(F("Input string too long."));
        //                error_flag = true;
        break;
      }
    }
    else {
      break;
    }
  }
}

void  cmdExec()
{
  cmdParse();
  cmdChar = args[0][0];
  switch (cmdChar)
  {
    case 'D':
    case 'd':
      if (strlen(args[1]) != 0)
      {
        newDrive = atoi(args[1]);
        if (newDrive < MAXDRIVE)
        {
          driveSelect();
          Serial.print(F("Changed Drive to: "));
          Serial.println(newDrive);
        }
        else
          Serial.println(F("ERROR: REQUESTED DRIVE NOT VALID"));
      }
      else
      {
        Serial.print(F("Current Drive is: "));
        Serial.println(curDrive);
      }
      break;
    case 'N':
    case 'n':
      if (noWrite)
      {
        noWrite = false;
        Serial.println(F("No Write DISABLED - BE CAREFUL"));
      }
      else
      {
        noWrite = true;
        Serial.println(F("No Write ENABLED - Safety First!"));
      }
      //          gotoTrack(0);
      //          cmdStr = "";
      updOled();
      break;
    case 'S':
    case 's':
      if (strlen(args[1]) != 0)
      {
        testTrack1 = atoi(args[1]);
        Serial.println("First Track = ");
        Serial.println(testTrack1);

        if (strlen(args[2]) != 0)
        {
          testTrack2 = atoi(args[2]);
          Serial.print("Second Track = ");
          Serial.println(testTrack2);
          twoTrack = true;
        }

        if (testTrack1 <= MAXTRACK)
        {
          gotoTrack(testTrack1);
          Serial.print("Changed Track to: ");
          Serial.println(currentTrack);
        }
        else
          Serial.println(F("ERROR: REQUESTED TRACK NOT VALID"));
      }
      else
      {
        Serial.print(F("Current Track is: "));
        Serial.println(currentTrack);
      }
      break;

    case 'C':
    case 'c':
      Serial.println(F("Compare Track on disk"));
      break;
    case 'E':
    case 'e':
      if (curSide)
        curSide = 0;
      else
        curSide = 1;
      digitalWrite(SIDESEL, curSide);
      Serial.print("sidE now: ");
      Serial.println(!curSide);
      updStat();
      updOled();
      break;


    case 'F':
    case 'f':
      Serial.println("Format Track ");
      break;
    case 'G':
    case 'g':
      updStat();
      updOled();
      Serial.print("Current Drive: ");
      Serial.print(curDrive);
      Serial.print(" Current Track: ");
      Serial.print(currentTrack);
      Serial.print(" Current Sector: ");
      Serial.print(curSec);
      Serial.print(" Drive is: ");
      if (driveReady)
        Serial.print("READY ");
      else
        Serial.print("NOT READY ");
      Serial.print(" Drive Write Protect: ");
      if (writeProtect)
        Serial.print(" ON ");
      else
        Serial.print(" OFF ");
      Serial.print(" Drive Curent Side: ");
      if (curSide)
        Serial.print("Side 0 ");
      else
        Serial.print("Side 1 ");



      Serial.print(" Head is: ");
      if (headLoaded)
        Serial.println("Loaded");
      else
        Serial.println("unLoaded");
      break;
    case 'H':
    case 'h':
      Serial.println("Help message ");
      break;

    case 'W':
    case 'w':
      if (strlen(args[1]) != 0)
      {
        newSec = atoi(args[1]);
        if (newSec < MAXSEC)
        {
          curSec = newSec;
          Serial.print("Changed Sector to: ");
          Serial.println(newSec);
        }
        else
          Serial.println("ERROR: REQUESTED SECTOR NOT VALID");
      }
      else
      {
        Serial.print("Current Sector is: ");
        Serial.println(curSec);
      }
      updOled();
      Serial.print("Write Sector: ");
      Serial.print(curSec);
      if (strlen(args[2]) != 0)
      {
        hexData = strtol(args[2], 0, 16);
        Serial.print(" Hex data field: ");
        Serial.println(hexData, HEX);
      }

      Serial.println("Write hh Hex to Sector: ");
      break;
    case 'R':
    case 'r':
      if (strlen(args[1]) != 0)
      {
        newSec = atoi(args[1]);
        if (newSec < MAXSEC)
        {
          curSec = newSec;
          Serial.print("Changed Sector to: ");
          Serial.println(newSec);
        }
        else
          Serial.println("ERROR: REQUESTED SECTOR NOT VALID");
      }
      else
      {
        Serial.print("Current Sector is: ");
        Serial.println(curSec);
      }
      updOled();
      Serial.print("Read Sector: ");
      Serial.print(curSec);
      break;
    case '1':
      if (!noWrite && !writeProtect)
      {
        if (driveReady)
        {
          digitalWrite(HEADLD, LOW);
          while (!driveIndex)
          {
            updStat();
          }
          digitalWrite(WRGATE, LOW);
          FastPwmPin::enablePwmPin(10, 250000L, 88);  // FD1771 manual for 00: 4usec, 500-600ns pulse, positive (7407)
          delay(167);
          digitalWrite(WRDATA, HIGH);
          //          FastPwmPin::enablePwmPin(10, 800000L, 2);  // Turn OFF data pulses
          digitalWrite(WRGATE, HIGH);
          digitalWrite(HEADLD, HIGH);
        }
        Serial.println("Set TRACK xx to all zero");
      }
      else
      {
        Serial.println("ERROR:  WRITE NOT ENABLED");
      }
      break;
    case '2':
      if (!noWrite && !writeProtect)
      {
        if (driveReady)
        {
          digitalWrite(HEADLD, LOW);
          while (!driveIndex)
          {
            updStat();
          }
          digitalWrite(WRGATE, LOW);
          FastPwmPin::enablePwmPin(10, 500000L, 75);  // FD1771 manual for FF: 2usec, 500-600ns pulse, positive (7407)
          delay(167);
          digitalWrite(WRDATA, HIGH);
          //        FastPwmPin::enablePwmPin(10, 800000L, 2);  // Turn OFF data pulses
          digitalWrite(WRGATE, HIGH);
          digitalWrite(HEADLD, HIGH);
        }
        Serial.println("Set TRACK xx to 2F: all one");
      }
      else
      {
        Serial.println("ERROR:  WRITE NOT ENABLED");
      }
      break;
    case '0':
      if (!noWrite && !writeProtect)
      {
        if (driveReady)
        {
          digitalWrite(HEADLD, LOW);
          while (!driveIndex)
          {
            updStat();
          }
          digitalWrite(WRGATE, LOW);
          digitalWrite(WRDATA, LOW);
          //          FastPwmPin::enablePwmPin(10, 500000L, 75);  // FD1771 manual for FF: 2usec, 500-600ns pulse, positive (7407)
          delay(167);
          digitalWrite(WRDATA, HIGH);
          //        FastPwmPin::enablePwmPin(10, 800000L, 2);  // Turn OFF data pulses
          digitalWrite(WRGATE, HIGH);
          digitalWrite(HEADLD, HIGH);
        }
        Serial.println("Erase Track xx");
      }
      else
      {
        Serial.println("ERROR:  WRITE NOT ENABLED");
      }
      break;


    default:
      Serial.println("COMMAND Default HIT");
      break;
  }
  for (index == 0; index < cmdLen; index++)
  {
    cmdBfr[index] = '\0';
  }
  index = 0;
  updStat();
}
