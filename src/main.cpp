//Jeroen Garritsen's B&O McKenzie Division - Staging Yard Project
//by: Mark Kellogg - Began: 4/23/2020
//
//---------------------------Github Repository------------------------------
// NAME:
// The repository has all code etc. along with flow control overview and 
// module graphics.  
//
// Designed for use on Jeroen Gerrisen's B&O McKenzie Div layout.
// A main control panel uses a Pro-Mini to set up active staging yard
// tracks with a second Pro-Mini to drive the rr-CirKits MotorMan 
// boards, control track power, etc. The project requires remote 
// control panels, so the second Pro_Mini is used to eliminate
// a large cable bundle to the remote panel.
//
// Panel
//
// Track power timer
//
// Entrance and Exit sensors - 
//
//----------------------------Track Sensors Descriptions--------------------
// All four staging yards have a single yard lead, from which all 
// the dead end staging tracks fan out.  The yard lead of three of the four 
// yards continue on to a reverse loop.
//
//   Sensors - The yard lead entry turnout and reverse loop turnout each have a 
//   sensor pair to track entry and exits from that point.
//
//   Sensor Names - Sensors are named "mainSens" for the yard throat, and 
//   "revSens" for the reverse loop leadout.
//
//   Direction Naming Convention - In all cases you may think of a point "between"
//   the yard lead turnout and the reverse loop turnout as the center of the
//   universe.  Therefore INBOUND is always towards that point on either sensor
//   and OUTBOUND obviously the reverse.   
//
//
// Module Output- Each sensor pair returns: 
//   Train Direction- mainDirection or revDirection: their output(s) are 
//   INBOUND, OUTBOUND, or CLEAR; and are active when the train is within the 
//   small sensor area.
//   
//   Train PassBy - which senses the train has passed by the sensor completely in 
//   the direction it arrived from.  If a train backs out without completely
//   passing by PassBy will not report true. It stays active as long as the 
//   train is within the sensor pair.
//
//   Sensor Busy - Sensor reports busy when either of the sensor pair is true
//   and remains so only until the both go false.
//---------------------------------------------------------------------------

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Bounce2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//------------Setup sensor debounce from Bounce2 library-----
#define mainSensInpin 11
#define mainSensOutpin 12
#define revSensInpin 10 
#define revSensOutpin 9

#define INBOUND 1    
#define OUTBOUND 2
#define CLEAR 0
#define ON 0
#define OFF 1

#define LED_PIN 13  //debug
#define trackPowerLED_PIN 7  //debug


// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();

//------------Set up OLED Screen-----
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//-------Declaration for an SSD1306 display - using I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//--RotaryEncoder DEFINEs for numbers of tracks to access with encoder
#define ROTARYSTEPS 1
#define ROTARYMIN   7
#define ROTARYMAX  12





//--- Setup a RotaryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);
byte lastPos = -1;                //--- Last known rotary position.
const int rotarySwitch = 2;      //---Setup Rotary Encoder switch on 
                                 //   pin D2 - active low ----------- 

//------RotaryEncoder Setup and variables are in this section---------
byte tracknumChoice  = ROTARYMAX;
byte tracknumActive  = ROTARYMAX;
byte tracknumDisplay = ROTARYMAX;
byte tracknumLast    = ROTARYMAX;

//Rotary Encoder Switch Variables
byte knobPosition = ROTARYMAX;
bool knobToggle   = true;       //active low 
void readEncoder();           //--RotaryEncoder Function------------------

//---Timer Variables---
const long tortiTimerInterval   = 1000 * 4;
const long trainTimerInterval   = 1000 * 15 * 1;
const long displayTimerInterval = 1000 * 10 * 1;
unsigned long startDisplayTime  = 0;


//---------------------OLED Display Functions------------------//
void bandoText(String text, int x, int y, int size, boolean d);

//---------------SETUP STATE Machine and State Functions----------------------
enum {HOUSEKEEP, STAND_BY, TRACK_SETUP, TRACK_ACTIVE, OCCUPIED,} mode;
void runHOUSEKEEP();
void runSTAND_BY();
void runTRACK_SETUP();
void runTRACK_ACTIVE();
void runOCCUPIED();

void leaveTrack_Setup();
void leaveTrack_Active();


//---State Machine Variables
byte railPower = ON;

//---Sensor variables
byte mainSensTotal      = 0, mainSens_Report = 0; 
byte mainPassByState    = false, mainPassByTotal = 0;
byte mainInValue        = 1, mainIn_LastValue = 1; 
byte mainOutValue       = 1, mainOut_LastValue = 1;
byte main_LastDirection = 0;
byte mainDirection      = 0;

byte revSensTotal       = 0, revSens_Report = 0; 
byte revPassByState     = false; 
byte revPassByTotal     = 0;
byte revInValue         = 1;
byte revIn_LastValue    = 1; 
byte revOutValue        = 1, revOut_LastValue = 1;
byte revDirection       = 0;
byte rev_LastDirection  = 0;

bool entry_ExitBusy = false;
//----end of sensor variables

//DEBUG SECTION

const int leaveTtimer = 8;  // wht wire rev
byte      bailOut = 1;  //active low


//----END DEBUG--------------- //


//---Sensor Function Declarations---------------
void readMainSens();
void readRevSens();
void rptMainDirection();
void rptRevDirection();
void readAllSens();
//--end sensor functions---


//--------------------------------------------------------------//
//                         void setup()                         //
//--------------------------------------------------------------//

void setup() 
{
  Serial.begin(115200);
  tracknumLast = ROTARYMIN;
  
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
  
  //---Setup the button (using external pull-up) :
  pinMode(mainSensInpin, INPUT); pinMode(mainSensOutpin, INPUT);
  pinMode(revSensInpin, INPUT);  pinMode(revSensOutpin, INPUT);

  // After setting up the button, setup the Bounce instances :
  debouncer1.attach(mainSensInpin); debouncer2.attach(mainSensOutpin);
  debouncer3.attach(revSensInpin);  debouncer4.attach(revSensOutpin);
  debouncer1.interval(5);           debouncer2.interval(5); // interval in ms
  debouncer3.interval(5);           debouncer4.interval(5); 

//DEBUG Section - these are manual switches until functions are ready
  //pinMode(mainPassByOff, INPUT_PULLUP);
  pinMode(leaveTtimer, INPUT_PULLUP);
  //pinMode(LED_PIN, OUTPUT);
  pinMode(trackPowerLED_PIN, OUTPUT);
  //----END DEBUG---------------

  encoder.setPosition(ROTARYMIN / ROTARYSTEPS); // start with the value of ROTARYMIN 

  pinMode(rotarySwitch, INPUT_PULLUP);
  //mode = HOUSEKEEP;

  digitalWrite(trackPowerLED_PIN, HIGH);
  display.clearDisplay();
  bandoText("B&O RAIL",25,0,2,false);
  bandoText("JEROEN GARRITSEN'S",8,20,1,true);
  bandoText("McKENZIE",0,33,2,true);
  bandoText("DIVISION",30,50,2,true);
  display.display();
  delay(5000);
  display.clearDisplay();
  digitalWrite(trackPowerLED_PIN, LOW);
  
  
  

  
}  //End setup

//--------------------------------------------------------------//
//                          void loop()                         //
//--------------------------------------------------------------//

void loop() 
{
      if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
      else  digitalWrite(trackPowerLED_PIN, LOW);

      if (mode == HOUSEKEEP)
  {
      runHOUSEKEEP();
  }

  else if (mode == STAND_BY)
  {
      runSTAND_BY();
  }

  else if (mode == TRACK_SETUP)
  {
    runTRACK_SETUP();
  }

  else if (mode == TRACK_ACTIVE)
  {
    runTRACK_ACTIVE();
  }

  else if (mode == OCCUPIED)
  {
    runOCCUPIED();
  }
  
  //----debug terminal print----------------
      
      //Serial.print("mainOutValue: ");
      //Serial.print(mainOutValue);
      //Serial.print("        revOutValue: ");
      //Serial.println(revOutValue);
      //Serial.print("mainSens_Report: ");
      //Serial.print(mainSens_Report);
      //Serial.print("     revSens_Report: ");
      //Serial.println(revSens_Report);
      Serial.print("mainSensTotal:      ");
      Serial.print(mainSensTotal);
      Serial.print("           revSensTotal:  ");
      Serial.println(revSensTotal);
      //Serial.print("mainPassByTotal: ");
      //Serial.print(mainPassByTotal);
      /////Serial.print("     revPassByTotal: ");
      /////Serial.println(revPassByTotal); 
      Serial.print("mainPassByState:    ");
      Serial.print(mainPassByState);
      Serial.print("          revPassByState: ");
      Serial.println(revPassByState); 
      //Serial.print("entryExitBusy: ");
      //Serial.println(entry_ExitBusy); 
      //Serial.print("mainDirection: ");
      //Serial.print(mainDirection);  
      //Serial.print("   revDirection: ");
      //Serial.println(revDirection);  
      Serial.print("main_LastDirection: ");
      Serial.print(main_LastDirection);
      Serial.print("       rev_lastDirection: ");
      Serial.println(rev_LastDirection);
      Serial.print("tracknumActive:    ");
      Serial.print(tracknumActive);
      Serial.print("           tracknumLast: ");
      Serial.println(tracknumLast);
      /*
      Serial.println();
      Serial.println("=======Report Starts Here!=======");
      delay(307);
      //---end debug printing   
      */

}  //  END void loop
 
// ---------------State Machine Functions Section----------------//
//                          BEGINS HERE                          //
//---------------------------------------------------------------//



//--------------------HOUSEKEEP Function-----------------
void runHOUSEKEEP()
{
  display.ssd1306_command(0xAF);  // turn OLED on

  Serial.println();
  Serial.println("-----------------------------------------HOUSEKEEP---");

  if(tracknumLast < ROTARYMAX) railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
  
  //Serial.print("railPower Status: ");
  //Serial.println(railPower);

  tracknumChoice = tracknumLast;

    enum {BufSize=3};  
    char buf[BufSize];
    snprintf (buf, BufSize, "%2d", tracknumLast);
    display.clearDisplay();
    bandoText("SELECT NOW",0,0,2,false);
    bandoText("TRACK",0,20,2,false);
    if(tracknumLast == ROTARYMAX) bandoText("RevL",70,20,2,false);
    else bandoText(buf,80,20,2,false);
    bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
    bandoText("TRACK POWER  -HK-",0,56,1,true);

  
  //Serial.println(tracknumActive);
  //delay(1000);
  mode = STAND_BY;
}  

//-----------------------STAND_BY Function-----------------
void runSTAND_BY()
{
    
    
  Serial.println("-----------------------------------------STAND_BY---");
  do
  {
    readEncoder();
   
    knobToggle = digitalRead(rotarySwitch);
    
    readAllSens();

    if((mainSens_Report > 0) || (revSens_Report > 0))
    {
      Serial.println("---to OCCUPIED from STAND_BY---");
      
      //display.clearDisplay();
      //bandoText("YARD LEAD",0,0,2,false);
      //bandoText("OCCUPIED",0,20,2,true);

      //mode = OCCUPIED;
      runOCCUPIED();
    }
  }
  while (knobToggle == true);    //check rotary switch pressed to select a track (active low)

  knobToggle = true;          //---reset so readEncoder will run in stand_by 
  
  mode = TRACK_SETUP;
} 


//-----------------------TRACK_SETUP- State Function-----------------------
void runTRACK_SETUP()
{
  readAllSens();
  
  railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);

  Serial.println("-----------------------------------------TRACK_SETUP---");
  tracknumActive = tracknumChoice;  
  tracknumLast = tracknumActive;

  display.clearDisplay();
  enum {BufSize=3};  
  char buf[BufSize];
  snprintf (buf, BufSize, "%2d", tracknumActive);
  display.clearDisplay();
  bandoText("ALIGNING",0,0,2,false);
  bandoText("TRACK",0,20,2,false);
  if(tracknumActive == ROTARYMAX) bandoText("RevL",70,20,2,false);
  else bandoText(buf,80,20,2,false);
  bandoText("HAVE A NICE DAY",0,46,1,false);
  bandoText("TRACK POWER  -OFF-",0,56,1,true);
  

  
  
  unsigned long startTortiTime = millis();
  while((millis() - startTortiTime) <= tortiTimerInterval)
  {
   readAllSens();
  }
  railPower = ON;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
  leaveTrack_Setup();
  
}  //---end track setup function-------------------


void leaveTrack_Setup()
{
  Serial.println("---Entering leaveTrack_Setup---");
  readAllSens();
 
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
    Serial.println("---to OCCUPIED from leaveTrack_Setup---");
    mode = OCCUPIED;
  }
  else 
  {
    Serial.println("--times up--leaving TrackSetup--");
    mode = TRACK_ACTIVE;
  }
}


//-----------------------TRACK_ACTIVE State Function------------------
void runTRACK_ACTIVE()
{
  readAllSens();
    display.clearDisplay();
    bandoText("PROCEED ",20,0,2,false);
    bandoText("TIMER ON",0,20,2,false);
    //bandoText("3 MINUTE TIMER -ON-",0,46,1,false);
    bandoText("TRACK POWER  -ON-",0,56,1,true);

  Serial.println("-----------------------------------------TRACK_ACTIVE---");
  rev_LastDirection = 0; //reset for use during the next TRACK_ACTIVE call
  main_LastDirection = 0;
 
  unsigned long startTrainTime = millis();
  do
  {
     readAllSens();
     bailOut = digitalRead(leaveTtimer);
     //---debug     
     Serial.print("main_LastDirection: ");
     Serial.print(rev_LastDirection);
     Serial.println(main_LastDirection);
     Serial.println("-----Waiting for Train to Exit!");
     //debug
     
        //--true when outbound train completely leaves sensor  
    if (((mainPassByState == 1) && (main_LastDirection == 2)) ||
         ((rev_LastDirection == 2) && (revPassByState == 1)))           
    {
      
      break;
      //startTrainTime = startTrainTime + trainTimerInterval;  //adds time to startTrainTime to force end of while loop
    }

    if(bailOut == 0)
    {
      break;
      //startTrainTime = startTrainTime + trainTimerInterval;
    }
    
  }
  while (millis() - startTrainTime <= trainTimerInterval);
  mainPassByState = false;
  revPassByState = false;

  leaveTrack_Active();
 
}  //--end runTrack_Active---


void leaveTrack_Active()
{
  readAllSens();
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
    Serial.println("----to OCCUPIED from leavTrack_Active---");
    mode = OCCUPIED;
  }
  else 
  {
    Serial.println("--times up leaving TrackActive--");
    mode = HOUSEKEEP;
  }
}

//-------------------------OCCUPIED State Function--------------------
void runOCCUPIED()
{
  Serial.println("OCCUPIED");

  display.clearDisplay();
  
  //bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
  //bandoText("TRACK POWER  -ON-",0,56,1,true);
    
  while((mainSens_Report > 0) || (revSens_Report > 0))
  {
    
    readAllSens();
    Serial.println("----to OCCUPIED from OCCUPIED---");
    bandoText("YARD LEAD",0,0,2,false);
    bandoText("OCCUPIED",0,20,2,false);
    bandoText("STOP!",20,42,2,true);
    //mode = OCCUPIED;
  }
   
  Serial.println("----Leaving OCCUPIED---");
  runHOUSEKEEP();
}

//------------------------ReadEncoder Function----------------------

void readEncoder()
{
  /*if((millis() - startDisplayTime) >= (displayTimerInterval))  //display off
    {
      display.ssd1306_command(0xAE);   //turn OLED off  and see line 320
      startDisplayTime = (displayTimerInterval + millis());
    }
  */
  encoder.tick();
  // get the current physical position and calc the logical position
  int newPos = encoder.getPosition() * ROTARYSTEPS;

  if (newPos < ROTARYMIN) {
    encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    newPos = ROTARYMIN;
  } 
  else if (newPos > ROTARYMAX) {
    encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    newPos = ROTARYMAX;
  } 

  if (lastPos != newPos) 
  {
    Serial.print(newPos);
    Serial.println();

    display.ssd1306_command(0xAF);  // turn OLED on
    display.clearDisplay();

    //startDisplayTime = millis();  //display off

    delay(20);

    lastPos = newPos;
    tracknumChoice = newPos;
      
    enum {BufSize=3};  
    char buf[BufSize];
    snprintf (buf, BufSize, "%2d", tracknumChoice);
    display.clearDisplay();
    bandoText("SELECT NOW",0,0,2,false);
    bandoText("TRACK",0,20,2,false);
    if(tracknumChoice == ROTARYMAX) bandoText("RevL",70,20,2,false);
    else bandoText(buf,80,20,2,false);
    bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
    bandoText("TRACK POWER  -OFF-",0,56,1,true);
  }
}     




//---------------------Updating Sensor Functions------------------
//  All in this section update and track sensor information: Busy,
//  Direction, PassBy.  Only the mainOut sensor is documented.  
//  The remaining three work identically.
//------------------------------end of note-----------------------

void readMainSens() {
  debouncer1.update();  
  int mainInValue = debouncer1.read();
    
  if(mainInValue != mainIn_LastValue)     
    {
      if(mainInValue == 0) bitSet(mainSens_Report, 0);
      else bitClear(mainSens_Report, 0); 

      mainIn_LastValue = mainInValue;

      if (mainSens_Report > 0) 
      {
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else mainSensTotal = 0;   
    }
    
  debouncer2.update();  //read mainOut sensor
  int mainOutValue = debouncer2.read();

      //---update history register:*Sens_Report    
  if(mainOutValue != mainOut_LastValue)   
    {
      if(mainOutValue == 0) bitSet(mainSens_Report, 1);
      else bitClear(mainSens_Report, 1); 

      mainOut_LastValue = mainOutValue;

      //---add running total to "*"SensTotal to track PassBy status
      if (mainSens_Report > 0) 
      {
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else mainSensTotal = 0; 
    }
    //---PassByTotal of "6" means train has cleared the sensor success-
    //  fully.  If train were to back out of sensors the sensors would
    //  fire and up the count, the condition would not ever be met.
    //---------end of note------  

    if(mainSensTotal == 0 && mainPassByTotal == 6) 
      {
       mainPassByState = true;
       mainPassByTotal = 0;
      }
    else if(mainSensTotal == 0) mainPassByTotal = 0;

    //--report mainLine Direction
    if((mainSensTotal == 2) && (mainSens_Report == 2)) 
    { 
      mainDirection = 2;
      main_LastDirection = 2;
    }
    else if((mainSensTotal == 1) && (mainSens_Report == 1)) 
    {
      mainDirection = 1;
      main_LastDirection = 1;
    }
    if((mainSensTotal == 0) && (mainSens_Report == 0)) 
    {
     mainDirection = 0;
    } 
    
}  // end readMainSen--

void readRevSens() 
{ 
  //byte revPassByTotal = 0;
  //byte revDirection = 0;

  debouncer3.update();
  int revInValue = debouncer3.read();
    
  if(revInValue != revIn_LastValue)     
  {
    if(revInValue == 0) bitSet(revSens_Report, 0);
    else bitClear(revSens_Report, 0); 

    revIn_LastValue = revInValue;

     if (revSens_Report > 0) 
    {
      revSensTotal = revSensTotal + revSens_Report;
      revPassByTotal = revSensTotal;
    }
    else revSensTotal = 0;   
  }
      
  debouncer4.update();
  int revOutValue = debouncer4.read();
    
  if(revOutValue != revOut_LastValue)     
  {
    if(revOutValue == 0) bitSet(revSens_Report, 1);
    else bitClear(revSens_Report, 1); 

    revOut_LastValue = revOutValue;

    if (revSens_Report > 0) 
    {
      revSensTotal = revSensTotal + revSens_Report;
      revPassByTotal = revSensTotal;
    }
    else revSensTotal = 0; 
  }
    if(revSensTotal == 0 && revPassByTotal == 6) 
    {
      revPassByState = true;
      revPassByTotal = 0;
    }
    else if(revSensTotal == 0) revPassByTotal = 0;

    //--report revLoop Direction
    if((revSensTotal == 2) && (revSens_Report == 2)) 
    {
      revDirection = 2;
      rev_LastDirection = 2;
    }
    else if((revSensTotal == 1) && (revSens_Report == 1)) 
    {
      revDirection = 1;
      rev_LastDirection = 1;
    }
    if((revSensTotal == 0) && (revSens_Report == 0)) 
    {
      revDirection = 0;
    }

}  // end readrevSen--

  
void readAllSens() 
  {
    readMainSens();
    readRevSens();
  }   

// ------------------Display Functions Section-------------------//
//                          BEGINS HERE                          //
//---------------------------------------------------------------//

void bandoText(String text, int x, int y, int size, boolean d){
  display.setTextSize(size);
  display.setTextColor(WHITE);
  display.setCursor(x,y);
  display.println(text);
  if(d){
    display.display();
  }
}

//--------------------------------------------------

