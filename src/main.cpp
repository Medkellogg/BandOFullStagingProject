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

// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();

//--RotaryEncoder DEFINEs for numbers of tracks to access with encoder
#define ROTARYSTEPS 1
#define ROTARYMIN 7
#define ROTARYMAX 12





//--- Setup a RotaryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);
int lastPos = -1;                //--- Last known rotary position.
const int rotarySwitch = 2;      //---Setup Rotary Encoder switch on 
                                 //   pin D2 - active low ----------- 

//------RotaryEncoder Setup and variables are in this section---------
int tracknumChoice =  ROTARYMAX;
int tracknumActive =  ROTARYMAX;
int tracknumDisplay =  ROTARYMAX;
int tracknumLast =  ROTARYMAX;

//Rotary Encoder Switch Variables
int knobPosition = ROTARYMAX;
bool knobToggle = true;       //active low 
void readEncoder();           //--RotaryEncoder Function------------------

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
byte mainSensTotal = 0, mainSens_Report = 0; 
byte mainPassByState = false, mainPassByTotal = 0;
byte mainInValue = 1, mainIn_LastValue = 1; 
byte mainOutValue = 1, mainOut_LastValue = 1;
byte main_LastDirection = 0;
byte mainLineDirection = 0;
byte getMainDirection = 0;
byte mainDirection = 0;
bool mainOutbound = 0;
bool mainInbound = 0;

byte revSensTotal = 0, revSens_Report = 0; 
byte revPassByState = false, revPassByTotal = 0;
byte revInValue = 1, revIn_LastValue = 1; 
byte revOutValue = 1, revOut_LastValue = 1;
byte revLoopDirection = 0;
byte rev_LastDirection = 0;
byte getRevLoopDirection = 0;
byte revDirection = 0;
bool revOutbound = 0;
bool revInbound = 0;

bool entry_ExitBusy = false;
//----end of sensor variables

//DEBUG SECTION
const int mainPassByOff = 7;  // green wire main
int mainPassByToZero = 1;
const int revPassByOff = 8;  // wht wire rev
int revPassByToZero = 1;
//----END DEBUG---------------


//---Sensor Function Declarations---------------
int readMainSens();
int readRevSens();
byte rptMainDirection();
byte rptRevDirection();
bool readAllSens();
//--end sensor functions---


// ---------------------------Void Setup------------------------------------
void setup() {
  
  //---Setup the button (using external pull-up) :
  pinMode(mainSensInpin, INPUT); pinMode(mainSensOutpin, INPUT);
  pinMode(revSensInpin, INPUT);  pinMode(revSensOutpin, INPUT);

  // After setting up the button, setup the Bounce instances :
  debouncer1.attach(mainSensInpin); debouncer2.attach(mainSensOutpin);
  debouncer3.attach(revSensInpin);  debouncer4.attach(revSensOutpin);
  debouncer1.interval(5); debouncer2.interval(5); // interval in ms
  debouncer3.interval(5); debouncer4.interval(5); 

//DEBUG Section - these are manual switches until functions are ready
  pinMode(mainPassByOff, INPUT_PULLUP);
  pinMode(revPassByOff, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  //----END DEBUG---------------

  encoder.setPosition(ROTARYMIN / ROTARYSTEPS); // start with the value of ROTARYMIN 

  pinMode(rotarySwitch, INPUT_PULLUP);
  mode = HOUSEKEEP;

  Serial.begin(115200);
  Serial.println("---ANOTHER INOVATIVE PRODUCT FROM THE B&O McKENZIE DIVISION---");
  
}  //End setup

//------------------------------Void Loop-----------------------


void loop() 
{
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
  

  /*---debug-Check new sensor module
  entry_ExitBusy = readAllSens();
  getMainDirection = rptMainDirection();
  getRevLoopDirection = rptRevDirection();
  
  mainPassByToZero = digitalRead(mainPassByOff);  //debug

    if(mainPassByToZero == 0){
       mainPassByState = 0;
      }

  revPassByToZero = digitalRead(revPassByOff);  //debug

    if(revPassByToZero == 0){
       revPassByState = 0;
      }  
    */
  //----debug terminal print----------------
      /*
      Serial.print("mainOutValue: ");
      Serial.print(mainOutValue);
      Serial.print("        revOutValue: ");
      Serial.println(revOutValue);
      Serial.print("mainSens_Report: ");
      Serial.print(mainSens_Report);
      Serial.print("     revSens_Report: ");
      Serial.println(revSens_Report);
      Serial.print("mainSensTotal: ");
      Serial.print(mainSensTotal);
      Serial.print("       revSensTotal: ");
      Serial.println(revSensTotal);
      Serial.print("mainPassByTotal: ");
      Serial.print(mainPassByTotal);
      Serial.print("     revPassByTotal: ");
      Serial.println(revPassByTotal); 
      Serial.print("mainPassByState: ");
      Serial.print(mainPassByState);
      Serial.print("     revPassByState: ");
      Serial.println(revPassByState);
      Serial.print("entryExitBusy: ");
      Serial.println(entry_ExitBusy); 
      Serial.print("getLineDirection: ");
      Serial.print(getMainDirection);  
      Serial.print("   revLoopDirection: ");
      Serial.println(getRevLoopDirection);  
      Serial.print("main_LastDirection: ");
      Serial.print(main_LastDirection);
      Serial.print("   rev_lastDirection: ");
      Serial.println(rev_LastDirection);
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
  
  Serial.println();
  Serial.println("HOUSEKEEP-------------------------");

  railPower = OFF;
  //Serial.print("railPower Status: ");
  //Serial.println(railPower);

  tracknumChoice = tracknumActive;
  //Serial.print("Make Active track the display track: ");
  //Serial.println(tracknumActive);
  //delay(1000);

  //runSTAND_BY();
  mode = STAND_BY;
}  


//-----------------------STAND_BY Function-----------------
void runSTAND_BY()
{
  Serial.println("-----STAND_BY");

  
  
do
  {
    
    readEncoder();
   
    knobToggle = digitalRead(rotarySwitch);
    
    mode = STAND_BY;

  } while (knobToggle == true);    //check rotary switch pressed to select a track (active low)

  knobToggle = true;               //reset so readEncoder will run in stand_by next pass

  

  mode = TRACK_SETUP;
  
  
} 


//-----------------------TRACK_SETUP Functions-----------------------
void runTRACK_SETUP()
{
  //entry_ExitBusy = readAllSens();
  
  Serial.println("----------TRACK_SETUP--------");
  Serial.print("entry_ExitBusy: ");
  Serial.println(entry_ExitBusy);

  delay(3000);
    
  
  Serial.print("entry_ExitBusy: ");
  Serial.println(entry_ExitBusy);

  leaveTrack_Setup();
 

}  //---end track setup function-------------------


void leaveTrack_Setup()
{
  if( entry_ExitBusy == true)
  
  {
    Serial.println("----to OCCUPIED from Setup---");
    //mode = OCCUPIED;
    runOCCUPIED();
  }
  else 
  {
    Serial.println("--times up going on--");

    mode = TRACK_ACTIVE;
    //runTRACK_ACTIVE();
  }
  
}


//-----------------------TRACK_ACTIVE Function------------------
void runTRACK_ACTIVE()
{
  entry_ExitBusy = readAllSens();
  Serial.println("---------------TRACK_ACTIVE-----");
  Serial.print("entry_ExitBusy: ");
  Serial.println(entry_ExitBusy);

  
  

  delay(10000);

  //TODO check for knobToggle - back to Standby if true
  //Serial.println("Check Click: ");

  leaveTrack_Active();

  //mode = HOUSEKEEP;
  //runHOUSEKEEP()

}  //--end runTrack_Active---

void leaveTrack_Active()
{
  if( entry_ExitBusy == true)
  {
    Serial.println("----to OCCUPIED from Active---");
    mode = OCCUPIED;
    
  }
  else 
  {
    Serial.println("--times up, going on from Track_Active----");
    mode = HOUSEKEEP;
    runHOUSEKEEP();
  }
}

//-------------------------OCCUPIED Function--------------------
void runOCCUPIED()
{
  Serial.println("OCCUPIED");
  //delay(20);

  entry_ExitBusy = readAllSens();
  Serial.print("entry_ExitBusy: ");
  Serial.println(entry_ExitBusy);

  if( entry_ExitBusy == true)
  {
    Serial.println("----to OCCUPIED from OCCUPIED---");
    //mode = OCCUPIED;
    runOCCUPIED();
  }
  else 
  {    
   Serial.println("-------LEAVING OCCUPIED----------------");
   runSTAND_BY();
  }
  
  
     
}

//------------------------ReadEncoder Function----------------------

void readEncoder()
{
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
    } // if

    if (lastPos != newPos) {
      Serial.print(newPos);
      Serial.println();
      lastPos = newPos;
      tracknumChoice = newPos;
    }
}     




//---------------------Updating Sensor Functions------------------
//  All in this section update and track sensor information: Busy,
//  Direction, PassBy.  Only the mainOut sensor is documented.  
//  The remaining three work identically.
//------------------------------end of note-----------------------

int readMainSens() {
  debouncer1.update();  
  int mainInValue = debouncer1.read();
    
  if(mainInValue != mainIn_LastValue)     
    {
      if(mainInValue == 0) bitSet(mainSens_Report, 0);
      else bitClear(mainSens_Report, 0); 

      mainIn_LastValue = mainInValue;

      if (mainSens_Report > 0) {
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
      if (mainSens_Report > 0) {
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
       
  return mainSensTotal;
}  // end readMainSen--

int readRevSens() 
{  
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

  return revSensTotal;
}  // end readrevSen--

  
bool readAllSens() {
  bool sensBusy;
  int sensMainBusy = 0;
  int sensRevBusy = 0;

  sensMainBusy = readMainSens();
  sensRevBusy = readRevSens();

  if(sensMainBusy > 0 || sensRevBusy >0){
    sensBusy = true;
  }
  else sensBusy = false;

  return sensBusy;
}


//--------------------------------------------------

byte rptMainDirection() 
  {      
    if((mainSensTotal == 2) && (mainSens_Report = 2)) 
    {
      mainOutbound = 1;
      mainInbound = 0;
    }
    else if((mainSensTotal == 1) && (mainSens_Report = 1)) 
    {
      mainOutbound = 0;
      mainInbound = 1;
    }

    if((mainDirection == 0) && (mainSensTotal == 0)) 
    {
      mainOutbound = 0;
      mainInbound = 0;
    } 
    
    if(mainOutbound == 1)
    {
      mainLineDirection = 2;
      main_LastDirection = 2;
    }
    if(mainInbound == 1)
    {
      mainLineDirection = 1;
      main_LastDirection = 1;
    }
    if((mainOutbound == 0) && (mainInbound == 0)){
     mainLineDirection = 0;
    }

    return mainLineDirection;
  }  //End function

//-----------------FUNCTION rptRevDirection()---------------------------
//  This function is the same as above, but reports on the Reverse Loop.
//----------------------------------------------------------------------

  byte rptRevDirection() 
  {
    if((revSensTotal == 2) && (revSens_Report = 2)) {
      revOutbound = 1;
      revInbound = 0;
      }
    else if((revSensTotal == 1) && (revSens_Report = 1)) {
      revOutbound = 0;
      revInbound = 1;
      }
    if((revDirection == 0) && (revSensTotal == 0)) {
      revOutbound = 0;
      revInbound = 0;
      }
    
    if(revOutbound == 1){
      revLoopDirection = 2;
      rev_LastDirection = 2;
    }
    if(revInbound == 1){
      revLoopDirection = 1;
      rev_LastDirection = 1;
    }
    if((revOutbound == 0) && (revInbound == 0)){
     revLoopDirection = 0;
    }

    return revLoopDirection;
  }  //End function   