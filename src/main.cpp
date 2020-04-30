/* 
Throat and reverse loop sensor module
by Mark Kellogg 4/23/2020
---------------------------Github Repository------------------------------
NAME:
The repository has all code etc. along with flow control overview and 
module graphics.  

----------------------------Module Description-----------------------------
All four staging yards have a single yard lead, off of which all 
the dead end staging tracks fan out.  The yard lead of three of the four yards
continue on to a reverse loop.

 Sensors - The yard lead entry turnout and reverse loop turnout each have a 
 sensor pair to track entry and exits from that point.

 Sensor Names - Sensors are named "mainSens" for the yard throat, and 
 "revSens" for the reverse loop leadout.

 Direction Naming Convention - In all cases you may think of a point "between"
 the yard lead turnout and the reverse loop turnout as the center of the
 universe.  Therefore INBOUND is always towards that point on either sensor
 and OUTBOUND obviously the reverse.   


 Module Output- Each sensor pair returns: 
   Train Direction- mainDirection or revDirection: their output(s) are 
   INBOUND, OUTBOUND, or CLEAR; and are active when the train is within the small sensor area.
   
   Last Train Direction = main_LastDirection or rev_LastDirection with the same
   output(s) as above, which stays true until the next train activates the
   sensor pair.

   Train PassBy - which senses the train has passed by the sensor completely in the direction it arrived from.  If a train backs out without completely
   passing by PassBy will not report true. It stays active as long as the 
   train is within the sensor pair.

   Sensor Busy - Sensor reports busy when either of the sensor pair is true
   and remains so only until the both go false.
   */

#include <Arduino.h>

//---------------------Debounce Routine Setup and explanation---------------------------
// Buttons with Pull-Ups are "backwards"
// Some DEFINEs to make reading code easier
#define PUSHED false
#define NOT_PUSHED true
#define WATCH_BUTTON true
#define IGNORE_BUTTON false

//--sensorInfo to make code easier to read 
#define INBOUND 1    
#define OUTBOUND 2
#define CLEAR 0 

//-----Main sensorInfo variables-------
byte mainSens_Report = 0;   //Bit 1 is written hi when mainOutValue is true 
byte mainSens_LastReport = 0;
byte mainSensTotal = 0;     //Totals value of mainSens_Report register 
byte mainPassByTotal = 0;
byte mainPassByState = false;
byte mainInValue = 1;  //---Sensors report active low so initial setting is 1
byte mainIn_LastValue = 1;
byte mainOutValue = 1;
byte mainOut_LastValue = 1;

byte mainDirection = 0;
byte main_LastDirection = 0;
boolean mainSensorsBusy = false;

//-----revLoop sensorInfo variables-------
byte revSens_Report = 0;   //Bit 1 is written hi when mainOutValue is true 
byte revSens_LastReport = 0;
byte revSensTotal = 0;     //Totals value of mainSens_Report register 
byte revPassByTotal = 0;
byte revPassByState = false;
byte revInValue = 1;  //---Sensors report active low so initial setting is 1
byte revIn_LastValue = 1;
byte revOutValue = 1;
byte revOut_LastValue = 1;

byte revDirection = 0;
byte rev_LastDirection = 0;
boolean revSensorsBusy = false;



 
// Time to wait for bounce, in MICROsconds
const int sensorWaitInterval = 6000;
// Pins for LED and Button
const int LEDpin = 13;
const int mainSensInpin = 11, mainSensOutpin = 12, revSensInpin =10, revSensOutpin = 9;
 
// Used to track how long between "bounces"
unsigned long previousmicrosMainIn = 0, previousmicrosMainOut = 0; 
unsigned long previousmicrosRevIn = 0, previousmicrosRevOut = 0; 
 
// Used to track state of button (high or low)
boolean previousMainInState = NOT_PUSHED, previousMainOutState = NOT_PUSHED;
boolean previousRevInState = NOT_PUSHED, previousRevOutState = NOT_PUSHED;
 
// Variable reporting de-bounced state.
boolean debouncedMainInState = NOT_PUSHED, debouncedMainOutState = NOT_PUSHED;
boolean debouncedRevInState = NOT_PUSHED, debouncedRevOutState = NOT_PUSHED;

 
// Tracks if we are waiting for a "bounce" event
boolean bounceMainInState = false, bounceMainOutState = false;
boolean bounceRevInState = false, bounceRevOutState = false;


//--------------Functions for each sensor-------------------------
void updateMainInSens();
void updateMainOutSens();
void readMainSens();
void rptMainDirection();
void rptMainSensActive();

void updateRevInSens();
void updateRevOutSens();
void readRevSens();
void rptRevDirection();
void rptRevSensActive();



//DEBUG SECTION
const int mainPassByOff = 7;  // green wire main
int mainPassByToZero = 1;
const int revPassByOff = 8;  // wht wire rev
int revPassByToZero = 1;
//----END DEBUG---------------
 
// ---------------------------Void Setup------------------------------------
void setup() {
  pinMode(LEDpin, OUTPUT);
  pinMode(mainSensInpin, INPUT);
  pinMode(mainSensOutpin, INPUT);
  pinMode(revSensInpin, INPUT);
  pinMode(revSensOutpin, INPUT);

  //DEBUG Section
  Serial.begin(115200);
  pinMode(mainPassByOff, INPUT_PULLUP);
  pinMode(revPassByOff, INPUT_PULLUP);

  //----END DEBUG---------------

}

//------------------------------Void Loop------------------------------------


void loop() 
{
    
    readMainSens();
    readRevSens();
    rptMainSensActive();
    rptRevSensActive();
    
    
    
    //*---DEBUG - all print routines here are debug
    Serial.print("mainsensorReport: ");   
    Serial.print(mainSens_Report);
    Serial.print("     revSensorReport: ");   
    Serial.println(revSens_Report);

    Serial.print("mainSensTotal: ");   
    Serial.print(mainSensTotal);
    Serial.print("        revSensTotal: ");   
    Serial.println(revSensTotal);

    Serial.print("mainPassByTotal: ");  
    Serial.print(mainPassByTotal);
    Serial.print("      revPassByTotal: ");  
    Serial.println(revPassByTotal);

    Serial.print("mainPassByState: ");  
    Serial.print(mainPassByState);
    Serial.print("      revPassByState: ");  
    Serial.println(revPassByState);

    Serial.print("MainPassbyToZero: ");  
    Serial.print(mainPassByToZero);
    Serial.print("     revPassbyToZero: ");  
    Serial.println(revPassByToZero);
    Serial.println();

    Serial.print("mainSensorBusy: ");  
    Serial.print(mainSensorsBusy);
    Serial.print("       revSensorBusy: ");  
    Serial.println(revSensorsBusy);
    Serial.println();

    Serial.print("MainDirection: ");
    Serial.print(mainDirection);
    Serial.print("        revDirection: ");
    Serial.println(revDirection);

    Serial.print("main_LastDirection: ");
    Serial.print(main_LastDirection);
    Serial.print("   rev_LastDirection: ");
    Serial.println(rev_LastDirection);
    Serial.println("------------------------------------------------------");
    Serial.println();

    delay(500); 

    //--END DEBUG-----------------------------------------*/

}
 
// ---------------Main INBOUND Sensor Function------------------
void updateMainInSens() {
  // We are waiting for any activity on the button
  if (bounceMainInState == WATCH_BUTTON) {
    // Get and store current button state
    boolean currentMainInState = digitalRead(mainSensInpin);
    // Check to see if a transition has occured (and only one)
    if (previousMainInState != currentMainInState) {
      // A transition was detected, ignore the others for a while
      bounceMainInState = IGNORE_BUTTON;
      // Store current time (start the clock)
      previousmicrosMainIn = micros();
    }
    // Keep storing existing button state, if we're watching
    previousMainInState = currentMainInState;
  }
  // We are waiting for the buttonWaitInterval to elapse
  if (bounceMainInState == IGNORE_BUTTON) {
    // Compare current value of micros to previously stored, enough time yet?
    unsigned long currentMicrosMainIn = micros();
    if ((unsigned long)(currentMicrosMainIn - previousmicrosMainIn) >= sensorWaitInterval) {
      // Store the state of the button/pin to debouncedButtonState, which "reports"
      // the correct value. This allows for the code to handle active high or low inputs
      debouncedMainInState = digitalRead(mainSensInpin);
      //Serial.print("MainIN: ");
      //Serial.println(debouncedMainInState);
            // Go back to watching the button again.
      bounceMainInState = WATCH_BUTTON;
    }
  }
}
//-----------------------Main OUTBOUND Sensor Function---------------------------------------------
void updateMainOutSens() 
{  
    if (bounceMainOutState == WATCH_BUTTON) 
    {
      boolean currentMainOutState = digitalRead(mainSensOutpin);
      if (previousMainOutState != currentMainOutState) 
      {
        bounceMainOutState = IGNORE_BUTTON;
        previousmicrosMainOut = micros();
      }
      previousMainOutState = currentMainOutState;
    }
    if (bounceMainOutState == IGNORE_BUTTON) 
    {
      unsigned long currentMicrosMainOut = micros();
      if ((unsigned long)(currentMicrosMainOut - previousmicrosMainOut) >= sensorWaitInterval) 
      {
        debouncedMainOutState = digitalRead(mainSensOutpin);
        bounceMainOutState = WATCH_BUTTON;
      }
    }
} 

//-----------------------Reverse Loop INBOUND Sensor Function---------------------------------------------
void updateRevInSens() 
{
  if (bounceRevInState == WATCH_BUTTON) 
    {
      boolean currentRevInState = digitalRead(revSensInpin);
      if (previousRevInState != currentRevInState) 
      {
        bounceRevInState = IGNORE_BUTTON;
        previousmicrosRevIn = micros();
      }
      previousRevInState = currentRevInState;
    }
  if (bounceRevInState == IGNORE_BUTTON) 
    {
    unsigned long currentMicrosRevIn = micros();
      if ((unsigned long)(currentMicrosRevIn - previousmicrosRevIn) >= sensorWaitInterval) 
      {
        debouncedRevInState = digitalRead(revSensInpin);
        bounceRevInState = WATCH_BUTTON;
      }
    }
}  

//-----------------------Reverse Loop OUTBOUND Sensor Function---------------------------------
void updateRevOutSens() 
{
    if (bounceRevOutState == WATCH_BUTTON) 
    {
      boolean currentRevOutState = digitalRead(revSensOutpin);
      if (previousRevOutState != currentRevOutState) 
      {
        bounceRevOutState = IGNORE_BUTTON;
        previousmicrosRevOut = micros();
      }
      previousRevOutState = currentRevOutState;
    }
    if (bounceRevOutState == IGNORE_BUTTON) 
    {
      unsigned long currentMicrosRevOut = micros();
      if ((unsigned long)(currentMicrosRevOut - previousmicrosRevOut) >= sensorWaitInterval) 
      {
        debouncedRevOutState = digitalRead(revSensOutpin);
        bounceRevOutState = WATCH_BUTTON;
      }
    }
}

  //-------------------------Function--readMainSens-------------------

  void readMainSens()
  { 
    /*--THIS CODE NEEDS TO BE RELOCATED INTO TRACK_ACTIVE OR HOUSEKEEP ONCE BRANCHES
    ARE COMBINED--see notes in readRevSens for complete info*/
    mainPassByToZero = digitalRead(mainPassByOff);

    if(mainPassByToZero == 0){
       mainPassByState = 0;
      }
     //----- End of code to be relocated

     //--beginning of function 
    //--------Reset Totals after train enters sensor zone but backs out-----
     if(mainPassByState == 0 && mainSensTotal == 0) mainPassByTotal = 0; 
    
    updateMainInSens();
    digitalWrite(mainSensInpin, debouncedMainInState);
    mainInValue = debouncedMainInState;

    if(mainInValue != mainIn_LastValue)
    {
      if(mainInValue == 0) { 
        bitSet(mainSens_Report, 0);
      }
      else{
        bitClear(mainSens_Report, 0);
      }
     mainIn_LastValue = mainInValue;
    
      if(mainSens_Report > 0) {
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else{
        mainSensTotal = 0;
      }
    } 
    updateMainOutSens();
    digitalWrite(mainSensOutpin, debouncedMainOutState);
    mainOutValue = debouncedMainOutState;

    if(mainOutValue != mainOut_LastValue)
    {
      if(mainOutValue == 0) { 
        bitSet(mainSens_Report, 1);
      }
      else{
        bitClear(mainSens_Report, 1);
      }
      mainOut_LastValue = mainOutValue;

      if(mainSens_Report > 0) {    //This 
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else{
        mainSensTotal = 0;
      }
    }

    if(mainSensTotal == 0 && mainPassByTotal == 6) {
       mainPassByState = true;
       mainPassByTotal = 0;
    }
    rptMainDirection();
  }


 //-------------------------Function--readRevSens-------------------

  void readRevSens()
  {
    /*--THIS CODE NEEDS TO BE RELOCATED INTO TRACK_ACTIVE OR HOUSEKEEP ONCE BRANCHES
    ARE COMBINED--*/
    revPassByToZero = digitalRead(revPassByOff); // read switch on breadboard to clear revPassByState
    if(revPassByToZero == 0){   //switch on breadboard
       revPassByState = 0;   //placed in function where sensers are read
      }
     //----- End of code to be relocated

     //--beginning of function 
    //--------Reset Totals after train enters sensor zone but backs out-----
     if(revPassByState == 0 && revSensTotal == 0) revPassByTotal = 0; 
    
    updateRevInSens();
    digitalWrite(revSensInpin, debouncedRevInState);
    revInValue = debouncedRevInState;

    if(revInValue != revIn_LastValue)
    {
      if(revInValue == 0) { 
        bitSet(revSens_Report, 0);
      }
      else{
        bitClear(revSens_Report, 0);
      }
     revIn_LastValue = revInValue;
    
      if(revSens_Report > 0) {
        revSensTotal = revSensTotal + revSens_Report;
        revPassByTotal = revSensTotal;
      }
      else{
        revSensTotal = 0;
      }
    }

    updateRevOutSens();
    digitalWrite(revSensOutpin, debouncedRevOutState);
    revOutValue = debouncedRevOutState;

    if(revOutValue != revOut_LastValue)
    {
      if(revOutValue == 0) { 
        bitSet(revSens_Report, 1);
      }
      else{
        bitClear(revSens_Report, 1);
      }
      revOut_LastValue = revOutValue;

      if(revSens_Report > 0) {    //This 
        revSensTotal = revSensTotal + revSens_Report;
        revPassByTotal = revSensTotal;
      }
      else{
        revSensTotal = 0;
      }
    }
    if(revSensTotal == 0 && revPassByTotal == 6) {
       revPassByState = true;
       revPassByTotal = 0;
    }
    rptRevDirection();
  }

  /*-----------------FUNCTION rptMainDirection()---------------------

    This function reports train direction at each sensor while it's 
    moving through the sensor.

    Store a second variable: last_MainDirection, to indicate the 
    direction of the last train to hit the sensors. NOTE: this will
    record and remember the direction of a train that hits the sensor
    but backs out before completely passing by.  The next train to hit
    either in or out sensor will start the routine again.
  ------------------------------------------------------------------*/

  void rptMainDirection() 
  {
    if ((mainSensTotal == OUTBOUND) && (mainSens_Report = OUTBOUND)) {
      mainDirection = OUTBOUND;
      main_LastDirection = mainDirection;
      }
    else if ((mainSensTotal == INBOUND) && (mainSens_Report = INBOUND)) {
      mainDirection = INBOUND;
      main_LastDirection = mainDirection;
      }
    else if ((mainDirection == OUTBOUND) && (mainSensTotal > OUTBOUND)) {
      mainDirection = OUTBOUND;
      main_LastDirection = mainDirection;
      }
    else if ((mainDirection == INBOUND) && (mainSensTotal > CLEAR)) {
      mainDirection = INBOUND;
      main_LastDirection = mainDirection;
      }
    else if ((mainDirection > CLEAR) && (mainSens_Report == CLEAR)) mainDirection = CLEAR;
  }  //End function

/*-----------------FUNCTION rptRevDirection()--------------------------
    This function is the same as above, but reports on the Reverse Loop.
 ---------------------------------------------------------------------*/

  void rptRevDirection() 
  {
    if ((revSensTotal == OUTBOUND) && (revSens_Report = OUTBOUND)) {
      revDirection = OUTBOUND;
      rev_LastDirection = revDirection;
      }
    else if ((revSensTotal == INBOUND) && (revSens_Report = INBOUND)) {
      revDirection = INBOUND;
      rev_LastDirection = revDirection;
      }
    else if ((revDirection == OUTBOUND) && (revSensTotal > OUTBOUND)) {
      revDirection = OUTBOUND;
      rev_LastDirection = revDirection;
      }
    else if ((revDirection == INBOUND) && (revSensTotal > CLEAR)) {
      revDirection = INBOUND;
      rev_LastDirection = revDirection;
      }
    else if ((revDirection > CLEAR) && (revSens_Report == CLEAR)) revDirection = CLEAR;
  }   //End function 

/*----------------------FUNCTIONS rptMainSensActive()------------------------
                                  rptRevSensActive()
    These two functions report any presense of trains at the specific 
    sensor pair, one at the yard throat ("main"), or the other at the reverse
    loop ("rev").  Unlike other sensor functions they are called separately, not within the main readMainSens or readRevSens function calls.
 --------------------------------------------------------------------------*/

void rptMainSensActive()
{
  if (mainSens_Report > 0) mainSensorsBusy = true;
  else { mainSensorsBusy = false; }
}


void rptRevSensActive()
{
  if (revSens_Report > 0)revSensorsBusy = true;
  else { revSensorsBusy = false; }
}
   

 