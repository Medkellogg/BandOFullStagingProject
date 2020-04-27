/* 
Throat and REVLOOP sensor module
by Mark Kellogg 4/23/2020
 */

#include <Arduino.h>

//---------------------Debounce Routine Setup and explanation---------------------------
// Buttons with Pull-Ups are "backwards"
// Some DEFINEs to make reading code easier
#define PUSHED false
#define NOT_PUSHED true
#define WATCH_BUTTON true
#define IGNORE_BUTTON false

//--sensorInfo to make info easier to read 
#define INBOUND true    
#define OUTBOUND false

//--sensorInfo variables
byte mainSens_Report = 0;   //Bit 1 is written hi when mainOutValue is true 
byte mainSens_LastReport = 0;
byte mainSensTotal = 0;     //Totals value of mainSens_Report register 
byte mainPassByTotal = 0;
byte mainPassByState = false;
byte mainInValue = 1;  //---Sensors report active low so initial setting is 1
byte mainIn_LastValue = 1;
byte mainOutValue = 1;
byte mainOut_LastValue = 1;
boolean throatDirection = INBOUND;
boolean revloopDirction = INBOUND;
boolean sensorsBusy = false;
boolean sensorsPassedBy = false;


 
// Time to wait for bounce, in MICROsconds
const int sensorWaitInterval = 6000;
// Pins for LED and Button
const int LEDpin = 13;
const int mainSensInpin = 11, mainSensOutpin = 12, revSensInpin = 9, revSensOutpin = 10;
 
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
void updateRevInSens();
void updateRevOutSens();

void readMainSens();
void readMainOutSens();

//DEBUG SECTION
const int passByOff = 6;
int passByToZero = 1;
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
  pinMode(passByOff, INPUT_PULLUP);
  //----END DEBUG---------------

}

//------------------------------Void Loop------------------------------------


void loop() 
{
    readMainSens();

    //DEBUG but still needs work for post sensor time once code is combined
    /*passByToZero = digitalRead(passByOff);

    if(passByToZero == 0){
       mainPassByState = 0;
      }
    
     if(mainPassByState == 0 && mainSensTotal == 0) mainPassByTotal = 0; 
    */
    //---DEBUG - all print routines here are debug
    Serial.print("sensorReportOUT: ");   
    Serial.println(mainSens_Report);
    Serial.print("mainSensTotal: ");   
    Serial.println(mainSensTotal);
    Serial.print("mainPassByTotal: ");  
    Serial.println(mainPassByTotal);
    Serial.print("mainPassByState: ");  
    Serial.println(mainPassByState);
    Serial.print("passbyToZero: ");  
    Serial.println(passByToZero);
    
    Serial.println();

    
    delay(500);
    //--END DEBUG----------

}
 
// ------------------------------------Main INBOUND Sensor Function----------------------------------
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
      
        //Serial.print("MainOut: "); //debug
        //Serial.println(debouncedMainOutState); //debug

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
      
        //Serial.print("RevIn: "); //debug
        //Serial.println(debouncedRevInState); //debug
                
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
      
        //Serial.print("RevOut: "); //debug
        //Serial.println(debouncedRevOutState); //debug
        //delay(500);

        

        bounceRevOutState = WATCH_BUTTON;
      }
    }
}

  //-------------------------Function--readMainInsens-------------------

  void readMainSens()
  {
    
    /*--THIS CODE NEEDS TO BE RELOCATED INTO TRACK_ACTIVE OR HOUSEKEEP ONCE BRANCHES
    ARE COMBINED--*/
    passByToZero = digitalRead(passByOff);

    if(passByToZero == 0){
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
  }



 