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
byte sensorReport = 0;
boolean direction = INBOUND;
boolean busy = false;
boolean passedBy = false;


 
// Time to wait for bounce, in MICROsconds
const int sensorWaitInterval = 6000;
// Pins for LED and Button
const int LEDpin = 13;
const int mainSensInpin = 7, mainSensOutpin = 8, revSensInpin = 9, revSensOutpin = 12;
 
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

 
// ---------------------------Void Setup------------------------------------
void setup() {
  pinMode(LEDpin, OUTPUT);
  pinMode(mainSensInpin, INPUT_PULLUP);
  pinMode(mainSensOutpin, INPUT_PULLUP);
  pinMode(revSensInpin, INPUT_PULLUP);
  pinMode(revSensOutpin, INPUT_PULLUP);
  Serial.begin(115200);
}

//------------------------------Void Loop------------------------------------


void loop() 
{
  // This needs to be called periodically to
  // update the timers and button status
  
  updateMainOutSens();
  digitalWrite(mainSensOutpin, debouncedMainOutState);

  updateMainInSens();
  // This replaces: digitalRead(BUTTONpin);
  digitalWrite(mainSensInpin, debouncedMainInState);

  //updateRevInSens();
  //digitalWrite(revSensInpin, debouncedRevInState);

  //updateRevOutSens();
  //digitalWrite(revSensOutpin, debouncedRevOutState);

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

      Serial.print("MainIN: ");
      Serial.println(debouncedMainInState);
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
      
        Serial.print("MainOut: "); //debug
        Serial.println(debouncedMainOutState); //debug

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
      
        Serial.print("RevIn: "); //debug
        Serial.println(debouncedRevInState); //debug

        bounceRevInState = WATCH_BUTTON;
      }
    }
}  

//-----------------------Reverse Loop OUTBOUND Sensor Function---------------------------------------------
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
      
        Serial.print("RevOut: "); //debug
        Serial.println(debouncedRevOutState); //debug

        bounceRevOutState = WATCH_BUTTON;
      }
    }
}  