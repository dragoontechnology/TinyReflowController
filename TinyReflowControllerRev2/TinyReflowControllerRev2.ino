/**

  Required Libraries
  ==================
  - Adafruit MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

  Revision  Description
  ========  ===========
  2.00      Support V2 of the Tiny Reflow Controller:
            - select the arduino board “Tools” -> “Board” -> “Arduino Pro or Pro Mini”
            - Based on ATMega328P 3.3V @ 8MHz
            - Uses SSD1306 128x64 OLED

**/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MAX31856.h> 
#include "utilities.h"

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,  
  REFLOW_STATE_HOLD,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position


// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Idle",
  "Pre",
  "Soak",
  "Flow",
  "Hold",
  "Cool",
  "Done!",
  "Hot!",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****

const unsigned char ssrPin = A0;
const unsigned char fanPin = A1;
const unsigned char thermocoupleCSPin = 10;
const unsigned char ledPin = 4;
const unsigned char buzzerPin = 5;
const unsigned char switchStartStopPin = 3;
const unsigned char switch2pin = 2;


// ***** PID CONTROL VARIABLES *****
const float soakTemp = 180.0;
const float preheatTemp = 150.0;
float tempCmd = 0.0;
const float reflowTemp = 230.0;
const float holdTemp = 217.0;
const float coolTemp = 100.0;
const unsigned long soakPeriod = 100000;
const unsigned long reflowPeriod = 110000;
float ovenTemp = 0.0;
float kp = 5.0;
float ki = 0.0045;
float kd = 250.0;
float command = 0.0;
float setTemp = 0.0;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timer;
unsigned long buzzerPeriod;

// Reflow oven controller state machine state variable
reflowState_t reflowState = REFLOW_STATE_IDLE;
// Reflow oven controller status
reflowStatus_t reflowStatus = REFLOW_STATUS_OFF;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;
// Thermocouple fault status
unsigned char fault;
bool atReflowTemp = false;
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;


// PID control interface
PID ovenPID(kp, ki, kd,0.0,255);

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// MAX31856 thermocouple interface
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);

void setup()
{
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialize thermocouple interface
  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  // Start-up splash
  digitalWrite(buzzerPin, HIGH);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();
  digitalWrite(buzzerPin, LOW);
  delay(500);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("     Tiny Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("     Dragoon v1.4"));
  oled.println();
  oled.println(F("      07-01-22"));
  oled.display();
  delay(2000);
  oled.clearDisplay();

  // Serial communication at 115200 bps
  Serial.begin(115200);

  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

void loop()
{
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
  secondLoop();
  }

  if (millis() > updateLcd)
  {
  updateLCD();
  }

  /////////////////////////////////////////////////////////////////////////////// Reflow oven controller state machine/////////////////////////////////////////////////////////
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      setTemp = 0.0; //turn off the oven
      // If switch is pressed to start reflow process
      if (switchStatus == SWITCH_1)
      {
        // Send header for CSV file
        Serial.println(F("Time,Setpoint,ovenTemp,Output"));
        // Intialize seconds timer for serial debug information
        timerSeconds = 0;
        
        // Initialize reflow plot update timer
        timerUpdate = 0;
        
        for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++)
        {
          temperature[x] = 0;
        }
        // Initialize index for average temperature array used for reflow plot
        x = 0;
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 500;
        // Turn on buzzer to indicate start of reflow process
        digitalWrite(buzzerPin, HIGH);
        // Proceed to preheat stage
        reflowState = REFLOW_STATE_PREHEAT;
        ovenPID.I = 0.0; //reset integral term
      }
  
      break;

    case REFLOW_STATE_PREHEAT:
      setTemp = preheatTemp;
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (ovenTemp >= (preheatTemp - 5.0))
      {
                // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 150;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // find out when we can end the soak period
        timer = millis() + soakPeriod;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
        //get ready for next phase of slow heating
        setTemp = preheatTemp;
      }
      break;

    case REFLOW_STATE_SOAK:
    //slowly ish ramp up the temp command to the soak temp
      if (setTemp < soakTemp)
      {
        setTemp += 0.004;
      } 
      // if we have soaked for enough time
      if (millis() > timer)
      {
                // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 150;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW;
      }
      break;

    case REFLOW_STATE_REFLOW:
      setTemp = reflowTemp;
      //check to see if we have gotten to temp
      if (ovenTemp >= holdTemp)
      {
                // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 150;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
      // Proceed to hold state
      reflowState = REFLOW_STATE_HOLD;
      timer = millis() + reflowPeriod;
      }
      break;

    case REFLOW_STATE_HOLD:
    //keep the oven at the reflow temp
      setTemp = reflowTemp;
      //if we have held for long enough
      if (millis() > timer)
      { 
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 5000;
        // Turn on buzzer to indicate start of reflow process
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        //proceed to cool state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      setTemp = 0; //turn off the oven
      // If minimum cool temperature is achieve
      if (ovenTemp <= coolTemp)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 5000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }






///////////////////////////////////////////////////////////////switch stuff////////////////////////////////////////////////////////////////////////////
  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2)
  {
    
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  /////////////////////// Simple switch debounce state machine (analog switch)////////////////////////////////////////////
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If either switch is pressed
      if (switchValue != SWITCH_NONE)
      {
        // Keep track of the pressed switch
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask)
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  ////////////////////////////////////////////////////////////////// PID computation and SSR control  //////////////////////////////////////////////////////////////////
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    command = ovenPID.calculate(ovenTemp,setTemp);
    analogWrite(ssrPin, command);

  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }

  //turn off the buzzer when done
  if(buzzerPeriod < millis())
  {
    digitalWrite(buzzerPin, LOW);
  }
}

switch_t readSwitch(void)
{
  int switchAdcValue = 0;
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switch2pin) == LOW) return SWITCH_2;

  return SWITCH_NONE;
}

void secondLoop(void)
{
    // Read thermocouple next sampling period
    nextRead += 1000;
    // Read current temperature
    ovenTemp = thermocouple.readThermocoupleTemperature();//IIRfilterf(thermocouple.readThermocoupleTemperature(),ovenTemp,75);
    // Check for thermocouple fault
    fault = thermocouple.readFault();
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(F(","));
      Serial.print(setTemp);
      Serial.print(F(","));   
      Serial.print(ovenTemp);
      Serial.print(F(","));      
      Serial.println(command);

    }
    else
    {
      // Turn off red LED
      digitalWrite(ledPin, LOW);
    }
    // If any thermocouple fault is detected
    if ((fault & MAX31856_FAULT_CJRANGE) ||
        (fault & MAX31856_FAULT_TCRANGE) ||
        (fault & MAX31856_FAULT_CJHIGH) ||
        (fault & MAX31856_FAULT_CJLOW) ||
        (fault & MAX31856_FAULT_TCHIGH) ||
        (fault & MAX31856_FAULT_TCLOW) ||
        (fault & MAX31856_FAULT_OVUV) ||
        (fault & MAX31856_FAULT_OPEN))
    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println(F("Error"));
    }
}
void updateLCD(void)
{
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(lcdMessagesReflowStatus[reflowState]);
    oled.setTextSize(1);
    // Right align temperature reading
    if (setTemp < 10) oled.setCursor(106, 0);
    else if (setTemp < 100) oled.setCursor(100,0);
    else oled.setCursor(94, 0);
    oled.print(((int)setTemp));
    oled.print((char)247);
    oled.print(F("C"));

    
    // Temperature markers
    oled.setCursor(0, 18);
    oled.print(F("250"));
    oled.setCursor(0, 36);
    oled.print(F("150"));
    oled.setCursor(0, 54);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63, WHITE);
    oled.drawLine(18, 63, 127, 63, WHITE);
    oled.setCursor(115, 0);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      oled.setCursor(80, 9);
      oled.print(F("TC Error"));
    }
    else
    {
      // Right align temperature reading
      if (ovenTemp < 10) oled.setCursor(91, 9);
      else if (ovenTemp < 100) oled.setCursor(85,9);
      else oled.setCursor(80, 9);
      // Display current temperature
      oled.print(ovenTemp);
      oled.print((char)247);
      oled.print(F("C"));

    }
    
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      oled.setCursor(110, 50);
      oled.print(timerSeconds);
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate)
      {
        // Store temperature reading every 3 s
        if ((timerSeconds % 5) == 0)
        {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(ovenTemp, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[x++] = averageReading;
          }

        }
      }
    }
    
    unsigned char timeAxis;
    for (timeAxis = 0; timeAxis < x; timeAxis++)
    {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], WHITE);
    }
    
    // Update screen
    oled.display();
}
