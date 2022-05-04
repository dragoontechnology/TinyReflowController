/**

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
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
#include <Adafruit_GFX.h>      // Comment for VERSION 1
#include <Adafruit_SSD1306.h>  // Comment for VERSION 1 
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



// ***** CONSTANTS *****
// ***** GENERAL *****

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position


// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre",
  "Soak",
  "Reflow",
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

unsigned char ssrPin = A0;
unsigned char fanPin = A1;
unsigned char thermocoupleCSPin = 10;
unsigned char ledPin = 4;
unsigned char buzzerPin = 5;
unsigned char switchStartStopPin = 3;
unsigned char switch2pin = 2;


// ***** PID CONTROL VARIABLES *****
float preheatTemp = 150;
float soakTemp = 180;
float reflowTemp = 250;
float coolTemp = 100;
unsigned long soakPeriod = 120000;
unsigned long reflowPeriod = 30000;
double setpoint = 0;
float ovenTemp = 0;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
float command = 0;
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
PID ovenPID(kp, ki, kd);

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
  delay(2000);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("     Tiny Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("       Dragoon v1.0"));
  oled.println();
  oled.println(F("      05-03-22"));
  oled.display();
  delay(3000);
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
        command = ovenPID.calculate(ovenTemp,0); //turn off the oven
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
          
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
  
      break;

    case REFLOW_STATE_PREHEAT:
      command = ovenPID.calculate(ovenTemp,preheatTemp);
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (ovenTemp >= soakTemp)
      {
        // find out when we can end the soak period
        timer = millis() + soakPeriod;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      command = ovenPID.calculate(ovenTemp,soakTemp);
      // If micro soak temperature is achieved
      if (millis() > timer)
      {
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW;
      }
      break;

    case REFLOW_STATE_REFLOW:
      command = ovenPID.calculate(ovenTemp,reflowTemp);
      //check to see if we have gotten to temp
      if ((ovenTemp >= (reflowTemp - 5)))
      {
      // Proceed to hold state
      reflowState = REFLOW_STATE_HOLD;
      timer = millis() + reflowPeriod;
      }
      break;

    case REFLOW_STATE_HOLD:
      command = ovenPID.calculate(ovenTemp,reflowTemp);
      if (millis() > timer)
      {      
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      command = ovenPID.calculate(ovenTemp,0); //turn off the oven
      // If minimum cool temperature is achieve
      if (ovenTemp <= coolTemp)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
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
    analogWrite(ssrPin, map(command,0,1000,0,255)); //this pin might not have pwm capabilities, may need to bit bang something
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
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
    ovenTemp = thermocouple.readThermocoupleTemperature();
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
      Serial.print(setpoint);
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
    oled.setCursor(110, 0);

      oled.print(F("GOON"));

    
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
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate)
      {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0)
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
