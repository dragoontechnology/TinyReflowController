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
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>      // Comment for VERSION 1
#include <Adafruit_SSD1306.h>  // Comment for VERSION 1 
#include <Adafruit_MAX31856.h> 
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
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

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED,
  REFLOW_PROFILE_TPU
} reflowProfile_t;

// ***** CONSTANTS *****
// ***** GENERAL *****
#define VERSION 2 // Replace with 1 or 2

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define TEMPERATURE_SOAK_MAX 190
#define TEMPERATURE_REFLOW_MAX 250
#define SOAK_PERIOD 12000
#define REFLOW_PERIOD 30000

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

#if VERSION == 2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position
#endif

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
#if VERSION == 1
unsigned char ssrPin = 3;
unsigned char thermocoupleCSPin = 2;
unsigned char lcdRsPin = 10;
unsigned char lcdEPin = 9;
unsigned char lcdD4Pin = 8;
unsigned char lcdD5Pin = 7;
unsigned char lcdD6Pin = 6;
unsigned char lcdD7Pin = 5;
unsigned char buzzerPin = 14;
unsigned char switchPin = A1;
unsigned char ledPin = LED_BUILTIN;
#elif VERSION == 2
unsigned char ssrPin = A0;
unsigned char fanPin = A1;
unsigned char thermocoupleCSPin = 10;
unsigned char ledPin = 4;
unsigned char buzzerPin = 5;
unsigned char switchStartStopPin = 3;
unsigned char switchLfPbPin = 2;
#endif

// ***** PID CONTROL VARIABLES *****
double setpoint;
double ovenTemp;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakPeriod = 60000;
unsigned long reflowTimer = 0;
unsigned long reflowPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState = REFLOW_STATE_IDLE;
// Reflow oven controller status
reflowStatus_t reflowStatus = REFLOW_STATUS_OFF;
// Reflow profile type
reflowProfile_t reflowProfile;
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
#ifdef VERSION == 2
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;
#endif

// PID control interface
PID reflowOvenPID(&ovenTemp, &output, &setpoint, kp, ki, kd, DIRECT);
#if VERSION == 1
// LCD interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
#elif VERSION == 2
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
#endif
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
#if VERSION == 1
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();
  lcd.print(F(" Tiny  "));
  lcd.setCursor(0, 1);
  lcd.print(F(" Reflow "));
#elif VERSION == 2
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();
#endif
  digitalWrite(buzzerPin, LOW);
  delay(2000);
#if VERSION == 1
  lcd.clear();
  lcd.print(F(" v1.00  "));
  lcd.setCursor(0, 1);
  lcd.print(F("26-07-17"));
  delay(2000);
  lcd.clear();
#elif VERSION == 2
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("     Tiny Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("       v2.02"));
  oled.println();
  oled.println(F("      05-03-22"));
  oled.display();
  delay(3000);
  oled.clearDisplay();
#endif

  // Serial communication at 115200 bps
  Serial.begin(115200);

  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    ovenTemp = thermocouple.readThermocoupleTemperature();
    // Check for thermocouple fault
    fault = thermocouple.readFault();

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

  if (millis() > nextCheck)
  {
    // Check ovenTemp in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
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
      Serial.println(output);
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledPin, LOW);
    }
  }

  if (millis() > updateLcd)
  {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;
#if VERSION == 1
    // Clear LCD
    lcd.clear();
    // Print current system state
    lcd.print(lcdMessagesReflowStatus[reflowState]);
    lcd.setCursor(6, 0);
    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
	    lcd.print(F("LF"));
    }
    else if (reflowProfile == REFLOW_PROFILE_LEADED)
    {
      lcd.print(F("PB"));
    }
    else
    {
      lcd.print(F("TPU"));
    }
    lcd.setCursor(0, 1);
    
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // Thermocouple error (open, shorted)
      lcd.print(F("TC Error"));
    }
    else
    {
      // Display current temperature
      lcd.print(ovenTemp);
#if ARDUINO >= 100
      // Display degree Celsius symbol
      lcd.write((uint8_t)0);
#else
      // Display degree Celsius symbol
      lcd.print(0, BYTE);
#endif
      lcd.print("C ");
    }
#elif VERSION == 2
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(lcdMessagesReflowStatus[reflowState]);
    oled.setTextSize(1);
    oled.setCursor(110, 0);

    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
      oled.print(F("LF"));
    }
    else if (reflowProfile == REFLOW_PROFILE_LEADED)
    {
      oled.print(F("PB"));
    }
    else
    {
      oled.print(F("TPU"));
    }
    
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
#endif
  }

  /////////////////////////////////////////////////////////////////////////////// Reflow oven controller state machine/////////////////////////////////////////////////////////
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (ovenTemp >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          Serial.println(F("Time,Setpoint,ovenTemp,Output"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          
          #if VERSION == 2
          // Initialize reflow plot update timer
          timerUpdate = 0;
          
          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++)
          {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;
          #endif
          
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile  constant
          soakTemperatureMax = TEMPERATURE_SOAK_MAX;
          reflowTemperatureMax = TEMPERATURE_REFLOW_MAX;
          soakPeriod = SOAK_PERIOD;
          reflowPeriod = REFLOW_PERIOD;
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (ovenTemp >= TEMPERATURE_SOAK_MIN)
      {
        // find out when we can end the soak period
        timerSoak = millis() + soakPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // set soak temp
        setpoint = TEMPERATURE_SOAK_MAX;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = reflowTemperatureMax;
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW;
        reflowTimer = millis() + reflowPeriod;
      }
      break;

    case REFLOW_STATE_REFLOW:
    
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if ((ovenTemp >= (reflowTemperatureMax - 5)))// && (timerSeconds > reflowTimer))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (ovenTemp <= TEMPERATURE_COOL_MIN)
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






//////////////////////////////////////////////switch stuff//////////////////////////////////////////////////////////
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
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE)
    {
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
      // Currently using leaded reflow profile
      else if (reflowProfile == REFLOW_PROFILE_LEADED)
      {
        // Switch to TPU profile
        reflowProfile = REFLOW_PROFILE_TPU;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 2);
      }
      else
      {
        // Switch to lead-free profile
        reflowProfile = REFLOW_PROFILE_LEADFREE;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
      }
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
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

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
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
#if VERSION == 1
  // Analog multiplexing switch
  switchAdcValue = analogRead(switchPin);

  // Add some allowance (+10 ADC step) as ADC reading might be off a little
  // due to 3V3 deviation and also resistor value tolerance
  if (switchAdcValue >= 1000) return SWITCH_NONE;
  if (switchAdcValue <= 10) return SWITCH_1;
  if (switchAdcValue <= 522) return SWITCH_2;

#elif VERSION == 2
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;

#endif

  return SWITCH_NONE;
}
