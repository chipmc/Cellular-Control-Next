/*
* Project Cellular-Next - converged software for Monitoring Control Systems
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Based on the new Well Head Next board and the Particle Series E
* Author: Chip McClelland chip@mcclellands.org
* Date:19 Jan 2018
*/

/*  The idea of this release is to unify the code base between sensors
    Both implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge

    The mode will be set and recoded in the controlRegisterAddr so resets will not change the mode
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Pumping, 0 - Low Power Mode
    We won't use Solar or Low Power modes at this time but will keep control register structure
*/

// Easy place to change global numbers
//These defines let me change the memory map and configuration without hunting through the whole program
namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number
    resetCountAddr        = 0x3,                    // This is where we keep track of how often the Electron was reset
    timeZoneAddr          = 0x4,                    // Store the local time zone data
    openTimeAddr          = 0x5,                    // Hour for opening the park / store / etc - military time (e.g. 6 is 6am)
    closeTimeAddr         = 0x6,                    // Hour for closing of the park / store / etc - military time (e.g 23 is 11pm)
    controlRegisterAddr   = 0x7,                    // This is the control register for storing the current state
    currentHourlyCountAddr =0x8,                    // Current Hourly Count - 16 bits
    dailyPumpMinutesAddr  = 0xA,                    // Current Daily Pump Minutes - 16 bits
    currentDailyCountAddr = 0xC,                    // Current Daily Count - 16 bits
    currentCountsTimeAddr = 0xE,                    // Time of last count - 32 bits
  };
};

const int versionNumber = 9;                        // Increment this number each time the memory map is changed
const char releaseNumber[6] = "0.63";               // Displays the release on the menu


// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                      //Initalize the PMIC class so you can call the Power Management functions below.

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, PUMPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
State state = INITIALIZATION_STATE;

// Pin Defines for Sketch
const int analogIn = A0;         // Analog in pin 0-20v reduced to 0-3.3V
const int analogOut = DAC;    // Analog out 0-20V
const int currentSns1 = A1;     // AC Current Sense 4-30A
const int currentSns2 = A2;
const int control0 = D0;         // Controls the SSR1
const int control1 = D1;
const int senseAC0 = C5;      // Power on or off (jumpers for inverting)
const int senseAC1 = C4;
const int senseAC2 = C3;
const int senseAC3 = C2;
const int senseAC4 = C1;
const int senseAC5 = C0;
const int powerCycle =  A5;                       // Power Cycles the Electron and the Carrier Board
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int wakeUpPin = A7;
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D5;                       // User switch with a pull-up resistor
const int anyOn = D4;                               // Active high when any of the voltage sense lines are active

// Timing Variables
unsigned long webhookWait = 45000;              // How long we will wair for a webhook response
unsigned long resetWait = 30000;                // Honw long we will wait before resetting on an error
unsigned long publishFrequency = 1000;          // How often can we publish to Particle
unsigned long sampleFrequency = 2000;           // How often will we take a sample
unsigned long webhookTimeStamp = 0;
unsigned long resetTimeStamp = 0;
unsigned long sampleTimeStamp = 0;
unsigned long publishTimeStamp = 0;             // Keep track of when we publish a webhook
unsigned long lastPublish = 0;
unsigned long lastSample = 0;

// Program Variables
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
byte controlRegister;                               // Stores the control register values
int lowBattLimit = 30;                              // Trigger for Low Batt State
bool verboseMode;                                   // Enables more active communications for configutation and setup
retained char SignalString[17];                           // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};
bool pettingEnabled = true;

// FRAM and Unix time variables
time_t t;
byte lastHour = 0;                          // For recording the startup values
byte lastDate = 0;                          // These values make sure we record events if time has lapsed
byte alertValue = 0;                        // Current Active Alerts
int alertValueInt = 0;                    // Last value - not zeroed
bool dataInFlight = false;                  // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte currentHourlyPeriod;                   // This is where we will know if the period changed

// Battery monitor
int stateOfCharge = 0;                      // stores battery charge level value

// Pump control and monitoriing
int pumpAmps = 0;
int pumpCurrentRaw = 0;
time_t pumpingStart = 0;
int dailyPumpingMins = 0;


void setup()                                                      // Note: Disconnected Setup()
{
  pinMode(analogIn,INPUT);
  pinMode(analogOut,OUTPUT);
  pinMode(currentSns1,INPUT);                                // Senses the pump current
  pinMode(currentSns2,INPUT);                                // Senses the pump current
  pinMode(control0,OUTPUT);                               // Turns on the pump
  pinMode(control1,OUTPUT);                               // Turns on the pump
  pinMode(senseAC0,INPUT);                               // Voltage Sensor Interrupt pin
  pinMode(senseAC1,INPUT);                                     // Voltage Sensor Interrupt pin
  pinMode(senseAC2,INPUT);                                   // Voltage Sensor Interrupt pin
  pinMode(senseAC3,INPUT);                               // Voltage Sensor Interrupt pin
  pinMode(senseAC4,INPUT);                                     // Voltage Sensor Interrupt pin
  pinMode(senseAC5,INPUT);                                   // Voltage Sensor Interrupt pin
  pinMode(powerCycle,OUTPUT);
  pinMode(donePin,OUTPUT);                                        // Allows us to pet the watchdog
  pinMode(wakeUpPin,INPUT);                                       // This pin is active HIGH
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(anyOn,INPUT);

  watchdogISR();                                                  // Pet the watchdog

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("Alerts", alertValueInt);
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("pumpAmps",pumpAmps);
  Particle.variable("pumpMinutes",dailyPumpingMins);

  Particle.function("Reset-FRAM", resetFRAM);
  Particle.function("PumpControl",pumpControl);
  Particle.function("Reset-Counts",resetCounts);
  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Send-Now",sendNow);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);

  if (!fram.begin()) {                                                  // You can stick the new i2c addr in here, e.g. begin(0x51);
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }
  else if (FRAMread8(FRAM::versionAddr) != versionNumber) {                   // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                        // Reset the FRAM to correct the issue
    if (FRAMread8(FRAM::versionAddr) != versionNumber) {
      resetTimeStamp = millis();
      state = ERROR_STATE;   // Resetting did not fix the issue
    }
    else {
      FRAMwrite8(FRAM::controlRegisterAddr,0);                                    // Need to reset so not in low power or low battery mode
    }
  }

  resetCount = FRAMread8(FRAM::resetCountAddr);                                   // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(FRAM::resetCountAddr,static_cast<uint8_t>(resetCount));            // If so, store incremented number - watchdog must have done This
  }
  if (resetCount >=6) {                                                 // If we get to resetCount 4, we are resetting without entering the main loop
    FRAMwrite8(FRAM::resetCountAddr,4);                                            // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                   // This will reset the modem and the device will reboot
  }

  int8_t tempTimeZoneOffset = FRAMread8(FRAM::timeZoneAddr);                  // Load Time zone data from FRAM
  if (tempTimeZoneOffset > -12 && tempTimeZoneOffset < 12) Time.zone((float)tempTimeZoneOffset);
  else Time.zone(-5);                                               // Defaults to EST if invalid value in FRAM

  controlRegister = FRAMread8(FRAM::controlRegisterAddr);                         // Read the Control Register for system modes so they stick even after reset
  verboseMode = (0b00001000 & controlRegister);                         // verboseMode
  dailyPumpingMins = FRAMread16(FRAM::dailyPumpMinutesAddr);                      // Reload so we don't loose track
  if (controlRegister & 0b00000010) {                                   // This means we reset while pumpting
    pumpingStart = FRAMread32(FRAM::currentCountsTimeAddr);                       // Reload the pumping start time
  }

  stateOfCharge = int(batteryMonitor.getSoC());                         // Percentage of full charge
  if (stateOfCharge > lowBattLimit) connectToParticle();                // If not low battery, we can connect

  attachInterrupt(wakeUpPin, watchdogISR, RISING);   // The watchdog timer will signal us and we have to respond

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    waitUntil(meterSampleRate);
    if(takeMeasurements()) state = REPORTING_STATE;
    lastSample = millis();
    if (Time.hour() != currentHourlyPeriod) {
      state = REPORTING_STATE;                                    // We want to report on the hour
      if (Time.hour() == 0) {                                     // Check to see if it is midnight
        dailyPumpingMins = 0;                                     // Reset each day.
        FRAMwrite16(FRAM::dailyPumpMinutesAddr,0);                          // And zero the value in FRAM
        Particle.syncTime();                                      // This is needed since these devices never disconnect
      }
    }
    if (stateOfCharge <= lowBattLimit) LOW_BATTERY_STATE;         // The battery is low - sleep
    break;

  case PUMPING_STATE:
    break;

  case LOW_BATTERY_STATE: {
      if (Particle.connected()) {
        disconnectFromParticle();                               // If connected, we need to disconned and power down the modem
      }
      ledState = false;
      digitalWrite(blueLED,LOW);                                // Turn off the LED
      watchdogISR();                                            // Pet the watchdog
      int secondsToHour = (60*(60 - Time.minute()));            // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);              // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE: {
    watchdogISR();                                    // Pet the watchdog once an hour
    pettingEnabled = false;                           // see this reporint cycle through
    if (!Particle.connected()) {
      if (!connectToParticle()) {
        resetTimeStamp = millis();
        state = ERROR_STATE;
        break;
      }
    }
    if (alertValue != 0) resolveAlert();
    sendEvent();
    webhookTimeStamp = millis();
    currentHourlyPeriod = Time.hour();                                // Change the time period since we have reported for this one
    waitUntil(meterParticlePublish);
    pettingEnabled = true;
    if (verboseMode) Particle.publish("State","Waiting for Response");
    lastPublish = millis();
    state = RESP_WAIT_STATE;                                          // Wait for Response
    } break;

  case RESP_WAIT_STATE:
    if (!dataInFlight)                                                // Response received
    {
      state = IDLE_STATE;
      waitUntil(meterParticlePublish);
      if (verboseMode) Particle.publish("State","Idle");
      lastPublish = millis();
    }
    else if (millis() - webhookTimeStamp >= webhookWait) {            // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;  // Response timed out
      waitUntil(meterParticlePublish);
      Particle.publish("State","Response Timeout Error");
      lastPublish = millis();
    }
    break;

    case ERROR_STATE:                                          // To be enhanced - where we deal with errors
      if (millis() - resetTimeStamp >= resetWait)
      {
        if(verboseMode) Particle.publish("State","ERROR_STATE - Resetting");
        delay(2000);
        if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
        else {
          FRAMwrite8(FRAM::resetCountAddr,0);                           // If so, store incremented number - watchdog must have done This
          fullModemReset();                                   // Full Modem reset and reboot
        }
      }
      break;
  }
}

void resolveAlert()
{
  char data[128] = "";
  if (alertValue & 0b00000001) strcat(data,"Control Power - ");
  if (alertValue & 0b00000010) strcat(data,"Low Level - ");
  if (alertValue & 0b00000100) strcat(data,"Pump On - ");
  if (alertValue & 0b10000000) strcat(data,"Particle Power");
  waitUntil(meterParticlePublish);
  if(verboseMode) Particle.publish("Alerts",data,PRIVATE);
  lastPublish = millis();
}

void sendEvent()
{
  char data[256];                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"alertValue\":%i, \"pumpAmps\":%i, \"pumpMins\":%i, \"battery\":%i, \"resets\":%i}",alertValue, pumpAmps, dailyPumpingMins, stateOfCharge,resetCount);
  Particle.publish("Monitoring_Event", data, PRIVATE);
  dataInFlight = true; // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                            // First check to see if there is any data
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", "No Data");
    lastPublish = millis();
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    waitUntil(meterParticlePublish);
    if(verboseMode) Particle.publish("State","Response Received");
    lastPublish = millis();
    dataInFlight = false;                                 // Data has been received
  }
  else Particle.publish("Ubidots Hook", data);             // Publish the response code
}


void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(SignalString,17, "%s: %d", levels[strength], rssi);
}

// Here is were we will put the timer and other ISRs
void watchdogISR()
{
  if (pettingEnabled) {
    digitalWrite(donePin, HIGH);                              // Pet the watchdog
    digitalWrite(donePin, LOW);
  }
}

// These functions manage our connecion to Particle
bool connectToParticle()
{
  if (!Cellular.ready())
  {
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;         // Connect to cellular - give it 90 seconds
  }
  Particle.process();
  Particle.connect();                                      // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;     // Connect to Particle - give it 30 seconds
  Particle.process();
  return true;
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}


// Take measurements
bool takeMeasurements() {
  controlRegister = FRAMread8(FRAM::controlRegisterAddr);                                 // Check the control register
  byte lastAlertValue = alertValue;                                             // Last value - so we can detect a
  alertValueInt = int(alertValue);                                              // For reporting
  bool pumpAmpsSignificantChange = false;                                       // Don't want to waste bandwidth reporting small changes
  int lastPumpAmps = pumpAmps;                                                  // What was the pumpAmps measurement last time
  alertValue = 0b00000000;                                                      // Reset for each run through
  if (Cellular.ready()) getSignalStrength();                                    // Test signal strength if the cellular modem is on and ready
  stateOfCharge = int(batteryMonitor.getSoC());                                 // Percentage of full charge
  if (!pinReadFast(senseAC0)) alertValue = alertValue | 0b00000001;       // Set the value for alertValue - This is opposite - power is good
  if (!pinReadFast(senseAC1)) alertValue = alertValue | 0b00000010;          // Set the value for alertValue
  if (!pinReadFast(senseAC2))                                                  // If the pump is on, we need to sample the current
  {
    alertValue = alertValue | 0b00000100;                                       // Set the value for alertValue
    pumpCurrentRaw = analogRead(currentSns1);                                // Current sensor is fairly linear from 1 to 32 Amps
    pumpAmps = map(pumpCurrentRaw,0,4095,0,32);                                 // Map analog voltage to current
    if (pumpAmps >= lastPumpAmps + 2 || pumpAmps <= lastPumpAmps - 2) pumpAmpsSignificantChange = true;
    if (!(controlRegister & 0b00000010)) {                                      // This is a new pumping session
      pumpingStart = Time.now();
      FRAMwrite32(FRAM::currentCountsTimeAddr,pumpingStart);                              // Write to FRAM in case of a reset
      FRAMwrite8(FRAM::controlRegisterAddr,controlRegister | 0b00000010);                 // Turn on the pumping bit
    }
  }
  else if (controlRegister & 0b00000010) {                                      // If the pump is off but the pumping flag is set
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegister ^ 0b00000010);                   // It is on and I want to turn the pumping bit off with an xor
    time_t pumpingStop = Time.now();
    dailyPumpingMins += int(difftime(pumpingStop,pumpingStart)/60);                 // Add to the total for the day
    FRAMwrite16(FRAM::dailyPumpMinutesAddr,dailyPumpingMins);                             // Store it in FRAM in case of a reset
  }
  else pumpAmps = 0;
  if (getLostPower()) alertValue = alertValue | 0b10000000;                      // Set the value for alertValue
  if (alertValue != lastAlertValue || pumpAmpsSignificantChange) return 1;
  else return 0;
}


/* These are the particle functions that allow you to configure and run the device
 * They are intended to allow for customization and control during installations
 * and to allow for management.
*/

int pumpControl(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    digitalWrite(control0,HIGH);
    digitalWrite(blueLED,HIGH);
    delay(10000);
    digitalWrite(control0,LOW);
    digitalWrite(blueLED,LOW);
    return 1;
  }
  else return 0;
}


int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite8(FRAM::resetCountAddr,0);          // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    dataInFlight = false;
    dailyPumpingMins = 0;
    FRAMwrite16(FRAM::dailyPumpMinutesAddr,0);
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(powerCycle,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegister);                        // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Set Verbose Mode");
    lastPublish = millis();
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegister);                        // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Cleared Verbose Mode");
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(FRAM::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  t = Time.now();
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  waitUntil(meterParticlePublish);
  Particle.publish("Time",data);
  lastPublish = millis();
  waitUntil(meterParticlePublish);
  Particle.publish("Time",Time.timeStr(t));
  lastPublish = millis();
  return 1;
}

bool getLostPower() {
	// Bit 2 (mask 0x4) == PG_STAT. If non-zero, power is good but we want to return 1 if power is lost.
	// This means we're powered off USB or VIN, so we don't know for sure if there's a battery
	byte systemStatus = power.getSystemStatus();
	return ((systemStatus & 0x04) == 0);
}

bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}

bool meterSampleRate(void)
{
  if(millis() - lastSample >= sampleFrequency) return 1;
  else return 0;
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample

	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}
