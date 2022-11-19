/* RC engine sound & light controller for Arduino ESP32. Written by TheDIYGuy999
    Based on the code for ATmega 328: https://github.com/TheDIYGuy999/Rc_Engine_Sound

    ***** ESP32 CPU frequency must be set to 240MHz! *****
    ESP32 macOS Big Sur fix see: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/BigSurFix.md

   Sound files converted with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/tools/Audio2Header.html
   Original converter code by bitluni (send him a high five, if you like the code)

   Parts of automatic transmision code from Wombii's fork: https://github.com/Wombii/Rc_Engine_Sound_ESP32

   Dashboard, Neopixel and SUMD support by Gamadril: https://github.com/Gamadril/Rc_Engine_Sound_ESP32

   Contributors:
   - Christian Fiebig https://github.com/fiechr

   NEW: Visual Studio Code IDE support added (you need to install PlatformIO)
   Arduino IDE is supported as before, but stuff was renamed and moved to different folders!
*/

char codeVersion[] = "9.10.0"; // Software revision.

// This stuff is required for Visual Studio Code IDE, if .ino is renamed into .cpp!
#include <Arduino.h>
#include "ow_control_config.h"
#include <VescUart.h> //VESC uart communication
void Task1code(void *parameters);
float batteryVolts();

//
// =======================================================================================================
// ERROR CODES (INDICATOR LIGHTS, BEEPS)
// =======================================================================================================
//
/*
   Indicators:
   Constantly on = no SBUS signal (check "sbusInverted" true / false in "2_adjustmentsRemote.h")
   Number of blinks = this channel signal is not between 1400 and 1600 microseconds and can't be auto calibrated
   (check channel trim settings)

   Beeps (only, if "#define BATTERY_PROTECTION" in "3_adjustmentsESC.h"):
   - Number of beeps = number of detected battery cells in series
   - 10 fast beeps = battery error, disconnect it!
*/

//
// =======================================================================================================
// ! ! I M P O R T A N T ! !   ALL USER SETTINGS ARE DONE IN THE FOLLOWING TABS, WHICH ARE DISPLAYED ABOVE
// (ADJUST THEM BEFORE CODE UPLOAD), DO NOT CHANGE ANYTHING IN THIS TAB EXCEPT THE DEBUG OPTIONS
// =======================================================================================================
//

// All the required user settings are done in the following .h files:
#include "1_adjustmentsVehicle.h" // <<------- Select the vehicle you want to simulate
// #include "2_adjustmentsRemote.h"        // <<------- Remote control system related adjustments
#include "3_adjustmentsESC.h"          // <<------- ESC related adjustments
#include "4_adjustmentsTransmission.h" // <<------- Transmission related adjustments
// #include "5_adjustmentsShaker.h"        // <<------- Shaker related adjustments
#include "6_adjustmentsLights.h" // <<------- Lights related adjustments
// #include "7_adjustmentsServos.h"        // <<------- Servo output related adjustments
#include "8_adjustmentsSound.h" // <<------- Sound related adjustments
// #include "9_adjustmentsDashboard.h"     // <<------- Dashboard related adjustments
// #include "10_adjustmentsTrailer.h"      // <<------- Trailer related adjustments

// DEBUG options can slow down the playback loop! Only uncomment them for debugging, may slow down your system!
// #define CHANNEL_DEBUG // uncomment it for input signal & general debugging informations
// #define ESC_DEBUG // uncomment it to debug the ESC
// #define AUTO_TRANS_DEBUG // uncomment it to debug the automatic transmission
// #define MANUAL_TRANS_DEBUG // uncomment it to debug the manual transmission
// #define TRACKED_DEBUG // debugging tracked vehicle mode
// #define SERVO_DEBUG // uncomment it for servo calibration in BUS communication mode
// #define ESPNOW_DEBUG  // uncomment for additional ESP-NOW messages
// #define CORE_DEBUG // Don't use this!
//#define VESC_DEBUG
//#define FAKE_VESC_DATA // random Number for test
#ifdef FAKE_VESC_DATA
#define PI 3.14159265
#endif
// TODO = Things to clean up!

//
// =======================================================================================================
// LIRBARIES & HEADER FILES, REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder.
// No manual library download is required in Visual Studio Code IDE (see platformio.ini)
#include <statusLED.h>          // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control
#include <FastLED.h>            // https://github.com/FastLED/FastLED        <<------- required for Neopixel support. Use V3.3.3
#include <ESP32AnalogRead.h>    // https://github.com/madhephaestus/ESP32AnalogRead <<------- required for battery voltage measurement
// Additional headers (included)
#include "src/curves.h"  // Nonlinear throttle curve arrays
#include "soc/rtc_wdt.h" // for watchdog timer
#include "src/helper.h"  // map fun for any type
// The following tasks only required for Arduino IDE! ----
// Install ESP32 board according to: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
// Warning: Use Espressif ESP32 board definition v1.06! v2.x is not working
// Adjust board settings according to: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/pictures/settings.png

// Visual Studio Code IDE instructions: ----
// - General settings are done in platformio.ini
// - !!IMPORTANT!! Rename src.ini to scr.cpp -> This will prevent potential compiler errors

// Make sure to remove -master from your sketch folder name

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES (Do not play around here)
// =======================================================================================================
//
// Pin assignment and wiring instructions ****************************************************************

// ------------------------------------------------------------------------------------
// Use a 330Ohm resistor in series with all I/O pins! allows to drive LED directly and
// provides short circuit protection. Also works on the serial Rx pin "VP" (36)
// ------------------------------------------------------------------------------------

#define BATTERY_DETECT_PIN 39

#define DAC1 25 // connect pin25 (do not change the pin) to a 10kOhm resistor
#define DAC2 26 // connect pin26 (do not change the pin) to a 10kOhm resistor
// both outputs of the resistors above are connected together and then to the outer leg of a
// 10kOhm potentiometer. The other outer leg connects to GND. The middle leg connects to both inputs
// of a PAM8403 amplifier and allows to adjust the volume. This way, two speakers can be used.
// Objects *************************************************************************************
#ifdef USE_DUAL_HEAD_LIGHT
statusLED headLight0(true);
statusLED headLight1(true);
#else
statusLED headLight(false);
#endif
statusLED tailLight(false);
// Neopixel LED
#ifdef USE_RGB_LED
CRGB rgb1LEDs[RGB_LED1_COUNT];
CRGB rgb2LEDs[RGB_LED2_COUNT];
#endif

#ifdef USE_FAN_COOLING
statusLED fan(false);
#endif

// Battery voltage
ESP32AnalogRead battery;

// Global variables **********************************************************************

// Sound
volatile boolean engineOn = false;                // Signal for engine on / off
volatile boolean engineStart = false;             // Active, if engine is starting up
volatile boolean engineRunning = false;           // Active, if engine is running
volatile boolean engineStop = false;              // Active, if engine is shutting down
volatile boolean jakeBrakeRequest = false;        // Active, if engine jake braking is requested
volatile boolean engineJakeBraking = false;       // Active, if engine is jake braking
volatile boolean wastegateTrigger = false;        // Trigger wastegate (blowoff) after rapid throttle drop
volatile boolean blowoffTrigger = false;          // Trigger jake brake sound (blowoff) after rapid throttle drop
volatile boolean dieselKnockTrigger = false;      // Trigger Diesel ignition "knock"
volatile boolean dieselKnockTriggerFirst = false; // The first Diesel ignition "knock" per sequence
volatile boolean airBrakeTrigger = false;         // Trigger for air brake noise
volatile boolean parkingBrakeTrigger = false;     // Trigger for air parking brake noise
volatile boolean shiftingTrigger = false;         // Trigger for shifting noise
volatile boolean hornTrigger = false;             // Trigger for horn on / off
volatile boolean sirenTrigger = false;            // Trigger for siren  on / off
volatile boolean sound1trigger = false;           // Trigger for sound1  on / off
volatile boolean couplingTrigger = false;         // Trigger for trailer coupling  sound
volatile boolean uncouplingTrigger = false;       // Trigger for trailer uncoupling  sound
volatile boolean bucketRattleTrigger = false;     // Trigger for bucket rattling  sound
volatile boolean indicatorSoundOn = false;        // active, if indicator bulb is on
volatile boolean outOfFuelMessageTrigger = false; // Trigger for out of fuel message

// Sound latches
volatile boolean hornLatch = false;  // Horn latch bit
volatile boolean sirenLatch = false; // Siren latch bit

// Sound volumes
volatile uint16_t throttleDependentVolume = 0;        // engine volume according to throttle position
volatile uint16_t throttleDependentRevVolume = 0;     // engine rev volume according to throttle position
volatile uint16_t rpmDependentJakeBrakeVolume = 0;    // Engine rpm dependent jake brake volume
volatile uint16_t throttleDependentKnockVolume = 0;   // engine Diesel knock volume according to throttle position
volatile uint16_t rpmDependentKnockVolume = 0;        // engine Diesel knock volume according to engine RPM
volatile uint16_t throttleDependentTurboVolume = 0;   // turbo volume according to rpm
volatile uint16_t throttleDependentFanVolume = 0;     // cooling fan volume according to rpm
volatile uint16_t throttleDependentChargerVolume = 0; // cooling fan volume according to rpm
volatile uint16_t rpmDependentWastegateVolume = 0;    // wastegate volume according to rpm
volatile uint16_t tireSquealVolume = 0;                  // Tire squeal volume according to speed and cornering radius
volatile uint16_t trackRattleVolume = 0; // track rattling volume

volatile uint64_t dacDebug = 0;      // DAC debug variable TODO
volatile int16_t masterVolume = 100; // Master volume percentage
volatile uint8_t dacOffset = 0;      // 128, but needs to be ramped up slowly to prevent popping noise, if switched on

// Throttle
int16_t currentThrottle = 0;      // 0 - 500 (Throttle trigger input)
int16_t currentThrottleFaded = 0; // faded throttle for volume calculations etc.

// Engine
const int16_t maxRpm = 500;       // always 500
const int16_t minRpm = 0;         // always 0
int32_t currentRpm = 0;           // 0 - 500 (signed required!)
volatile uint8_t engineState = 0; // Engine state
enum EngineState                  // Engine state enum
{
  OFF,
  STARTING,
  RUNNING,
  STOPPING,
  PARKING_BRAKE
};

int16_t engineLoad = 0;                 // 0 - 500
volatile uint16_t engineSampleRate = 0; // Engine sample rate
int32_t speedLimit = maxRpm;            // The speed limit, depending on selected virtual gear

// Clutch
boolean clutchDisengaged = true; // Active while clutch is disengaged

// Transmission
uint8_t selectedGear = 1;             // The currently used gear of our shifting gearbox
uint8_t selectedAutomaticGear = 1;    // The currently used gear of our automatic gearbox
boolean gearUpShiftingInProgress;     // Active while shifting upwards
boolean doubleClutchInProgress;       // Double-clutch (Zwischengas)
boolean gearDownShiftingInProgress;   // Active while shifting downwards
boolean gearUpShiftingPulse;          // Active, if shifting upwards begins
boolean gearDownShiftingPulse;        // Active, if shifting downwards begins
volatile boolean neutralGear = false; // Transmission in neutral
boolean lowRange = false;             // Transmission range (off road reducer)

// ESC
volatile boolean owIsBraking = false; // ESC is in a braking state
volatile boolean owIsDriving = false; // ESC is in a driving state
volatile boolean owInReverse = false; // ESC is driving or braking backwards
volatile boolean brakeDetect = false; // Additional brake detect signal, enabled immediately, if brake applied
int8_t driveState = 0;                // for ESC state machine
uint16_t escPulseMax = 2000;          // ESC calibration variables (values will be changed later)
uint16_t escPulseMin = 1000;
uint16_t escPulseMaxNeutral = 1500;
uint16_t escPulseMinNeutral = 1500;
uint16_t currentSpeed = 0;         // 0 - 500 (current ESC power)
volatile bool crawlerMode = false; // Crawler mode intended for crawling competitons (withouth sound and virtual inertia)

// Lights

// Battery
float batteryCutoffvoltage;
float batteryVoltage;
uint8_t numberOfCells;
bool batteryProtection = false;

// VESC var
#define NEUTRAL_ERPM 600.0 //  rang in -200 < 0 < 200 erpm is neutral 
#define NEUTRAL_PID 10 //  rang in -5 < 0 < 5 erpm is neutral 
#define MAX_ERPM 6000 
#define MIN_ERPM 600 
volatile float vescErpm;
volatile float vescPid;
volatile SwitchState vescSwitchState;
const uint32_t engineOffDelay = 5000; // engine off delay
unsigned long engineOffTimer ;
unsigned long sourceCheckTimer ; //check the vesc id , 0 is ble audio , 1 is engine sound.

// audio source
volatile AudioSource source;

volatile unsigned long timelast;
unsigned long timelastloop;
//// Initiate VescUart class
VescUart VESC;
// DEBUG stuff
volatile uint8_t coreId = 99;
// Our main tasks
TaskHandle_t Task1;
// Loop time (for debug)
uint16_t loopTime;
// Sampling intervals for interrupt timer (adjusted according to your sound file sampling rate)
uint32_t maxSampleInterval = 4000000 / sampleRate;
uint32_t minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;
// Interrupt timer for variable sample rate playback (engine sound)
hw_timer_t *variableTimer = NULL;
portMUX_TYPE variableTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t variableTimerTicks = maxSampleInterval;
// Interrupt timer for fixed sample rate playback (horn etc., playing in parallel with engine sound)
hw_timer_t *fixedTimer = NULL;
portMUX_TYPE fixedTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fixedTimerTicks = maxSampleInterval;
// Declare a mutex Semaphore Handles.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xPwmSemaphore;
SemaphoreHandle_t xRpmSemaphore;
//
// =======================================================================================================
// INTERRUPT FOR VARIABLE SPEED PLAYBACK (Engine sound, turbo sound)
// =======================================================================================================
//

void IRAM_ATTR variablePlaybackTimer()
{

  // coreId = xPortGetCoreID(); // Running on core 1

  static uint32_t attenuatorMillis = 0;
  static uint32_t curEngineSample = 0;          // Index of currently loaded engine sample
  static uint32_t curRevSample = 0;             // Index of currently loaded engine rev sample
  static uint32_t curTurboSample = 0;           // Index of currently loaded turbo sample
  static uint32_t curFanSample = 0;             // Index of currently loaded fan sample
  static uint32_t curChargerSample = 0;         // Index of currently loaded charger sample
  static uint32_t curStartSample = 0;           // Index of currently loaded start sample
  static uint32_t curJakeBrakeSample = 0;       // Index of currently loaded jake brake sample
  static uint32_t curHydraulicPumpSample = 0;   // Index of currently loaded hydraulic pump sample
  static uint32_t curTrackRattleSample = 0;     // Index of currently loaded train track rattle sample
  static uint32_t lastDieselKnockSample = 0;    // Index of last Diesel knock sample
  static uint16_t attenuator = 0;               // Used for volume adjustment during engine switch off
  static uint16_t speedPercentage = 0;          // slows the engine down during shutdown
  static int32_t a, a1, a2, a3, b, c, d, e = 0; // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
  static int32_t f = 0;                         // Input signals for mixer: f = hydraulic pump
  static int32_t g = 0;                         // Input signals for mixer: g = train track rattle
  uint8_t a1Multi = 0;                          // Volume multipliers

  // portENTER_CRITICAL_ISR(&variableTimerMux); // disables C callable interrupts (on the current core) and locks the mutex by the current core.

  switch (engineState)
  {

  case OFF:                                                   // Engine off -----------------------------------------------------------------------
    variableTimerTicks = 4000000 / startSampleRate;           // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    a = 0; // volume = zero
    if (engineOn)
    {
      engineState = STARTING;
      engineStart = true;
    }
    break;

  case STARTING:                                              // Engine start --------------------------------------------------------------------
    variableTimerTicks = 4000000 / startSampleRate;           // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    if (curStartSample < startSampleCount - 1)
    {

      a = (startSamples[curStartSample] * throttleDependentVolume / 100 * startVolumePercentage / 100);

      curStartSample++;
    }
    else
    {
      curStartSample = 0;
      engineState = RUNNING;
      engineStart = false;
      engineRunning = true;
      airBrakeTrigger = true;
    }
    break;

  case RUNNING: // Engine running ------------------------------------------------------------------

    // Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
    variableTimerTicks = engineSampleRate;                    // our variable idle sampling rate!
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    if (!engineJakeBraking && !blowoffTrigger)
    {
      if (curEngineSample < sampleCount - 1)
      {
        a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100); // Idle sound
        a3 = 0;
        curEngineSample++;

        // Optional rev sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
        // idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
        // or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
#ifdef REV_SOUND
        a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100); // Rev sound
        if (curRevSample < revSampleCount)
          curRevSample++;
#endif

        // Trigger throttle dependent Diesel ignition "knock" sound (played in the fixed sample rate interrupt)
        if (curEngineSample - lastDieselKnockSample > (sampleCount / dieselKnockInterval))
        {
          dieselKnockTrigger = true;
          dieselKnockTriggerFirst = false;
          lastDieselKnockSample = curEngineSample;
        }
      }
      else
      {
        curEngineSample = 0;
        if (jakeBrakeRequest)
          engineJakeBraking = true;
#ifdef REV_SOUND
        curRevSample = 0;
#endif
        lastDieselKnockSample = 0;
        dieselKnockTrigger = true;
        dieselKnockTriggerFirst = true;
      }
      curJakeBrakeSample = 0;
    }
    else
    { // Jake brake sound ----
#ifdef JAKE_BRAKE_SOUND
      a3 = (jakeBrakeSamples[curJakeBrakeSample] * rpmDependentJakeBrakeVolume / 100 * jakeBrakeVolumePercentage / 100); // Jake brake sound
      a2 = 0;
      a1 = 0;
      if (curJakeBrakeSample < jakeBrakeSampleCount - 1)
        curJakeBrakeSample++;
      else
      {
        curJakeBrakeSample = 0;
        if (!jakeBrakeRequest)
          engineJakeBraking = false;
      }

      curEngineSample = 0;
      curRevSample = 0;
#endif
    }

    // Engine sound mixer ----
#ifdef REV_SOUND
    // Mixing the idle and rev sounds together, according to engine rpm
    // Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @ max. rpm.
    // The total of idle and rev volume percentage is always 100%

    if (currentRpm > revSwitchPoint)
      a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
    else
      a1Multi = idleVolumeProportionPercentage; // 90 - 100% proportion
    if (currentRpm > idleEndPoint)
      a1Multi = 0;

    a1 = a1 * a1Multi / 100;         // Idle volume
    a2 = a2 * (100 - a1Multi) / 100; // Rev volume

    a = a1 + a2 + a3; // Idle and rev sounds mixed together
#else
    a = a1 + a3; // Idle sound only
#endif

    // Turbo sound ----------------------------------
    if (curTurboSample < turboSampleCount - 1)
    {
      c = (turboSamples[curTurboSample] * throttleDependentTurboVolume / 100 * turboVolumePercentage / 100);
      curTurboSample++;
    }
    else
    {
      curTurboSample = 0;
    }

    // Fan sound -----------------------------------
    if (curFanSample < fanSampleCount - 1)
    {
      d = (fanSamples[curFanSample] * throttleDependentFanVolume / 100 * fanVolumePercentage / 100);
      curFanSample++;
    }
    else
    {
      curFanSample = 0;
    }
#if defined GEARBOX_WHINING
    if (neutralGear)
      d = 0; // used for gearbox whining simulation, so not active in gearbox neutral
#endif

    // Supercharger sound --------------------------
    if (curChargerSample < chargerSampleCount - 1)
    {
      e = (chargerSamples[curChargerSample] * throttleDependentChargerVolume / 100 * chargerVolumePercentage / 100);
      curChargerSample++;
    }
    else
    {
      curChargerSample = 0;
    }

    if (!engineOn)
    {
      speedPercentage = 100;
      attenuator = 1;
      engineState = STOPPING;
      engineStop = true;
      engineRunning = false;
    }
    break;

  case STOPPING:                                                       // Engine stop --------------------------------------------------------------------
    variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true);          // // change timer ticks, autoreload true

    if (curEngineSample < sampleCount - 1)
    {
      a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
      curEngineSample++;
    }
    else
    {
      curEngineSample = 0;
    }

    // fade engine sound out
    if (millis() - attenuatorMillis > 100)
    { // Every 50ms
      attenuatorMillis = millis();
      attenuator++;          // attenuate volume
      speedPercentage += 20; // make it slower (10)
    }

    if (attenuator >= 50 || speedPercentage >= 500)
    { // 50 & 500
      a = 0;
      speedPercentage = 100;
      parkingBrakeTrigger = true;
      engineState = PARKING_BRAKE;
      engineStop = false;
    }
    break;

  case PARKING_BRAKE: // parking brake bleeding air sound after engine is off ----------------------------

    if (!parkingBrakeTrigger)
    {
      engineState = OFF;
    }
    break;

  } // end of switch case

  // DAC output (groups a, b, c mixed together) ************************************************************************

  dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f + g) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write  to DAC
  // dacWrite(DAC1, constrain(a * masterVolume / 100 + dacOffset, 0, 255));
  // dacWrite(DAC1, constrain(a + 128, 0, 255));

  // portEXIT_CRITICAL_ISR(&variableTimerMux);
}

//
// =======================================================================================================
// INTERRUPT FOR FIXED SPEED PLAYBACK (Horn etc., played in parallel with engine sound)
// =======================================================================================================
//

void IRAM_ATTR fixedPlaybackTimer()
{

  // coreId = xPortGetCoreID(); // Running on core 1

  static uint32_t curHornSample = 0;                        // Index of currently loaded horn sample
  static uint32_t curSirenSample = 0;                       // Index of currently loaded siren sample
  static uint32_t curSound1Sample = 0;                      // Index of currently loaded sound1 sample
  static uint32_t curReversingSample = 0;                   // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample = 0;                   // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample = 0;                   // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample = 0;                       // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample = 0;                // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample = 0;                    // Index of currently loaded shifting sample
  static uint32_t curDieselKnockSample = 0;                 // Index of currently loaded Diesel knock sample
  static uint32_t curTrackRattleSample = 0;                 // Index of currently loaded track rattle sample
  static uint32_t curTireSquealSample = 0;                     // Index of currently loaded tire squeal sample
  static uint32_t curOutOfFuelSample = 0;                   // Index of currently loaded out of fuel sample
  static int32_t a, a1, a2 = 0;                             // Input signals "a" for mixer
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b9 = 0; // Input signals "b" for mixer
  static int32_t c, c1, c2, c3 = 0;                         // Input signals "c" for mixer
  static int32_t d, d1, d2 = 0;                             // Input signals "d" for mixer
  static boolean knockSilent = 0;                           // This knock will be more silent
  static boolean knockMedium = 0;                           // This knock will be medium
  static uint8_t curKnockCylinder = 0;                      // Index of currently ignited zylinder

  // portENTER_CRITICAL_ISR(&fixedTimerMux);

  // Group "a" (horn & siren) ******************************************************************

  if (hornTrigger || hornLatch)
  {
    fixedTimerTicks = 4000000 / hornSampleRate;         // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curHornSample < hornSampleCount - 1)
    {
      a1 = (hornSamples[curHornSample] * hornVolumePercentage / 100);
      curHornSample++;
#ifdef HORN_LOOP // Optional "endless loop" (points to be defined manually in horn file)
      if (hornTrigger && curHornSample == hornLoopEnd)
        curHornSample = hornLoopBegin; // Loop, if trigger still present
#endif
    }
    else
    { // End of sample
      curHornSample = 0;
      a1 = 0;
      hornLatch = false;
    }
  }

  if (sirenTrigger || sirenLatch)
  {
    fixedTimerTicks = 4000000 / sirenSampleRate;        // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curSirenSample < sirenSampleCount - 1)
    {
      a2 = (sirenSamples[curSirenSample] * sirenVolumePercentage / 100);
      curSirenSample++;
#ifdef SIREN_LOOP // Optional "endless loop" (points to be defined manually in siren file)
      if (sirenTrigger && curSirenSample == sirenLoopEnd)
        curSirenSample = sirenLoopBegin; // Loop, if trigger still present
#endif
    }
    else
    { // End of sample
      curSirenSample = 0;
      a2 = 0;
      sirenLatch = false;
    }
  }

  // Group "b" (other sounds) **********************************************************************

  // Sound 1 "b0" ----
  if (sound1trigger)
  {
    fixedTimerTicks = 4000000 / sound1SampleRate;       // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curSound1Sample < sound1SampleCount - 1)
    {
      b0 = (sound1Samples[curSound1Sample] * sound1VolumePercentage / 100);
      curSound1Sample++;
    }
    else
    {
      sound1trigger = false;
    }
  }
  else
  {
    curSound1Sample = 0; // ensure, next sound will start @ first sample
    b0 = 0;
  }

  // Reversing beep sound "b1" ----
  if (engineRunning && owInReverse)
  {
    fixedTimerTicks = 4000000 / reversingSampleRate;    // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curReversingSample < reversingSampleCount - 1)
    {
      b1 = (reversingSamples[curReversingSample] * reversingVolumePercentage / 100);
      curReversingSample++;
    }
    else
    {
      curReversingSample = 0;
    }
  }
  else
  {
    curReversingSample = 0; // ensure, next sound will start @ first sample
    b1 = 0;
  }

  // Indicator tick sound "b2" ----------------------------------------------------------------------
#if not defined NO_INDICATOR_SOUND
  if (indicatorSoundOn)
  {
    fixedTimerTicks = 4000000 / indicatorSampleRate;    // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curIndicatorSample < indicatorSampleCount - 1)
    {
      b2 = (indicatorSamples[curIndicatorSample] * indicatorVolumePercentage / 100);
      curIndicatorSample++;
    }
    else
    {
      indicatorSoundOn = false;
    }
  }
  else
  {
    curIndicatorSample = 0; // ensure, next sound will start @ first sample
    b2 = 0;
  }
#endif

  // Wastegate (blowoff) sound, triggered after rapid throttle drop -----------------------------------
  if (wastegateTrigger)
  {
    if (curWastegateSample < wastegateSampleCount - 1)
    {
      b3 = (wastegateSamples[curWastegateSample] * rpmDependentWastegateVolume / 100 * wastegateVolumePercentage / 100);
      curWastegateSample++;
    }
    else
    {
      wastegateTrigger = false;
    }
  }
  else
  {
    b3 = 0;
    curWastegateSample = 0; // ensure, next sound will start @ first sample
  }

  // Air brake release sound, triggered after stop -----------------------------------------------
  if (airBrakeTrigger)
  {
    if (curBrakeSample < brakeSampleCount - 1)
    {
      b4 = (brakeSamples[curBrakeSample] * brakeVolumePercentage / 100);
      curBrakeSample++;
    }
    else
    {
      airBrakeTrigger = false;
    }
  }
  else
  {
    b4 = 0;
    curBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Air parking brake attaching sound, triggered after engine off --------------------------------
  if (parkingBrakeTrigger)
  {
    if (curParkingBrakeSample < parkingBrakeSampleCount - 1)
    {
      b5 = (parkingBrakeSamples[curParkingBrakeSample] * parkingBrakeVolumePercentage / 100);
      curParkingBrakeSample++;
    }
    else
    {
      parkingBrakeTrigger = false;
    }
  }
  else
  {
    b5 = 0;
    curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
  if (shiftingTrigger && engineRunning && !automatic && !doubleClutch)
  {
    if (curShiftingSample < shiftingSampleCount - 1)
    {
      b6 = (shiftingSamples[curShiftingSample] * shiftingVolumePercentage / 100);
      curShiftingSample++;
    }
    else
    {
      shiftingTrigger = false;
    }
  }
  else
  {
    b6 = 0;
    curShiftingSample = 0; // ensure, next sound will start @ first sample
  }

  // Diesel ignition "knock" is played in fixed sample rate section, because we don't want changing pitch! ------
  if (dieselKnockTriggerFirst)
  {
    dieselKnockTriggerFirst = false;
    curKnockCylinder = 0;
  }

  if (dieselKnockTrigger)
  {
    dieselKnockTrigger = false;
    curKnockCylinder++; // Count ignition sequence
    curDieselKnockSample = 0;
  }

#ifdef V8 // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
  // Ford or Scania V8 ignition sequence: 1 - 5 - 4 - 2* - 6 - 3 - 7 - 8* (* = louder knock pulses, because 2nd exhaust in same manifold after 90°)
  if (curKnockCylinder == 4 || curKnockCylinder == 8)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V8_MEDIUM // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
  // This is EXPERIMENTAL!! TODO
  if (curKnockCylinder == 5 || curKnockCylinder == 1)
    knockMedium = false;
  else
    knockMedium = true;
#endif

#ifdef V8_468 // (Chevy 468, containing 16 ignition pulses)
  // 1th, 5th, 9th and 13th are the loudest
  // Ignition sequence: 1 - 8 - 4* - 3 - 6 - 5 - 7* - 2
  if (curKnockCylinder == 1 || curKnockCylinder == 5 || curKnockCylinder == 9 || curKnockCylinder == 13)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V2
  // V2 engine: 1st and 2nd knock pulses (of 4) will be louder
  if (curKnockCylinder == 1 || curKnockCylinder == 2)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6
  // R6 inline 6 engine: 6th knock pulse (of 6) will be louder
  if (curKnockCylinder == 6)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6_2
  // R6 inline 6 engine: 6th and 3rd knock pulse (of 6) will be louder
  if (curKnockCylinder == 6 || curKnockCylinder == 3)
    knockSilent = false;
  else
    knockSilent = true;
#endif

  if (curDieselKnockSample < knockSampleCount)
  {
#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#else // Just depending on throttle
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
    curDieselKnockSample++;
    if (knockSilent && !knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
    if (knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 75;
  }

  // Group "d" (additional sounds) **********************************************************************

#if defined TIRE_SQUEAL
  // Tire squeal sound -----------------------
  if (curTireSquealSample < tireSquealSampleCount - 1)
  {
    d1 = (tireSquealSamples[curTireSquealSample] * tireSquealVolumePercentage / 100 * tireSquealVolume / 100);
    curTireSquealSample++;
  }
  else
  {
    d1 = 0;
    curTireSquealSample = 0;
  }
#endif

#if defined BATTERY_PROTECTION
  // Out of fuel sound, triggered by battery voltage -----------------------------------------------
  if (outOfFuelMessageTrigger)
  {
    if (curOutOfFuelSample < outOfFuelSampleCount - 1)
    {
      d2 = (outOfFuelSamples[curOutOfFuelSample] * outOfFuelVolumePercentage / 100);
      curOutOfFuelSample++;
    }
    else
    {
      outOfFuelMessageTrigger = false;
    }
  }
  else
  {
    d2 = 0;
    curOutOfFuelSample = 0; // ensure, next sound will start @ first sample
  }
#endif

  // Mixing sounds together **********************************************************************
  a = a1 + a2; // Horn & siren
  // if (a < 2 && a > -2) a = 0; // Remove noise floor TODO, experimental
  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7; // b8 + b9;  Other sounds
  c = c1 + c2 + c3;                                  // Excavator sounds
  d = d1 + d2;                                       // Additional sounds

  // DAC output (groups mixed together) ****************************************************************************

  // dacDebug = constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255); // Mix signals, add 128 offset, write result to DAC
  dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, constrain( a2 * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, 0);

  // portEXIT_CRITICAL_ISR(&fixedTimerMux);
}

//
// =======================================================================================================
// BATTERY SETUP
// =======================================================================================================
//

void setupBattery()
{
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//
//
// =======================================================================================================
// Get the vesc main data
// =======================================================================================================
//

bool get_vesc_values(uint32_t loop_time)

{
  static uint32_t lastReadTime = millis();
  if (loop_time > 0)
  {
    if (millis() - lastReadTime > loop_time)
    {

      return VESC.updateCustomValues();

      lastReadTime = millis();
    }
  }
  else
  {

    return VESC.updateCustomValues();

  }

#ifdef VESC_DEBUG
  VESC.printCustomValues();
#endif // DEBUG
}

//
//===================================================== ===================================================== ===
// initialize one wheel
//===================================================== ===================================================== ===
//
void ow_setup()
{

  // Neopixel setup
  FastLED.addLeds<NEOPIXEL, RGB_LED1_DATA_PIN>(rgb1LEDs, RGB_LED1_COUNT);
  FastLED.addLeds<NEOPIXEL, RGB_LED2_DATA_PIN>(rgb2LEDs, RGB_LED2_COUNT);
#ifdef USE_DUAL_HEAD_LIGHT
  headLight0.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
  headLight1.begin(LED2_PIN, 1, 20000); // Timer 1, 20kHz
#else
  headLight.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
#endif
  tailLight.begin(LED3_PIN, 2, 20000); // Timer 2, 20kHz
  // VESC serial
  Serial2.begin(115200, SERIAL_8N1, ESP_VESC_TX_PIN, ESP_VESC_RX_PIN);
  // wait serial init ..
  while (!Serial2)
  {
    ;
  }
#ifdef VESC_DEBUG
  VESC.setDebugPort(&Serial);
#endif
  VESC.setSerialPort(&Serial2);
  // delay 100ms
  vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef FAKE_VESC_DATA
  source = SOURCE_ESP32;
#else
  vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 100mS for vesc Boot in to balance application
  get_vesc_values(0);
  source = (AudioSource)VESC.getVescId();
#endif
  if (source == SOURCE_CSR )
  {
    AUDIO_MUTE();
    CSR_POWER_ON();
    AUDIO_SOURCE_CSR()
    vTaskDelay(300 / portTICK_PERIOD_MS);
    AUDIO_UNMUTE();
  }
  else if (source == SOURCE_ESP32)
  {
    AUDIO_MUTE();
    CSR_POWER_OFF();
    AUDIO_SOURCE_ESP()
    vTaskDelay(300 / portTICK_PERIOD_MS);
    AUDIO_UNMUTE();
  }
  engineOn=false;
}

void setup()
{   
  Serial.begin(115200); // USB serial (for DEBUG) Mode, Rx pin (99 = not used), Tx pin
  // set pin mode
  pinMode(CSR_EN_PIN, OUTPUT);
  pinMode(AUDIO_SOURCE_PIN, OUTPUT);
  pinMode(PAM_MUTE_PIN, OUTPUT);
  // csr8645 off
  CSR_POWER_OFF();
  // sound source for welcome sound
  AUDIO_SOURCE_ESP();
  // Unmute Amp
  AUDIO_UNMUTE();
  // welcome sound trigger
  sound1trigger = true;
  // Watchdog timers need to be disabled, if task 1 is running without delay(1)
  disableCore0WDT();
  // disableCore1WDT(); // TODO leaving this one enabled is experimental!
  // Setup RTC (Real Time Clock) watchdog
  //rtc_wdt_protect_off(); // Disable RTC WDT write protection
  //rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  //rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  //rtc_wdt_set_time(RTC_WDT_STAGE0, 10000); // set 10s timeout
  //rtc_wdt_enable();                        // Start the RTC WDT timer
  // rtc_wdt_disable();            // Disable the RTC WDT timer
  //rtc_wdt_protect_on(); // Enable RTC WDT write protection
  // Serial setup

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the PWM variable.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if (xPwmSemaphore == NULL) // Check to confirm that the PWM Semaphore has not already been created.
  {
    xPwmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xPwmSemaphore) != NULL)
      xSemaphoreGive((xPwmSemaphore)); // Make the PWM variable available for use, by "Giving" the Semaphore.
  }

  if (xRpmSemaphore == NULL) // Check to confirm that the RPM Semaphore has not already been created.
  {
    xRpmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xRpmSemaphore) != NULL)
      xSemaphoreGive((xRpmSemaphore)); // Make the RPM variable available for use, by "Giving" the Semaphore.
  }

  // Refresh sample intervals (important, because MAX_RPM_PERCENTAGE was probably changed above)
  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

  // Task 1 setup (running on core 0)
  TaskHandle_t Task1;
  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      Task1code, // Task function
      "Task1",   // name of task
      8192,      // Stack size of task (8192)
      NULL,      // parameter of the task
      1,         // priority of the task (1 = low, 3 = medium, 5 = highest)
      &Task1,    // Task handle to keep track of created task
      0);        // pin task to core 0

  // Interrupt timer for variable sample rate playback
  variableTimer = timerBegin(0, 20, true);                           // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(variableTimer, variableTimerTicks, true);          // autoreload true
  timerAlarmEnable(variableTimer);                                   // enable

  // Interrupt timer for fixed sample rate playback
  fixedTimer = timerBegin(1, 20, true);                        // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(fixedTimer, fixedTimerTicks, true);          // autoreload true
  timerAlarmEnable(fixedTimer);                                // enable
  // wating the welcome sound stop
  while (sound1trigger)
  {
    ;
  }
  // setup for wheel
  ow_setup();
}

//
// =======================================================================================================
// DAC OFFSET FADER
// =======================================================================================================
//

static unsigned long dacOffsetMicros;
boolean dacInit;

void dacOffsetFade()
{
  if (!dacInit)
  {
    if (micros() - dacOffsetMicros > 100)
    { // Every 0.1ms
      dacOffsetMicros = micros();
      dacOffset++; // fade DAC offset slowly to prevent it from popping, if ESP32 powered up after amplifier
      if (dacOffset == 128)
        dacInit = true;
    }
  }
}


/**
 * @brief Get the vesc throttle object
 *
 * @return int16_t
 */
static int16_t get_vesc_throttle()
{
  uint32_t  throttle;
  throttle = fabsf(vescErpm);
  throttle =MAX( throttle ,MIN_ERPM); // limit min erpm
  throttle =MIN( throttle ,MAX_ERPM); //limit max erpm
  return (int16_t)map((uint32_t)throttle, MIN_ERPM, MAX_ERPM, minRpm, maxRpm);
}

//
// =======================================================================================================
// MAP PULSEWIDTH TO THROTTLE CH3
// =======================================================================================================
//

void mapThrottle()
{

  // Input is around 1000 - 2000us, output 0-500 for forward and backwards
  // Volume calculations --------------------------------------------------------------------------
  currentThrottle = get_vesc_throttle();
  // As a base for some calculations below, fade the current throttle to make it more natural
  static unsigned long throttleFaderMicros;
  static boolean blowoffLock;
  if (micros() - throttleFaderMicros > 500)
  { // Every 0.5ms

    throttleFaderMicros = micros();

    if (currentThrottleFaded < currentThrottle && !owIsBraking && currentThrottleFaded < 499)
      currentThrottleFaded += 2;
    if ((currentThrottleFaded > currentThrottle || owIsBraking) && currentThrottleFaded > 2)
      currentThrottleFaded -= 2;

    // Calculate throttle dependent engine idle volume
    if (!owIsBraking && !brakeDetect && engineRunning)
      throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);
    // else throttleDependentVolume = engineIdleVolumePercentage; // TODO
    else
    {
      if (throttleDependentVolume > engineIdleVolumePercentage)
        throttleDependentVolume--;
      else
        throttleDependentVolume = engineIdleVolumePercentage;
    }

    // Calculate throttle dependent engine rev volume
    if (!owIsBraking && !brakeDetect && engineRunning)
      throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);
    // else throttleDependentRevVolume = engineRevVolumePercentage; // TODO
    else
    {
      if (throttleDependentRevVolume > engineRevVolumePercentage)
        throttleDependentRevVolume--;
      else
        throttleDependentRevVolume = engineRevVolumePercentage;
    }

    // Calculate throttle dependent Diesel knock volume
    if (!owIsBraking && !brakeDetect && engineRunning && (currentThrottleFaded > dieselKnockStartPoint))
      throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);
    // else throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;
    else
    {
      if (throttleDependentKnockVolume > dieselKnockIdleVolumePercentage)
        throttleDependentKnockVolume--;
      else
        throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;
    }

    // Calculate engine rpm dependent jake brake volume
    if (engineRunning)
      rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
    else
      rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    // Calculate RPM dependent Diesel knock volume
    if (currentRpm > 400)
      rpmDependentKnockVolume = map(currentRpm, knockStartRpm, 500, minKnockVolumePercentage, 100);
    else
      rpmDependentKnockVolume = minKnockVolumePercentage;
#endif

    // Calculate engine rpm dependent turbo volume
    if (engineRunning)
      throttleDependentTurboVolume = map(currentRpm, 0, 500, turboIdleVolumePercentage, 100);
    else
      throttleDependentTurboVolume = turboIdleVolumePercentage;

    // Calculate engine rpm dependent cooling fan volume
    if (engineRunning && (currentRpm > fanStartPoint))
      throttleDependentFanVolume = map(currentRpm, fanStartPoint, 500, fanIdleVolumePercentage, 100);
    else
      throttleDependentFanVolume = fanIdleVolumePercentage;

    // Calculate throttle dependent supercharger volume
    if (!owIsBraking && !brakeDetect && engineRunning && (currentRpm > chargerStartPoint))
      throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, 500, chargerIdleVolumePercentage, 100);
    else
      throttleDependentChargerVolume = chargerIdleVolumePercentage;

    // Calculate engine rpm dependent wastegate volume
    if (engineRunning)
      rpmDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
    else
      rpmDependentWastegateVolume = wastegateIdleVolumePercentage;
  }

  // Calculate engine load (used for torque converter slip simulation)
  engineLoad = currentThrottle - currentRpm;

  if (engineLoad < 0 || owIsBraking || brakeDetect)
    engineLoad = 0; // Range is 0 - 180
  if (engineLoad > 180)
    engineLoad = 180;


      // Additional sounds volumes -----------------------------

  // // Tire squealing ----
  // uint8_t steeringAngle = 0;
  // uint8_t brakeSquealVolume = 0;


  // // Brake squealing
  // if ((driveState == 2 || driveState == 4) && currentSpeed > 50 && currentThrottle > 250) {
  //   tireSquealVolume += map(currentThrottle, 250, 500, 0, 100);
  // }

  // tireSquealVolume = constrain(tireSquealVolume, 0, 100);
}

//
// =======================================================================================================
// ENGINE MASS SIMULATION (running on core 0)
// =======================================================================================================
//

void engineMassSimulation()
{

  static int32_t targetRpm = 0;   // The engine RPM target
  static int32_t _currentRpm = 0; // Private current RPM (to prevent conflict with core 1)
  static int32_t _currentThrottle = 0;
  static int32_t lastThrottle;
  uint16_t converterSlip;
  static unsigned long throtMillis;
  static unsigned long wastegateMillis;
  static unsigned long blowoffMillis;
  uint8_t timeBase;

#ifdef SUPER_SLOW
  timeBase = 6; // super slow running, heavy engines, for example locomotive diesels
#else
  timeBase = 2;
#endif

  _currentThrottle = currentThrottle;

  if (millis() - throtMillis > timeBase)
  { // Every 2 or 6ms
    throtMillis = millis();

    if (_currentThrottle > 500)
      _currentThrottle = 500;

    // Normal mode ---
    // if ((currentSpeed < clutchEngagingPoint && _currentRpm < maxClutchSlippingRpm) || gearUpShiftingInProgress || gearDownShiftingInProgress || neutralGear || _currentRpm < 200) { // TODO Bug?
    if ((currentSpeed < clutchEngagingPoint && _currentRpm < maxClutchSlippingRpm) || gearUpShiftingInProgress || gearDownShiftingInProgress || neutralGear)
    {
      clutchDisengaged = true;
    }
    else
    {
      clutchDisengaged = false;
    }

    // Transmissions ***********************************************************************************

    // automatic transmission ----
    if (automatic)
    {
      // Torque converter slip calculation
      if (selectedAutomaticGear < 2)
        converterSlip = engineLoad * torqueconverterSlipPercentage / 100 * 2; // more slip in first and reverse gear
      else
        converterSlip = engineLoad * torqueconverterSlipPercentage / 100;

      if (!neutralGear)
        targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10 + converterSlip; // Compute engine RPM
      else
        targetRpm = reMap(curveLinear, _currentThrottle);
    }
    else if (doubleClutch)
    {
      // double clutch transmission
      if (!neutralGear)
        targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10; // Compute engine RPM
      else
        targetRpm = reMap(curveLinear, _currentThrottle);
    }
    else
    {
      // Manual transmission ----
      if (clutchDisengaged)
      { // Clutch disengaged: Engine revving allowed
#if defined VIRTUAL_16_SPEED_SEQUENTIAL
        targetRpm = _currentThrottle;
#else
        targetRpm = reMap(curveLinear, _currentThrottle);

#endif
      }
      else
      { // Clutch engaged: Engine rpm synchronized with ESC power (speed)

#if defined VIRTUAL_3_SPEED || defined VIRTUAL_16_SPEED_SEQUENTIAL                                  // Virtual 3 speed or sequential 16 speed transmission
        targetRpm = reMap(curveLinear, (currentSpeed * virtualManualGearRatio[selectedGear] / 10)); // Add virtual gear ratios
        if (targetRpm > 500)
          targetRpm = 500;
#else // Real 3 speed transmission
        targetRpm = reMap(curveLinear, currentSpeed);
#endif
      }
    }

    // Engine RPM **************************************************************************************

    if (owIsBraking && currentSpeed < clutchEngagingPoint)
      targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    if (targetRpm > 500)
      targetRpm = 500;

    // Accelerate engine
    if (targetRpm > (_currentRpm + acc) && (_currentRpm + acc) < maxRpm && engineState == RUNNING && engineRunning)
    {
      if (!airBrakeTrigger)
      { // No acceleration, if brake release noise still playing
        if (!gearDownShiftingInProgress)
          _currentRpm += acc;
        else
          _currentRpm += acc / 2; // less aggressive rpm rise while downshifting
        if (_currentRpm > maxRpm)
          _currentRpm = maxRpm;
      }
    }

    // Decelerate engine
    if (targetRpm < _currentRpm)
    {
      _currentRpm -= dec;
      if (_currentRpm < minRpm)
        _currentRpm = minRpm;
    }

#if (defined VIRTUAL_3_SPEED || defined VIRTUAL_16_SPEED_SEQUENTIAL)
    // Limit top speed, depending on manual gear ratio. Ensures, that the engine will not blow up!
    if (!automatic && !doubleClutch)
      speedLimit = maxRpm * 10 / virtualManualGearRatio[selectedGear];
#endif

    // Speed (sample rate) output
    engineSampleRate = map(_currentRpm, minRpm, maxRpm, maxSampleInterval, minSampleInterval); // Idle

    // if ( xSemaphoreTake( xRpmSemaphore, portMAX_DELAY ) )
    //{
    currentRpm = _currentRpm;
    // xSemaphoreGive( xRpmSemaphore ); // Now free or "Give" the semaphore for others.
    // }
  }

  // Prevent Wastegate from being triggered while downshifting
  if (gearDownShiftingInProgress)
    wastegateMillis = millis();

  // Trigger Wastegate, if throttle rapidly dropped
  if (lastThrottle - _currentThrottle > 70 && !owIsBraking && millis() - wastegateMillis > 1000)
  {
    wastegateMillis = millis();
    wastegateTrigger = true;
  }

#if defined JAKEBRAKE_ENGINE_SLOWDOWN && defined JAKE_BRAKE_SOUND
  // Use jake brake to slow down engine while releasing throttle in neutral or during upshifting while applying throttle
  // for some vehicles like Volvo FH open pipe. See example: https://www.youtube.com/watch?v=MU1iwzl33Zw&list=LL&index=4
  if (!wastegateTrigger)
    blowoffMillis = millis();
  blowoffTrigger = ((gearUpShiftingInProgress || neutralGear) && millis() - blowoffMillis > 20 && millis() - blowoffMillis < 250);
#endif

  lastThrottle = _currentThrottle;
}

//
// =======================================================================================================
// SWITCH ENGINE ON OR OFF (for automatic mode)
// =======================================================================================================
//

void engineOnOff()
{

  // static unsigned long pulseDelayMillis; // TODO
  static unsigned long idleDelayMillis;

  // Engine automatically switched on or off depending on throttle position and 15s delay timne
  if (currentThrottle > 80 || driveState != 0)
    idleDelayMillis = millis(); // reset delay timer, if throttle not in neutral

#ifdef AUTO_ENGINE_ON_OFF
  if (millis() - idleDelayMillis > 15000)
  {
    engineOn = false; // after delay, switch engine off
  }
#endif

#ifdef AUTO_LIGHTS
  if (millis() - idleDelayMillis > 10000)
  {
    lightsOn = false; // after delay, switch light off
  }
#endif

  // Engine start detection
  if (currentThrottle > 100 && !airBrakeTrigger)
  {
    engineOn = true;

#ifdef AUTO_LIGHTS
    lightsOn = true;
#endif
  }
}

//
// =======================================================================================================
// MANUAL GEARBOX DETECTION (Real 3 speed, virtual 3 speed, virtual 16 speed, semi automatic)
// =======================================================================================================
//

void gearboxDetection()
{

  static uint8_t previousGear = 1;
  static bool previousReverse;
  static bool sequentialLock;
  static bool overdrive = false;
  static unsigned long upShiftingMillis;
  static unsigned long downShiftingMillis;
  static unsigned long lastShiftingMillis;       // This timer is used to prevent transmission from oscillating!
#if defined OVERDRIVE && defined VIRTUAL_3_SPEED // Additional 4th gear mode for virtual 3 speed ********************************
  if (!crawlerMode)
  {
    // The 4th gear (overdrive) is engaged automatically, if driving @ full throttle in 3rd gear
    if (currentRpm > 490 && selectedGear == 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
    {
      overdrive = true;
    }
    if (!owIsBraking)
    { // Lower downshift point, if not braking
      if (currentRpm < 200 && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    else
    { // Higher downshift point, if braking
      if ((currentRpm < 400 || engineLoad > 150) && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    if (selectedGear < 3)
      overdrive = false;
  }
#endif // End of overdrive ******************************************************************************************************

#if defined SEMI_AUTOMATIC // gears not controlled by the 3 position switch but by RPM limits ************************************
  if (currentRpm > 490 && selectedGear < 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
  {
    selectedGear++;
  }
  if (!owIsBraking)
  { // Lower downshift point, if not braking
    if (currentRpm < 200 && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--; //
    }
  }
  else
  { // Higher downshift point, if braking
    if ((currentRpm < 400 || engineLoad > 150) && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--; // Higher downshift point, if braking
    }
  }
  if (neutralGear || owInReverse)
    selectedGear = 1;
#endif // End of SEMI_AUTOMATIC **************************************************************************************************

  // Gear upshifting detection
  if (selectedGear > previousGear)
  {
    gearUpShiftingInProgress = true;
    gearUpShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }

  // Gear upshifting duration
  static uint16_t upshiftingDuration = 700;
  if (!gearUpShiftingInProgress)
    upShiftingMillis = millis();
  if (millis() - upShiftingMillis > upshiftingDuration)
    gearUpShiftingInProgress = false;

    // Double-clutch (Zwischengas während dem Hochschalten)
#if defined DOUBLE_CLUTCH
  upshiftingDuration = 900;
  doubleClutchInProgress = (millis() - upShiftingMillis >= 500 && millis() - upShiftingMillis < 600); // Apply full throttle
#endif

  // Gear downshifting detection
  if (selectedGear < previousGear)
  {
    gearDownShiftingInProgress = true;
    gearDownShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }

  // Gear downshifting duration
  if (!gearDownShiftingInProgress)
    downShiftingMillis = millis();
  if (millis() - downShiftingMillis > 300)
    gearDownShiftingInProgress = false;

  // Reverse gear engaging / disengaging detection
  if (owInReverse != previousReverse)
  {
    previousReverse = owInReverse;
    shiftingTrigger = true; // Play shifting sound
  }

#ifdef MANUAL_TRANS_DEBUG
  static unsigned long manualTransDebugMillis;
  if (millis() - manualTransDebugMillis > 100)
  {
    manualTransDebugMillis = millis();
    Serial.printf("MANUAL_TRANS_DEBUG:\n");
    Serial.printf("currentThrottle: %i\n", currentThrottle);
    Serial.printf("selectedGear: %i\n", selectedGear);
    Serial.printf("overdrive: %i\n", overdrive);
    Serial.printf("engineLoad: %i\n", engineLoad);
    Serial.printf("sequentialLock: %s\n", sequentialLock ? "true" : "false");
    Serial.printf("currentRpm: %i\n", currentRpm);
    Serial.printf("currentSpeed: %i\n", currentSpeed);
    Serial.printf("---------------------------------\n");
  }
#endif // MANUAL_TRANS_DEBUG
}

//
// =======================================================================================================
// SIMULATED AUTOMATIC TRANSMISSION GEAR SELECTOR (running on core 0)
// =======================================================================================================
//

void automaticGearSelector()
{

  static unsigned long gearSelectorMillis;
  static unsigned long lastUpShiftingMillis;
  static unsigned long lastDownShiftingMillis;
  uint16_t downShiftPoint = 200;
  uint16_t upShiftPoint = 490;
  static int32_t _currentRpm = 0; // Private current RPM (to prevent conflict with core 1)

  // if ( xSemaphoreTake( xRpmSemaphore, portMAX_DELAY ) )
  //{
  _currentRpm = currentRpm;
  // xSemaphoreGive( xRpmSemaphore ); // Now free or "Give" the semaphore for others.
  // }

  if (millis() - gearSelectorMillis > 100)
  { // Waiting for 100ms is very important. Otherwise gears are skipped!
    gearSelectorMillis = millis();

    // compute load dependent shift points (less throttle = less rpm before shifting up, kick down will shift back!)
    upShiftPoint = map(engineLoad, 0, 180, 390, 490);   // 390, 490
    downShiftPoint = map(engineLoad, 0, 180, 150, 250); // 150, 250

    if (owInReverse)
    { // Reverse (only one gear)
      selectedAutomaticGear = 0;
    }
    else
    { // Forward (multiple gears)

      // Adaptive shift points
      if (millis() - lastDownShiftingMillis > 500 && _currentRpm >= upShiftPoint && engineLoad < 5)
      {                          // 500ms locking timer!
        selectedAutomaticGear++; // Upshifting (load maximum is important to prevent gears from oscillating!)
        lastUpShiftingMillis = millis();
      }
      if (millis() - lastUpShiftingMillis > 600 && selectedAutomaticGear > 1 && (_currentRpm <= downShiftPoint || engineLoad > 100))
      {                          // 600ms locking timer! TODO was 1000
        selectedAutomaticGear--; // Downshifting incl. kickdown
        lastDownShiftingMillis = millis();
      }

      selectedAutomaticGear = constrain(selectedAutomaticGear, 1, NumberOfAutomaticGears);
    }

#ifdef AUTO_TRANS_DEBUG
    Serial.printf("AUTO_TRANS_DEBUG:\n");
    Serial.printf("currentThrottle: %i\n", currentThrottle);
    Serial.printf("selectedAutomaticGear: %i\n", selectedAutomaticGear);
    Serial.printf("engineLoad: %i\n", engineLoad);
    Serial.printf("upShiftPoint: %i\n", upShiftPoint);
    Serial.printf("_currentRpm: %i\n", _currentRpm);
    Serial.printf("downShiftPoint: %i\n", downShiftPoint);
    Serial.printf("-----------------------------------\n");
#endif
  }
}

//
// =======================================================================================================
// ESC CONTROL (including optional battery protection)
// =======================================================================================================
//

static uint16_t escPulseWidth = 1500;
static uint16_t escPulseWidthOut = 1500;
static uint16_t escSignal = 1500;
static unsigned long escMillis;
// static int8_t pulse; // -1 = reverse, 0 = neutral, 1 = forward
// static int8_t escPulse; // -1 = reverse, 0 = neutral, 1 = forward
static int8_t driveRampRate;
static int8_t driveRampGain;
static int8_t brakeRampRate;
uint16_t escRampTime;
/**
 * @brief check the one wheel forward ,backward and neutral .
 * 
 * @param val  can be pid or erpm 
 * @param range neutral range 
 * @return int8_t 
 */
int8_t checkPulse(float val , uint32_t range )
{ // rpm direction
  int8_t _rpmPulse;
  if (((val+ range ) * (range - val)) > 0) // check val is in the range +/-range
  {
    _rpmPulse = 0; // 0 = Neutral

  } else 
  {
    if (val > range )
    _rpmPulse = 1; // 1 = Forward
    else 
     _rpmPulse = -1; // -1 = backwards
  }

  return _rpmPulse;
}

int8_t rpmPulse()
{ // rpm direction
  return checkPulse(vescErpm,NEUTRAL_ERPM);
}


int8_t pidPulse()
{ 
  return checkPulse(vescPid,NEUTRAL_PID);
}

// If you connect your ESC to pin 33, the vehicle inertia is simulated. Direct brake (crawler) ESC required
// *** WARNING!! Do it at your own risk!! There is a falisafe function in case, the signal input from the
// receiver is lost, but if the ESP32 crashes, the vehicle could get out of control!! ***

void esc()
{ // ESC main function ================================
  // Gear dependent ramp speed for acceleration & deceleration
#if defined VIRTUAL_3_SPEED
  escRampTime = escRampTimeThirdGear * 10 / virtualManualGearRatio[selectedGear];

#elif defined VIRTUAL_16_SPEED_SEQUENTIAL
  escRampTime = escRampTimeThirdGear * virtualManualGearRatio[selectedGear] / 5;

#elif defined STEAM_LOCOMOTIVE_MODE
  escRampTime = escRampTimeSecondGear;

#else // TAMIYA 3 speed shifting transmission
  if (selectedGear == 1)
    escRampTime = escRampTimeFirstGear; // about 20
  if (selectedGear == 2)
    escRampTime = escRampTimeSecondGear; // about 50
  if (selectedGear == 3)
    escRampTime = escRampTimeThirdGear; // about 75
#endif

  if (automatic || doubleClutch)
  {
    escRampTime = escRampTimeSecondGear; // always use 2nd gear acceleration for automatic transmissions
    if (owInReverse)
      escRampTime = escRampTime * 100 / automaticReverseAccelerationPercentage; // faster acceleration in automatic reverse, EXPERIMENTAL, TODO!
  }

  // Allows to scale vehicle file dependent acceleration
  escRampTime = escRampTime * 100 / globalAccelerationPercentage;

  // ESC ramp time compensation in low range
  if (lowRange)
    escRampTime = escRampTime * lowRangePercentage / 100;

  // Drive mode -------------------------------------------
  // Crawler mode for direct control -----
  crawlerMode = (masterVolume <= masterVolumeCrawlerThreshold); // Direct control, depending on master volume

  if (crawlerMode)
  { // almost no virtual inertia (just for drive train protection), for crawling competitions
    escRampTime = crawlerEscRampTime;
    brakeRampRate = map(currentThrottle, 0, 500, 1, 10);
    driveRampRate = 10;
  }
  else
  { // Virtual inertia mode -----
    // calulate throttle dependent brake & acceleration steps
    brakeRampRate = map(currentThrottle, 0, 500, 1, escBrakeSteps);
    driveRampRate = map(currentThrottle, 0, 500, 1, escAccelerationSteps);
  } // ----------------------------------------------------

  // Additional brake detection signal, applied immediately. Used to prevent sound issues, if braking very quickly
  brakeDetect = ((rpmPulse() == 1 && pidPulse() == -1) || (rpmPulse() == -1 && pidPulse() == 1));


  if (millis() - escMillis > escRampTime)
  { // About very 20 - 75ms
    escMillis = millis();

    // Drive state state machine **********************************************************************************
    switch (driveState)
    {

    case 0: // Standing still ---------------------------------------------------------------------
      owIsBraking = false;
      owInReverse = false;
      owIsDriving = false;
#ifdef VIRTUAL_16_SPEED_SEQUENTIAL
      selectedGear = 1;
#endif
      if (rpmPulse() == 1   && engineRunning && !neutralGear)
        driveState = 1; // Driving forward
      if (rpmPulse() == -1  && engineRunning && !neutralGear)
        driveState = 3; // Driving backwards
      break;

    case 1: // Driving forward ---------------------------------------------------------------------
      owIsBraking = false;
      owInReverse = false;
      owIsDriving = true;

      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // lowering RPM, if shifting up transmissio
        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // increasing RPM, if shifting down transmission

        gearDownShiftingPulse = false;
      }

      if (rpmPulse() == 1 && pidPulse() == -1)
        driveState = 2; // Braking forward
      if (rpmPulse() == -1 && pidPulse() == -1)
        driveState = 3; // Driving backwards, if ESC not yet moving. Prevents state machine from hanging! v9.7.0
      if (rpmPulse() == 0 ) // only check rpm for standing still 
        driveState = 0; // standing still
      break;

    case 2: // Braking forward ---------------------------------------------------------------------
      owIsBraking = true;
      owInReverse = false;
      owIsDriving = false;

      if (rpmPulse() == 1 && pidPulse() == 1 && !neutralGear)
      {
        driveState = 1; // Driving forward
        airBrakeTrigger = true;
      }
      if (rpmPulse() == 0 )
      {
        driveState = 0; // standing still
        airBrakeTrigger = true;
      }
      break;

    case 3: // Driving backwards ---------------------------------------------------------------------
      owIsBraking = false;
      owInReverse = true;
      owIsDriving = true;

      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // lowering RPM, if shifting up transmission

        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // increasing RPM, if shifting down transmission

        gearDownShiftingPulse = false;
      }

      if (rpmPulse() == -1 && pidPulse() == 1)
        driveState = 4; // Braking backwards
      if (rpmPulse() == 1 && pidPulse() == 1)
        driveState = 1; // Driving forward, if ESC not yet moving. Prevents state machine from hanging! v9.7.0
      if (rpmPulse() == 0 )
        driveState = 0; // standing still
      break;

    case 4: // Braking backwards ---------------------------------------------------------------------
      owIsBraking = true;
      owInReverse = true;
      owIsDriving = false;

      if (rpmPulse() == -1 && pidPulse() == -1 && !neutralGear)
      {
        driveState = 3; // Driving backwards
        airBrakeTrigger = true;
      }
      if (rpmPulse() == 0 )
      {
        driveState = 0; // standing still
        airBrakeTrigger = true;
      }
      break;

    } // End of state machine **********************************************************************************

    // Gain for drive ramp rate, depending on clutchEngagingPoint
    if (currentSpeed < clutchEngagingPoint)
    {

      if (!automatic && !doubleClutch)
        driveRampGain = 2; // prevent clutch from slipping too much (2)
      else
        driveRampGain = 4; // Automatic transmission needs to catch immediately (4)
    }
    else
    {
      driveRampGain = 1;
    }

    currentSpeed = get_vesc_throttle();

  }
}

//
// =======================================================================================================
// BATTERY MONITORING
// =======================================================================================================
//

float batteryVolts()
{
  static float raw[6];
  static bool initDone = false;
#define VOLTAGE_CALIBRATION (RESISTOR_TO_BATTTERY_PLUS + RESISTOR_TO_GND) / RESISTOR_TO_GND + DIODE_DROP

  if (!initDone)
  { // Init array, if first measurement (important for call in setup)!
    for (uint8_t i = 0; i <= 5; i++)
    {
      raw[i] = battery.readVoltage();
    }
    initDone = true;
  }

  raw[5] = raw[4]; // Move array content, then add latest measurement (averaging)
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];

  raw[0] = battery.readVoltage(); // read analog input

  float voltage = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6 * VOLTAGE_CALIBRATION;
  return voltage;
}

//
// =======================================================================================================
// LOOP TIME MEASUREMENT
// =======================================================================================================
//

unsigned long loopDuration()
{
  static unsigned long timerOld;
  unsigned long loopTime;
  unsigned long timer = millis();
  loopTime = timer - timerOld;
  timerOld = timer;
  return loopTime;
}

//
// =======================================================================================================
// HORN, BLUELIGHT & SIREN TRIGGERING BY CH4 (POT), WINCH CONTROL
// =======================================================================================================
//

void triggerHorn()
{

  //   if (!winchEnabled && !unlock5thWheel && !hazard) { // Horn & siren control mode *************
  //     winchPull = false;
  //     winchRelease = false;
  //     legsUp = false;
  //     legsDown = false;
  //     rampsUp = false;
  //     rampsDown = false;

  //     // detect horn trigger ( impulse length > 1900us) -------------
  //     if (pulseWidth[4] > 1900 && pulseWidth[4] < pulseMaxLimit[4]) {
  //       hornTrigger = true;
  //       hornLatch = true;
  //     }
  //     else {
  //       hornTrigger = false;
  //     }

  // #if not defined EXCAVATOR_MODE
  // #ifndef NO_SIREN
  //     // detect siren trigger ( impulse length < 1100us) ----------
  //     if (pulseWidth[4] < 1100 && pulseWidth[4] > pulseMinLimit[4]) {
  //       sirenTrigger = true;
  //       sirenLatch = true;
  //     }
  //     else {
  //       sirenTrigger = false;
  //     }
  // #endif
  // #endif

  //     // detect bluelight trigger ( impulse length < 1300us) ----------
  //     static uint32_t bluelightOffDelay = millis();
  //     if ((pulseWidth[4] < 1300 && pulseWidth[4] > pulseMinLimit[4]) || sirenLatch) {
  //       bluelightOffDelay = millis();
  //       blueLightTrigger = true;
  //     }
  //     if (millis() - bluelightOffDelay > 50)  { // Switch off delay
  //       blueLightTrigger = false;
  //     }
  //   }

  //   else if (unlock5thWheel) { // Trailer leg control mode *************************************
  //     winchPull = false;
  //     winchRelease = false;
  //     rampsUp = false;
  //     rampsDown = false;

  //     // legs down ( impulse length > 1900us) -------------
  //     if (pulseWidth[4] > 1900 && pulseWidth[4] < pulseMaxLimit[4]) legsDown = true;
  //     else legsDown = false;

  //     // legs up ( impulse length < 1100us) -------------
  //     if (pulseWidth[4] < 1100 && pulseWidth[4] > pulseMinLimit[4]) legsUp = true;
  //     else legsUp = false;
  //   }

  //   else if (hazard) { // Trailer ramps control mode ***************************************
  //     winchPull = false;
  //     winchRelease = false;
  //     legsUp = false;
  //     legsDown = false;

  //     // ramps down ( impulse length > 1900us) -------------
  //     if (pulseWidth[4] > 1900 && pulseWidth[4] < pulseMaxLimit[4]) rampsDown = true;
  //     else rampsDown = false;

  //     // ramps up ( impulse length < 1100us) -------------
  //     if (pulseWidth[4] < 1100 && pulseWidth[4] > pulseMinLimit[4]) rampsUp = true;
  //     else rampsUp = false;
  //   }

  //   else { // Winch control mode *****************************************************************
  //     legsUp = false;
  //     legsDown = false;
  //     rampsUp = false;
  //     rampsDown = false;

  //     // pull winch ( impulse length > 1900us) -------------
  //     if (pulseWidth[4] > 1900 && pulseWidth[4] < pulseMaxLimit[4]) winchPull = true;
  //     else winchPull = false;

  //     // release winch ( impulse length < 1100us) -------------
  //     if (pulseWidth[4] < 1100 && pulseWidth[4] > pulseMinLimit[4]) winchRelease = true;
  //     else winchRelease = false;
  //   }
}

//
// =======================================================================================================
// INDICATOR (TURN SIGNAL) TRIGGERING
// =======================================================================================================
//

void triggerIndicators()
{

  // #if not defined EXCAVATOR_MODE // Only used, if our vehicle is not an excavator!

  //   static boolean L;
  //   static boolean R;

  // #ifdef AUTO_INDICATORS // Automatic, steering triggered indicators ********
  //   // detect left indicator trigger -------------
  //   if (pulseWidth[1] > (1500 + indicatorOn)) {
  //     L = true;
  //     R = false;
  //   }
  //   if (pulseWidth[1] < (1500 + indicatorOn / 3)) L = false;

  //   // detect right indicator trigger -------------
  //   if (pulseWidth[1] < (1500 - indicatorOn)) {
  //     R = true;
  //     L = false;
  //   }
  //   if (pulseWidth[1] > (1500 - indicatorOn / 3)) R = false;

  // #else // Manually triggered indicators ********
  //   // detect left indicator trigger -------------
  //   if (pulseWidth[6] > 1900) {
  //     L = true;
  //     R = false;
  //   }
  //   if (pulseWidth[6] < (1500 - indicatorOn / 3)) L = false;

  //   // detect right indicator trigger -------------
  //   if (pulseWidth[6] < 1100) {
  //     R = true;
  //     L = false;
  //   }
  //   if (pulseWidth[6] > (1500 + indicatorOn / 3)) R = false;

  //   // Reset by steering -------------
  //   static int steeringOld;

  //   if (pulseWidth[1] < steeringOld - 50) {
  //     L = false;
  //     steeringOld = pulseWidth[1];
  //   }

  //   if (pulseWidth[1] > steeringOld + 50) {
  //     R = false;
  //     steeringOld = pulseWidth[1];
  //   }

  // #endif // End of manually triggered indicators

  //   // Indicator direction
  //   if (!INDICATOR_DIR) {
  //     indicatorLon = L;
  //     indicatorRon = R;
  //   }
  //   else {
  //     indicatorLon = R;
  //     indicatorRon = L;
  //   }

  //   if (indicatorLon || indicatorRon) hazard = false;

  // #endif
}

//
// =======================================================================================================
// NEOPIXEL WS2812 LED
// =======================================================================================================
//

void updateRGBLEDs()
{
}

// If you connect your ESC to pin 33, the vehicle inertia is simulated. Direct brake (crawler) ESC required
// *** WARNING!! Do it at your own risk!! There is a falisafe function in case, the signal input from the
// receiver is lost, but if the ESP32 crashes, the vehicle could get out of control!! ***

/**
 * @brief
 *
 */

void check_mute( uint32_t time )
{ 
  static bool unmute;
  static unsigned long _timer =millis();
  if (millis() - _timer > time && source == SOURCE_ESP32)
  {
    unmute = ((engineState > 0) || sirenTrigger || dieselKnockTrigger || airBrakeTrigger || parkingBrakeTrigger || shiftingTrigger || hornTrigger || sirenTrigger || sound1trigger || couplingTrigger || uncouplingTrigger || bucketRattleTrigger || indicatorSoundOn );
    if (unmute)
    {

      AUDIO_UNMUTE();
    }
    else
    {

      AUDIO_MUTE();
    }
    _timer = millis();
  }
}

void debug_print()
{
  static unsigned long tt = millis();

  if (millis() - tt > 1000)
  {
    Serial.print("engineOn ");
    Serial.println(engineOn);
    Serial.print("currentThrottle ");
    Serial.println(currentThrottle);
    Serial.print("engineState ");
    Serial.println(engineState);
    Serial.print("switchState ");
    Serial.println(vescSwitchState);
     Serial.print("erpm ");
    Serial.println(vescErpm);
    Serial.print("rpmPluse   ");
    Serial.println(rpmPulse());
    Serial.print("pidPluse   ");
    Serial.println(pidPulse());
     Serial.print("pidOutput  ");
    Serial.println(vescPid);
    Serial.print("owIsBraking  ");
    Serial.println(owIsBraking);
    Serial.print("owIsDriving  ");
    Serial.println(owIsDriving);
    Serial.print("owInReverse  ");
    Serial.println(owInReverse);
    Serial.print("Break detect  ");
    Serial.println(brakeDetect);
    tt = millis();
  }
}

void get_fake_data()
{ 
  static float angle =0  ;
  float val;
  val=2*PI/360;
  static unsigned long fakeTimer = millis();
  if (millis() - fakeTimer > 250)
  {
    vescSwitchState=SWITCH_ON;
    vescErpm=MAX_ERPM*sin(val*angle);
    Serial.print("Fake Vesc Erpm :");
    Serial.println( vescErpm  );
    vescPid=100*sin(90-(val*angle));
    Serial.print("Fake Vesc pid :");
    Serial.println( vescErpm  );
    angle+=1;
    fakeTimer = millis();
  }
}

// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//

void loop()
{ 

  // Horn triggering
  triggerHorn();
  // Indicator (turn signal) triggering
  triggerIndicators();

  if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
  {
#ifdef FAKE_VESC_DATA

    get_fake_data(); // generate sin of erpm data and cos of pid data.

#else
    // update vesc date for 5ms
    VESC.updateCustomValues(5);
    // check audio source
    source = (AudioSource)VESC.getVescId();
    vescErpm = VESC.getErpm();
    vescPid = VESC.getPidOUtput();
    vescSwitchState = (SwitchState)VESC.getSwitchState();
#endif
   
    if (source == SOURCE_ESP32 )
    {
       CSR_POWER_OFF();
      AUDIO_SOURCE_ESP();

      if (vescSwitchState == SWITCH_ON)
      {
        engineOn = true;
        AUDIO_UNMUTE();
        engineOffTimer = millis();
      }
      else
      {
        if (millis() - engineOffTimer > engineOffDelay)
        {
          engineOn = false;
        }
      }

       mapThrottle();
    }  else {
      // turn on csr
      engineOn =false; 
      CSR_POWER_ON();
      AUDIO_SOURCE_CSR()
      AUDIO_UNMUTE();

    }
    // Map pulsewidth to throttle
    xSemaphoreGive(xRpmSemaphore); // Now free or "Give" the semaphore for others.
  }

  // RGB LED control
#if defined NEOPIXEL_ENABLED
  updateRGBLEDs();
#endif

  // Core ID debug
#if defined CORE_DEBUG
  Serial.print("Running on core ");
  Serial.println(coreId);
#endif
 check_mute(500); //500mS to check mute or unmute.
// Feeding the RTC watchtog timer is essential!
rtc_wdt_feed();

}

//
// =======================================================================================================
// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
// =======================================================================================================
//

void Task1code(void *pvParameters)
{
  for (;;)
  {

    // coreId = xPortGetCoreID(); // Running on core 0

    // DAC offset fader
    dacOffsetFade();

    if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
    {
      // Simulate engine mass, generate RPM signal

      engineMassSimulation();
      // Call gear selector
      if (automatic || doubleClutch)
        automaticGearSelector();
      xSemaphoreGive(xRpmSemaphore); // Now free or "Give" the semaphore for others.
    }

    // Switch engine on or off
    //engineOnOff();

    // Gearbox detection
    gearboxDetection();

    // ESC control & low discharge protection
    esc();
   debug_print();
    // measure loop time
    loopTime = loopDuration(); // for debug only
  }
}
