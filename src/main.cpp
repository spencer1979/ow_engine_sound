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
// #define VESC_DEBUG
// #define FAKE_VESC_DATA  0// random Number for test

// TODO = Things to clean up!

//
// =======================================================================================================
// LIRBARIES & HEADER FILES, REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder.
// No manual library download is required in Visual Studio Code IDE (see platformio.ini)
#include <statusLED.h>       // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control
#include <FastLED.h>         // https://github.com/FastLED/FastLED        <<------- required for Neopixel support. Use V3.3.3
#include <ESP32AnalogRead.h> // https://github.com/madhephaestus/ESP32AnalogRead <<------- required for battery voltage measurement
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
volatile boolean sound1Trigger = false;           // Trigger for sound1  on / off
volatile boolean couplingTrigger = false;         // Trigger for trailer coupling  sound
volatile boolean uncouplingTrigger = false;       // Trigger for trailer uncoupling  sound
volatile boolean bucketRattleTrigger = false;     // Trigger for bucket rattling  sound
volatile boolean indicatorSoundOn = false;        // active, if indicator bulb is on
volatile boolean outOfFuelMessageTrigger = false; // Trigger for out of fuel message
// new for float
volatile boolean lowVoltageTrigger = false;     // Trigger for low voltage message
volatile boolean overSpeedTrigger = false;      // Trigger for over speed message
volatile boolean excuseMeTrigger = false;       // Trigger for excuse me message
volatile boolean startUpWarnTrigger = false;    // Trigger for start up safty warning message
volatile boolean vescNotConnectTrigger = false; // Trigger for vesc not onnection message

// Sound latches
volatile boolean hornLatch = false;  // Horn latch bit
volatile boolean sirenLatch = false; // Siren latch bit
volatile boolean excuseMeLatch = false;
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
volatile uint16_t tireSquealVolume = 0;               // Tire squeal volume according to speed and cornering radius
volatile uint16_t trackRattleVolume = 0;              // track rattling volume

volatile uint64_t dacDebug = 0;      // DAC debug variable TODO
volatile int16_t masterVolume = 100; // Master volume percentage
volatile uint8_t dacOffset = 0;      // 128, but needs to be ramped up slowly to prevent popping noise, if switched on

// Throttle
int16_t currentThrottle = 0;      // 0 - 500(Throttle trigger input)
int16_t currentThrottleFaded = 0; // faded throttle for volume calculations etc.

// Engine
const int16_t maxRpm = 500;       // always 1000
const int16_t minRpm = 0;         // always 0
int32_t currentRpm = 0;           // 0 - 500 (signed required!)
volatile uint8_t engineState = 0; // Engine state

enum EngineState // Engine state enum
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

//
// =======================================================================================================
// vesc variable
// =======================================================================================================
//
#define NEUTRAL_VESC_ERPM 300.0 //  rang in -200 < 0 < 200 erpm is neutral
#define NEUTRAL_VESC_PID 10     //  rang in -5 < 0 < 5 erpm is neutral
#define MAX_VESC_ERPM 6000
#define MIN_VESC_ERPM 300
#define MAX_VESC_MCURRENT 100
#define MIN_VESC_MCURRENT 0
#define NUM_MEASUREMENTS 7
#define ENGINE_OFF_DELAY 1200 // 引擎關閉延遲時間
volatile uint8_t soundTriggered;
volatile int8_t vescEnableData;
volatile bool lastVescConnected, vescConnected, engineSoundEnable, startWarnEnable, lowBattEnable, variablePlaybackTimerRunning;
// Eanble last state
volatile bool lastEngineSoundEnable, lastSafetyWarnEnable, lastLowVoltTrigger, lastOverSpeedTrigger = false;
volatile float speedAvg, lowBattLevel, batteryLevelNow, overSpeedLimit = 0;
float vescErpm = 0, vescAbsErpm = 0, vescPid = 0, vescMCurrent = 0, vsecAbsMCurrent = 0, soundSamplingData = 0;

bool lock_esp_sound = false;
volatile FloatSwitchState vescSwitchState, lastVescSwitchState = FLOAT_SWITCH_OFF;
volatile FloatSamplingType soundSamplingType = FLOAT_MOTOR_CURRENT;

unsigned long engineOffTimer, sourceCheckTimer, timelast, timelastloop;
// audio source
volatile Audio_Source source;
// QueueHandle
QueueHandle_t speedAvgDataQueue = NULL;
QueueHandle_t speedDataQueue = NULL;

// Initiate VescUart class
VescUart VESC(100);

// DEBUG stuff
volatile uint8_t coreId = 99;
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

SemaphoreHandle_t xVescSemaphore;
SemaphoreHandle_t xRpmSemaphore;
// taskv handle
TaskHandle_t engineTaskHandle = NULL;
TaskHandle_t vescTaskHandle = NULL;
TaskHandle_t warningTaskHandle = NULL;
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

  portENTER_CRITICAL_ISR(&variableTimerMux); // disables C callable interrupts (on the current core) and locks the mutex by the current core.

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

  portEXIT_CRITICAL_ISR(&variableTimerMux);
}

//
// =======================================================================================================
// INTERRUPT FOR FIXED SPEED PLAYBACK (Horn etc., played in parallel with engine sound)
// =======================================================================================================
//

void IRAM_ATTR fixedPlaybackTimer()
{

  // coreId = xPortGetCoreID(); // Running on core 1

  static uint32_t curHornSample = 0;         // Index of currently loaded horn sample
  static uint32_t curSirenSample = 0;        // Index of currently loaded siren sample
  static uint32_t curSound1Sample = 0;       // Index of currently loaded sound1 sample
  static uint32_t curReversingSample = 0;    // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample = 0;    // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample = 0;    // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample = 0;        // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample = 0; // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample = 0;     // Index of currently loaded shifting sample
  static uint32_t curDieselKnockSample = 0;  // Index of currently loaded Diesel knock sample
  static uint32_t curTrackRattleSample = 0;  // Index of currently loaded track rattle sample
  static uint32_t curTireSquealSample = 0;   // Index of currently loaded tire squeal sample
  static uint32_t curOutOfFuelSample = 0;    // Index of currently loaded out of fuel sample

  // new for float app
  static uint32_t curOverSpeedSample = 0;
  static uint32_t curLowVoltageSample = 0;
  static uint32_t curExcuseMeSample = 0;
  static uint32_t curStartUpWarningSample = 0;
  static uint32_t curVescNotConnectSample = 0;
  //
  static int32_t a, a1, a2 = 0;                                      // Input signals "a" for mixer
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10 = 0; // Input signals "b" for mixer
  static int32_t c, c1, c2, c3 = 0;                                  // Input signals "c" for mixer
  static int32_t d, d1, d2 = 0;                                  // Input signals "d" for mixer
  static boolean knockSilent = 0;                                    // This knock will be more silent
  static boolean knockMedium = 0;                                    // This knock will be medium
  static uint8_t curKnockCylinder = 0;                               // Index of currently ignited zylinder

  portENTER_CRITICAL_ISR(&fixedTimerMux);

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
  // b0 Start up warning sound
  if (startUpWarnTrigger)
  {
    fixedTimerTicks = 4000000 / startUpWarningSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);   // change timer ticks, autoreload true

    if (curStartUpWarningSample < startUpWarningSampleCount - 1)
    {
      b0 = (startUpWarningSamples[curStartUpWarningSample] * startUpWarningPercentage / 100);
      curStartUpWarningSample++;
    }
    else
    {
      startUpWarnTrigger = false;
    }
  }
  else
  {
    curStartUpWarningSample = 0; // ensure, next sound will start @ first sample
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

  // Wastegate (blowoff) sound, triggered after rapid throttle drop -----------------------------------
  if (wastegateTrigger)
  {
    if (curWastegateSample < wastegateSampleCount - 1)
    {
      b2 = (wastegateSamples[curWastegateSample] * rpmDependentWastegateVolume / 100 * wastegateVolumePercentage / 100);
      curWastegateSample++;
    }
    else
    {
      wastegateTrigger = false;
    }
  }
  else
  {
    b2 = 0;
    curWastegateSample = 0; // ensure, next sound will start @ first sample
  }

  // Air brake release sound, triggered after stop -----------------------------------------------
  if (airBrakeTrigger)
  {
    if (curBrakeSample < brakeSampleCount - 1)
    {
      b3 = (brakeSamples[curBrakeSample] * brakeVolumePercentage / 100);
      curBrakeSample++;
    }
    else
    {
      airBrakeTrigger = false;
    }
  }
  else
  {
    b3 = 0;
    curBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Air parking brake attaching sound, triggered after engine off --------------------------------
  if (parkingBrakeTrigger)
  {
    if (curParkingBrakeSample < parkingBrakeSampleCount - 1)
    {
      b4 = (parkingBrakeSamples[curParkingBrakeSample] * parkingBrakeVolumePercentage / 100);
      curParkingBrakeSample++;
    }
    else
    {
      parkingBrakeTrigger = false;
    }
  }
  else
  {
    b4 = 0;
    curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
  if (shiftingTrigger && engineRunning && !automatic && !doubleClutch)
  {
    if (curShiftingSample < shiftingSampleCount - 1)
    {
      b5 = (shiftingSamples[curShiftingSample] * shiftingVolumePercentage / 100);
      curShiftingSample++;
    }
    else
    {
      shiftingTrigger = false;
    }
  }
  else
  {
    b5 = 0;
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
    b6 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#else // Just depending on throttle
    b6 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
    curDieselKnockSample++;
    if (knockSilent && !knockMedium)
      b6 = b6 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
    if (knockMedium)
      b6 = b6 * dieselKnockAdaptiveVolumePercentage / 75;
  }

  // sound b8
  // low Voltage sound
  if (lowVoltageTrigger)
  {
    fixedTimerTicks = 4000000 / lowVoltageSampleRate;   // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // change timer ticks, autoreload true

    if (curLowVoltageSample < lowVoltageSampleCount - 1)
    {
      b7 = (lowVoltageSamples[curLowVoltageSample] * lowVoltageVolumePercentage / 100);
      curLowVoltageSample++;
    }
    else
    {
      lowVoltageTrigger = false;
    }
  }
  else
  {
    curLowVoltageSample = 0; // ensure, next sound will start @ first sample
    b7 = 0;
  }

  // over speed
  if (overSpeedTrigger)
  {
    fixedTimerTicks = 4000000 / overSpeedSampleRate;    // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // change timer ticks, autoreload true

    if (curOverSpeedSample < overSpeedSampleCount - 1)
    {
      b8 = (overSpeedSamples[curOverSpeedSample] * overSpeedVolumePercentage / 100);
      curOverSpeedSample++;
    }
    else
    {
      overSpeedTrigger = false;
    }
  }
  else
  {
    curOverSpeedSample = 0; // ensure, next sound will start @ first sample
    b8 = 0;
  }

  //  excuse me

  if (excuseMeTrigger || excuseMeLatch)
  {
    fixedTimerTicks = 4000000 / excuseMeSampleRate;     // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curExcuseMeSample < excuseMeSampleCount - 1)
    {
      a1 = (excuseMeSamples[curExcuseMeSample] * excuseMeVolumePercentage / 100);
      curExcuseMeSample++;
#ifdef EXCUSE_ME_LOOP // Optional "endless loop" (points to be defined manually in excuseMe file)
      if (excuseMeTrigger && curExcuseMeSample == excuseMeLoopEnd)
        curExcuseMeSample = excuseMeLoopBegin; // Loop, if trigger still present
#endif
    }
    else
    { // End of sample
      curExcuseMeSample = 0;
      a1 = 0;
      excuseMeLatch = false;
    }
  }

  //  vesc not connection
  if (vescNotConnectTrigger)
  {
    fixedTimerTicks = 4000000 / vescNotConnectSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);   // change timer ticks, autoreload true

    if (curVescNotConnectSample < vescNotConnectSampleRate - 1)
    {
      b10 = (vescNotConnectSamples[curVescNotConnectSample] * vescNotConnectVolumePercentage / 100);
      curVescNotConnectSample++;
    }
    else
    {
      vescNotConnectTrigger = false;
    }
  }
  else
  {
    curVescNotConnectSample = 0; // ensure, next sound will start @ first sample
    b10 = 0;
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
  b = b0 * 5 + b1 + b2 + b3 + b4 + b5 + b6 + b7 * 5 + b8 * 2 + b9 * 2 + b10 * 2; // Other sounds
  c = c1 + c2 + c3;                                                              // Excavator sounds
  d = d1 + d2 ;                                                              // Additional sounds

  // DAC output (groups mixed together) ****************************************************************************

  // dacDebug = constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255); // Mix signals, add 128 offset, write result to DAC
  dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, constrain( a2 * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, 0);

   portEXIT_CRITICAL_ISR(&fixedTimerMux);
}

// Utility Functions

static float biquad_process(Biquad *biquad, float in)
{
  float out = in * biquad->a0 + biquad->z1;
  biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
  biquad->z2 = in * biquad->a2 - biquad->b2 * out;
  return out;
}

static void biquad_config(Biquad *biquad, BiquadType type, float Fc)
{
  float K = tanf(M_PI * Fc); // -0.0159;
  float Q = 0.707;           // maximum sharpness (0.5 = maximum smoothness)
  float norm = 1 / (1 + K / Q + K * K);
  if (type == BQ_LOWPASS)
  {
    biquad->a0 = K * K * norm;
    biquad->a1 = 2 * biquad->a0;
    biquad->a2 = biquad->a0;
  }
  else if (type == BQ_HIGHPASS)
  {
    biquad->a0 = 1 * norm;
    biquad->a1 = -2 * biquad->a0;
    biquad->a2 = biquad->a0;
  }
  biquad->b1 = 2 * (K * K - 1) * norm;
  biquad->b2 = (1 - K / Q + K * K) * norm;
}

static void biquad_reset(Biquad *biquad)
{
  biquad->z1 = 0;
  biquad->z2 = 0;
}
//
//===================================================== ===================================================== ===
// initialize one wheel
//===================================================== ===================================================== ===
//

void initIO()
{

  // set pin mode
  pinMode(CSR_EN_PIN, OUTPUT);
  pinMode(AUDIO_SOURCE_PIN, OUTPUT);
  pinMode(PAM_MUTE_PIN, OUTPUT);
  pinMode(EPS_BUTTON_PIN, INPUT);
  // Neopixel setup
  // FastLED.addLeds<NEOPIXEL, RGB_LED1_DATA_PIN>(rgb1LEDs, RGB_LED1_COUNT);
  // FastLED.addLeds<NEOPIXEL, RGB_LED2_DATA_PIN>(rgb2LEDs, RGB_LED2_COUNT);
  // #ifdef USE_DUAL_HEAD_LIGHT
  //   headLight0.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
  //   headLight1.begin(LED2_PIN, 1, 20000); // Timer 1, 20kHz
  // #else
  //   headLight.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
  // #endif
  //   tailLight.begin(LED3_PIN, 2, 20000); // Timer 2, 20kHz
  SET_AUDIO_MUTE();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  SET_CSR_POWER_ON();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  SET_CSR_POWER_OFF();
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

int16_t get_vesc_sampling_source(FloatSamplingType type)
{

  uint16_t throttle = 0;

  switch (type)
  {
  case FLOAT_MOTOR_CURRENT:

    throttle = (int16_t)map(fabsf(vescMCurrent), MIN_VESC_MCURRENT, MAX_VESC_MCURRENT, minRpm, maxRpm);
    break;

  case FLOAT_ERPM:
    throttle = (int16_t) map(fabsf(vescErpm), MIN_VESC_ERPM, MAX_VESC_ERPM, minRpm, maxRpm);
    
    break;

  case FLOAT_PID:

    throttle = (int16_t)map(fabsf(vescPid) ,MIN_VESC_MCURRENT, MAX_VESC_MCURRENT, minRpm, maxRpm);
   
    break;
  }

  if (throttle > maxRpm)
  {
    throttle = maxRpm;
  }
  if (throttle < minRpm)
  {
    throttle = minRpm;
  }
  return throttle;
}

int16_t get_vesc_speed(float d)
{
  int16_t speed;

  speed = (int16_t)map(constrain(fabsf(vescMCurrent), MIN_VESC_MCURRENT, MAX_VESC_MCURRENT), MIN_VESC_MCURRENT, MAX_VESC_MCURRENT, minRpm, maxRpm);

  if (speed > maxRpm)
  {
    speed = maxRpm;
  }
  if (speed < minRpm)
  {
    speed = minRpm;
  }
  return speed;
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
      rpmDependentJakeBrakeVolume = map(currentRpm, 0, maxRpm, jakeBrakeIdleVolumePercentage, 100);
    else
      rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    // Calculate RPM dependent Diesel knock volume
    if (currentRpm > (maxRpm * 0.8))
      rpmDependentKnockVolume = map(currentRpm, knockStartRpm, maxRpm, minKnockVolumePercentage, 100);
    else
      rpmDependentKnockVolume = minKnockVolumePercentage;
#endif

    // Calculate engine rpm dependent turbo volume
    if (engineRunning)
      throttleDependentTurboVolume = map(currentRpm, 0, maxRpm, turboIdleVolumePercentage, 100);
    else
      throttleDependentTurboVolume = turboIdleVolumePercentage;

    // Calculate engine rpm dependent cooling fan volume
    if (engineRunning && (currentRpm > fanStartPoint))
      throttleDependentFanVolume = map(currentRpm, fanStartPoint, maxRpm, fanIdleVolumePercentage, 100);
    else
      throttleDependentFanVolume = fanIdleVolumePercentage;

    // Calculate throttle dependent supercharger volume
    if (!owIsBraking && !brakeDetect && engineRunning && (currentRpm > chargerStartPoint))
      throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, maxRpm, chargerIdleVolumePercentage, 100);
    else
      throttleDependentChargerVolume = chargerIdleVolumePercentage;

    // Calculate engine rpm dependent wastegate volume
    if (engineRunning)
      rpmDependentWastegateVolume = map(currentRpm, 0, maxRpm, wastegateIdleVolumePercentage, 100);
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
  //   tireSquealVolume += map(currentThrottle, 250, maxRpm, 0, 100);
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

    if (_currentThrottle > maxRpm)
      _currentThrottle = maxRpm;

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
        if (targetRpm > maxRpm)
          targetRpm = maxRpm;
#else // Real 3 speed transmission
        targetRpm = reMap(curveLinear, currentSpeed);
#endif
      }
    }

    // Engine RPM **************************************************************************************

    if (owIsBraking && currentSpeed < clutchEngagingPoint)
      targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    if (targetRpm > maxRpm)
      targetRpm = maxRpm;

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

    // if ( xSemaphoreTake( xVescSemaphore, portMAX_DELAY ) )
    //{
    currentRpm = _currentRpm;
    // xSemaphoreGive( xVescSemaphore ); // Now free or "Give" the semaphore for others.
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
    if (currentRpm > (maxRpm * 0.98) && selectedGear == 3 && engineLoad < 5 && currentThrottle > 980 && millis() - lastShiftingMillis > 2000)
    {
      overdrive = true;
    }
    if (!owIsBraking)
    { // Lower downshift point, if not braking
      if (currentRpm < (maxRpm * 0.4) && millis() - lastShiftingMillis > 2000)
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
  if (currentRpm > (maxRpm * 0.7) && selectedGear < 3 && engineLoad < 5 && currentThrottle > (maxRpm * 0.7) && millis() - lastShiftingMillis > 2000)
  {
    selectedGear++;
  }
  if (!owIsBraking)
  { // Lower downshift point, if not braking
    if (currentRpm < (maxRpm * 0.3) && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--; //
    }
  }
  else
  { // Higher downshift point, if braking
    if ((currentRpm < (maxRpm * 0.65) || engineLoad > 150) && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
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
  if (millis() - downShiftingMillis > 400)
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
    DEBUG_PRINT("MANUAL_TRANS_DEBUG:\n");
    DEBUG_PRINT("currentThrottle: %i\n", currentThrottle);
    DEBUG_PRINT("selectedGear: %i\n", selectedGear);
    DEBUG_PRINT("overdrive: %i\n", overdrive);
    DEBUG_PRINT("engineLoad: %i\n", engineLoad);
    DEBUG_PRINT("sequentialLock: %s\n", sequentialLock ? "true" : "false");
    DEBUG_PRINT("currentRpm: %i\n", currentRpm);
    DEBUG_PRINT("currentSpeed: %i\n", currentSpeed);
    DEBUG_PRINT("---------------------------------\n");
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
  uint16_t downShiftPoint = (maxRpm * 0.3);
  uint16_t upShiftPoint = (maxRpm * 0.7);
  static int32_t _currentRpm = 0; // Private current RPM (to prevent conflict with core 1)

  // if ( xSemaphoreTake( xVescSemaphore, portMAX_DELAY ) )
  //{
  _currentRpm = currentRpm;
  // xSemaphoreGive( xVescSemaphore ); // Now free or "Give" the semaphore for others.
  // }

  if (millis() - gearSelectorMillis > 100)
  { // Waiting for 100ms is very important. Otherwise gears are skipped!
    gearSelectorMillis = millis();

    // compute load dependent shift points (less throttle = less rpm before shifting up, kick down will shift back!)
    upShiftPoint = map(engineLoad, 0, 180, 340, 440);   // 390, 490
    downShiftPoint = map(engineLoad, 0, 180, 100, 2); // 150, 250

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
    DEBUG_PRINT("AUTO_TRANS_DEBUG:\n");
    DEBUG_PRINT("currentThrottle: %i\n", currentThrottle);
    DEBUG_PRINT("selectedAutomaticGear: %i\n", selectedAutomaticGear);
    DEBUG_PRINT("engineLoad: %i\n", engineLoad);
    DEBUG_PRINT("upShiftPoint: %i\n", upShiftPoint);
    DEBUG_PRINT("_currentRpm: %i\n", _currentRpm);
    DEBUG_PRINT("downShiftPoint: %i\n", downShiftPoint);
    DEBUG_PRINT("-----------------------------------\n");
#endif
  }
}

//
// =======================================================================================================
// ESC CONTROL (including optional battery protection)
// =======================================================================================================
//

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
int8_t checkPulse(float val, uint32_t range)
{ // rpm direction
  int8_t _rpmPulse;
  if (((val + range) * (range - val)) > 0) // check val is in the range +/-range
  {
    _rpmPulse = 0; // 0 = Neutral
  }
  else
  {
    if (val > range)
      _rpmPulse = 1; // 1 = Forward
    else
      _rpmPulse = -1; // -1 = backwards
  }

  return _rpmPulse;
}

int8_t rpmPulse()
{ // rpm direction
  return checkPulse(vescErpm, NEUTRAL_VESC_ERPM);
}

int8_t pidPulse()
{
  return checkPulse(vescPid, NEUTRAL_VESC_PID);
}

// If you connect your ESC to pin 33, the vehicle inertia is simulated. Direct brake (crawler) ESC required
// *** WARNING!! Do it at your own risk!! There is a falisafe function in case, the signal input from the
// receiver is lost, but if the ESP32 crashes, the vehicle could get out of control!! ***

void esc()
{

  // ESC main function ================================
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
    brakeRampRate = map(currentThrottle, 0, maxRpm, 1, 10);
    driveRampRate = 10;
  }
  else
  { // Virtual inertia mode -----
    // calulate throttle dependent brake & acceleration steps
    brakeRampRate = map(currentThrottle, 0, maxRpm, 1, escBrakeSteps);
    driveRampRate = map(currentThrottle, 0, maxRpm, 1, escAccelerationSteps);
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
      if (rpmPulse() == 1 && engineRunning && !neutralGear)
        driveState = 1; // Driving forward
      if (rpmPulse() == -1 && engineRunning && !neutralGear)
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
        driveState = 3;    // Driving backwards, if ESC not yet moving. Prevents state machine from hanging! v9.7.0
      if (rpmPulse() == 0) // only check rpm for standing still
        driveState = 0;    // standing still
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
      if (rpmPulse() == 0)
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
      if (rpmPulse() == 0)
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
      if (rpmPulse() == 0)
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

    currentSpeed = get_vesc_speed(vescErpm);
  }
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



//啟動可變音源計時器（引擎聲）
void start_variable_playback_timer()
{
  // Interrupt timer for variable sample rate playback
  if (!variablePlaybackTimerRunning)
  {
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // autoreload true
    timerAlarmEnable(variableTimer);
    variablePlaybackTimerRunning = true; // enable
    DEBUG_PRINT("Start variable playback timer \n");
  }
}
//停止可變音源計時器（引擎聲）
void stop_variable_playback_timer()
{
  if (variablePlaybackTimerRunning)
  {
    timerAlarmDisable(variableTimer);
    variablePlaybackTimerRunning = false;
    DEBUG_PRINT("Stop variable playback timer \n");
  }
}
//改變音頻源
void change_audio_source(Audio_Source source)
{
  if (source == AUDIO_SOURCE_CSR)
  {
    SET_AUDIO_MUTE();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    SET_AUDIO_SOURCE_CSR();
    // delay 100ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
    SET_AUDIO_UNMUTE();
  }
  else if (source == AUDIO_SOURCE_ESP32)
  {

    SET_AUDIO_MUTE();
    // delay 100ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
    SET_AUDIO_SOURCE_ESP();
    // delay 100ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
    SET_AUDIO_UNMUTE();
  }
}

#ifdef FAKE_VESC_DATA

void get_fake_data()
{
  static float angle = 0;
  float val;
  val = 2 * PI / 360;
  static unsigned long fakeTimer = millis();
  if (millis() - fakeTimer > 250 && vescSwitchState)
  {


    vescErpm = MAX_VESC_ERPM * sin(val * angle);
    // DEBUG_PRINT("Fake Vesc Erpm :%.2f\n",vescErpm);
    vescPid = 100 * sin(90 - (val * angle));
    // DEBUG_PRINT("Fake Vesc pid : %.2f\n",vescPid);
    angle += 1;
    if (angle > 180)
    {
      angle = 0;
    }

    fakeTimer = millis();
  }
}
#endif

/// @brief 讀取vesc 觸發音效按鈕
/// @param sound 
void check_sound_triggered(uint8_t sound)
{
  //重設按鈕按下次數
  static uint8_t hornClickCount = 0;
  static uint8_t sirenClickCount = 0;
  static uint8_t excuseMeClickcount = 0;
  //  b   0    0    0     0     0     0     0     0 
  //                                  ^     ^     ^
  //                                  2     1     0
  // bit 0 :喇叭聲
  // bit 1 :借過
  // bit 2 :警笛聲
  
  if (soundTriggered > 0)
  {
    if (CHECK_BIT(soundTriggered, SOUND_HORN_TRIGGERED))
    {
      hornClickCount++;
      //檢查是否按鈕次數是否為奇數（開啟）
      if (hornClickCount % 2 == 1)
      {
        hornLatch = true;
        hornTrigger = true;
      }
      else //檢查是否按鈕次數是否為偶數 （關閉）
      {
        hornLatch = true;
        hornTrigger = false;
      }
      DEBUG_PRINT(" excuse triggered \n");
    }

    if (CHECK_BIT(soundTriggered, SOUND_EXCUSE_ME_TRIGGERED))
    {
      excuseMeClickcount++;
      if (excuseMeClickcount % 2 == 1)  //檢查是否按鈕次數是否為奇數（開啟）
      {
        excuseMeTrigger = true;
        excuseMeLatch = true;
      }
      else //檢查是否按鈕次數是否為偶數 （關閉）
      {
        excuseMeLatch = true;
        excuseMeTrigger = false;
      }

      DEBUG_PRINT(" excuse triggered \n");
    }

    if (CHECK_BIT(soundTriggered, SOUND_POLICE_TRIGGERED))  //檢查是否按鈕次數是否為奇數（開啟）
    {
      sirenClickCount++;
      if (sirenClickCount % 2 == 1)
      {
        sirenLatch = true;
        sirenTrigger = true;
      }
      else //檢查是否按鈕次數是否為偶數 （關閉）
      {
        sirenLatch = true;
        sirenTrigger = false;
      }
      DEBUG_PRINT(" excuse triggered \n");
    }
   //清除等待下次按鈕按下
   // soundTriggered = 0;
  }
}

//
// =======================================================================================================
// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
// =======================================================================================================
//

void reset_val()
{
  vescConnected = false;
  variablePlaybackTimerRunning = false;
  lastVescConnected = false;
  lastEngineSoundEnable = false;
  engineSoundEnable = false;
  vescMCurrent = 0;
  vescPid = 0;
  vescErpm = 0;
  vescAbsErpm = 0;
  vescEnableData = 0;
}

void reset_trigger(void)
{
  lowVoltageTrigger = false;     // Trigger for low voltage message
  overSpeedTrigger = false;      // Trigger for over speed message
  excuseMeTrigger = false;       // Trigger for excuse me message
  startUpWarnTrigger = false;    // Trigger for start up safty warning message
  vescNotConnectTrigger = false; // Trigger for vesc not onnection message
  // Sound latches
  hornLatch = false;  // Horn latch bit
  sirenLatch = false; // Siren latch bit
  excuseMeLatch = false;
}



/// @brief 用於產生動態引擎音效
void engineTask_func(void *pvParameters)
{

  for (;;)
  {

    // coreId = xPortGetCoreID(); // Running on core 0

    // DAC offset fader
    dacOffsetFade();

    if (xSemaphoreTake(xVescSemaphore, portMAX_DELAY))
    {

      engineMassSimulation();
      // Call gear selector
      if (automatic || doubleClutch)
        automaticGearSelector();
      // Gearbox detection
      gearboxDetection();

      xSemaphoreGive(xVescSemaphore); // Now free or "Give" the semaphore for others.
    }

    // Switch engine on or off
    // engineOnOff();

    // ESC control & low discharge protection
    esc();
    // debug_print();
    //  measure loop time
    loopTime = loopDuration(); // for debug only
  }
}




///
static void vesc_check_engine_enable_loop()
{
  if (vescConnected)
  {
    // 獲取信號防止同時使用uart讀取
    if (xSemaphoreTake(xVescSemaphore, portMAX_DELAY))
    {
      vescEnableData= VESC.get_enable_item_data();
      xSemaphoreGive(xVescSemaphore); // 釋放信號
    }
    // 需要在main loop呼叫 VESC.get_enable_item_data();
    engineSoundEnable = CHECK_BIT(vescEnableData, ENGINE_SOUND_ENABLE_MASK_BIT);

    // 引擎聲開/關 狀態改變
    if (lastEngineSoundEnable != engineSoundEnable)
    {
      // 將引擎聲設為關閉
      engineOn = false;
      //! 引擎為開啟

      if (engineSoundEnable)
      {
        DEBUG_PRINT("ENGINE SOUND is ON !\n");
        // 檢查task 是否是已經關閉的
        // if (engineTaskHandle == NULL)
        // { // 建立task
        //   if (xTaskCreatePinnedToCore(engineTask_func, "engineTask", 1024 * 3, NULL, 1, &engineTaskHandle, 1) == pdPASS)
        //   {
        //     DEBUG_PRINT("Task engineTask created successfully\n");
            
        //   }
        //   else
        //   { // 建立task失敗
        //     DEBUG_PRINT("Task Failed to create engineTask\n");
        //   }
        // }
        // 啟動可變音效計時器
  
            start_variable_playback_timer();
            // 關閉csr8645 電源
            SET_CSR_POWER_OFF();
            // 將音源切換至esp32
            change_audio_source(AUDIO_SOURCE_ESP32);
      }
      //! 引擎聲為關閉
      else
      {
        DEBUG_PRINT("ENGINE SOUND if OFF!\n");

        // // 刪除freertos任務
        // if (engineTaskHandle != NULL)
        // {

        //   vTaskDelete(engineTaskHandle);
        //   engineTaskHandle = NULL;
        //   // 建立task失敗
        //   if (engineTaskHandle == NULL)
        //     DEBUG_PRINT("Task vescTask deleted successfully!\n");
        // }
        // 停止可變音效計時器
        stop_variable_playback_timer();
        // 將音源切換至csr8645藍芽
        change_audio_source(AUDIO_SOURCE_CSR);
        // 打開csr8645電源
        SET_CSR_POWER_ON();
      }
       // 更新狀態
    lastEngineSoundEnable = engineSoundEnable;
    }
   
  }
  else
  {
    // 可以寫一個連續偵測uart ,當突然連線
    return;
  }
}


// @brief This task is used to check overspeed, low battery , horn sound and check engine sound source.
/// @param pvParameters
void vesc_warningTask_func(void *pvParameters)
{
   //重設所有觸發音效
  reset_trigger();
  // 超速
  int speedCount = 0;
  float speedAverage = 0, speedKmh = 0;
  float speedRawData[NUM_MEASUREMENTS];
 //檢查音效觸發變數,使用於音源切換
  bool setAudioSourceToESP = 0;
  //低電壓警告延遲時間
  unsigned long lowVoltTriggerTime = 0;
  //超速警告延遲時間
  unsigned long overSpeedTriggerTime = 0;

  for (;;)
  { 
    if (vescConnected)  //!VESC 已連接
    {
      if (xSemaphoreTake(xVescSemaphore, portMAX_DELAY))
      { // read data from vesc
        VESC.advancedUpdate();
        soundTriggered = VESC.get_sound_triggered();
        //! Over speed setting cheaged
        if (overSpeedLimit != (float)VESC.get_over_speed_value())
        {

          overSpeedLimit = (float)VESC.get_over_speed_value();
          DEBUG_PRINT("overSpeed limit changed!! %.2f\n", overSpeedLimit);
        }
        // Low battery warning level setting cheaged
        if (lowBattLevel != VESC.get_low_battery_warning_level())
        {

          lowBattLevel = VESC.get_low_battery_warning_level();
          DEBUG_PRINT("lowBattLevel changed!!%.2f\n", lowBattLevel);
        }
        // Engine sound volume setting changed
        if (masterVolume != VESC.get_engine_sound_volume())

        {
          masterVolume = VESC.get_engine_sound_volume();
          DEBUG_PRINT("masterVolume changed!!%d\n", masterVolume);
        }
        // Engine sound source changed
        if (soundSamplingType != (FloatSamplingType)VESC.get_engine_sampling())
        {

          soundSamplingType = (FloatSamplingType)VESC.get_engine_sampling();
          DEBUG_PRINT("soundSamplingType %d\n", soundSamplingType);
        }
        xSemaphoreGive(xVescSemaphore); // 釋放信號量
      }
      
      // 檢查按鈕是否按下,觸發音效
      check_sound_triggered(soundTriggered);
      // 檢查電量是否低於某設定值
      if (lowBattLevel > 0.0)
      {
        // 低電壓警告
        if ((VESC.get_battery_level() * 100.00) < lowBattLevel && !lowVoltageTrigger && ((millis() - lowVoltTriggerTime) > 10000))
        {
          // 低電量音效觸發
          lowVoltageTrigger = true;
          DEBUG_PRINT("low Battery Level: %.2f\n", lowBattLevel);
          // 低電量音效觸發時關閉引擎聲
          if (engineSoundEnable)
          {
            stop_variable_playback_timer();
          }
          lowVoltTriggerTime = millis();
        }
      }
      else
      { // 低電量解除
        lowVoltageTrigger = false;
      }
      // 檢測超速
      if (overSpeedLimit > 0)
      {
        // 從vescTask 讀取ERPM , 做速度平均運算
        if (xQueueReceive(speedAvgDataQueue, &speedRawData[speedCount], (TickType_t)10) == pdTRUE)
        {
          /**計算行駛速度
           * rpm = erpm / (poles / 2.0)
            kmh = ((rpm / 60.0) * wheel_d * M_PI / gearing) * 3.6;
           * Speed = ((motor poles/2) * 60 *gear ratio ) / (wheel diameter * 3.1415)
           *
           */
          speedCount++;
          if (speedCount == NUM_MEASUREMENTS)
          {
            speedRawData[5] = speedRawData[4];
            speedRawData[4] = speedRawData[3];
            speedRawData[3] = speedRawData[2];
            speedRawData[2] = speedRawData[1];
            speedRawData[1] = speedRawData[0];
            speedRawData[0] = speedRawData[6];
            speedAverage = (speedRawData[0] + speedRawData[1] + speedRawData[2] + speedRawData[3] + speedRawData[4] + speedRawData[5]) / 6;
            // store average in global variable speedAvg
            speedKmh = (speedAverage / (MOTOR_POLES / 2));
            speedKmh = ((speedKmh / 60) * WHEEL_DIAMETER * M_PI / GEAR_RATIO) * 3.6;
            DEBUG_PRINT(" Speed average data  : %.2f km/H \n", speedKmh);
            speedCount = 0;
          }
        }
        // 檢測是否超速
        if (speedKmh > overSpeedLimit && !overSpeedTrigger && (millis() - overSpeedTriggerTime > 6000))
        {
          DEBUG_PRINT("Over Speed !!real speed :%.2f\n  limit speed :%.2f", speedKmh, overSpeedLimit);
          overSpeedTrigger = true;
          // 如果有使用引擎聲功能,超速須先停止引擎聲,讓使用者聽到超速聲.
          if (engineSoundEnable)
          {
            stop_variable_playback_timer();
          }
          speedKmh = 0;
          // reset time
          overSpeedTriggerTime = millis();
        }
      }
      else
      {
        overSpeedTrigger = false;
      }
      // 超速狀態改變
      //  如果超速音效已經觸發結束,且引擎是開啟的,將引擎聲計時器打開,繼續產生引擎聲.
      if (lastOverSpeedTrigger != overSpeedTrigger)
      {

        if (!overSpeedTrigger && engineSoundEnable)
        {
          start_variable_playback_timer();
          if (engineOn == true)
            engineState = RUNNING;
        }
      }

      // 低電量狀態改變
      //  如果低電量音效已經觸發結束,且引擎是開啟的,將引擎聲計時器打開,繼續產生引擎聲.
      if (lastLowVoltTrigger != lowVoltageTrigger && engineSoundEnable)
      {
        if (lowVoltageTrigger == false)
        {
          start_variable_playback_timer();
          if (engineOn == true)

            engineState = RUNNING;
        }
      }
      // 當引擎聲關閉時,檢查音效是否觸發,如果是將切換音源到esp端
      setAudioSourceToESP = engineSoundEnable || sirenTrigger || excuseMeTrigger || hornTrigger || overSpeedTrigger || lowVoltageTrigger;

      if (setAudioSourceToESP)
      {
        SET_AUDIO_SOURCE_ESP();
        // DEBUG_PRINT("Sound on ESP32\n");
      }
      else
      {
        SET_AUDIO_SOURCE_CSR();
        // DEBUG_PRINT("Sound on CSR\n");
      }
      lastOverSpeedTrigger = overSpeedTrigger;
      lastLowVoltTrigger = lowVoltageTrigger;
    }
    vesc_check_engine_enable_loop();
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
// vesc 連線 ,用於中途斷線使用
static void vesc_connected()
{
  if (vescConnected)
  {

    // 檢查哪些選項是啟動的
    // EXT_DCDC_ENABLE_MASK_BIT=0,
    // ENGINE_SOUND_ENABLE_MASK_BIT,
    // START_UP_WARNING_ENABLE_MASK_BIT,
    vescEnableData = VESC.get_enable_item_data();
    // EXT_DCDC_ENABLE_MASK_BIT is not avalible
    startWarnEnable = CHECK_BIT(vescEnableData, START_UP_WARNING_ENABLE_MASK_BIT);
    engineSoundEnable = CHECK_BIT(vescEnableData, ENGINE_SOUND_ENABLE_MASK_BIT);
    DEBUG_PRINT("Start warning Enable : %s\n", startWarnEnable ? "true" : "false");
    DEBUG_PRINT("Enable sound Enable : %s\n", engineSoundEnable ? "true" : "false");
    DEBUG_PRINT("Low Battery Warn Enable : %s\n", engineSoundEnable ? "true" : "false");

    /****
     * 讀取資料
        uint8_t lights_mode;
        uint8_t idle_warning_time;
        uint16_t engine_sound_volume;
        uint8_t over_speed_warning;
        float battery_level;
        float low_battery_warning_level;
        uint8_t engine_sampling_data;
    */
    if (VESC.advancedUpdate())
    {
      masterVolume = (uint16_t)VESC.get_engine_sound_volume();
      lowBattLevel = VESC.get_low_battery_warning_level();
      batteryLevelNow = VESC.get_battery_level();
      overSpeedLimit = VESC.get_over_speed_value();
      soundSamplingType = (FloatSamplingType)VESC.get_engine_sampling();
    }
    else
    {
      masterVolume = 100;
      lowBattLevel = 0;
      batteryLevelNow = 0;
      overSpeedLimit = 0;
      soundSamplingType = FLOAT_MOTOR_CURRENT;
    }

    // 啟動安全聲判斷
    if (startWarnEnable)
    {
      change_audio_source(AUDIO_SOURCE_ESP32);
      startUpWarnTrigger = true; // Turn on the startup warning sound
      DEBUG_PRINT("Start warning sound \n");
      while (startUpWarnTrigger)
      {
        ;
      }
    }
 // 建立引擎聲task
      if (xTaskCreatePinnedToCore(engineTask_func, "engineTask", 1204 * 3, NULL, 1, &engineTaskHandle, 1) == pdPASS)
      {
        DEBUG_PRINT("Task engineTask created successfully\r\n");
      }
      else
      {
        DEBUG_PRINT("Task Failed to create engineTask\r\n");
      }

    // 檢查引擎聲是否開啟
    if (engineSoundEnable)
    {
     
      // 起動引擎聲計時器
      start_variable_playback_timer();
      // 建立task
      engineOn = false;
      vescSwitchState = FLOAT_SWITCH_OFF;
      // 切換音源到ESP32
      change_audio_source(AUDIO_SOURCE_ESP32);
      SET_CSR_POWER_OFF();
    }
    else
    { // 切換音源到csr8645
      change_audio_source(AUDIO_SOURCE_CSR);
      stop_variable_playback_timer();
      SET_CSR_POWER_ON();
    }

    // 建立警告聲觸發Task
    if (xTaskCreatePinnedToCore(vesc_warningTask_func, "warningTask", 1024 * 10, NULL, 2, &warningTaskHandle, 0) == pdPASS)

    {
      DEBUG_PRINT("Task warningTask created successfully\r\n");
    }
    else
    {
      DEBUG_PRINT("Task Failed to create warningTask\r\n");
    }
  }
}
//
static void vesc_not_connected()
{
   if (!vescConnected)
  {
    //切換音源到csr8645
    change_audio_source(AUDIO_SOURCE_ESP32);
    //播放未連線音效
    vescNotConnectTrigger = true;
    while (vescNotConnectTrigger)
    {
      ;
    }
    //延遲0.5秒
    vTaskDelay(500/portTICK_RATE_MS);
    //播放啟動安全警告
    startUpWarnTrigger = true;
    while (startUpWarnTrigger)
    {
      ;
    }
     //延遲0.5秒
    vTaskDelay(500/portTICK_RATE_MS);
    //切換音源至csr8645
    change_audio_source(AUDIO_SOURCE_CSR);
    //打開csr8645
    SET_CSR_POWER_ON();
    DEBUG_PRINT("ENTER CSR SPEAKER  mode only.\n");
  }
}
void ow_setup()
{ // 初始化io
  initIO();

  Serial.begin(115200); // USB serial (for DEBUG) Mode, Rx pin (99 = not used), Tx pin
  // VESC serial
  Serial2.begin(115200, SERIAL_8N1, ESP_VESC_TX_PIN, ESP_VESC_RX_PIN);

// #define   VESC_DEBUG
#ifdef VESC_DEBUG
  VESC.setDebugPort(&Serial);
#endif
  // 設置與vesc連線的串列
  VESC.setSerialPort(&Serial2);
  //
  reset_val();

  int tries = 0;

  do
  {
    unsigned long startTime = millis();
    while (millis() - startTime < 10000)
    {
      if (VESC.get_vesc_ready()) // check vesc fw version , if not connect will get 0.0
      {
        vescConnected = true;
        DEBUG_PRINT(" VESC is connected\n");
        break;
      }
    }
    if (vescConnected)
    {
      break;
    }
    else
    {
      tries++;
      DEBUG_PRINT("UART retry time:%d\n", tries);
    }

  } while (tries < 2);

  vesc_connected();
  vesc_not_connected();
  // update status
  lastVescConnected = vescConnected;
  lastEngineSoundEnable = engineSoundEnable;
}


static void vesc_engine_loop()
{
  if (vescConnected)
  {

    // 獲取信號防止同時使用uart讀取
    if (xSemaphoreTake(xVescSemaphore, portMAX_DELAY))
    {
      VESC.soundUpdate();
      xSemaphoreGive(xVescSemaphore); // 釋放信號
    }
    // 讀取vesc 馬達電流
    vescMCurrent = VESC.get_motor_current();
    // 讀取float app 計算後的pid (也就是馬達所需電流)
    vescPid = VESC.get_pid_output();
    // 讀取vesc erpm ,並限制最大及最小值防止資料型態轉換時overflow
    //?是否使用機械轉速？
    vescErpm = constrain(VESC.get_erpm(), 0.0, (float)MAX_VESC_ERPM);
    // 轉換為絕對電子轉速
    vescAbsErpm = fabsf(vescErpm);
    // 利用隊列發送數據至vesc_warinig task ,來做超速平均計算
    xQueueSend(speedAvgDataQueue, &vescAbsErpm, (TickType_t)0);
    // 檢查獨輪車開關是否開啟
    vescSwitchState = (FloatSwitchState)VESC.get_switch_state();
    // 開關狀態改變,並且引擎聲功能須開啟

    // 獨輪車開關開啟,
    if (vescSwitchState == FLOAT_SWITCH_ON)
    {
      //! 觸發引擎啟動聲
      engineOn = true;
      engineOffTimer = millis();
    }
    else //! 獨輪車開關關閉
    {    // 將油門設為0
      // currentThrottle = 0;
      // 延遲關閉引擎聲,直接關閉比較不自然
      if (millis() - engineOffTimer > ENGINE_OFF_DELAY)
      {
        engineOn = false;
      }
    }
    // 重設開關
    lastVescSwitchState = vescSwitchState;

    currentThrottle = get_vesc_sampling_source(soundSamplingType);
    mapThrottle();
  }
  else
  {
    return;
  }
}
// Setup
void setup()
{

  // Watchdog timers need to be disabled, if task 1 is running without delay(1)
  disableCore0WDT();
  disableCore1WDT(); // TODO leaving this one enabled is experimental!
  // Setup RTC (Real Time Clock) watchdog
  // rtc_wdt_protect_off(); // Disable RTC WDT write protection
  // rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  // rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  // rtc_wdt_set_time(RTC_WDT_STAGE0, 10000); // set 10s timeout
  // rtc_wdt_enable();                        // Start the RTC WDT timer
  // rtc_wdt_disable();            // Disable the RTC WDT timer
  // rtc_wdt_protect_on(); // Enable RTC WDT write protection
  // Serial setup

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the PWM variable.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.

  if (xVescSemaphore == NULL) // Check to confirm that the RPM Semaphore has not already been created.
  {
    xVescSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xVescSemaphore) != NULL)
      xSemaphoreGive((xVescSemaphore)); // Make the RPM variable available for use, by "Giving" the Semaphore.
  }

  // Refresh sample intervals (important, because MAX_RPM_PERCENTAGE was probably changed above)
  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

  // Interrupt timer for fixed sample rate playback
  fixedTimer = timerBegin(1, 20, true);                              // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true);       // edge (not level) triggered
  timerAlarmWrite(fixedTimer, fixedTimerTicks, true);                // autoreload true
  timerAlarmEnable(fixedTimer);                                      // enable
                                                                     // Interrupt timer for variable sample rate playback
  variableTimer = timerBegin(0, 20, true);                           // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true); // edge (not level) triggered
  // create queue for average data
  speedAvgDataQueue = xQueueCreate(NUM_MEASUREMENTS, sizeof(float));
  speedDataQueue = xQueueCreate(100, sizeof(int16_t));
  // setup for wheel
  ow_setup();
}


// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//
void loop()
{
 
    vesc_engine_loop();
    

}

