/* RC engine sound & LED controller for Arduino ESP32. Written by TheDIYGuy999
    Based on the code for ATmega 328: https://github.com/TheDIYGuy999/Rc_Engine_Sound

 *  ***** ESP32 CPU frequency must be set to 240MHz! *****
    ESP32 macOS Big Sur fix see: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/BigSurFix.md

   Sound files converted with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/Audio2Header.html
   Original converter code by bitluni (send him a high five, if you like the code)

   Parts of automatic transmision code from Wombii's fork: https://github.com/Wombii/Rc_Engine_Sound_ESP32

   Dashboard, Neopixel and SUMD support by Gamadril: https://github.com/Gamadril/Rc_Engine_Sound_ESP32

   NEW: Visual Studio Code IDE support added (you need to install PlatformIO)
   Arduino IDE is supported as before, but stuff was renamed and moved to different folders!
*/

const float codeVersion = 8.2; // Software revision.

// This stuff is required for Visual Studio Code IDE, if .ino is renamed into .cpp!
#include <Arduino.h>
#include "ow_control_config.h"
#include <VescUart.h> //VESC uart communication

void Task1code(void *parameters);

//
// =======================================================================================================
// ERROR CODES (INDICATOR LIGHTS)
// =======================================================================================================
//
/* Constantly on = no SBUS signal (check "sbusInverted" true / false in "2_adjustmentsRemote.h")
   Number of blinks = this channel signal is not between 1400 and 1600 microseconds and can't be auto calibrated
   (check channel trim settings)
*/

//
// =======================================================================================================
// ! ! I M P O R T A N T ! !   ALL USER SETTINGS ARE DONE IN THE FOLLOWING TABS, WHICH ARE DISPLAYED ABOVE
// (ADJUST THEM BEFORE CODE UPLOAD), DO NOT CHANGE ANYTHING IN THIS TAB EXCEPT THE DEBUG OPTIONS
// =======================================================================================================
//

// All the required user settings are done in the following .h files:
#include "1_adjustmentsVehicle.h"      // <<------- Select the vehicle you want to simulate
#include "4_adjustmentsTransmission.h" // <<------- Transmission related adjustments
#include "8_adjustmentsSound.h"        // <<------- Sound related adjustments
#
// DEBUG options can slow down the playback loop! Only uncomment them for debugging, may slow down your system!
//#define CHANNEL_DEBUG // uncomment it for input signal debugging informations
//#define ESC_DEBUG // uncomment it to debug the ESC
//#define AUTO_TRANS_DEBUG // uncomment it to debug the automatic transmission
//#define MANUAL_TRANS_DEBUG // uncomment it to debug the manual transmission
//#define TRACKED_DEBUG // debugging tracked vehicle mode
//#define SERVO_DEBUG // uncomment it for servo calibration in BUS communication mode
//#define DEBUG_THROTTLE
// TODO = Things to clean up!

//
// =======================================================================================================
// LIRBARIES & HEADER FILES, REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder.
// No manual library download is required in Visual Studio Code IDE (see platformio.ini)
#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control
#include <FastLED.h>   // https://github.com/FastLED/FastLED        <<------- required for Neopixel support
// Additional headers
#include "src/curves.h"         // Nonlinear throttle curve arrays
#include <ESP32AnalogRead.h>    //esp32 adc read
#include <AwesomeClickButton.h> //Handle click on a button
// Objects *************************************************************************************
// Status LED objects (also used for PWM shaker motor and ESC control) -----
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
volatile boolean dieselKnockTriggerFirst = false; // The first  Diesel ignition "knock" per sequence
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
volatile uint16_t tireSquealVolume = 0;               // Tire squeal volume according to speed and cornering radius
// for excavator mode:
volatile uint16_t hydraulicPumpVolume = 0;             // hydraulic pump volume
volatile uint16_t hydraulicFlowVolume = 0;             // hydraulic flow volume
volatile uint16_t trackRattleVolume = 0;               // track rattling volume
volatile uint16_t hydraulicDependentKnockVolume = 100; // engine Diesel knock volume according to hydraulic load
volatile uint16_t hydraulicLoad = 0;                   // Hydraulic load dependent RPM drop

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
boolean gearDownShiftingInProgress;   // Active while shifting downwards
boolean gearUpShiftingPulse;          // Active, if shifting upwards begins
boolean gearDownShiftingPulse;        // Active, if shifting downwards begins
volatile boolean neutralGear = false; // Transmission in neutral

// ESC
volatile boolean escIsBraking = false; // ESC is in a braking state
volatile boolean escIsDriving = false; // ESC is in a driving state
volatile boolean escInReverse = false; // ESC is driving or braking backwards
int8_t driveState = 0;                 // for ESC state machine
int16_t escPulseMax;                   // ESC calibration variables
int16_t escPulseMin;
int16_t escPulseMaxNeutral;
int16_t escPulseMinNeutral;
uint16_t currentSpeed = 0; // 0 - 500 (current ESC power)

// Lights
int8_t lightsState = 0;                        // for lights state machine
volatile boolean lightsOn = false;             // Lights on
volatile boolean headLightsFlasherOn = false;  // Headlights flasher impulse (Lichthupe)
volatile boolean headLightsHighBeamOn = false; // Headlights high beam (Fernlicht)
volatile boolean blueLightTrigger = false;     // Bluelight on (Blaulicht)
boolean indicatorLon = false;                  // Left indicator (Blinker links)
boolean indicatorRon = false;                  // Right indicator (Blinker rechts)
boolean fogLightOn = false;                    // Fog light is on
boolean cannonFlash = false;                   // Flashing cannon fire
// VESC var
const int16_t maxDuty = 95; // 65% as max duty
const int16_t minDuty = 1;  //  0%
const int16_t maxErpm = 4000;
const int16_t minErpm = 100;
const uint32_t engineOffDelay = 5000; // engine off delay
unsigned long vescReadDelay;
volatile unsigned long timelast;
unsigned long timelastloop;
//// Initiate VescUart class
VescUart VESC;
//// push button switch trigger houn sound on esp32 pin 0
AwesomeClickButton soundButton(PUSH_BUTTON_PIN);
// Battery voltage
#ifdef READ_BATTERY_VOL_FROM_ADC
ESP32AnalogRead battery;
#endif
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

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xRpmSemaphore;
SemaphoreHandle_t xVescSemaphore;
//
// =======================================================================================================
// INTERRUPT FOR VARIABLE SPEED PLAYBACK (Engine sound, turbo sound)
// =======================================================================================================
//

void IRAM_ATTR variablePlaybackTimer()
{

  static uint32_t attenuatorMillis = 0;
  static uint32_t curEngineSample = 0;    // Index of currently loaded engine sample
  static uint32_t curRevSample = 0;       // Index of currently loaded engine rev sample
  static uint32_t curTurboSample = 0;     // Index of currently loaded turbo sample
  static uint32_t curFanSample = 0;       // Index of currently loaded fan sample
  static uint32_t curChargerSample = 0;   // Index of currently loaded charger sample
  static uint32_t curStartSample = 0;     // Index of currently loaded start sample
  static uint32_t curJakeBrakeSample = 0; // Index of currently loaded jake brake sample

  static uint32_t lastDieselKnockSample = 0;    // Index of last Diesel knock sample
  static uint16_t attenuator = 0;               // Used for volume adjustment during engine switch off
  static uint16_t speedPercentage = 0;          // slows the engine down during shutdown
  static int32_t a, a1, a2, a3, b, c, d, e = 0; // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
  static int32_t f = 0;                         // Input signals for mixer: f = hydraulic pump
  uint8_t a1Multi = 0;                          // Volume multipliers

  // portENTER_CRITICAL_ISR(&variableTimerMux);

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

    // Hydraulic pump sound -----------------------
#if defined EXCAVATOR_MODE
    static uint32_t curHydraulicPumpSample = 0; // Index of currently loaded hydraulic pump sample
    if (curHydraulicPumpSample < hydraulicPumpSampleCount - 1)
    {
      f = (hydraulicPumpSamples[curHydraulicPumpSample] * hydraulicPumpVolumePercentage / 100 * hydraulicPumpVolume / 100);
      curHydraulicPumpSample++;
    }
    else
    {
      curHydraulicPumpSample = 0;
    }
#endif

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

  dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
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
  static uint32_t curCouplingSample = 0;     // Index of currently loaded trailer coupling sample
  static uint32_t curUncouplingSample = 0;   // Index of currently loaded trailer uncoupling sample
#if defined EXCAVATOR_MODE
  static uint32_t curHydraulicFlowSample = 0; // Index of currently loaded hydraulic flow sample
  static uint32_t curTrackRattleSample = 0;   // Index of currently loaded track rattle sample
  static uint32_t curBucketRattleSample = 0;  // Index of currently loaded bucket rattle sample
#endif
#if defined TIRE_SQUEAL
  static uint32_t curTireSquealSample = 0; // Index of currently loaded tire squeal sample
#endif
  static int32_t a, a1, a2 = 0;                                 // Input signals "a" for mixer
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9 = 0; // Input signals "b" for mixer
  static int32_t c, c1, c2, c3 = 0;                             // Input signals "c" for mixer
  static int32_t d = 0;                                         // Input signals "d" for mixer
  static boolean knockSilent = 0;                               // This knock will be more silent
  static boolean knockMedium = 0;                               // This knock will be medium
  static uint8_t curKnockCylinder = 0;                          // Index of currently ignited zylinder

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
  if (curSirenSample > 10 && curSirenSample < 500)
    cannonFlash = true; // Tank cannon flash triggering in TRACKED_MODE
  else
    cannonFlash = false;

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
  if (engineRunning && escInReverse)
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

  if (curDieselKnockSample < knockSampleCount)
  {
#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#elif defined EXCAVATOR_MODE // knock volume also depending on hydraulic load
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * hydraulicDependentKnockVolume / 100);
#else                        // Just depending on throttle
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
    curDieselKnockSample++;
    if (knockSilent && !knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
    if (knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 75;
  }

#if not defined EXCAVATOR_MODE
  // Trailer coupling sound, triggered by switch -----------------------------------------------
#ifdef COUPLING_SOUND
  if (couplingTrigger)
  {
    if (curCouplingSample < couplingSampleCount - 1)
    {
      b8 = (couplingSamples[curCouplingSample] * couplingVolumePercentage / 100);
      curCouplingSample++;
    }
    else
    {
      couplingTrigger = false;
    }
  }
  else
  {
    b8 = 0;
    curCouplingSample = 0; // ensure, next sound will start @ first sample
  }

  // Trailer uncoupling sound, triggered by switch -----------------------------------------------
  if (uncouplingTrigger)
  {
    if (curUncouplingSample < uncouplingSampleCount - 1)
    {
      b9 = (uncouplingSamples[curUncouplingSample] * couplingVolumePercentage / 100);
      curUncouplingSample++;
    }
    else
    {
      uncouplingTrigger = false;
    }
  }
  else
  {
    b9 = 0;
    curUncouplingSample = 0; // ensure, next sound will start @ first sample
  }
#endif
#endif

  // Group "c" (excavator sounds) **********************************************************************

#if defined EXCAVATOR_MODE

  // Hydraulic fluid flow sound -----------------------
  if (curHydraulicFlowSample < hydraulicFlowSampleCount - 1)
  {
    c1 = (hydraulicFlowSamples[curHydraulicFlowSample] * hydraulicFlowVolumePercentage / 100 * hydraulicFlowVolume / 100);
    curHydraulicFlowSample++;
  }
  else
  {
    curHydraulicFlowSample = 0;
  }

  // Track rattle sound -----------------------
  if (curTrackRattleSample < trackRattleSampleCount - 1)
  {
    c2 = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
    curTrackRattleSample++;
  }
  else
  {
    curTrackRattleSample = 0;
  }

  // Track rattle sound -----------------------
  if (bucketRattleTrigger)
  {
    if (curBucketRattleSample < bucketRattleSampleCount - 1)
    {
      c3 = (bucketRattleSamples[curBucketRattleSample] * bucketRattleVolumePercentage / 100);
      curBucketRattleSample++;
    }
    else
    {
      bucketRattleTrigger = false;
    }
  }
  else
  {
    c3 = 0;
    curBucketRattleSample = 0; // ensure, next sound will start @ first sample
  }
#endif

  // Group "d" (additional sounds) **********************************************************************

#if defined TIRE_SQUEAL

  // Tire squeal sound -----------------------
  if (curTireSquealSample < tireSquealSampleCount - 1)
  {
    d = (tireSquealSamples[curTireSquealSample] * tireSquealVolumePercentage / 100 * tireSquealVolume / 100);
    curTireSquealSample++;
  }
  else
  {
    curTireSquealSample = 0;
  }
#endif

  // Mixing sounds together **********************************************************************
  a = a1 + a2;                                                 // Horn & siren
  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds
  c = c1 + c2 + c3;                                            // Excavator sounds

  // DAC output (groups mixed together) ****************************************************************************

  dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, constrain( c * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
  // dacWrite(DAC2, 0);

  // portEXIT_CRITICAL_ISR(&fixedTimerMux);
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup()
{

  // Watchdog timers need to be disabled, if task 1 is running without delay(1)
  disableCore0WDT();
  // disableCore1WDT();

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the PWM variable.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if (xRpmSemaphore == NULL) // Check to confirm that the PWM Semaphore has not already been created.
  {
    xRpmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xRpmSemaphore) != NULL)
      xSemaphoreGive((xRpmSemaphore)); // Make the PWM variable available for use, by "Giving" the Semaphore.
  }
  if (xVescSemaphore == NULL) // Check to confirm that the PWM Semaphore has not already been created.
  {
    xVescSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xVescSemaphore) != NULL)
      xSemaphoreGive((xVescSemaphore)); // Make the PWM variable available for use, by "Giving" the Semaphore.
  }

#ifdef USE_DUAL_HEAD_LIGHT
  headLight0.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
  headLight1.begin(LED2_PIN, 1, 20000); // Timer 1, 20kHz
#else
  headLight.begin(LED1_PIN, 1, 20000); // Timer 1, 20kHz
#endif
  tailLight.begin(LED3_PIN, 2, 20000); // Timer 2, 20kHz
  // setup onewheel
  ow_setup();
  // Neopixel setup
  FastLED.addLeds<NEOPIXEL, RGB_LED1_DATA_PIN>(rgb1LEDs, RGB_LED1_COUNT);
  FastLED.addLeds<NEOPIXEL, RGB_LED2_DATA_PIN>(rgb2LEDs, RGB_LED2_COUNT);
  // Serial setup
  Serial.begin(115200); // USB serial (for DEBUG)

  // Refresh sample intervals (important, because MAX_RPM_PERCENTAGE was probably changed above)
  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;
  // Time
  timelast = micros();
  timelastloop = timelast;

  // Task 1 setup (running on core 0)
  TaskHandle_t Task1;
  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      100000,    /* Stack size of task (10000) */
      NULL,      /* parameter of the task */
      1,         /* priority of the task (1 = low, 3 = medium, 5 = highest)*/
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */

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

  // wait for RC receiver to initialize
  while (millis() <= 1000)
    ;
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
//
// =======================================================================================================
// MAP VESC Duty TO THROTTLE
// =======================================================================================================
//

void mapThrottle()
{
  // TODO :Map vesc Duty to throttle or erpm
  // currentThrottle = map(pulseWidth[3], pulseMinNeutral[3], pulseMin[3], 0, 500);

  static float throttle = 0;
  int max, min;

#ifdef DUTY_TO_THROTTLE
  throttle = fabsf(VESC.appData.pitch);
  max = 90;
  min = 1;

#else
  throttle = fabsf(VESC.appData.erpm);
  max = maxErpm;
  min = minErpm;
#endif

  if (throttle > min)
  {
    currentThrottle = map((uint16_t)throttle, min, max, 0, 500);
  }
  else
  {
    currentThrottle = 0;
  }

#ifdef DEBUG_THROTTLE
  Serial.print("throttle:");
  Serial.println(throttle);
  Serial.print("currentRpm:");
  Serial.println(currentRpm);
  Serial.print("currentThrottle : ");
  Serial.println(currentThrottle);
  Serial.print("vescData.dutyCycleNow : ");
  Serial.println(vescData.dutyCycleNow);
#endif

  // Volume calculations --------------------------------------------------------------------------
  // As a base for some calculations below, fade the current throttle to make it more natural
  static unsigned long throttleFaderMicros;
  // static boolean blowoffLock;

  if (micros() - throttleFaderMicros > 500)
  { // Every 0.5ms
    throttleFaderMicros = micros();

    if (currentThrottleFaded < currentThrottle && currentThrottleFaded < 499)
      currentThrottleFaded += 2;
    if (currentThrottleFaded > currentThrottle && currentThrottleFaded > 2)
      currentThrottleFaded -= 2;
    // Serial.println(currentThrottleFaded);
  }

  // Calculate throttle dependent engine idle volume
  if (!escIsBraking && engineRunning)
    throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);
  else
    throttleDependentVolume = engineIdleVolumePercentage;

  // Calculate throttle dependent engine rev volume
  if (!escIsBraking && engineRunning)
    throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);
  else
    throttleDependentRevVolume = engineRevVolumePercentage;

  // Calculate engine rpm dependent jake brake volume
  if (engineRunning)
    rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
  else
    rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

  // Calculate throttle dependent Diesel knock volume
  if (!escIsBraking && engineRunning && (currentThrottleFaded > dieselKnockStartPoint))
    throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);
  else
    throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;

  // Calculate RPM dependent Diesel knock volume
  if (currentRpm > 400)
    rpmDependentKnockVolume = map(currentRpm, 400, 500, 5, 100);
  else
    rpmDependentKnockVolume = 5;

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
  if (!escIsBraking && engineRunning && (currentRpm > chargerStartPoint))
    throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, 500, chargerIdleVolumePercentage, 100);
  else
    throttleDependentChargerVolume = chargerIdleVolumePercentage;

  // Calculate engine rpm dependent wastegate volume
  if (engineRunning)
    rpmDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
  else
    rpmDependentWastegateVolume = wastegateIdleVolumePercentage;

  // Calculate engine load (used for torque converter slip simulation)
  engineLoad = currentThrottle - currentRpm;

  if (engineLoad < 0 || escIsBraking)
    engineLoad = 0; // Range is 0 - 180
  if (engineLoad > 180)
    engineLoad = 180;

  // Additional sounds volumes -----------------------------
  // Brake squealing
  if ((driveState == 2 || driveState == 4) && currentSpeed > 50 && currentThrottle > 250)
  {
    tireSquealVolume += map(currentThrottle, 250, 500, 0, 100);
  }

  tireSquealVolume = constrain(tireSquealVolume, 0, 100);
}

//
// =======================================================================================================
// ENGINE MASS SIMULATION
// =======================================================================================================
//

void engineMassSimulation()
{

  static int32_t targetRpm = 0; // The engine RPM target
  static int32_t lastThrottle;
  uint16_t converterSlip;
  static unsigned long throtMillis;
  static unsigned long wastegateMillis;

  uint8_t timeBase;

#ifdef SUPER_SLOW
  timeBase = 6; // super slow running, heavy engines, for example locomotive diesels
#else
  timeBase = 2;
#endif

  if (millis() - throtMillis > timeBase)
  { // Every 2 or 6ms
    throtMillis = millis();

    if (currentThrottle > 500)
      currentThrottle = 500;

      // Virtual clutch **********************************************************************************
#if defined EXCAVATOR_MODE // Excavator mode ---
    clutchDisengaged = true;

    targetRpm = currentThrottle - hydraulicLoad;
    targetRpm = constrain(targetRpm, 0, 500);

#else // Normal mode ---
    if ((currentSpeed < clutchEngagingPoint && currentRpm < maxClutchSlippingRpm) || gearUpShiftingInProgress || gearDownShiftingInProgress || neutralGear || currentRpm < 200)
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
        targetRpm = reMap(curveLinear, currentThrottle);
    }
    else if (doubleClutch)
    {
      // double clutch transmission
      if (!neutralGear)
        targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10; // Compute engine RPM
      else
        targetRpm = reMap(curveLinear, currentThrottle);
    }

#endif

    // Engine RPM **************************************************************************************

    if (escIsBraking && currentSpeed < clutchEngagingPoint)
      targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    if (targetRpm > 500)
      targetRpm = 500;

    // Accelerate engine
    if (targetRpm > (currentRpm + acc) && (currentRpm + acc) < maxRpm && engineState == RUNNING && engineRunning)
    {
      if (!airBrakeTrigger)
      { // No acceleration, if brake release noise still playing
        if (!gearDownShiftingInProgress)
          currentRpm += acc;
        else
          currentRpm += acc / 2; // less aggressive rpm rise while downshifting
        if (currentRpm > maxRpm)
          currentRpm = maxRpm;
      }
    }

    // Decelerate engine
    if (targetRpm < currentRpm)
    {
      currentRpm -= dec;
      if (currentRpm < minRpm)
        currentRpm = minRpm;
    }
    // Speed (sample rate) output
    engineSampleRate = map(currentRpm, minRpm, maxRpm, maxSampleInterval, minSampleInterval); // Idle
  }

  // Prevent Wastegate from being triggered while downshifting
  if (gearDownShiftingInProgress)
    wastegateMillis = millis();

  // Trigger Wastegate, if throttle rapidly dropped
  if (lastThrottle - currentThrottle > 70 && !escIsBraking && millis() - wastegateMillis > 1000)
  {
    wastegateMillis = millis();
    wastegateTrigger = true;
  }

#if defined JAKEBRAKE_ENGINE_SLOWDOWN && defined JAKE_BRAKE_SOUND
  // Use jake brake to slow down engine while releasing throttle in neutral or during upshifting while applying throttle
  // for some vehicles like Volvo FH open pipe. See example: https://www.youtube.com/watch?v=MU1iwzl33Zw&list=LL&index=4
  static unsigned long blowoffMillis;
  if (!wastegateTrigger)
    blowoffMillis = millis();
  blowoffTrigger = ((gearUpShiftingInProgress || neutralGear) && millis() - blowoffMillis > 20 && millis() - blowoffMillis < 250);
#endif

  lastThrottle = currentThrottle;
}

//
// =======================================================================================================
// MANUAL TAMIYA 3 SPEED GEARBOX DETECTION
// =======================================================================================================
//
void gearboxDetection()
{

  static uint8_t previousGear = 1;
  static boolean previousReverse;
  static boolean sequentialLock;
  static unsigned long upShiftingMillis;
  static unsigned long downShiftingMillis;

#if defined TRACKED_MODE // CH2 is used for left throttle in TRACKED_MODE --------------------------------
  selectedGear = 2;

#else // only active, if not in TRACKED_MODE -------------------------------------------------------------
  // if automatic transmission, always 2nd gear
  if (automatic || doubleClutch)
    selectedGear = 2;
  // Gear upshifting detection
  if (selectedGear > previousGear)
  {
    gearUpShiftingInProgress = true;
    gearUpShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
  }

  // Gear upshifting duration
  if (!gearUpShiftingInProgress)
    upShiftingMillis = millis();
  if (millis() - upShiftingMillis > 700)
    gearUpShiftingInProgress = false;

  // Gear downshifting detection
  if (selectedGear < previousGear)
  {
    gearDownShiftingInProgress = true;
    gearDownShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
  }

  // Gear downshifting duration
  if (!gearDownShiftingInProgress)
    downShiftingMillis = millis();
  if (millis() - downShiftingMillis > 300)
    gearDownShiftingInProgress = false;

  // Reverse gear engaging / disengaging detection
  if (escInReverse != previousReverse)
  {
    previousReverse = escInReverse;
    shiftingTrigger = true; // Play shifting sound
  }

#ifdef MANUAL_TRANS_DEBUG
  static unsigned long manualTransDebugMillis;
  if (millis() - manualTransDebugMillis > 100)
  {
    manualTransDebugMillis = millis();
    Serial.println(currentThrottle);
    Serial.println(selectedGear);
    Serial.println(sequentialLock);
    Serial.println(currentRpm);
    Serial.println(currentSpeed);
    Serial.println("");
  }
#endif

#endif // End of not TRACKED_MODE -----------------------------------------------------------------------
}

//
// =======================================================================================================
// SWITCH ENGINE ON OR OFF (for automatic mode)
// =======================================================================================================
//

void engineOnOff()
{
}

//
// =======================================================================================================
// SIMULATED AUTOMATIC TRANSMISSION GEAR SELECTOR
// =======================================================================================================
//

void automaticGearSelector()
{

  static unsigned long gearSelectorMillis;
  static unsigned long lastUpShiftingMillis;
  static unsigned long lastDownShiftingMillis;
  uint16_t downShiftPoint = 200;
  uint16_t upShiftPoint = 490;

  if (millis() - gearSelectorMillis > 100)
  { // Waiting for 100ms is very important. Otherwise gears are skipped!
    gearSelectorMillis = millis();

    // compute load dependent shift points (less throttle = less rpm before shifting up, kick down will shift back!)
    upShiftPoint = map(engineLoad, 0, 180, 390, 490);   // 390, 490
    downShiftPoint = map(engineLoad, 0, 180, 150, 250); // 150, 250

    if (escInReverse)
    { // Reverse (only one gear)
      selectedAutomaticGear = 0;
    }
    else
    { // Forward (multiple gears)

      // Adaptive shift points
      if (millis() - lastDownShiftingMillis > 500 && currentRpm >= upShiftPoint && engineLoad < 5)
      {                          // 500ms locking timer!
        selectedAutomaticGear++; // Upshifting (load maximum is important to prevent gears from oscillating!)
        lastUpShiftingMillis = millis();
      }
      if (millis() - lastUpShiftingMillis > 600 && selectedAutomaticGear > 1 && (currentRpm <= downShiftPoint || engineLoad > 100))
      {                          // 600ms locking timer! TODO was 1000
        selectedAutomaticGear--; // Downshifting incl. kickdown
        lastDownShiftingMillis = millis();
      }

      selectedAutomaticGear = constrain(selectedAutomaticGear, 1, NumberOfAutomaticGears);
    }

#ifdef AUTO_TRANS_DEBUG
    Serial.println(currentThrottle);
    Serial.println(selectedAutomaticGear);
    Serial.println(engineLoad);
    Serial.println(upShiftPoint);
    Serial.println(currentRpm);
    Serial.println(downShiftPoint);
    Serial.println("");
#endif
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

//
// =======================================================================================================
// HORN, BLUELIGHT & SIREN TRIGGERING BY CH4 (POT), WINCH CONTROL
// =======================================================================================================
//

void triggerHorn()
{
}
//
//===================================================== ===================================================== ===
// initialize one wheel
//===================================================== ===================================================== ===
//
void ow_setup()
{
#define VESC_DEBUG
  // set pin mode
  pinMode(CSR_EN_PIN, OUTPUT);
  pinMode(AUDIO_SOURCE_PIN, OUTPUT);
  pinMode(PAM_MUTE_PIN, OUTPUT);
  // audio mute
  AUDIO_MUTE();
  // csr8645 off
  CSR_POWER_OFF();
  // Serial port for debug
  Serial.begin(115200);
  // VESC serial
  Serial2.begin(115200, SERIAL_8N1, ESP_VESC_TX_PIN, ESP_VESC_RX_PIN);
#ifdef VESC_DEBUG
  VESC.setDebugPort(&Serial);
#endif
  VESC.setSerialPort(&Serial2);

  AUDIO_SOURCE_ESP();
  AUDIO_UNMUTE();
  engineOn = false;
  sound1trigger=true;
}

//
// =======================================================================================================
// Get the vesc main data
// =======================================================================================================
//

bool get_vesc_values(uint32_t loop_time)

{
  bool is_read = false;
  static uint32_t lastReadTime = millis();

  if (millis() - lastReadTime > loop_time)
  {

    is_read = VESC.getCustomValues();

    lastReadTime = micros();
  }

#ifdef VESC_DEBUG
  VESC.printCustomValues();
#endif // DEBUG

  return is_read;
}

//
// =======================================================================================================
// ESC CONTROL
// =======================================================================================================
//

// If you connect your ESC to pin 33, the vehicle inertia is simulated. Direct brake (crawler) ESC required
// *** WARNING!! Do it at your own risk!! There is a falisafe function in case, the signal input from the
// receiver is lost, but if the ESP32 crashes, the vehicle could get out of control!! ***

void vesc()
{

#if not defined TRACKED_MODE // No ESC control in TRACKED_MODEl
  static unsigned long escMillis;
  static unsigned long lastStateTime;
  static int8_t pulse;    // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t escPulse; // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t driveRampRate;
  static int8_t driveRampGain;
  static int8_t brakeRampRate;
  static int16_t speed;
  uint8_t escRampTime;
  uint16_t max, min;
  // Gear dependent ramp speed for acceleration & deceleration
#if defined VIRTUAL_3_SPEED
  escRampTime = escRampTimeThirdGear * 10 / virtualManualGearRatio[selectedGear];

#elif defined VIRTUAL_16_SPEED_SEQUENTIAL
  // escRampTime = escRampTimeThirdGear;// * 10 / map(virtualManualGearRatio[selectedGear], 155, 10, 23, 10);
  escRampTime = escRampTimeThirdGear * virtualManualGearRatio[selectedGear] / 5;

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
    if (escInReverse)
      escRampTime = escRampTime * 100 / automaticReverseAccelerationPercentage; // faster acceleration in automatic reverse, EXPERIMENTAL, TODO!
  }

  if (millis() - escMillis > escRampTime)
  { // About very 20 - 75ms
    escMillis = millis();

    // calulate throttle dependent brake & acceleration steps
    brakeRampRate = map(currentThrottle, 0, 500, 1, escBrakeSteps);
    driveRampRate = map(currentThrottle, 0, 500, 1, escAccelerationSteps);

    // Comparators
    if (fabsf(VESC.appData.erpm) > 60) // Not standing still
    {

      // positive erpm
      if (VESC.appData.erpm > 0) // forward
      {
        if (VESC.appData.pidOutput > -2) // positive current(pid)    balanceData.motorCurrent or balanceData.pidOutput
        {
          // forward
          escPulse = 1;
          pulse = 1;
        }
        else // Negative current(pid)

        {
          // braking forward
          pulse = 0;
          escPulse = 1;
        }
      }
      else if (VESC.appData.erpm < 0) // backward

      {
        if (VESC.appData.pidOutput < -2) // positive current(pid)    balanceData.motorCurrent or balanceData.pidOutput
        {
          // backward
          escPulse = -1;
          pulse = 0;
        }
        else // Negative current(pid)
        {

          // braking backward
          pulse = 1;
          escPulse = -1;
        }
      }
    }
    else ////  standing still
    {
      pulse = 1;
      escPulse = 1;
    }

#ifdef ESC_DEBUG
    if (millis() - lastStateTime > 300)
    { // Print the data every 300ms
      lastStateTime = millis();
      Serial.println(driveState);
      Serial.println(pulse);
      Serial.println(escPulse);
      Serial.println(escPulseMin);
      Serial.println(escPulseMax);
      Serial.println(brakeRampRate);
      Serial.println(currentRpm);
      Serial.println(escPulseWidth);
      Serial.println(escPulseWidthOut);
      Serial.println(currentSpeed);
      Serial.println(speedLimit);
      Serial.println("");
    }
#endif

    // Drive state state machine **********************************************************************************
    switch (driveState)
    {

    case 0: // Standing still ---------------------------------------------------------------------
      escIsBraking = false;
      escInReverse = false;
      escIsDriving = false;
#ifdef VIRTUAL_16_SPEED_SEQUENTIAL
      selectedGear = 1;
#endif

      if (pulse == 1 && engineRunning && !neutralGear)
        driveState = 1; // Driving forward
      if (pulse == -1 && engineRunning && !neutralGear)
        driveState = 3; // Driving backwards
      break;

    case 1: // Driving forward ---------------------------------------------------------------------
      escIsBraking = false;
      escInReverse = false;
      escIsDriving = true;

      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // lowering RPM, if shifting up transmission

        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // increasing RPM, if shifting down transmission
        gearDownShiftingPulse = false;
      }

      if (pulse == -1 && escPulse == 1)
        driveState = 2; // Braking forward
      if (pulse == 0 && escPulse == 0)
        driveState = 0; // standing still
      break;

    case 2: // Braking forward ---------------------------------------------------------------------
      escIsBraking = true;
      escInReverse = false;
      escIsDriving = false;

      if (pulse == 0 && escPulse == 1 && !neutralGear)
      {
        driveState = 1; // Driving forward
        airBrakeTrigger = true;
      }
      if (pulse == 0 && escPulse == 0)
      {
        driveState = 0; // standing still
        airBrakeTrigger = true;
      }
      break;

    case 3: // Driving backwards ---------------------------------------------------------------------
      escIsBraking = false;
      escInReverse = true;
      escIsDriving = true;
      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // lowering RPM, if shifting up transmission
        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      { // increasing RPM, if shifting down transmission

        gearDownShiftingPulse = false;
      }

      if (pulse == 1 && escPulse == -1)
        driveState = 4; // Braking backwards
      if (pulse == 0 && escPulse == 0)
        driveState = 0; // standing still
      break;

    case 4: // Braking backwards ---------------------------------------------------------------------
      escIsBraking = true;
      escInReverse = true;
      escIsDriving = false;

      if (pulse == 0 && escPulse == -1 && !neutralGear)
      {
        driveState = 3; // Driving backwards
        airBrakeTrigger = true;
      }
      if (pulse == 0 && escPulse == 0)
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

    // Calculate a speed value from the pulsewidth signal (used as base for engine sound RPM while clutch is engaged)

#ifdef DUTY_TO_THROTTLE
    speed = fabsf(VESC.appData.pitch);
    max = 90;
    min = 1;
#else

    speed = fabsf(VESC.appData.erpm);
    max = maxErpm;
    min = minErpm;
#endif

    if (speed > min)
    {
      currentSpeed = map((uint16_t)speed, min, max, 0, 500);
    }
    else
    {
      currentSpeed = 0;
    }
  }
#endif
}

//
// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//

void loop()
{
  // TODO: vesc get data from here

  //
  if (xSemaphoreTake(xVescSemaphore, portMAX_DELAY))
  {
    if (get_vesc_values(100))
    {
      static unsigned long engineOffTimer = millis();
      if (VESC.appData.switchState < 2)
      {
        if (millis() - engineOffTimer > engineOffDelay)
        {
          engineOn = false;

        }
      }
      else
      {
      
        engineOn = true;
        AUDIO_UNMUTE();
        engineOffTimer = millis();
      }
    }
    else

    {
      engineOn = false;
     
    }
    xSemaphoreGive(xVescSemaphore); // Now free or "Give" the semaphore for others.
  }
  // Map pulsewidth to throttle
  if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
  {

    // Map pulsewidth to throttle
    mapThrottle();
    xSemaphoreGive(xRpmSemaphore); // Now free or "Give" the semaphore for others.
  }

  // Horn triggering
  triggerHorn();

  // RGB LED control

  // Feeding the RTC watchtog timer is essential!
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

    // Gearbox detection
    gearboxDetection();

    // Switch engine on or off
    // engineOnOff();

    vesc();
    // measure loop time
    // loopTime = loopDuration(); // for debug only
  }
}
