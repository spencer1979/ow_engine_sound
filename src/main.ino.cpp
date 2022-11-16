# 1 "C:\\Users\\Spencer-Dell\\AppData\\Local\\Temp\\tmpf90sby5i"
#include <Arduino.h>
# 1 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
# 21 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
char codeVersion[] = "9.10.0";


#include <Arduino.h>
#include "ow_control_config.h"
#include <VescUart.h>
void Task1code(void *parameters);
float batteryVolts();
# 54 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
#include "1_adjustmentsVehicle.h"

#include "3_adjustmentsESC.h"
#include "4_adjustmentsTransmission.h"

#include "6_adjustmentsLights.h"

#include "8_adjustmentsSound.h"
# 76 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
#define PI 3.14159265
# 88 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
#include <statusLED.h>
#include <FastLED.h>
#include <ESP32AnalogRead.h>
#include <AwesomeClickButton.h>

#include "src/curves.h"
#include "soc/rtc_wdt.h"
#include "src/helper.h"
# 119 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
#define BATTERY_DETECT_PIN 39

#define DAC1 25
#define DAC2 26




#ifdef USE_DUAL_HEAD_LIGHT
statusLED headLight0(true);
statusLED headLight1(true);
#else
statusLED headLight(false);
#endif
statusLED tailLight(false);

#ifdef USE_RGB_LED
CRGB rgb1LEDs[RGB_LED1_COUNT];
CRGB rgb2LEDs[RGB_LED2_COUNT];
#endif

#ifdef USE_FAN_COOLING
statusLED fan(false);
#endif


ESP32AnalogRead battery;




volatile boolean engineOn = false;
volatile boolean engineStart = false;
volatile boolean engineRunning = false;
volatile boolean engineStop = false;
volatile boolean jakeBrakeRequest = false;
volatile boolean engineJakeBraking = false;
volatile boolean wastegateTrigger = false;
volatile boolean blowoffTrigger = false;
volatile boolean dieselKnockTrigger = false;
volatile boolean dieselKnockTriggerFirst = false;
volatile boolean airBrakeTrigger = false;
volatile boolean parkingBrakeTrigger = false;
volatile boolean shiftingTrigger = false;
volatile boolean hornTrigger = false;
volatile boolean sirenTrigger = false;
volatile boolean sound1trigger = false;
volatile boolean couplingTrigger = false;
volatile boolean uncouplingTrigger = false;
volatile boolean bucketRattleTrigger = false;
volatile boolean indicatorSoundOn = false;
volatile boolean outOfFuelMessageTrigger = false;


volatile boolean hornLatch = false;
volatile boolean sirenLatch = false;


volatile uint16_t throttleDependentVolume = 0;
volatile uint16_t throttleDependentRevVolume = 0;
volatile uint16_t rpmDependentJakeBrakeVolume = 0;
volatile uint16_t throttleDependentKnockVolume = 0;
volatile uint16_t rpmDependentKnockVolume = 0;
volatile uint16_t throttleDependentTurboVolume = 0;
volatile uint16_t throttleDependentFanVolume = 0;
volatile uint16_t throttleDependentChargerVolume = 0;
volatile uint16_t rpmDependentWastegateVolume = 0;
volatile uint16_t tireSquealVolume = 0;
volatile uint16_t trackRattleVolume = 0;

volatile uint64_t dacDebug = 0;
volatile int16_t masterVolume = 100;
volatile uint8_t dacOffset = 0;


int16_t currentThrottle = 0;
int16_t currentThrottleFaded = 0;


const int16_t maxRpm = 500;
const int16_t minRpm = 0;
int32_t currentRpm = 0;
volatile uint8_t engineState = 0;
enum EngineState
{
  OFF,
  STARTING,
  RUNNING,
  STOPPING,
  PARKING_BRAKE
};

int16_t engineLoad = 0;
volatile uint16_t engineSampleRate = 0;
int32_t speedLimit = maxRpm;


boolean clutchDisengaged = true;


uint8_t selectedGear = 1;
uint8_t selectedAutomaticGear = 1;
boolean gearUpShiftingInProgress;
boolean doubleClutchInProgress;
boolean gearDownShiftingInProgress;
boolean gearUpShiftingPulse;
boolean gearDownShiftingPulse;
volatile boolean neutralGear = false;
boolean lowRange = false;


volatile boolean owIsBraking = false;
volatile boolean owIsDriving = false;
volatile boolean owInReverse = false;
volatile boolean brakeDetect = false;
int8_t driveState = 0;
uint16_t escPulseMax = 2000;
uint16_t escPulseMin = 1000;
uint16_t escPulseMaxNeutral = 1500;
uint16_t escPulseMinNeutral = 1500;
uint16_t currentSpeed = 0;
volatile bool crawlerMode = false;




float batteryCutoffvoltage;
float batteryVoltage;
uint8_t numberOfCells;
bool batteryProtection = false;


#define NEUTRAL_ERPM 200.0
#define NEUTRAL_PID 5.0
#define MAX_ERPM 4000
#define MIN_ERPM 300
volatile float vescErpm;
volatile float vescPid;
volatile SwitchState vescSwitchState;
const uint32_t engineOffDelay = 5000;
unsigned long engineOffTimer ;
unsigned long sourceCheckTimer ;


volatile AudioSource source;

volatile unsigned long timelast;
unsigned long timelastloop;


VescUart VESC;

AwesomeClickButton soundButton(PUSH_BUTTON_PIN);


volatile uint8_t coreId = 99;


TaskHandle_t Task1;


uint16_t loopTime;


uint32_t maxSampleInterval = 4000000 / sampleRate;
uint32_t minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;


hw_timer_t *variableTimer = NULL;
portMUX_TYPE variableTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t variableTimerTicks = maxSampleInterval;


hw_timer_t *fixedTimer = NULL;
portMUX_TYPE fixedTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fixedTimerTicks = maxSampleInterval;



SemaphoreHandle_t xPwmSemaphore;
SemaphoreHandle_t xRpmSemaphore;
void IRAM_ATTR variablePlaybackTimer();
void IRAM_ATTR fixedPlaybackTimer();
void setupBattery();
bool get_vesc_values(uint32_t loop_time);
void ow_setup();
void setup();
void dacOffsetFade();
static int16_t get_vesc_throttle();
void mapThrottle();
void engineMassSimulation();
void engineOnOff();
void gearboxDetection();
void automaticGearSelector();
int8_t checkPulse(float val , uint32_t range );
int8_t rpmPulse();
int8_t pidPulse();
void esc();
unsigned long loopDuration();
void triggerHorn();
void triggerIndicators();
void updateRGBLEDs();
void check_mute( uint32_t time );
void debug_print();
void get_fake_data();
void loop();
void Task1code(void *pvParameters);
#line 307 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
void IRAM_ATTR variablePlaybackTimer()
{



  static uint32_t attenuatorMillis = 0;
  static uint32_t curEngineSample = 0;
  static uint32_t curRevSample = 0;
  static uint32_t curTurboSample = 0;
  static uint32_t curFanSample = 0;
  static uint32_t curChargerSample = 0;
  static uint32_t curStartSample = 0;
  static uint32_t curJakeBrakeSample = 0;
  static uint32_t curHydraulicPumpSample = 0;
  static uint32_t curTrackRattleSample = 0;
  static uint32_t lastDieselKnockSample = 0;
  static uint16_t attenuator = 0;
  static uint16_t speedPercentage = 0;
  static int32_t a, a1, a2, a3, b, c, d, e = 0;
  static int32_t f = 0;
  static int32_t g = 0;
  uint8_t a1Multi = 0;



  switch (engineState)
  {

  case OFF:
    variableTimerTicks = 4000000 / startSampleRate;
    timerAlarmWrite(variableTimer, variableTimerTicks, true);

    a = 0;
    if (engineOn)
    {
      engineState = STARTING;
      engineStart = true;
    }
    break;

  case STARTING:
    variableTimerTicks = 4000000 / startSampleRate;
    timerAlarmWrite(variableTimer, variableTimerTicks, true);

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

  case RUNNING:


    variableTimerTicks = engineSampleRate;
    timerAlarmWrite(variableTimer, variableTimerTicks, true);

    if (!engineJakeBraking && !blowoffTrigger)
    {
      if (curEngineSample < sampleCount - 1)
      {
        a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100);
        a3 = 0;
        curEngineSample++;




#ifdef REV_SOUND
        a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100);
        if (curRevSample < revSampleCount)
          curRevSample++;
#endif


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
    {
#ifdef JAKE_BRAKE_SOUND
      a3 = (jakeBrakeSamples[curJakeBrakeSample] * rpmDependentJakeBrakeVolume / 100 * jakeBrakeVolumePercentage / 100);
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


#ifdef REV_SOUND




    if (currentRpm > revSwitchPoint)
      a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
    else
      a1Multi = idleVolumeProportionPercentage;
    if (currentRpm > idleEndPoint)
      a1Multi = 0;

    a1 = a1 * a1Multi / 100;
    a2 = a2 * (100 - a1Multi) / 100;

    a = a1 + a2 + a3;
#else
    a = a1 + a3;
#endif


    if (curTurboSample < turboSampleCount - 1)
    {
      c = (turboSamples[curTurboSample] * throttleDependentTurboVolume / 100 * turboVolumePercentage / 100);
      curTurboSample++;
    }
    else
    {
      curTurboSample = 0;
    }


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
      d = 0;
#endif


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

  case STOPPING:
    variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100;
    timerAlarmWrite(variableTimer, variableTimerTicks, true);

    if (curEngineSample < sampleCount - 1)
    {
      a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
      curEngineSample++;
    }
    else
    {
      curEngineSample = 0;
    }


    if (millis() - attenuatorMillis > 100)
    {
      attenuatorMillis = millis();
      attenuator++;
      speedPercentage += 20;
    }

    if (attenuator >= 50 || speedPercentage >= 500)
    {
      a = 0;
      speedPercentage = 100;
      parkingBrakeTrigger = true;
      engineState = PARKING_BRAKE;
      engineStop = false;
    }
    break;

  case PARKING_BRAKE:

    if (!parkingBrakeTrigger)
    {
      engineState = OFF;
    }
    break;

  }



  dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f + g) * masterVolume / 100 + dacOffset, 0, 255));




}







void IRAM_ATTR fixedPlaybackTimer()
{



  static uint32_t curHornSample = 0;
  static uint32_t curSirenSample = 0;
  static uint32_t curSound1Sample = 0;
  static uint32_t curReversingSample = 0;
  static uint32_t curIndicatorSample = 0;
  static uint32_t curWastegateSample = 0;
  static uint32_t curBrakeSample = 0;
  static uint32_t curParkingBrakeSample = 0;
  static uint32_t curShiftingSample = 0;
  static uint32_t curDieselKnockSample = 0;
  static uint32_t curTrackRattleSample = 0;
  static uint32_t curTireSquealSample = 0;
  static uint32_t curOutOfFuelSample = 0;
  static int32_t a, a1, a2 = 0;
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b9 = 0;
  static int32_t c, c1, c2, c3 = 0;
  static int32_t d, d1, d2 = 0;
  static boolean knockSilent = 0;
  static boolean knockMedium = 0;
  static uint8_t curKnockCylinder = 0;





  if (hornTrigger || hornLatch)
  {
    fixedTimerTicks = 4000000 / hornSampleRate;
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);

    if (curHornSample < hornSampleCount - 1)
    {
      a1 = (hornSamples[curHornSample] * hornVolumePercentage / 100);
      curHornSample++;
#ifdef HORN_LOOP
      if (hornTrigger && curHornSample == hornLoopEnd)
        curHornSample = hornLoopBegin;
#endif
    }
    else
    {
      curHornSample = 0;
      a1 = 0;
      hornLatch = false;
    }
  }

  if (sirenTrigger || sirenLatch)
  {
    fixedTimerTicks = 4000000 / sirenSampleRate;
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);

    if (curSirenSample < sirenSampleCount - 1)
    {
      a2 = (sirenSamples[curSirenSample] * sirenVolumePercentage / 100);
      curSirenSample++;
#ifdef SIREN_LOOP
      if (sirenTrigger && curSirenSample == sirenLoopEnd)
        curSirenSample = sirenLoopBegin;
#endif
    }
    else
    {
      curSirenSample = 0;
      a2 = 0;
      sirenLatch = false;
    }
  }




  if (sound1trigger)
  {
    fixedTimerTicks = 4000000 / sound1SampleRate;
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);

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
    curSound1Sample = 0;
    b0 = 0;
  }


  if (engineRunning && owInReverse)
  {
    fixedTimerTicks = 4000000 / reversingSampleRate;
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);

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
    curReversingSample = 0;
    b1 = 0;
  }


#if not defined NO_INDICATOR_SOUND
  if (indicatorSoundOn)
  {
    fixedTimerTicks = 4000000 / indicatorSampleRate;
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true);

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
    curIndicatorSample = 0;
    b2 = 0;
  }
#endif


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
    curWastegateSample = 0;
  }


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
    curBrakeSample = 0;
  }


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
    curParkingBrakeSample = 0;
  }


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
    curShiftingSample = 0;
  }


  if (dieselKnockTriggerFirst)
  {
    dieselKnockTriggerFirst = false;
    curKnockCylinder = 0;
  }

  if (dieselKnockTrigger)
  {
    dieselKnockTrigger = false;
    curKnockCylinder++;
    curDieselKnockSample = 0;
  }

#ifdef V8

  if (curKnockCylinder == 4 || curKnockCylinder == 8)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V8_MEDIUM

  if (curKnockCylinder == 5 || curKnockCylinder == 1)
    knockMedium = false;
  else
    knockMedium = true;
#endif

#ifdef V8_468


  if (curKnockCylinder == 1 || curKnockCylinder == 5 || curKnockCylinder == 9 || curKnockCylinder == 13)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V2

  if (curKnockCylinder == 1 || curKnockCylinder == 2)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6

  if (curKnockCylinder == 6)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6_2

  if (curKnockCylinder == 6 || curKnockCylinder == 3)
    knockSilent = false;
  else
    knockSilent = true;
#endif

  if (curDieselKnockSample < knockSampleCount)
  {
#if defined RPM_DEPENDENT_KNOCK
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#else
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
    curDieselKnockSample++;
    if (knockSilent && !knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100;
    if (knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 75;
  }



#if defined TIRE_SQUEAL

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
    curOutOfFuelSample = 0;
  }
#endif


  a = a1 + a2;

  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7;
  c = c1 + c2 + c3;
  d = d1 + d2;




  dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255));




}







void setupBattery()
{
}
# 930 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
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
#endif
}






void ow_setup()
{


  FastLED.addLeds<NEOPIXEL, RGB_LED1_DATA_PIN>(rgb1LEDs, RGB_LED1_COUNT);
  FastLED.addLeds<NEOPIXEL, RGB_LED2_DATA_PIN>(rgb2LEDs, RGB_LED2_COUNT);
#ifdef USE_DUAL_HEAD_LIGHT
  headLight0.begin(LED1_PIN, 1, 20000);
  headLight1.begin(LED2_PIN, 1, 20000);
#else
  headLight.begin(LED1_PIN, 1, 20000);
#endif
  tailLight.begin(LED3_PIN, 2, 20000);

  Serial2.begin(115200, SERIAL_8N1, ESP_VESC_TX_PIN, ESP_VESC_RX_PIN);

  while (!Serial2)
  {
    ;
  }
#ifdef VESC_DEBUG
  VESC.setDebugPort(&Serial);
#endif
  VESC.setSerialPort(&Serial2);

  vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef FAKE_VESC_DATA
  source = SOURCE_ESP32;
#else
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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
  Serial.begin(115200);

  pinMode(CSR_EN_PIN, OUTPUT);
  pinMode(AUDIO_SOURCE_PIN, OUTPUT);
  pinMode(PAM_MUTE_PIN, OUTPUT);

  CSR_POWER_OFF();

  AUDIO_SOURCE_ESP();

  AUDIO_UNMUTE();

  sound1trigger = true;

  disableCore0WDT();
# 1045 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
  if (xPwmSemaphore == NULL)
  {
    xPwmSemaphore = xSemaphoreCreateMutex();
    if ((xPwmSemaphore) != NULL)
      xSemaphoreGive((xPwmSemaphore));
  }

  if (xRpmSemaphore == NULL)
  {
    xRpmSemaphore = xSemaphoreCreateMutex();
    if ((xRpmSemaphore) != NULL)
      xSemaphoreGive((xRpmSemaphore));
  }


  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;


  TaskHandle_t Task1;

  xTaskCreatePinnedToCore(
      Task1code,
      "Task1",
      8192,
      NULL,
      1,
      &Task1,
      0);


  variableTimer = timerBegin(0, 20, true);
  timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true);
  timerAlarmWrite(variableTimer, variableTimerTicks, true);
  timerAlarmEnable(variableTimer);


  fixedTimer = timerBegin(1, 20, true);
  timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true);
  timerAlarmWrite(fixedTimer, fixedTimerTicks, true);
  timerAlarmEnable(fixedTimer);

  while (sound1trigger)
  {
    ;
  }

  ow_setup();
}







static unsigned long dacOffsetMicros;
boolean dacInit;

void dacOffsetFade()
{
  if (!dacInit)
  {
    if (micros() - dacOffsetMicros > 100)
    {
      dacOffsetMicros = micros();
      dacOffset++;
      if (dacOffset == 128)
        dacInit = true;
    }
  }
}







static int16_t get_vesc_throttle()
{
  uint32_t throttle;
  throttle = fabsf(vescErpm);
  throttle =MAX( throttle ,MIN_ERPM);
  throttle =MIN( throttle ,MAX_ERPM);
  return (int16_t)map((uint32_t)throttle, MIN_ERPM, MAX_ERPM, minRpm, maxRpm);
}







void mapThrottle()
{



  currentThrottle = get_vesc_throttle();

  static unsigned long throttleFaderMicros;
  static boolean blowoffLock;
  if (micros() - throttleFaderMicros > 500)
  {

    throttleFaderMicros = micros();

    if (currentThrottleFaded < currentThrottle && !owIsBraking && currentThrottleFaded < 499)
      currentThrottleFaded += 2;
    if ((currentThrottleFaded > currentThrottle || owIsBraking) && currentThrottleFaded > 2)
      currentThrottleFaded -= 2;


    if (!owIsBraking && !brakeDetect && engineRunning)
      throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);

    else
    {
      if (throttleDependentVolume > engineIdleVolumePercentage)
        throttleDependentVolume--;
      else
        throttleDependentVolume = engineIdleVolumePercentage;
    }


    if (!owIsBraking && !brakeDetect && engineRunning)
      throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);

    else
    {
      if (throttleDependentRevVolume > engineRevVolumePercentage)
        throttleDependentRevVolume--;
      else
        throttleDependentRevVolume = engineRevVolumePercentage;
    }


    if (!owIsBraking && !brakeDetect && engineRunning && (currentThrottleFaded > dieselKnockStartPoint))
      throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);

    else
    {
      if (throttleDependentKnockVolume > dieselKnockIdleVolumePercentage)
        throttleDependentKnockVolume--;
      else
        throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;
    }


    if (engineRunning)
      rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
    else
      rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

#if defined RPM_DEPENDENT_KNOCK

    if (currentRpm > 400)
      rpmDependentKnockVolume = map(currentRpm, knockStartRpm, 500, minKnockVolumePercentage, 100);
    else
      rpmDependentKnockVolume = minKnockVolumePercentage;
#endif


    if (engineRunning)
      throttleDependentTurboVolume = map(currentRpm, 0, 500, turboIdleVolumePercentage, 100);
    else
      throttleDependentTurboVolume = turboIdleVolumePercentage;


    if (engineRunning && (currentRpm > fanStartPoint))
      throttleDependentFanVolume = map(currentRpm, fanStartPoint, 500, fanIdleVolumePercentage, 100);
    else
      throttleDependentFanVolume = fanIdleVolumePercentage;


    if (!owIsBraking && !brakeDetect && engineRunning && (currentRpm > chargerStartPoint))
      throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, 500, chargerIdleVolumePercentage, 100);
    else
      throttleDependentChargerVolume = chargerIdleVolumePercentage;


    if (engineRunning)
      rpmDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
    else
      rpmDependentWastegateVolume = wastegateIdleVolumePercentage;
  }


  engineLoad = currentThrottle - currentRpm;

  if (engineLoad < 0 || owIsBraking || brakeDetect)
    engineLoad = 0;
  if (engineLoad > 180)
    engineLoad = 180;





  uint8_t steeringAngle = 0;
  uint8_t brakeSquealVolume = 0;



  if ((driveState == 2 || driveState == 4) && currentSpeed > 50 && currentThrottle > 250) {
    tireSquealVolume += map(currentThrottle, 250, 500, 0, 100);
  }

  tireSquealVolume = constrain(tireSquealVolume, 0, 100);
}







void engineMassSimulation()
{

  static int32_t targetRpm = 0;
  static int32_t _currentRpm = 0;
  static int32_t _currentThrottle = 0;
  static int32_t lastThrottle;
  uint16_t converterSlip;
  static unsigned long throtMillis;
  static unsigned long wastegateMillis;
  static unsigned long blowoffMillis;
  uint8_t timeBase;

#ifdef SUPER_SLOW
  timeBase = 6;
#else
  timeBase = 2;
#endif

  _currentThrottle = currentThrottle;

  if (millis() - throtMillis > timeBase)
  {
    throtMillis = millis();

    if (_currentThrottle > 500)
      _currentThrottle = 500;



    if ((currentSpeed < clutchEngagingPoint && _currentRpm < maxClutchSlippingRpm) || gearUpShiftingInProgress || gearDownShiftingInProgress || neutralGear)
    {
      clutchDisengaged = true;
    }
    else
    {
      clutchDisengaged = false;
    }




    if (automatic)
    {

      if (selectedAutomaticGear < 2)
        converterSlip = engineLoad * torqueconverterSlipPercentage / 100 * 2;
      else
        converterSlip = engineLoad * torqueconverterSlipPercentage / 100;

      if (!neutralGear)
        targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10 + converterSlip;
      else
        targetRpm = reMap(curveLinear, _currentThrottle);
    }
    else if (doubleClutch)
    {

      if (!neutralGear)
        targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10;
      else
        targetRpm = reMap(curveLinear, _currentThrottle);
    }
    else
    {

      if (clutchDisengaged)
      {
#if defined VIRTUAL_16_SPEED_SEQUENTIAL
        targetRpm = _currentThrottle;
#else
        targetRpm = reMap(curveLinear, _currentThrottle);

#endif
      }
      else
      {

#if defined VIRTUAL_3_SPEED || defined VIRTUAL_16_SPEED_SEQUENTIAL
        targetRpm = reMap(curveLinear, (currentSpeed * virtualManualGearRatio[selectedGear] / 10));
        if (targetRpm > 500)
          targetRpm = 500;
#else
        targetRpm = reMap(curveLinear, currentSpeed);
#endif
      }
    }



    if (owIsBraking && currentSpeed < clutchEngagingPoint)
      targetRpm = 0;
    if (targetRpm > 500)
      targetRpm = 500;


    if (targetRpm > (_currentRpm + acc) && (_currentRpm + acc) < maxRpm && engineState == RUNNING && engineRunning)
    {
      if (!airBrakeTrigger)
      {
        if (!gearDownShiftingInProgress)
          _currentRpm += acc;
        else
          _currentRpm += acc / 2;
        if (_currentRpm > maxRpm)
          _currentRpm = maxRpm;
      }
    }


    if (targetRpm < _currentRpm)
    {
      _currentRpm -= dec;
      if (_currentRpm < minRpm)
        _currentRpm = minRpm;
    }

#if (defined VIRTUAL_3_SPEED || defined VIRTUAL_16_SPEED_SEQUENTIAL)

    if (!automatic && !doubleClutch)
      speedLimit = maxRpm * 10 / virtualManualGearRatio[selectedGear];
#endif


    engineSampleRate = map(_currentRpm, minRpm, maxRpm, maxSampleInterval, minSampleInterval);



    currentRpm = _currentRpm;


  }


  if (gearDownShiftingInProgress)
    wastegateMillis = millis();


  if (lastThrottle - _currentThrottle > 70 && !owIsBraking && millis() - wastegateMillis > 1000)
  {
    wastegateMillis = millis();
    wastegateTrigger = true;
  }

#if defined JAKEBRAKE_ENGINE_SLOWDOWN && defined JAKE_BRAKE_SOUND


  if (!wastegateTrigger)
    blowoffMillis = millis();
  blowoffTrigger = ((gearUpShiftingInProgress || neutralGear) && millis() - blowoffMillis > 20 && millis() - blowoffMillis < 250);
#endif

  lastThrottle = _currentThrottle;
}







void engineOnOff()
{


  static unsigned long idleDelayMillis;


  if (currentThrottle > 80 || driveState != 0)
    idleDelayMillis = millis();

#ifdef AUTO_ENGINE_ON_OFF
  if (millis() - idleDelayMillis > 15000)
  {
    engineOn = false;
  }
#endif

#ifdef AUTO_LIGHTS
  if (millis() - idleDelayMillis > 10000)
  {
    lightsOn = false;
  }
#endif


  if (currentThrottle > 100 && !airBrakeTrigger)
  {
    engineOn = true;

#ifdef AUTO_LIGHTS
    lightsOn = true;
#endif
  }
}







void gearboxDetection()
{

  static uint8_t previousGear = 1;
  static bool previousReverse;
  static bool sequentialLock;
  static bool overdrive = false;
  static unsigned long upShiftingMillis;
  static unsigned long downShiftingMillis;
  static unsigned long lastShiftingMillis;
#if defined OVERDRIVE && defined VIRTUAL_3_SPEED
  if (!crawlerMode)
  {

    if (currentRpm > 490 && selectedGear == 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
    {
      overdrive = true;
    }
    if (!owIsBraking)
    {
      if (currentRpm < 200 && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    else
    {
      if ((currentRpm < 400 || engineLoad > 150) && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    if (selectedGear < 3)
      overdrive = false;
  }
#endif

#if defined SEMI_AUTOMATIC
  if (currentRpm > 490 && selectedGear < 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
  {
    selectedGear++;
  }
  if (!owIsBraking)
  {
    if (currentRpm < 200 && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--;
    }
  }
  else
  {
    if ((currentRpm < 400 || engineLoad > 150) && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--;
    }
  }
  if (neutralGear || owInReverse)
    selectedGear = 1;
#endif


  if (selectedGear > previousGear)
  {
    gearUpShiftingInProgress = true;
    gearUpShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }


  static uint16_t upshiftingDuration = 700;
  if (!gearUpShiftingInProgress)
    upShiftingMillis = millis();
  if (millis() - upShiftingMillis > upshiftingDuration)
    gearUpShiftingInProgress = false;


#if defined DOUBLE_CLUTCH
  upshiftingDuration = 900;
  doubleClutchInProgress = (millis() - upShiftingMillis >= 500 && millis() - upShiftingMillis < 600);
#endif


  if (selectedGear < previousGear)
  {
    gearDownShiftingInProgress = true;
    gearDownShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }


  if (!gearDownShiftingInProgress)
    downShiftingMillis = millis();
  if (millis() - downShiftingMillis > 300)
    gearDownShiftingInProgress = false;


  if (owInReverse != previousReverse)
  {
    previousReverse = owInReverse;
    shiftingTrigger = true;
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
#endif
}







void automaticGearSelector()
{

  static unsigned long gearSelectorMillis;
  static unsigned long lastUpShiftingMillis;
  static unsigned long lastDownShiftingMillis;
  uint16_t downShiftPoint = 200;
  uint16_t upShiftPoint = 490;
  static int32_t _currentRpm = 0;



  _currentRpm = currentRpm;



  if (millis() - gearSelectorMillis > 100)
  {
    gearSelectorMillis = millis();


    upShiftPoint = map(engineLoad, 0, 180, 390, 490);
    downShiftPoint = map(engineLoad, 0, 180, 150, 250);

    if (owInReverse)
    {
      selectedAutomaticGear = 0;
    }
    else
    {


      if (millis() - lastDownShiftingMillis > 500 && _currentRpm >= upShiftPoint && engineLoad < 5)
      {
        selectedAutomaticGear++;
        lastUpShiftingMillis = millis();
      }
      if (millis() - lastUpShiftingMillis > 600 && selectedAutomaticGear > 1 && (_currentRpm <= downShiftPoint || engineLoad > 100))
      {
        selectedAutomaticGear--;
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







static uint16_t escPulseWidth = 1500;
static uint16_t escPulseWidthOut = 1500;
static uint16_t escSignal = 1500;
static unsigned long escMillis;
static unsigned long brakeMillis;


static int8_t driveRampRate;
static int8_t driveRampGain;
static int8_t brakeRampRate;
uint16_t escRampTime;







int8_t checkPulse(float val , uint32_t range )
{
  int8_t _rpmPulse;
  if (((val+ range ) * (range - val)) > 0)
  {
    _rpmPulse = 0;

  } else
  {
    if (val > range )
    _rpmPulse = 1;
    else
     _rpmPulse = -1;
  }

  return _rpmPulse;
}

int8_t rpmPulse()
{
  return checkPulse(vescErpm,NEUTRAL_ERPM);
}


int8_t pidPulse()
{
  return checkPulse(vescPid,NEUTRAL_PID);
}





void esc()
{

#if defined VIRTUAL_3_SPEED
  escRampTime = escRampTimeThirdGear * 10 / virtualManualGearRatio[selectedGear];

#elif defined VIRTUAL_16_SPEED_SEQUENTIAL
  escRampTime = escRampTimeThirdGear * virtualManualGearRatio[selectedGear] / 5;

#elif defined STEAM_LOCOMOTIVE_MODE
  escRampTime = escRampTimeSecondGear;

#else
  if (selectedGear == 1)
    escRampTime = escRampTimeFirstGear;
  if (selectedGear == 2)
    escRampTime = escRampTimeSecondGear;
  if (selectedGear == 3)
    escRampTime = escRampTimeThirdGear;
#endif

  if (automatic || doubleClutch)
  {
    escRampTime = escRampTimeSecondGear;
    if (owInReverse)
      escRampTime = escRampTime * 100 / automaticReverseAccelerationPercentage;
  }


  escRampTime = escRampTime * 100 / globalAccelerationPercentage;


  if (lowRange)
    escRampTime = escRampTime * lowRangePercentage / 100;



  crawlerMode = (masterVolume <= masterVolumeCrawlerThreshold);

  if (crawlerMode)
  {
    escRampTime = crawlerEscRampTime;
    brakeRampRate = map(currentThrottle, 0, 500, 1, 10);
    driveRampRate = 10;
  }
  else
  {

    brakeRampRate = map(currentThrottle, 0, 500, 1, escBrakeSteps);
    driveRampRate = map(currentThrottle, 0, 500, 1, escAccelerationSteps);
  }
  brakeMillis=millis();


  if (((rpmPulse() == 1 && pidPulse() == -1) || (rpmPulse() == -1 && pidPulse() == 1)) && (millis() - brakeMillis > 100))

  {
    brakeDetect = true;
    brakeMillis = millis();
  }
  else
  {
    brakeDetect = false;
  }


  if (millis() - escMillis > escRampTime)
  {
    escMillis = millis();


    switch (driveState)
    {

    case 0:
      owIsBraking = false;
      owInReverse = false;
      owIsDriving = false;
#ifdef VIRTUAL_16_SPEED_SEQUENTIAL
      selectedGear = 1;
#endif
      if (rpmPulse() == 1 && engineRunning && !neutralGear)
        driveState = 1;
      if (rpmPulse() == -1 && engineRunning && !neutralGear)
        driveState = 3;
      break;

    case 1:
      owIsBraking = false;
      owInReverse = false;
      owIsDriving = true;

      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      {
        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      {

        gearDownShiftingPulse = false;
      }

      if (rpmPulse() == 1 && pidPulse() == -1)
        driveState = 2;
      if (rpmPulse() == -1 && pidPulse() == -1)
        driveState = 3;
      if (rpmPulse() == 0 )
        driveState = 0;
      break;

    case 2:
      owIsBraking = true;
      owInReverse = false;
      owIsDriving = false;

      if (rpmPulse() == 1 && pidPulse() == 1 && !neutralGear)
      {
        driveState = 1;
        airBrakeTrigger = true;
      }
      if (rpmPulse() == 0 )
      {
        driveState = 0;
        airBrakeTrigger = true;
      }
      break;

    case 3:
      owIsBraking = false;
      owInReverse = true;
      owIsDriving = true;

      if (gearUpShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      {

        gearUpShiftingPulse = false;
      }
      if (gearDownShiftingPulse && shiftingAutoThrottle && !automatic && !doubleClutch)
      {

        gearDownShiftingPulse = false;
      }

      if (rpmPulse() == -1 && pidPulse() == 1)
        driveState = 4;
      if (rpmPulse() == 1 && pidPulse() == 1)
        driveState = 1;
      if (rpmPulse() == 0 )
        driveState = 0;
      break;

    case 4:
      owIsBraking = true;
      owInReverse = true;
      owIsDriving = false;

      if (rpmPulse() == -1 && pidPulse() == -1 && !neutralGear)
      {
        driveState = 3;
        airBrakeTrigger = true;
      }
      if (rpmPulse() == 0 )
      {
        driveState = 0;
        airBrakeTrigger = true;
      }
      break;

    }


    if (currentSpeed < clutchEngagingPoint)
    {

      if (!automatic && !doubleClutch)
        driveRampGain = 2;
      else
        driveRampGain = 4;
    }
    else
    {
      driveRampGain = 1;
    }

    currentSpeed = get_vesc_throttle();

  }
}







float batteryVolts()
{
  static float raw[6];
  static bool initDone = false;
#define VOLTAGE_CALIBRATION (RESISTOR_TO_BATTTERY_PLUS + RESISTOR_TO_GND) / RESISTOR_TO_GND + DIODE_DROP

  if (!initDone)
  {
    for (uint8_t i = 0; i <= 5; i++)
    {
      raw[i] = battery.readVoltage();
    }
    initDone = true;
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];

  raw[0] = battery.readVoltage();

  float voltage = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6 * VOLTAGE_CALIBRATION;
  return voltage;
}







unsigned long loopDuration()
{
  static unsigned long timerOld;
  unsigned long loopTime;
  unsigned long timer = millis();
  loopTime = timer - timerOld;
  timerOld = timer;
  return loopTime;
}







void triggerHorn()
{
# 2043 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
}







void triggerIndicators()
{
# 2117 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
}







void updateRGBLEDs()
{
}
# 2138 "C:/Users/Spencer-Dell/Desktop/git/ow_engine_sound/src/main.ino"
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
  static float angle =0 ;
  float val;
  val=2*PI/360;
  static unsigned long fakeTimer = millis();
  if (millis() - fakeTimer > 250)
  {
    vescSwitchState=SWITCH_ON;
    vescErpm=MAX_ERPM*sin(val*angle);
    Serial.print("Fake Vesc Erpm :");
    Serial.println( vescErpm );
    vescPid=100*sin(90-(val*angle));
    Serial.print("Fake Vesc pid :");
    Serial.println( vescErpm );
    angle+=1;
    fakeTimer = millis();
  }
}






void loop()
{


  triggerHorn();

  triggerIndicators();

  if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
  {
#ifdef FAKE_VESC_DATA

    get_fake_data();

#else

    VESC.updateCustomValues(5);

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
    } else {

      engineOn =false;
      CSR_POWER_ON();
      AUDIO_SOURCE_CSR()
      AUDIO_UNMUTE();

    }

    xSemaphoreGive(xRpmSemaphore);
  }


#if defined NEOPIXEL_ENABLED
  updateRGBLEDs();
#endif


#if defined CORE_DEBUG
  Serial.print("Running on core ");
  Serial.println(coreId);
#endif
 check_mute(500);

rtc_wdt_feed();

}







void Task1code(void *pvParameters)
{
  for (;;)
  {




    dacOffsetFade();

    if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
    {


      engineMassSimulation();

      if (automatic || doubleClutch)
        automaticGearSelector();
      xSemaphoreGive(xRpmSemaphore);
    }





    gearboxDetection();


    esc();
   debug_print();

    loopTime = loopDuration();
  }
}