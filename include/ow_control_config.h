/**
 * @file ow_control_config.h
 * @author spencer.chen7@gmail.com
 * @brief
 * @version 0.1
 * @date 2022-09-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef OW_CONTROL_CONFIG_H_
#define OW_CONTROL_CONFIG_H_

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SET_BIT(value, bit) (value |= (1 << bit))
#define CLEAR_BIT(value, bit) (value &= ~(1 << bit))
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

#include <Arduino.h>
//#define DUTY_TO_THROTTLE // you want to vesc dutycycle as throttle else use abd ermp as throttle 
#define DEBUG 0
#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#define USE_DUAL_HEAD_LIGHT // USE dual led  for headlight
#define USE_RGB_LED         // use ws2812 RGB led
#define USE_FAN_COOLING             // use fan for cooling
#define READ_BATTERY_VOL_FROM_ADC //read Battery voltage from adc 
//#define BATTERY_PROTECTION
// PWM LED driver pin
#define LED1_PIN 23
#define LED2_PIN 22
#define LED3_PIN 32
// FAN
#define FAN_PIN 27
// audio source selection 0:CSR8645 Audio  (CH442E IN pin pull low ) 1:ESP32 DAC output(CH442E IN pin pull high )
#define AUDIO_SOURCE_PIN 33
#define SET_AUDIO_SOURCE_ESP()  digitalWrite(AUDIO_SOURCE_PIN,HIGH)
#define SET_AUDIO_SOURCE_CSR()  digitalWrite(AUDIO_SOURCE_PIN,LOW)
// PAM8606 audio amplifier mute control
#define PAM_MUTE_PIN 13 //pcb board will modify the pin 
#define SET_AUDIO_MUTE() digitalWrite(PAM_MUTE_PIN,LOW) //active lOW 
#define SET_AUDIO_UNMUTE() digitalWrite(PAM_MUTE_PIN,HIGH) 
// CSR8645 BLE module power on/off (control en pin )
#define CSR_EN_PIN 5
#define SET_CSR_POWER_ON() digitalWrite(CSR_EN_PIN, HIGH)
#define SET_CSR_POWER_OFF() digitalWrite(CSR_EN_PIN,LOW) // pull en pin low to power off 
// ESP32 DAC
#define ESP_DAC1_PIN 25
#define ESP_DAC2_PIN 26
// TX RX for VESC
#define ESP_VESC_TX_PIN 16
#define ESP_VESC_RX_PIN 17
#define EPS_BUTTON_PIN 0
// Voltage divider resistors Low R5=? ,up R3=?
//#ifdef READ_BATTERY_VOL_FROM_ADC 

//#endif
//#define VOLTAGE_CALIBRATION (RE_TO_BATTTERY_PLUS + RESTO_GND) / RES_TO_GND + DIODE_DROP
//#endif
// push button for trigger Horn & siren
#define PUSH_BUTTON_PIN 0
// RGB LED ws2812 driver pin
#define RGB_LED1_DATA_PIN 4
#define RGB_LED1_COUNT 8
#define RGB_LED2_DATA_PIN 2
#define RGB_LED2_COUNT 8
/**
 * rpm = erpm / (poles / 2.0)
	kmh = ((rpm / 60.0) * wheel_d * M_PI / gearing) * 3.6;
 * Speed = ((motor poles/2) * 60 *gear ratio ) / (wheel diameter * 3.1415)
 * 
 */
#define MOTOR_POLES 30
#define GEAR_RATIO 1
#define WHEEL_DIAMETER 0.27 

/// @brief Data source for calculating throttle
typedef enum
{
	FLOAT_MOTOR_CURRENT = 0,
	FLOAT_ERPM,
	FLOAT_PID,
} FloatSamplingType;
/**idle warning time */
typedef enum
{
  FLOAT_IDLE_WARNING_TIME_DISABLE = 0,
  FLOAT_IDLE_WARNING_TIME_1M,
  FLOAT_IDLE_WARNING_TIME_5M,
  FLOAT_IDLE_WARNING_TIME_10M,
  FLOAT_IDLE_WARNING_TIME_30M,
  FLOAT_IDLE_WARNING_TIME_60M,
  FLOAT_IDLE_WARNING_TIME_120M

} floatIdleTime;


// balance state
typedef enum
{
	FLOAT_STARTUP = 0,
	FLOAT_RUNNING = 1,
	FLOAT_RUNNING_TILTBACK = 2,
	FLOAT_RUNNING_WHEELSLIP = 3,
	FLOAT_RUNNING_UPSIDEDOWN = 4,
	FLOAT_FAULT_ANGLE_PITCH = 5,
	FLOAT_FAULT_ANGLE_ROLL = 6,
	FLOAT_FAULT_SWITCH_HALF = 7,
	FLOAT_FAULT_SWITCH_FULL = 8,
	FLOAT_FAULT_STARTUP = 9,
	FLOAT_FAULT_REVERSE = 10,
	FLOAT_FAULT_QUICKSTOP = 11
} FloatState;


typedef enum
{
	FLOAT_SWITCH_OFF = 0,
	FLOAT_SWITCH_HALF,
	FLOAT_SWITCH_ON
} FloatSwitchState;

/**light mode*/
typedef enum
{
	FLOAT_LIGHT_OFF = 0,
	FLOAT_LIGHT_FLASH,
	FLOAT_LIGHT_FULL_ON
} floatLightMode;

/**
 * aoudio source 
 * AUDIO_SOURCE_CSR
 * AUDIO_SOURCE_ESP32
 */
typedef enum
{
  AUDIO_SOURCE_CSR,
  AUDIO_SOURCE_ESP32,
} Audio_Source;



//additional sound setting 
volatile int startUpWarningPercentage=60; //start-up warning sound 
#include "vehicles/sounds/welcome.h"
volatile int overSpeedVolumePercentage=200; // over speed sound 
#include "vehicles/sounds/overSpeed.h"
volatile int lowVoltageVolumePercentage=100;
#include "vehicles/sounds/lowVoltage.h"
volatile int excuseMeVolumePercentage=200;
#include "vehicles/sounds/hurley.h"
volatile int vescNotConnectVolumePercentage=100;
#include "vehicles/sounds/vescNC.h"

//#define ENABLE_NOTIFY
#ifdef ENABLE_NOTIFY
volatile int notifyVolumePercentage=150;
#include "vehicles/sounds/notify.h"
#endif

typedef struct{
	float a0, a1, a2, b1, b2;
	float z1, z2;
} Biquad;

typedef enum {
	BQ_LOWPASS,
	BQ_HIGHPASS
} BiquadType;


#endif
