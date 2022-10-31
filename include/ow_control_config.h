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
#include <Arduino.h>
//#define DUTY_TO_THROTTLE // you want to vesc dutycycle as throttle else use abd ermp as throttle 
#define ESP32_DEBUG // DEBUG ESP32 
#define USE_DUAL_HEAD_LIGHT // USE dual led  for headlight
#define USE_RGB_LED         // use ws2812 RGB led
#define USE_FAN_COOLING             // use fan for cooling
#define READ_BATTERY_VOL_FROM_ADC //read Battery voltage from adc 
#define BATTERY_PROTECTION
// PWM LED driver pin
#define LED1_PIN 23
#define LED2_PIN 22
#define LED3_PIN 32
// FAN
#define FAN_PIN 27
// audio source selection 0:CSR8645 Audio  (CH442E IN pin pull low ) 1:ESP32 DAC output(CH442E IN pin pull high )
#define AUDIO_SOURCE_PIN 33
#define AUDIO_SOURCE_ESP()  digitalWrite(AUDIO_SOURCE_PIN,HIGH);
#define AUDIO_SOURCE_CSR()  digitalWrite(AUDIO_SOURCE_PIN,LOW);
// PAM8606 audio amplifier mute control
#define PAM_MUTE_PIN 13 //pcb board will modify the pin 
#define AUDIO_MUTE() digitalWrite(PAM_MUTE_PIN,LOW) //active lOW 
#define AUDIO_UNMUTE() digitalWrite(PAM_MUTE_PIN,HIGH) 
// CSR8645 BLE module power on/off (control en pin )
#define CSR_EN_PIN 5
#define CSR_POWER_ON() digitalWrite(CSR_EN_PIN, HIGH)
#define CSR_POWER_OFF() digitalWrite(CSR_EN_PIN,LOW) // pull en pin low to power off 
// ESP32 DAC
#define ESP_DAC1_PIN 25
#define ESP_DAC2_PIN 26
// TX RX for VESC
#define ESP_VESC_TX_PIN 16
#define ESP_VESC_RX_PIN 17
// Voltage divider resistors Low R5=? ,up R3=?
#ifdef READ_BATTERY_VOL_FROM_ADC 
#define BATTERY_DETECT_PIN 34
#define RES_TO_GND 2200                // R5 2.2k
#define RES_TO_BATTTERY_PLUS 68000    // R3  68K
#define BATTERY_CUTOFF_VOLTAGE 3.3;        // Usually 3.3 V per LiPo cell. NEVER below 3.2 V!
#define BATTERY_FULLY_CHARGED_VOLTAGE 4.2; // Usually 4.2 V per LiPo cell, NEVER above!
#define BATTERY_RECOVERY_HYSTERESIS 0.2;   // around 0.2 V
#define DIODE_DROP 0.0
#ifdef BATTERY_PROTECTION
volatile int outOfFuelVolumePercentage = 80; // Adjust the message volume in %

#endif
#define VOLTAGE_CALIBRATION (RE_TO_BATTTERY_PLUS + RESTO_GND) / RES_TO_GND + DIODE_DROP
#endif
// push button for trigger Horn & siren
#define PUSH_BUTTON_PIN 0
// RGB LED ws2812 driver pin
#define RGB_LED1_DATA_PIN 4
#define RGB_LED1_COUNT 8
#define RGB_LED2_DATA_PIN 2
#define RGB_LED2_COUNT 8
// balance state
typedef enum
{
  STARTUP = 0,
  RUN = 1,
  RUN_TILTBACK_DUTY = 2,
  RUN_TILTBACK_HIGH_VOLTAGE = 3,
  RUN_TILTBACK_LOW_VOLTAGE = 4,
  FAULT_ANGLE_PITCH = 6,
  FAULT_ANGLE_ROLL = 7,
  FAULT_SWITCH_HALF = 8,
  FAULT_SWITCH_FULL = 9,
  FAULT_DUTY = 10,
  FAULT_STARTUP = 11,

} BalanceState;
// switch state
typedef enum
{
  SWITCH_OFF = 0,
  SWITCH_HALF,
  SWITCH_ON
} SwitchState;

//audio source
//using VESC controll id to get source 
typedef enum
{ 
  SOURCE_CSR,
  SOURCE_ESP32,
} AudioSource;


#endif
