#include <Arduino.h>

/*一般傳輸設置 ******************************************************************************************************************
*
*大多數變速箱設置，如自動、雙離合器等，都在 /vehicles/目錄下的車輛配置文件中完成。
*
*/
//在“自動”或“雙離合器”車輛中忽略以下傳輸模式 =================================== ================
//永遠不要取消評論超過一個！如果沒有定義選項，您可以使用真正的 3 速傳輸，例如來自 TAMIYA
//#define VIRTUAL_3_SPEED 允許模擬 3 檔變速箱，如果您的車輛沒有真正的變速箱。
//使用 3 檔開關，齒輪實際上是換檔的。示例：您的履帶具有 2 速變速器，用作越野減速器，
//但沒有真正的 3 檔變速箱。如果是電動或靜液壓驅動或自動變速器的車輛，請不要取消註釋！
//也不要將它用於 STEAM_LOCOMOTIVE_MODE
//#define VIRTUAL_3_SPEED
//#define VIRTUAL_16_SPEED_SEQUENTIAL 將啟用順序傳輸，通過 3 位置開關按上/下脈衝移動
//#define VIRTUAL_16_SPEED_SEQUENTIAL //這仍然是實驗性的，不能正常工作！不要使用它。
//附加傳輸選項================================================ ==================================================== ========
//帶超速檔的自動變速器（最高檔時轉速較低，齒輪比低於 1:1，僅限 4 和 6 速）
//也可與 VIRTUAL_3_SPEED 結合使用。在這種情況下，如果以 3 檔行駛 @ 全油門，則 4 檔會自動切換
#define OVERDRIVE//不要將其用於：doubleClutch。不使用 SEMI_AUTOMATIC，但在這種情況下您可以將其保持打開狀態。
//在某些情況下，我們希望自動變速箱車輛具有不同的反向加速。
uint16_t automaticReverseAccelerationPercentage = 100;

// Low range percentage is used for MODE1_SHIFTING (off road reducer)
uint16_t lowRangePercentage = 58;// WPL 2 speed ratios = 29:1, 17:1 = 58% in low range. You may want to change this for other 2 speed transmissions

// Transmission controls options ===========================================================================================================
// #define SEMI_AUTOMATIC This will simulate a semi automatic transmission. Shifting is not controlled by the 3 position switch in this mode!
//#define SEMI_AUTOMATIC // Works for VIRTUAL_3_SPEED or real 3 speed transmission. Don't select this @ the same time as VIRTUAL_16_SPEED_SEQUENTIAL

//#define MODE1_SHIFTING The 2 speed transmission is shifted by the "Mode 1" button instead of the 3 position switch.
// This is often used in WPL vehicles with 2 speed transmission, used as off road reducer, shifted while driving slowly in order to engage properly.
//#define MODE1_SHIFTING

//#define TRANSMISSION_NEUTRAL Allows to put the transmission in neutral. This can't be used, if the "Mode 1" button is used for other stuff!
// You can leave it on, if defined MODE1_SHIFTING. It is disabled automatically in this case.
#define TRANSMISSION_NEUTRAL

// Clutch options ==========================================================================================================================
uint16_t maxClutchSlippingRpm = 250; // The clutch will never slip above this limit! (about 250) 500 for vehicles like locomotives
// and the Kirovets tractor with hydrostatic or electric drive! Mainly required for "VIRTUAL_3_SPEED" mode

//#define DOUBLE_CLUTCH // Double-clutch (Zwischengas) Enable this for older manual transmission trucks without synchronised gears

#define HIGH_SLIPPINGPOINT // Clutch will engage @ higher RPM, if defined. Comment this out for heavy vehicles like semi trucks
