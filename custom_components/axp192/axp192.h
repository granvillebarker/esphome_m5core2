#ifndef __AXP192_H__
#define __AXP192_H__

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace axp192 {

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)
#define AXP192_DC_VOLT_STEP  25
#define AXP192_DC_VOLT_MIN   700
#define AXP192_DC_VOLT_MAX   3500

#define AXP192_LDO_VOLT_STEP 100
#define AXP192_LDO_VOLT_MIN  1800
#define AXP192_LDO_VOLT_MAX  3300

#define AXP192_VOFF_VOLT_STEP 100
#define AXP192_VOFF_VOLT_MIN  2600
#define AXP192_VOFF_VOLT_MAX  3300

#define AXP192_LDO23_DC123_EXT_CTL_REG 0x12

#define AXP192_DC1_EN_BIT   (0)
#define AXP192_DC3_EN_BIT   (1)
#define AXP192_LDO2_EN_BIT  (2)
#define AXP192_LDO3_EN_BIT  (3)
#define AXP192_DC2_EN_BIT   (4)
#define AXP192_EXT_EN_BIT   (6)

#define AXP192_DC1_VOLT_REG         0x26
#define AXP192_DC2_VOLT_REG         0x23
#define AXP192_DC3_VOLT_REG         0x27
#define AXP192_LDO23_VOLT_REG       0x28
#define AXP192_VBUS_IPSOUT_CTL_REG  0x30 // not support yet
#define AXP192_VOFF_VOLT_REG        0x31 // PWRON short press in here
#define AXP192_POWEROFF_REG         0x32 // CHGLED in here
#define AXP192_CHG_CTL1_REG         0x33 
#define AXP192_CHG_CTL2_REG         0x34
#define AXP192_SPARE_CHG_CTL_REG    0x35
#define AXP192_PEK_CTL_REG          0x36
#define AXP192_CHG_BOOL_REG         0x01
#define AXP192_ADC1_ENABLE_REG      0x82
#define BAT_VOLT_BIT        (7)
#define BAT_CURRENT_BIT     (6)
#define ACIN_VOLT_BIT       (5)
#define ACIN_CURRENT_BIT    (4)
#define VBUS_VOLT_BIT       (3)
#define VBUS_CURRENT_BIT    (2)
#define APS_VOLT_BIT        (1)
#define TS_BIT              (0)

#define AXP192_ACIN_ADC_VOLTAGE_REG         0x56
#define AXP192_ACIN_ADC_CURRENT_REG         0x58

#define AXP192_VBUS_ADC_VOLTAGE_REG         0x5A
#define AXP192_VBUS_ADC_CURRENT_REG         0x5C

#define AXP192_BAT_ADC_VOLTAGE_REG          0x78
#define AXP192_BAT_ADC_CURRENT_IN_REG       0x7A
#define AXP192_BAT_ADC_CURRENT_OUT_REG      0x7C

#define AXP192_GPIO0_CTL_REG                0x90                   
#define AXP192_GPIO0_VOLT_REG               0x91                   
#define AXP192_GPIO1_CTL_REG                0x92                   
#define AXP192_GPIO2_CTL_REG                0x93
#define AXP192_GPIO34_CTL_REG               0x95

#define AXP192_GPIO34_STATE_REG             0x96
#define AXP192_GPIO012_STATE_REG            0x94


class AXP192Component : public PollingComponent, public i2c::I2CDevice  {
public:
    enum CHGCurrent{
        kCHG_100mA = 0,
        kCHG_190mA,
        kCHG_280mA,
        kCHG_360mA,
        kCHG_450mA,
        kCHG_550mA,
        kCHG_630mA,
        kCHG_700mA,
        kCHG_780mA,
        kCHG_880mA,
        kCHG_960mA,
        kCHG_1000mA,
        kCHG_1080mA,
        kCHG_1160mA,
        kCHG_1240mA,
        kCHG_1320mA,
    };
    /**
     * @brief List of available charging voltages.
     */
    /* @[declare_axp192_chargevolt] */
    typedef enum {
        CHARGE_VOLT_4100mV = 0b0000, /**< @brief Charge at 4.10v. */
        CHARGE_VOLT_4150mV = 0b0001, /**< @brief Charge at 4.15v. */
        CHARGE_VOLT_4200mV = 0b0010, /**< @brief Charge at 4.20v. */
        CHARGE_VOLT_4360mV = 0b0011, /**< @brief Charge at 4.36v. */
    } ChargeVolt_t;
    
    /**
     * @brief List of available charging current rates.
     */
    typedef enum {
        CHARGE_Current_100mA = 0b0000, /**< @brief Charge at 100mA. */
        CHARGE_Current_190mA,          /**< @brief Charge at 190mA. */
        CHARGE_Current_280mA,          /**< @brief Charge at 280mA. */
        CHARGE_Current_360mA,          /**< @brief Charge at 360mA. */
        CHARGE_Current_450mA,          /**< @brief Charge at 450mA. */
        CHARGE_Current_550mA,          /**< @brief Charge at 550mA. */
        CHARGE_Current_630mA,          /**< @brief Charge at 630mA. */
        CHARGE_Current_700mA,          /**< @brief Charge at 700mA. */
        CHARGE_Current_780mA,          /**< @brief Charge at 780mA. */
        CHARGE_Current_880mA,          /**< @brief Charge at 880mA. */
        CHARGE_Current_960mA,          /**< @brief Charge at 960mA. */
        CHARGE_Current_1000mA,         /**< @brief Charge at 1000mA. */
        CHARGE_Current_1080mA,         /**< @brief Charge at 1080mA. */
        CHARGE_Current_1160mA,         /**< @brief Charge at 1160mA. */
        CHARGE_Current_1240mA,         /**< @brief Charge at 1240mA. */
        CHARGE_Current_1320mA,         /**< @brief Charge at 1320mA. */
    } ChargeCurrent_t;
    
    typedef enum {
        SPARE_CHARGE_VOLT_3100mV = 0x00,
        SPARE_CHARGE_VOLT_3000mV = 0x01,
        SPARE_CHARGE_VOLT_2500mV = 0x03,
    } SpareChargeVolt_t;

    typedef enum {
        SPARE_CHARGE_Current_50uA = 0x00,
        SPARE_CHARGE_Current_100uA = 0x01,
        SPARE_CHARGE_Current_200uA = 0x02,
        SPARE_CHARGE_Current_400uA = 0x03,
    } SpareChargeCurrent_t;

   typedef enum {
    STARTUP_128mS = 0x00, /**< @brief Power button hold for 128ms to turn on. */
    STARTUP_512mS = 0x01, /**< @brief Power button hold for 512ms to turn on. */
    STARTUP_1S = 0x02, /**< @brief Power button hold for 1s to turn on. */
    STARTUP_2S = 0x03, /**< @brief Power button hold for 2s to turn on. */
   } StartupTime_t;
/* @[declare_axp192_startuptime] */

/**
 * @brief List of possible durations the power button must 
 * be held to power off the Core2 for AWS IoT EduKit.
 */
/* @[declare_axp192_powerofftime] */
   typedef enum {
    POWEROFF_4S = 0x00, /**< @brief Power button hold for 4s to turn off. */
    POWEROFF_6S = 0x01, /**< @brief Power button hold for 6s to turn off. */
    POWEROFF_8S = 0x02, /**< @brief Power button hold for 8s to turn off. */
    POWEROFF_10S = 0x03, /**< @brief Power button hold for 10s to turn off. */
    } PoweroffTime_t;
/* @[declare_axp192_powerofftime] */

  void set_batterylevel_sensor(sensor::Sensor *batterylevel_sensor) { batterylevel_sensor_ = batterylevel_sensor; }
  void set_brightness(float brightness) { brightness_ = brightness; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  //void write_state(bool state) override;

protected:
    sensor::Sensor *batterylevel_sensor_;


    float brightness_{1.0f};
    float curr_brightness_{-1.0f};

    /**
     * LDO2: Display backlight
     * LDO3: Display Control
     * RTC: Don't set GPIO1 as LDO
     * DCDC1: Main rail. When not set the controller shuts down.
     * DCDC3: Use unknown
     */
    void  begin(bool disableLDO2 = false, bool disableLDO3 = false, bool disableRTC = false, bool disableDCDC1 = false, bool disableDCDC3 = false);
    void  UpdateBrightness();
    bool  GetBatState();
    uint8_t  GetBatData();

    void  EnableCoulombcounter(void);
    void  DisableCoulombcounter(void);
    void  StopCoulombcounter(void);
    void  ClearCoulombcounter(void);
    uint32_t GetCoulombchargeData(void);
    uint32_t GetCoulombdischargeData(void);
    float GetCoulombData(void); 
    
    uint16_t GetVbatData(void) __attribute__((deprecated));
    uint16_t GetIchargeData(void) __attribute__((deprecated));
    uint16_t GetIdischargeData(void) __attribute__((deprecated));
    uint16_t GetTempData(void) __attribute__((deprecated));
    uint32_t GetPowerbatData(void) __attribute__((deprecated));
    uint16_t GetVinData(void) __attribute__((deprecated));
    uint16_t GetIinData(void) __attribute__((deprecated));
    uint16_t GetVusbinData(void) __attribute__((deprecated));
    uint16_t GetIusbinData(void) __attribute__((deprecated));
    uint16_t GetVapsData(void) __attribute__((deprecated));
    uint8_t GetBtnPress(void);

      // -- sleep
    void SetSleep(void);
    void DeepSleep(uint64_t time_in_us = 0);
    void LightSleep(uint64_t time_in_us = 0);

    // void SetChargeVoltage( uint8_t );
    void  SetChargeCurrent( uint8_t );
    float GetBatVoltage();
    float GetBatCurrent();
    float GetVinVoltage();
    float GetVinCurrent();
    float GetVBusVoltage();
    float GetVBusCurrent();
    float GetTempInAXP192();
    float GetBatPower();
    float GetBatChargeCurrent();
    float GetAPSVoltage();
    float GetBatCoulombInput();
    float GetBatCoulombOut();
    uint8_t GetWarningLevel(void);	
    void SetCoulombClear();
    void SetLDO1( bool State );
    void SetLDO2( bool State );
    void SetLDO3( bool State );
    void SetVoffVolt(uint16_t voltage);
    void SetAdcState(bool State);
    void ScreenBreath(uint8_t brightness);
    
    void PowerOff();
 
    void SetLDOVoltage(uint8_t number , uint16_t voltage);
    void SetDCVoltage(uint8_t number , uint16_t voltage);
    void SetESPVoltage(uint16_t voltage);
    void SetLcdVoltage(uint16_t voltage);
    void SetLDOEnable( uint8_t number ,bool state );
    void SetLCDRSet( bool state );
    void SetBUSPwr( bool state );
    void SetBusPowerMode( uint8_t state );
    void SetLed(uint8_t state);
    void SetSpkEnable(uint8_t state);
    void SetCHGCurrent(uint8_t state);
	void SetDCDC3( bool State );
    void EnableLDODCExt(uint8_t value) ;
    void EnableExten(uint8_t state) ;
    void Write1Byte( uint8_t Addr ,  uint8_t Data );
    uint8_t Read8bit( uint8_t Addr );
    uint16_t Read12Bit( uint8_t Addr);
    uint16_t Read13Bit( uint8_t Addr);
    uint16_t Read16bit( uint8_t Addr );
    uint32_t Read24bit( uint8_t Addr );
    uint32_t Read32bit( uint8_t Addr );
    void ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff );
/* @[declare_axp192_setpresspowerofftime] */

/**
 * @brief Sets how long the power button needs to be held 
 * to power on all components connected to the AXP192.
 * 
 * Default is @ref STARTUP_128mS (128 milliseconds).
 * 
 * @param[in] time Desired time to hold the power button 
 * to turn on power. 
 */
void SetPressStartupTime(StartupTime_t time);
void SetPressPoweroffTime(PoweroffTime_t time);

void WriteDataStash();

void ReadDataStash();

/**
 * @brief Sets the mode of GPIO 4 on the AXP192 for
 * charging control.
 * 
 * @param[in] mode Desired mode of GPIO 4.
 * 0 for NMOS open-drain ouput, 1 for universal input. 
 * Default value for the Core2 for AWS 
 * is 1, and not meant to be changed. 
 */

/* @[declare_axp192_setgpio4mode] */
void SetGPIO4Mode(uint8_t mode);
/* @[declare_axp192_setgpio4mode] */

/**
 * @brief Sets the signal status level of GPIO 4.
 * 
 * @param[in] level Desired signal status level 
 * of GPIO 4.
 */
/* @[declare_axp192_setgpio4level] */
void SetGPIO4Level(uint8_t level);
/* @[declare_axp192_setgpio4level] */

/**
 * @brief Sets the mode of GPIO 2 on the AXP192.
 * 
 * @param[in] mode Desired mode of GPIO 2.
 * 0 for PWM, 1 for ADC input. 
 * Default value for the Core2 for AWS 
 * is 1, and not meant to be changed. 
 */
/* @[declare_axp192_setgpio2mode] */
void SetGPIO2Mode(uint8_t mode);
/* @[declare_axp192_setgpio2mode] */

/**
 * @brief Sets the average voltage level of GPIO 2 
 * on the AXP192 if the mode is set to PWM.
 * 
 * @param[in] level Desired average voltage level 
 * of GPIO 2.
 */

/* @[declare_axp192_setgpio2level] */
void SetGPIO2Level(uint8_t level);
/* @[declare_axp192_setgpio2level] */

/**
 * @brief Sets the mode of GPIO 0 on the AXP192.
 * 
 * @param[in] mode Desired mode of GPIO 0.
 * 0 for Low-Dropout, 1 for ADC input. 
 * Default value for the Core2 for AWS 
 * is 0, and not meant to be changed. 
 */
/* @[declare_axp192_setgpio0mode] */
void SetGPIO0Mode(uint8_t mode);
/* @[declare_axp192_setgpio0mode] */

/**
 * @brief Sets the voltage of GPIO 0 on the AXP192.
 * 
 * @param[in] volt Desired voltage of GPIO 0.
 * Default value for the Core2 for AWS is 3300mV.
 */
/* @[declare_axp192_setgpio0volt] */
void SetGPIO0Volt(uint16_t volt);
/* @[declare_axp192_setgpio0volt] */

/**
 * @brief Sets the mode of GPIO 1 on the AXP192.
 * 
 * @param[in] mode Desired mode of GPIO 1.
 * 0 for PWM, 1 for ADC input. 
 * Default value for the Core2 for AWS 
 * is 1, and not meant to be changed. 
 */
/* @[declare_axp192_setgpio1mode] */
void SetGPIO1Mode(uint8_t mode);
/* @[declare_axp192_setgpio1mode] */

/**
 * @brief Sets the average voltage level of GPIO 1 
 * on the AXP192 if the mode is set to PWM.
 * 
 * @param[in] level Desired average voltage level 
 * of GPIO 1.
 */
/* @[declare_axp192_setgpio1level] */
void SetGPIO1Level(uint8_t level);
/* @[declare_axp192_setgpio1level] */

void SetLDO23Volt(uint16_t ldo2_voltage, uint16_t ldo3_voltage) ;
uint16_t VALUE_LIMIT(uint16_t value, uint16_t low, uint16_t high);
void EnableCharge(uint16_t state) ;
void SetChargeVoltage(ChargeVolt_t volt) ;
void SetChargeCurrent(ChargeCurrent_t cur) ;
void SetLDO2Volt(uint16_t voltage) ;
void SetLDO3Volt(uint16_t voltage) ;
void SetDCDC1Volt(uint16_t voltage) ;
void SetDCDC2Volt(uint16_t voltage) ;
void SetDCDC3Volt(uint16_t voltage) ;
void SetAdc1Enable(uint8_t value) ;
void WriteBits(uint8_t reg_addr, uint8_t data, uint8_t bit_pos, uint8_t bit_length) ;
}; 

}
}

#endif
