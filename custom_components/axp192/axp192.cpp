#include "axp192.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cstddef>

namespace esphome {
namespace axp192 {
static const char *TAG = "axp192.sensor";
void AXP192Component::setup() 
{
  begin(false, true, false, false, false);
}

void AXP192Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP192:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
}

float AXP192Component::get_setup_priority() const { return setup_priority::BUS; }

void AXP192Component::update() {

    if (this->batterylevel_sensor_ != nullptr) {
      // To be fixed
      // This is not giving the right value - mostly there to have some sample sensor...
      float vbat = GetBatVoltage();
      float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));

      float cbat = GetBatCurrent();
      float vvin = GetVinVoltage();
      float cvin = GetVinCurrent();
      float temp = GetTempInAXP192();

      ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
      ESP_LOGD(TAG, "Battery Current Draw=(%f)", cbat);
      ESP_LOGD(TAG, "Vin Voltage = (%f)", vvin);
      ESP_LOGD(TAG, "Vin Current = (%f)", cvin);
      ESP_LOGD(TAG, "AXP Temp = (%f)", temp);
      if (batterylevel > 100.) {
        batterylevel = 100;
      }
      this->batterylevel_sensor_->publish_state(batterylevel);
    }

    UpdateBrightness();
}


//void AXP192Component::write_state(bool state) override {
    // Turn motor on 
//    SetLDO3(state);
//   publish_state(state);
//}


void AXP192Component::begin(bool disableLDO2, bool disableLDO3, bool disableRTC, bool disableDCDC1, bool disableDCDC3)
{  
   ESP_LOGD(TAG, "Initializing Axp192");
   /* 
    Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0X02);
    Write1Byte(0x92, Read8bit(0x92) & 0xf8);
    Write1Byte(0x93, Read8bit(0x93) & 0xf8);
    Write1Byte(0x35, (Read8bit(0x35) & 0x1c) | 0xa2);

    SetLDOVoltage(2, 2800); //Periph power voltage preset (LCD_logic, SD card)
    SetLDOVoltage(3, 2000); //Vibrator power voltage preset
    SetESPVoltage(3300);
    SetLcdVoltage(2700);
    SetLDOEnable(2, true);   // turn off perhipherals
    
    SetDCDC3(true); // LCD backlight turn off too
    SetLed(false);   // led on  
    SetLDO2(true);  // LCD Controller ON
    //SetBUSPwr(true);
*/

    //pinMode(39, INPUT_PULLUP);

    //AXP192 GPIO4
//    Write1Byte(0X95, (Read8bit(0x95) & 0x72) | 0X84);

    uint16_t dc2_volt = 0;
    uint16_t ldo3_volt = 0;

    uint16_t ldo2_volt = 3300;
    uint16_t dc3_volt = 2800;
    
    // comment out next two lines to turn display back on, or uncommend for off
    
    //ldo2_volt = 0;    // Display OFF
    //dc3_volt = 0;     // Display OFF

    uint8_t value = 0x00;
    value |= 0x01 << AXP192_DC1_EN_BIT;
    EnableLDODCExt(value);                          // Disable LDO2/LDO3/DC2/DC3 while we configure them
    SetLDO23Volt(0,0);
    SetDCDC2Volt(0);
    SetDCDC3Volt(0);
    SetLCDRSet(0);
    delay(250);
    SetLCDRSet(1);
    ESP_LOGD(TAG, "Power Cycling Display");


    value |= (ldo2_volt > 0) << AXP192_LDO2_EN_BIT;
    value |= (ldo3_volt > 0) << AXP192_LDO3_EN_BIT;
    value |= (dc2_volt > 0) << AXP192_DC2_EN_BIT;
    value |= (dc3_volt > 0) << AXP192_DC3_EN_BIT;


    SetLDO23Volt(ldo2_volt, ldo3_volt);
    // Axp192_SetDCDC1Volt(3300);
    SetDCDC2Volt(dc2_volt);
    SetDCDC3Volt(dc3_volt);
    SetVoffVolt(3000);

    ESP_LOGD(TAG, "Display Powered up");
    SetLDO3Volt(2000);
    SetLDO3(true);
    delay(150);
    SetLDO3(false); 
    ESP_LOGD(TAG, "Vibrator Tested");

    SetChargeCurrent(CHARGE_Current_100mA);
    SetChargeVoltage(CHARGE_VOLT_4100mV);
    EnableCharge(1);
    SetPressStartupTime(STARTUP_128mS);
    SetPressPoweroffTime(POWEROFF_4S);
    
    EnableLDODCExt(value);
    SetGPIO4Mode(1);
    SetGPIO2Mode(1);
    SetGPIO2Level(0);

    //SetGPIO0Volt(3300);
    SetAdc1Enable(0xfe);
    SetGPIO1Mode(1);
    
    SetGPIO0Mode(1);
    EnableExten(1);
   
 //   Write1Byte(0X36, 0X4C);

  //  Write1Byte(0x82,0xff);

    //SetLCDRSet(0);
    //delay(300);
    //SetLCDRSet(1);
    //SetBusPowerMode(0);
    ESP_LOGD(TAG, "Display Reset");
    SetLCDRSet(0);
    delay(100);
    SetLCDRSet(1);
    ESP_LOGD(TAG, "Display Reset Complete");
    
    brightness_ = 10;
    UpdateBrightness();

}




void AXP192Component::Write1Byte( uint8_t Addr ,  uint8_t Data )
{
    this->write_byte(Addr, Data);
}

void AXP192Component::WriteBits(uint8_t reg_addr, uint8_t data, uint8_t bit_pos, uint8_t bit_length) {
    if ((bit_pos + bit_length) > 8) {
        return ;
    }

    uint8_t value = Read8bit(reg_addr);

    value &= ~(((1 << bit_length) - 1) << bit_pos);
    data &= (1 << bit_length) - 1;
    value |= data << bit_pos;

    this->write_byte(reg_addr, value);
}

void AXP192Component::EnableLDODCExt(uint8_t value) {
    uint8_t data = Read8bit(AXP192_LDO23_DC123_EXT_CTL_REG);
    data &= 0xa0;
    value |= 0x01 << AXP192_DC1_EN_BIT;
    value |= data;
    this->write_byte(0x12, value);
}

void AXP192Component::EnableExten(uint8_t state) {
    uint8_t value = state ? 1 : 0;
    WriteBits(AXP192_LDO23_DC123_EXT_CTL_REG, value, AXP192_EXT_EN_BIT, 1);
}

void AXP192Component::SetDCDC3(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if (State == true)
        buf = (1 << 1) | buf;
    else
        buf = ~(1 << 1) & buf;
    Write1Byte(0x12, buf);
}

uint8_t AXP192Component::Read8bit( uint8_t Addr )
{
    uint8_t data;
    this->read_byte(Addr, &data);
    return data;
}

uint16_t AXP192Component::Read12Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 4) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read13Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP192Component::ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff )
{
    this->read_bytes(Addr, Buff, Size);
}

void AXP192Component::UpdateBrightness()
{
    ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    if (brightness_ == curr_brightness_)
    {
        return;
    }
    curr_brightness_ = brightness_;
    this->ScreenBreath(brightness_);
}

void AXP192Component::ScreenBreath(uint8_t brightness)
{
    if (brightness > 12)
    {
        brightness = 12;
    }
    //uint8_t buf = Read8bit(0x28);
    //Write1Byte(0x28, ((buf & 0x0f) | (brightness << 4)));
}

bool AXP192Component::GetBatState()
{
    if( Read8bit(0x01) | 0x20 )
        return true;
    else
        return false;
}

uint8_t AXP192Component::GetBatData()
{
    return Read8bit(0x75);
}

void AXP192Component::SetLDOVoltage(uint8_t number, uint16_t voltage)
{
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number)
    {
    //uint8_t reg, data;
    case 2:
        Write1Byte(0x28, (Read8bit(0x28) & 0X0F) | (voltage << 4));
        break;
    case 3:
        Write1Byte(0x28, (Read8bit(0x28) & 0XF0) | voltage);
        break;
    }
}

void AXP192Component::SetDCVoltage(uint8_t number, uint16_t voltage)
{
    uint8_t addr;
    if (number > 2)
        return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number)
    {
    case 0:
        addr = 0x26;
        break;
    case 1:
        addr = 0x25;
        break;
    case 2:
        addr = 0x27;
        break;
    }
    Write1Byte(addr, (Read8bit(addr) & 0X80) | (voltage & 0X7F));
}

void AXP192Component::SetESPVoltage(uint16_t voltage)
{
    if (voltage >= 3000 && voltage <= 3400)
    {
        SetDCVoltage(0, voltage);
    }
}
void AXP192Component::SetLcdVoltage(uint16_t voltage)
{
    if (voltage >= 2500 && voltage <= 3300)
    {
        SetDCVoltage(2, voltage);
    }
}

void AXP192Component::SetLDOEnable(uint8_t number, bool state)
{
    uint8_t mark = 0x01;
    if ((number < 2) || (number > 3))
        return;

    mark <<= number;
    if (state)
    {
        Write1Byte(0x12, (Read8bit(0x12) | mark));
    }
    else
    {
        Write1Byte(0x12, (Read8bit(0x12) & (~mark)));
    }
}

void AXP192Component::SetLCDRSet(bool state)
{
    uint8_t reg_addr = 0x96;
    uint8_t gpio_bit = 0x02;
    uint8_t data;
    data = Read8bit(reg_addr);

    if (state)
    {
        data |= gpio_bit;
    }
    else
    {
        data &= ~gpio_bit;
    }
}


void AXP192Component::SetBUSPwr(bool state)
{
    uint8_t reg_addr = 0x96;
    uint8_t gpio_bit = 0x01;
    uint8_t data;
    data = Read8bit(reg_addr);

    if (state)
    {
        data |= gpio_bit;
    }
    else
    {
        data &= ~gpio_bit;
    }
}

void AXP192Component::SetBusPowerMode(uint8_t state)
{
    uint8_t data;
    if (state == 0)
    {
        data = Read8bit(0x91);
        Write1Byte(0x91, (data & 0X0F) | 0XF0);

        data = Read8bit(0x90);
        Write1Byte(0x90, (data & 0XF8) | 0X02); //set GPIO0 to LDO OUTPUT , pullup N_VBUSEN to disable supply from BUS_5V

        data = Read8bit(0x91);

        data = Read8bit(0x12);         //read reg 0x12
        Write1Byte(0x12, data | 0x40); //set EXTEN to enable 5v boost
    }
    else
    {
        data = Read8bit(0x12);         //read reg 0x10
        Write1Byte(0x12, data & 0XBF); //set EXTEN to disable 5v boost

        //delay(2000);

        data = Read8bit(0x90);
        Write1Byte(0x90, (data & 0xF8) | 0X01); //set GPIO0 to float , using enternal pulldown resistor to enable supply from BUS_5VS
    }
}

void AXP192Component::SetLed(uint8_t state)
{
    uint8_t reg_addr=0x94;
    uint8_t data;
    data=Read8bit(reg_addr);

    if(state)
    {
      data=data&0XFD;
    }
    else
    {
      data|=0X02;
    }

    Write1Byte(reg_addr,data);
}

//set led state(GPIO high active,set 1 to enable amplifier)
void AXP192Component::SetSpkEnable(uint8_t state)
{
    uint8_t reg_addr=0x94;
    uint8_t gpio_bit=0x04;
    uint8_t data;
    data=Read8bit(reg_addr);

    if(state)
    {
      data|=gpio_bit;
    }
    else
    {
      data&=~gpio_bit;
    }

    Write1Byte(reg_addr,data);
}

void AXP192Component::SetCHGCurrent(uint8_t state)
{
    uint8_t data = Read8bit(0x33);
    data &= 0xf0;
    data = data | ( state & 0x0f );
    Write1Byte(0x33,data);
}


//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void); 
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void  AXP192Component::EnableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x80 );
}

void  AXP192Component::DisableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x00 );
}

void  AXP192Component::StopCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xC0 );
}

void  AXP192Component::ClearCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xA0 );
}

uint32_t AXP192Component::GetCoulombchargeData(void)
{
    return Read32bit(0xB0);
}

uint32_t AXP192Component::GetCoulombdischargeData(void)
{
    return Read32bit(0xB4);
}

float AXP192Component::GetCoulombData(void)
{

  uint32_t coin = 0;
  uint32_t coout = 0;

  coin = GetCoulombchargeData();
  coout = GetCoulombdischargeData();

  //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
  //Adc rate can be read from 84H ,change this variable if you change the ADC reate
  float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
  return ccc;

}
//----------coulomb_end_at_here----------

uint16_t AXP192Component::GetVbatData(void){

    uint16_t vbat = 0;
    uint8_t buf[2];
    ReadBuff(0x78,2,buf);
    vbat = ((buf[0] << 4) + buf[1]); // V
    return vbat;
}

uint16_t AXP192Component::GetVinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x56,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x58,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetVusbinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x5a,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIusbinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x5C,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetIchargeData(void)
{
    uint16_t icharge = 0;
    uint8_t buf[2];
    ReadBuff(0x7A,2,buf);
    icharge = ( buf[0] << 5 ) + buf[1] ;
    return icharge;
}

uint16_t AXP192Component::GetIdischargeData(void)
{
    uint16_t idischarge = 0;
    uint8_t buf[2];
    ReadBuff(0x7C,2,buf);
    idischarge = ( buf[0] << 5 ) + buf[1] ;
    return idischarge;
}

uint16_t AXP192Component::GetTempData(void)
{
    uint16_t temp = 0;
    uint8_t buf[2];
    ReadBuff(0x5e,2,buf);
    temp = ((buf[0] << 4) + buf[1]);
    return temp;
}

uint32_t AXP192Component::GetPowerbatData(void)
{
    uint32_t power = 0;
    uint8_t buf[3];
    ReadBuff(0x70,2,buf);
    power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
    return power;
}

uint16_t AXP192Component::GetVapsData(void)
{
    uint16_t vaps = 0;
    uint8_t buf[2];
    ReadBuff(0x7e,2,buf);
    vaps = ((buf[0] << 4) + buf[1]);
    return vaps;
}

void AXP192Component::SetSleep(void)
{
    Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Power off voltag 3.0v
    Write1Byte(0x90 , Read8bit(0x90) | 0x07); // GPIO1 floating
    Write1Byte(0x82, 0x00); // Disable ADCs
    Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
}


// 0 not press, 0x01 long press, 0x02 press
uint8_t AXP192Component::GetBtnPress()
{
    uint8_t state = Read8bit(0x46);
    if(state) 
    {
        Write1Byte( 0x46 , 0x03 );
    }
    return state;
}

uint8_t AXP192Component::GetWarningLevel(void)
{
    return Read8bit(0x47) & 0x01;
}

float AXP192Component::GetBatVoltage()
{
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit( 0x78 );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit( 0x7A );
    uint16_t CurrentOut = Read13Bit( 0x7C );
    return ( CurrentIn - CurrentOut ) * ADCLSB;
}

float AXP192Component::GetVinVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x56 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVinCurrent()
{
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit( 0x58 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x5A );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit( 0x5C );
    return ReData * ADCLSB;
}

float AXP192Component::GetTempInAXP192()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit( 0x5E );
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192Component::GetBatPower()
{
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit( 0x70 );
    return  VoltageLSB * CurrentLCS * ReData/ 1000.0;
}

float AXP192Component::GetBatChargeCurrent()
{
    float ADCLSB = 0.5;
    uint16_t ReData = Read13Bit( 0x7A );
    return ReData * ADCLSB;
}

float AXP192Component::GetAPSVoltage()
{
    float ADCLSB = 1.4  / 1000.0;
    uint16_t ReData = Read12Bit( 0x7E );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCoulombInput()
{
    uint32_t ReData = Read32bit( 0xB0 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

float AXP192Component::GetBatCoulombOut()
{
    uint32_t ReData = Read32bit( 0xB4 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

void AXP192Component::SetCoulombClear()
{
    Write1Byte(0xB8,0x20);
}

void AXP192Component::SetLDO1( bool State )
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<1) | buf;
    }
    else
    {
        buf = ~(1<<1) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetLDO2( bool State )
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<2) | buf;
    }
    else
    {
        buf = ~(1<<2) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetLDO3(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<3) | buf;
    }
    else
    {
        buf = ~(1<<3) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetLDO23Volt(uint16_t ldo2_voltage, uint16_t ldo3_voltage) {
    uint8_t value2 = 0, value3 = 0;
    ldo2_voltage = VALUE_LIMIT(ldo2_voltage, AXP192_LDO_VOLT_MIN, AXP192_LDO_VOLT_MAX);
    ldo3_voltage = VALUE_LIMIT(ldo3_voltage, AXP192_LDO_VOLT_MIN, AXP192_LDO_VOLT_MAX);
    value2 = (ldo2_voltage - AXP192_LDO_VOLT_MIN) / AXP192_LDO_VOLT_STEP;
    value3 = (ldo3_voltage - AXP192_LDO_VOLT_MIN) / AXP192_LDO_VOLT_STEP;
    Write1Byte(AXP192_LDO23_VOLT_REG, ((value2 & 0x0f) << 4) | (value3 & 0x0f));
}

void AXP192Component::SetLDO2Volt(uint16_t voltage) {
    uint8_t value = 0;
    voltage = VALUE_LIMIT(voltage, AXP192_LDO_VOLT_MIN, AXP192_LDO_VOLT_MAX);
    value = (voltage - AXP192_LDO_VOLT_MIN) / AXP192_LDO_VOLT_STEP;
    WriteBits(AXP192_LDO23_VOLT_REG, value, 4, 4);
}

void AXP192Component::SetLDO3Volt(uint16_t voltage) {
    uint8_t value = 0;
    voltage = VALUE_LIMIT(voltage, AXP192_LDO_VOLT_MIN, AXP192_LDO_VOLT_MAX);
    value = (voltage - AXP192_LDO_VOLT_MIN) / AXP192_LDO_VOLT_STEP;
    WriteBits(AXP192_LDO23_VOLT_REG, value, 0, 4);
}

void AXP192Component::SetDCDC1Volt(uint16_t voltage) {
    uint8_t value = 0;
    voltage = VALUE_LIMIT(voltage, AXP192_DC_VOLT_MIN, AXP192_DC_VOLT_MAX);
    value = (voltage - AXP192_DC_VOLT_MIN) / AXP192_DC_VOLT_STEP;
    Write1Byte(AXP192_DC1_VOLT_REG, value);
}

void AXP192Component::SetDCDC2Volt(uint16_t voltage) {
    uint8_t value = 0;
    voltage = VALUE_LIMIT(voltage, AXP192_DC_VOLT_MIN, AXP192_DC_VOLT_MAX);
    value = (voltage - AXP192_DC_VOLT_MIN) / AXP192_DC_VOLT_STEP;
    WriteBits(AXP192_DC2_VOLT_REG, value, 0, 6);
}

void AXP192Component::SetDCDC3Volt(uint16_t voltage) {
    uint8_t value = 0;
    voltage = VALUE_LIMIT(voltage, AXP192_DC_VOLT_MIN, AXP192_DC_VOLT_MAX);
    value = (voltage - AXP192_DC_VOLT_MIN) / AXP192_DC_VOLT_STEP;
    Write1Byte(AXP192_DC3_VOLT_REG, value);
}

void AXP192Component::SetChargeCurrent(uint8_t current)
{
    uint8_t buf = Read8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);
    Write1Byte(0x33, buf);
}

void AXP192Component::SetVoffVolt(uint16_t voltage) 
{
    uint8_t value = 0;
    if(voltage < AXP192_VOFF_VOLT_MIN)
       voltage = AXP192_VOFF_VOLT_MIN;
    if(voltage > AXP192_VOFF_VOLT_MAX)
       voltage = AXP192_VOFF_VOLT_MAX;
    
    value = (voltage - AXP192_VOFF_VOLT_MIN) / AXP192_VOFF_VOLT_STEP;
    WriteBits(AXP192_VOFF_VOLT_REG, value, 0, 3);
}

void AXP192Component::PowerOff()
{
    WriteBits(AXP192_POWEROFF_REG, 0x01, 7, 1);
}

void AXP192Component::SetAdcState(bool state)
{
    Write1Byte(0x82, state ? 0xff : 0x00);
}

void AXP192Component::SetAdc1Enable(uint8_t value) {
    Write1Byte(AXP192_ADC1_ENABLE_REG, value);
}

void AXP192Component::SetGPIO0Mode(uint8_t mode) {
    if (mode == 0) {
        WriteBits(AXP192_GPIO0_CTL_REG, 0x01, 0, 4);
    } else {
        WriteBits(AXP192_GPIO0_CTL_REG, 0x02, 0, 4);
    }
}

void AXP192Component::SetGPIO0Volt(uint16_t volt) {
    uint8_t value;
    if (volt < 1800) 
       volt = 1800;
    if (volt > 3300)
       volt = 3300;
    value = (volt - 1800) / 100;
    WriteBits(AXP192_GPIO0_VOLT_REG, value, 4, 4);
}

void AXP192Component::SetGPIO1Mode(uint8_t mode) {
    WriteBits(AXP192_GPIO1_CTL_REG, mode, 0, 3);
}

void AXP192Component::SetGPIO1Level(uint8_t level) {
    uint8_t value = level ? 1 : 0;
    WriteBits(AXP192_GPIO012_STATE_REG, value, 1, 1);
}

void AXP192Component::SetGPIO2Mode(uint8_t mode) {
    WriteBits(AXP192_GPIO2_CTL_REG, 0x00, 0, 3);
}

void AXP192Component::SetGPIO2Level(uint8_t level) {
    uint8_t value = level ? 1 : 0;
    WriteBits(AXP192_GPIO012_STATE_REG, value, 2, 1);
}

void AXP192Component::SetGPIO4Mode(uint8_t mode) {
    WriteBits(AXP192_GPIO34_CTL_REG, 0x01, 2, 2);
    WriteBits(AXP192_GPIO34_CTL_REG, 0x01, 7, 1);
}

void AXP192Component::SetGPIO4Level(uint8_t level) {
    uint8_t value = level ? 1 : 0;
    WriteBits(AXP192_GPIO34_STATE_REG, value, 1, 1);
}

void AXP192Component::EnableCharge(uint16_t state) {
    uint8_t value = state ? 1 : 0;
    WriteBits(AXP192_CHG_CTL1_REG, value, 7, 1);
}

void AXP192Component::SetChargeVoltage(ChargeVolt_t volt) {
    WriteBits(AXP192_CHG_CTL1_REG, volt, 5, 2);
}

void AXP192Component::SetChargeCurrent(ChargeCurrent_t cur) {
    WriteBits(AXP192_CHG_CTL1_REG, cur, 0, 4);
}

void AXP192Component::SetPressPoweroffTime(PoweroffTime_t time) {
    WriteBits(AXP192_PEK_CTL_REG, time, 0, 2);
}

void AXP192Component::SetPressStartupTime(StartupTime_t time) {
    WriteBits(AXP192_PEK_CTL_REG, time, 6, 2);
}

uint16_t AXP192Component::VALUE_LIMIT(uint16_t value, uint16_t low, uint16_t high) {
     return value < low ? low : value > high ? high : value;
} 

}
}

