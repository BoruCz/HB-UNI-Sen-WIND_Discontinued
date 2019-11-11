//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-02-26 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
// #define NSENSORS // if defined, only fake values are used

//#define NDEBUG
#define  EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#define USE_WOR
#include <AskSinPP.h>
#include <LowPower.h>

#include "PCF8583.h"
#include <Register.h>
#include <MultiChannelDevice.h>

// Arduino Pro mini 8 Mhz
// Arduino pin for the config button
#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4

#define WINDDIRECTION_PIN                    A3    // Pin, to which the wind direction indicator is connected
//#define WINDDIRECTION_USE_PULSE

//                             N                      O                       S                         W
//entspricht Windrichtung in ° 0 , 22.5, 45  , 67.5, 90  ,112.5, 135, 157.5, 180 , 202.5, 225 , 247.5, 270 , 292.5, 315 , 337.5
#ifdef WINDDIRECTION_USE_PULSE
const uint16_t WINDDIRS[] = { 70, 78, 86, 94, 102, 108, 116, 0, 8, 16, 24, 32, 40, 48, 56, 62 };
#else
const uint16_t WINDDIRS[] = { 497, 173, 205, 39, 42, 33, 74, 53, 115, 97, 327, 301, 790, 560, 665 , 397 };
#endif

#define WINDSPEED_MEASUREINTERVAL_SECONDS    5     // Measurement interval (seconds) for wind speed / gust

//some static definitions
#define WINDSPEED_MAX              0x3FFF
#define GUSTSPEED_MAX              0x7FFF
#define STORM_COND_VALUE_LO        100
#define STORM_COND_VALUE_HI        200
#define PEERS_PER_CHANNEL          4

using namespace as;

PCF8583 windcounter(0xA0);
volatile uint16_t _wind_isr_counter = 0;

enum eventMessageSources {EVENT_SRC_GUST};

const struct DeviceInfo PROGMEM devinfo = {
  {0xF1, 0xD2, 0x01},        // Device ID
  "JPWND00001",           	 // Device Serial
  {0xF1, 0xD2},            	 // Device Model
  0x05,                   	 // Firmware Version
  as::DeviceType::THSensor,  // Device Type
  {0x01, 0x01}             	 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AskSin<StatusLed<LED_PIN>, BatterySensor, Radio<AvrSPI<10, 11, 12, 13>, 2>> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
      // measure battery every 1h
      battery.init(seconds2ticks(60UL * 60), sysclock);
      battery.low(28);
      battery.critical(22);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;

class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t channel, uint16_t windspeed, uint8_t winddir, uint8_t winddirrange, uint16_t gustspeed, bool batlow, uint8_t volt) {
      Message::init(0x12, msgcnt, 0x70, BIDI | RPTEN, batlow ? 0x80 : 0x00, channel & 0xff );
	      pload[0] = (windspeed >> 8) & 0xff;
		    pload[1] = windspeed & 0xff;
		    pload[2] = winddir & 0xff;
		    pload[3] = winddirrange & 0xff;
		    pload[4] = (gustspeed >> 8) & 0xff;
		    pload[5] = gustspeed & 0xff;
	      pload[6] = volt & 0xff;
    }
};

class ExtraEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint16_t gustspeed) {
      Message::init(0x0d, msgcnt, 0x53, BIDI | RPTEN, 0x41, 0x00);
      pload[0] =  (gustspeed >> 8) & 0xff;
      pload[1] =  gustspeed & 0xff;
    }
};

DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x20, 0x21)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(26);
      Sendeintervall(180);
    }
};

DEFREGISTER(Reg1, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)
class SensorList1 : public RegList1<Reg1> {
  public:
    SensorList1 (uint16_t addr) : RegList1<Reg1>(addr) {}
    bool AnemometerRadius (uint8_t value) const {
      return this->writeRegister(0x01, value & 0xff);
    }
    uint8_t AnemometerRadius () const {
      return this->readRegister(0x01, 0);
    }
    bool AnemometerCalibrationFactor (uint16_t value) const {
      return this->writeRegister(0x02, (value >> 8) & 0xff) && this->writeRegister(0x03, value & 0xff);
    }
    uint16_t AnemometerCalibrationFactor () const {
      return (this->readRegister(0x02, 0) << 8) + this->readRegister(0x03, 0);
    }
    bool ExtraMessageOnGustThreshold (uint8_t value) const {
      return this->writeRegister(0x06, value & 0xff);
    }
    uint8_t ExtraMessageOnGustThreshold () const {
      return this->readRegister(0x06, 0);
    }
    bool StormUpperThreshold (uint8_t value) const {
      return this->writeRegister(0x07, value & 0xff);
    }
    uint8_t StormUpperThreshold () const {
      return this->readRegister(0x07, 0);
    }
    bool StormLowerThreshold (uint8_t value) const {
      return this->writeRegister(0x08, value & 0xff);
    }
    uint8_t StormLowerThreshold () const {
      return this->readRegister(0x08, 0);
    }

    void defaults () {
      clear();
      AnemometerRadius(65);
      AnemometerCalibrationFactor(10);
      ExtraMessageOnGustThreshold(0);
      StormUpperThreshold(0);
      StormLowerThreshold(0);
    }
};

class WeatherChannel : public Channel<Hal, SensorList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    uint16_t      windspeed;
    uint16_t      gustspeed;
    uint8_t       winddir;
    uint8_t       winddirrange;
    uint16_t      stormUpperThreshold;
    uint16_t      stormLowerThreshold;
    bool          initComplete;
    uint8_t       short_interval_measure_count;

  public:
    WeatherChannel () : Channel(), Alarm(seconds2ticks(60)), windspeed(0), stormUpperThreshold(0), stormLowerThreshold(0), initComplete(false), short_interval_measure_count(0), wind_measure(*this) {}
    virtual ~WeatherChannel () {}

    class WindSpeedMeasureAlarm : public Alarm {
        WeatherChannel& chan;
      public:
        WindSpeedMeasureAlarm (WeatherChannel& c) : Alarm (seconds2ticks(WINDSPEED_MEASUREINTERVAL_SECONDS)), chan(c) {}
        virtual ~WindSpeedMeasureAlarm () {}

        void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
          chan.measure_windspeed();
          tick = (seconds2ticks(WINDSPEED_MEASUREINTERVAL_SECONDS));
          DPRINT(F("WINDSPEED_MEASUREINTERVAL_TICK     : ")); DDECLN(tick);
          clock.add(*this);
          chan.short_interval_measure_count++;
          DPRINT(F("SHORT INTERVAL MEASURE COUNT     : ")); DDECLN(chan.short_interval_measure_count);
        }
    } wind_measure;

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      measure_winddirection();

      if (initComplete) {
        windspeed = windspeed / short_interval_measure_count;
        if (windspeed > WINDSPEED_MAX) windspeed = WINDSPEED_MAX;
      }

      DPRINT(F("GUSTSPEED     : ")); DDECLN(gustspeed);
      DPRINT(F("WINDSPEED     : ")); DDECLN(windspeed);

      WeatherEventMsg& msg = (WeatherEventMsg&)device().message();
      uint8_t msgcnt = device().nextcount();
      msg.init(msgcnt, number(), windspeed, winddir, winddirrange, gustspeed, device().battery().low(), device().battery().current());
      if (msgcnt % 10 == 1) {
        device().sendMasterEvent(msg);
      } else {
        device().broadcastEvent(msg, *this);
      }
      uint16_t updCycle = this->device().getList0().Sendeintervall();
      tick = (seconds2ticks(updCycle));

      initComplete = true;
      windspeed = 0;
      gustspeed = 0;
      short_interval_measure_count = 0;
      sysclock.add(*this);
    }

    void sendExtraMessage (uint8_t t) {
      DPRINT(F("SENDING EXTRA MESSAGE ")); DDECLN(t);
      ExtraEventMsg& extramsg = (ExtraEventMsg&)device().message();
      extramsg.init(device().nextcount(), gustspeed);
      device().sendMasterEvent(extramsg);
    }

    void measure_windspeed() {
#ifdef NSENSORS
      _wind_isr_counter = random(20);
#endif
      _wind_isr_counter = windcounter.getCount() / 4;
      windcounter.setCount(0);
      uint16_t kmph = ((226L * this->getList1().AnemometerRadius() * this->getList1().AnemometerCalibrationFactor() * _wind_isr_counter) / WINDSPEED_MEASUREINTERVAL_SECONDS) / 10000;
      if (kmph > gustspeed) {
        gustspeed = (kmph > GUSTSPEED_MAX) ? GUSTSPEED_MAX : kmph;
      }

      DPRINT(F("WIND PULSE COUNTER     : ")); DDECLN(_wind_isr_counter);
      DPRINT(F("WIND kmph     : ")); DDECLN(kmph);
      
      if (this->getList1().ExtraMessageOnGustThreshold() > 0 && kmph > (this->getList1().ExtraMessageOnGustThreshold() * 10)) {
        sendExtraMessage(EVENT_SRC_GUST);
      }
     
      //DPRINT(F("UPPER THRESH  : ")); DDECLN(stormUpperThreshold);
      //DPRINT(F("LOWER THRESH  : ")); DDECLN(stormLowerThreshold);

      static uint8_t STORM_COND_VALUE_Last = STORM_COND_VALUE_LO;
      static uint8_t STORM_COND_VALUE      = STORM_COND_VALUE_LO;

      if (stormUpperThreshold > 0) {
        if (kmph >= stormUpperThreshold || kmph <= stormLowerThreshold) {
          static uint8_t evcnt = 0;

          if (kmph >= stormUpperThreshold) STORM_COND_VALUE = STORM_COND_VALUE_HI;
          if (kmph <= stormLowerThreshold) STORM_COND_VALUE = STORM_COND_VALUE_LO;

          if (STORM_COND_VALUE != STORM_COND_VALUE_Last) {
            SensorEventMsg& rmsg = (SensorEventMsg&)device().message();
            DPRINT(F("PEER THRESHOLD DETECTED ")); DDECLN(STORM_COND_VALUE);
            rmsg.init(device().nextcount(), number(), evcnt++, STORM_COND_VALUE, false , false);
            device().sendPeerEvent(rmsg, *this);
          }
          STORM_COND_VALUE_Last = STORM_COND_VALUE;
        }
      }

      windspeed += kmph;
      _wind_isr_counter = 0;
    }

    void measure_winddirection() {
      //Windrichtung Grad/3: 60° = 20; 0° = Norden
      winddir = 0;
      uint8_t idxwdir = 0;
#ifdef NSENSORS
      idxwdir = random(15);
      winddir = (idxwdir * 15 + 2 / 2) / 2;
#else

#ifdef WINDDIRECTION_USE_PULSE
      uint8_t aVal = 0;
      uint8_t WINDDIR_TOLERANCE = 3;
      aVal = pulseIn(WINDDIRECTION_PIN, HIGH, 1000);
      DPRINT("AVAL = ");DDECLN(aVal);
#else
      uint16_t aVal = 0;
      for (uint8_t i = 0; i <= 0xf; i++) {
        aVal += analogRead(WINDDIRECTION_PIN);
      }
      aVal = aVal >> 4;

      uint8_t WINDDIR_TOLERANCE = 2;
      if ((aVal > 100) && (aVal < 250)) WINDDIR_TOLERANCE = 5;
      if (aVal >= 250) WINDDIR_TOLERANCE = 10;
#endif

      for (uint8_t i = 0; i < sizeof(WINDDIRS) / sizeof(uint16_t); i++) {
        if (aVal < WINDDIRS[i] + WINDDIR_TOLERANCE && aVal > WINDDIRS[i] - WINDDIR_TOLERANCE) {
          idxwdir = i;
          winddir = (idxwdir * 15 + 2 / 2) / 2;
          break;
        }
      }
      DPRINT(F("WINDDIR aVal  : ")); DDEC(aVal); DPRINT(F(" :: tolerance = ")); DDEC(WINDDIR_TOLERANCE); DPRINT(F(" :: i = ")); DDECLN(idxwdir);
#endif

      //Schwankungsbreite
      static uint8_t idxoldwdir = 0;
      winddirrange = 3; // 0  - 3 (0, 22,5, 45, 67,5°)
      int idxdiff = abs(idxwdir - idxoldwdir);

      if (idxdiff <= 3) winddirrange = idxdiff;
      if (idxwdir <= 2 && idxoldwdir >= 13) winddirrange = (sizeof(WINDDIRS) / sizeof(uint16_t)) - idxdiff;
      if (winddirrange > 3) winddirrange = 3;

      idxoldwdir = idxwdir;

      DPRINT(F("WINDDIR dir/3 : ")); DDECLN(winddir);
      DPRINT(F("WINDDIR range : ")); DDECLN(winddirrange);
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      tick = seconds2ticks(3);	// first message in 3 sec.
      sysclock.add(*this);
      sysclock.add(wind_measure);
    }

    void configChanged() {
      DPRINTLN("* Config changed       : List1");
      DPRINTLN(F("* ANEMOMETER           : "));
      DPRINT(F("*  - RADIUS            : ")); DDECLN(this->getList1().AnemometerRadius());
      DPRINT(F("*  - CALIBRATIONFACTOR : ")); DDECLN(this->getList1().AnemometerCalibrationFactor());
      DPRINT(F("*  - GUST MSG THRESHOLD: ")); DDECLN(this->getList1().ExtraMessageOnGustThreshold());
      DPRINT(F("PEERSETTING UPPER  = ")); DDECLN(this->getList1().StormUpperThreshold());
      stormUpperThreshold = this->getList1().StormUpperThreshold() * 10;
      DPRINT(F("PEERSETTING LOWER  = ")); DDECLN(this->getList1().StormLowerThreshold());
      stormLowerThreshold = this->getList1().StormLowerThreshold() * 10;
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class UType : public MultiChannelDevice<Hal, WeatherChannel, 1, UList0> {
  public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 1, UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINT(F("* LOW BAT Limit : ")); DDECLN(this->getList0().lowBatLimit());
      this->battery().low(this->getList0().lowBatLimit());
      DPRINT(F("* Sendeintervall: ")); DDECLN(this->getList0().Sendeintervall());
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
  pinMode(WINDDIRECTION_PIN, INPUT_PULLUP);
  windcounter.setMode(MODE_EVENT_COUNTER);
  windcounter.setCount(0);
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if (hal.battery.critical()) {
      hal.activity.sleepForever(hal);
    }
    hal.activity.savePower<Sleep<>>(hal);
  }
}

















