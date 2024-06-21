/*
 * That usermod implements support PCF8591 ADC/DAC module, DFPlayer mini MP3 player and PCA9685 PWM driver.
 *
 * 1. Attach PCF8591 module to i2c pins according to default pins for your board.
 * 2. Attach PCA9685 PWM Servo Driver module to i2c pins according to default pins for your board.

 * 2. Add "Adafruit BusIO @ ^1.16.1" , "Adafruit PWM Servo Driver Library @ ^3.0.2", "Adafruit PCF8591 @ ^1.0.4", "DFRobotDFPlayerMini @ ^1.0.6" dependency below to `lib_deps` like this:
 * lib_deps = lib_deps = adafruit/Adafruit BusIO @ ^1.16.1
 *   adafruit/Adafruit PWM Servo Driver Library @ ^3.0.2
 *   adafruit/Adafruit PCF8591 @ ^1.0.4
 *  dfrobot/DFRobotDFPlayerMini @ ^1.0.6
 *  ${esp32.lib_deps}
 */

#pragma once

#include <Wire.h>
#include "wled.h"

#include <Adafruit_PWMServoDriver.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_PCF8591.h>

#define RXD2 16
#define TXD2 17

#ifndef SENSOR_DELAY_MS
#define SENSOR_DELAY_MS 250
#endif

// for debug
#define MAX_I2C_DEVICES 127
byte deviceAddresses[MAX_I2C_DEVICES];
int numDevices = 0;
// end for debug

bool isI2C = false;
bool isSerial2 = false;
bool isPCF1 = false;
bool isPCF2 = false;
bool isPWM1 = false;
bool isPWM2 = false;
bool isDFPlayer = false;

#define PALETTE_SOLID_WRAP (strip.paletteBlend == 1 || strip.paletteBlend == 3)
#define PALETTE_MOVING_WRAP !(strip.paletteBlend == 2 || (strip.paletteBlend == 0 && SEGMENT.speed == 0))
/*
 * Color wipe function
 * LEDs are turned on (color1) in sequence, then turned off (color2) in sequence.
 * if (bool rev == true) then LEDs are turned off in reverse order
 */

uint16_t custom_color_wipe(bool rev)
{
  if (SEGENV.aux0 == 0)
  {                              // init
    SEGENV.aux1 = SEGMENT.speed; // store speed
    SEGENV.aux0 = 1;
  }
  else
  {
    SEGENV.aux0 += 1; // count frames
  }

  if (SEGENV.aux1 < 252 && SEGENV.aux0 % 20 == 0) // speed change every 100 frames
  {
    SEGENV.aux1 += 1;
  }

  uint32_t cycleTime = 100 + (255 - SEGMENT.aux1) * 150; //
  uint32_t perc = strip.now % cycleTime;                 //
  uint16_t prog = (perc * 65535) / cycleTime;            // 0 to 65535
  bool back = (prog > 32767);                            // 0 to 32767 to 0
  if (back)
  {
    prog -= 32767;
    if (SEGENV.step == 0)
      SEGENV.step = 1;
  }
  else
  {
    if (SEGENV.step == 2)
      SEGENV.step = 3; // trigger color change
  }

  uint16_t ledIndex = (prog * SEGLEN) >> 15; // 0 to SEGLEN
  uint16_t rem = 0;
  rem = (prog * SEGLEN) * 2; // mod 0xFFFF
  rem /= (SEGMENT.intensity + 1);
  if (rem > 255)
    rem = 255;

  uint32_t col1 = SEGCOLOR(1);

  SEGMENT.fadeToBlackBy(SEGMENT.intensity); // create fading trails
  for (int i = 0; i < SEGLEN; i++)
  {
    uint16_t index = (rev) ? SEGLEN - 1 - i : i;
    uint32_t col0 = SEGMENT.color_from_palette(index, true, PALETTE_SOLID_WRAP, 0);

    if (i < ledIndex)
    {
      SEGMENT.setPixelColor(index, back ? col1 : col0); // fill color
    }
    else
    {
      SEGMENT.setPixelColor(index, back ? col0 : col1); // clear color
      if (i == ledIndex)                                // fade in/out
        SEGMENT.setPixelColor(index, color_blend(back ? col0 : col1, back ? col1 : col0, rem));
    }
  }

  return FRAMETIME;
}

/*
 * color chase function.
 * color1 = background color
 * color2 and color3 = colors of two adjacent leds
 */
uint16_t custom_chase(uint32_t color1, uint32_t color2, uint32_t color3, bool do_palette)
{
  if (SEGENV.aux1 == 0)
  {
    SEGENV.aux1 = 1;
  }
  else
  {
    SEGENV.aux1 += 1;
  }

  uint16_t counter = strip.now * ((SEGMENT.speed >> 2) + 1) * 20;

  if (SEGENV.aux0 % 25 == 0)
  {
    uint16_t x = SEGENV.aux1 / 25;
    counter = strip.now * ((SEGMENT.speed >> 2) + 1) * x;
  }

  // uint16_t counter = strip.now * ((SEGMENT.speed >> 2) + 1) * 2; // increase speed, lower is faster (0 is fastest)
  uint16_t a = (counter * SEGLEN) >> 16; // position of the "dot"

  bool chase_random = (SEGMENT.mode == FX_MODE_CHASE_RANDOM);
  if (chase_random)
  {
    if (a < SEGENV.step) // we hit the start again, choose new color for Chase random
    {
      SEGENV.aux1 = SEGENV.aux0; // store previous random color
      SEGENV.aux0 = get_random_wheel_index(SEGENV.aux0);
    }
    color1 = SEGMENT.color_wheel(SEGENV.aux0);
  }
  SEGENV.step = a;

  // Use intensity setting to vary chase up to 1/2 string length
  uint8_t size = 1 + ((SEGMENT.intensity * SEGLEN) >> 10);

  uint16_t b = a + size; //"trail" of chase, filled with color1
  if (b > SEGLEN)
    b -= SEGLEN;
  uint16_t c = b + size;
  if (c > SEGLEN)
    c -= SEGLEN;

  // background
  if (do_palette)
  {
    for (int i = 0; i < SEGLEN; i++)
    {
      SEGMENT.setPixelColor(i, SEGMENT.color_from_palette(i, true, PALETTE_SOLID_WRAP, 1));
    }
  }
  else
    SEGMENT.fill(color1);

  // if random, fill old background between a and end
  if (chase_random)
  {
    color1 = SEGMENT.color_wheel(SEGENV.aux1);
    for (int i = a; i < SEGLEN; i++)
      SEGMENT.setPixelColor(i, color1);
  }

  // fill between points a and b with color2
  if (a < b)
  {
    for (int i = a; i < b; i++)
      SEGMENT.setPixelColor(i, color2);
  }
  else
  {
    for (int i = a; i < SEGLEN; i++) // fill until end
      SEGMENT.setPixelColor(i, color2);
    for (int i = 0; i < b; i++) // fill from start until b
      SEGMENT.setPixelColor(i, color2);
  }

  // fill between points b and c with color2
  if (b < c)
  {
    for (int i = b; i < c; i++)
      SEGMENT.setPixelColor(i, color3);
  }
  else
  {
    for (int i = b; i < SEGLEN; i++) // fill until end
      SEGMENT.setPixelColor(i, color3);
    for (int i = 0; i < c; i++) // fill from start until c
      SEGMENT.setPixelColor(i, color3);
  }

  return FRAMETIME;
}

uint16_t mode_custom_chase_color(void)
{
  return custom_chase(SEGCOLOR(1), (SEGCOLOR(2)) ? SEGCOLOR(2) : SEGCOLOR(0), SEGCOLOR(0), true);
}
static const char _data_FX_MODE_CUSTOM_CHASE_COLOR[] PROGMEM = "Turbine2@!,Width;!,!,!;!";

uint16_t mode_custom_color_wipe(void)
{
  return custom_color_wipe(false);
}

uint16_t mode_custom_color_wipe_reverse(void)
{
  return custom_color_wipe(true);
}

static const char _data_FX_MODE_CUSTOM_COLOR_WIPE[] PROGMEM = "Turbine@!,!;!,!;!";
static const char _data_FX_MODE_CUSTOM_COLOR_WIPE_REV[] PROGMEM = "Turbine_reverse@!,!;!,!;!";

uint8_t readPotentiometer(Adafruit_PCF8591 pcf, uint8_t pin, bool reverse = false)
{
  if (!isPCF1 || !isPCF2)
  {
    return 0;
  }
  if (reverse)
  {
    return 255 - pcf.analogRead(pin);
  }
  else
  {
    return pcf.analogRead(pin);
  }
}

const uint8_t switchThreshold = 128;

bool readSwitch(Adafruit_PCF8591 pcf, uint8_t pin, bool reverse = false)
{
  if (!isPCF1 || !isPCF2)
  {
    return 0;
  }
  bool switchState = false;
  if (pcf.analogRead(pin) >= switchThreshold)
  {
    switchState = true;
  }
  else
  {
    switchState = false;
  }
  if (reverse)
  {
    return !switchState;
  }
  else
  {
    return switchState;
  }
}

class UsermodReactor : public Usermod
{
private:
  bool enabled = false;
  unsigned long lastTime = 0;

  bool switch1 = false;
  bool switch2 = false;
  bool switch3 = false;
  bool switch4 = false;

  uint8_t potentiometer1Value = 0;
  uint8_t potentiometer2Value = 0;
  uint8_t potentiometer3Value = 0;
  uint8_t potentiometer4Value = 0;
  uint8_t potentiometerThreshold = 5;

  uint8_t lastVolume = 0;

  bool phase1Started = false;
  bool phase1Established = false;
  bool phase1Loop = false;
  unsigned long phase1Time = 0;
  uint16_t phase1StartTime = 2500;
  uint16_t phase1EstablishedTime = 60000;

  uint16_t shortTransition = 250;
  uint16_t defalutTransition = 750;
  uint16_t phase1StartTransition = 2000;
  uint16_t phase1FinishTransition = 6000;

  bool phase2Started = false;
  bool phase2Peak = false;
  bool phase2Flash = false;
  bool phase2Established = false;
  bool phase2Loop = false;
  unsigned long phase2Time = 0;
  uint16_t phase2PeakTime = 5500;
  uint16_t phase2FlashTime = 8000;
  uint16_t phase2StartTime = 15000;
  uint16_t phase2EstablishedTime = 60000;

  uint16_t phase2StartTransition = 500;
  uint16_t phase2MiddleTransition = 2000;
  uint16_t phase2FinishTransition = 4000;

  uint16_t phase3MiddleTransition = 2000;

  bool phase3Started = false;

  bool phase4Started = false;

  // // for debug
  // TwoWire wire = new TwoWire(1);
  // wire.setClock(400000);
  // // The SHT lib calls wire.begin() again without the SDA and SCL pins... So call it again here...
  // wire.begin(21, 22);
  // // end for debug

  Adafruit_PCF8591 pcf1 = Adafruit_PCF8591();
  Adafruit_PCF8591 pcf2 = Adafruit_PCF8591();
  Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
  Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
  DFRobotDFPlayerMini myDFPlayer;

  int fadeAmount = 100;
  bool currentEffectAndPaletteInitialized = false;
  uint8_t effectCurrentIndex = 0;
  uint8_t effectPaletteIndex = 0;
  byte *modes_alpha_indexes = nullptr;
  byte *palettes_alpha_indexes = nullptr;

public:
  void setup()
  {
    if (i2c_scl < 0 || i2c_sda < 0) // ESP32DEV GPIO21 - SDA GPIO22 - SCL
    {
      enabled = false;
      isI2C = false;
      return;
    }
    else
    {
      isI2C = true;
    }

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Hardware Serial of ESP32
    isSerial2 = true;

    if (!myDFPlayer.begin(Serial2, false /*isACK*/, true /*reset*/))
    {
      isDFPlayer = false;
    }
    else
    {
      isDFPlayer = true;
      // myDFPlayer.setTimeOut(500);
      // myDFPlayer.volume(20); // Set volume value. From 0 to 30
      // myDFPlayer.play(1);
      // myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
      myDFPlayer.EQ(DFPLAYER_EQ_BASS);
    }

    pwm1.begin();
    isPWM1 = true;
    pwm2.begin();
    isPWM2 = true;
    // pwm1.setOscillatorFrequency(27000000);
    pwm1.setPWMFreq(1600);

    // pwm2.setOscillatorFrequency(27000000);
    pwm2.setPWMFreq(1600);

    // https://kno.wled.ge/advanced/custom-features/#create-a-custom-effect-as-usermod
    strip.addEffect(255, &mode_custom_color_wipe, _data_FX_MODE_CUSTOM_COLOR_WIPE);
    strip.addEffect(254, &mode_custom_color_wipe_reverse, _data_FX_MODE_CUSTOM_COLOR_WIPE_REV);
    strip.addEffect(253, &mode_custom_chase_color, _data_FX_MODE_CUSTOM_CHASE_COLOR);

    for (uint8_t pwmnum = 0; pwmnum < 16; pwmnum++) // black all PWMs
    {
      pwm1.setPWM(pwmnum, 0, 4096);
      pwm2.setPWM(pwmnum, 0, 4096);
      // pwm1.setPWM(pwmnum, 4096, 0);
      // pwm2.setPWM(pwmnum, 4096, 0);
    }

    if (!pcf1.begin(0x48))
    {
      isPCF1 = false;
    }
    else
    {
      isPCF1 = true;
      switch1 = readSwitch(pcf1, 0);
      switch2 = readSwitch(pcf1, 1, true);
      switch3 = readSwitch(pcf1, 2);
      switch4 = readSwitch(pcf1, 3, false);
    }

    if (!pcf2.begin(0x4C))
    {
      isPCF2 = false;
    }
    else
    {
      isPCF2 = true;
      potentiometer1Value = readPotentiometer(pcf1, 0, true);
      potentiometer2Value = readPotentiometer(pcf1, 1, true);
      potentiometer3Value = readPotentiometer(pcf1, 2, true);
      potentiometer4Value = readPotentiometer(pcf1, 3, false);
    }

    if (isPCF1 && isPCF2)
    {
      enabled = true;
    }
  }

  void loop()
  {
    if (enabled)
    {
      if (millis() - lastTime > SENSOR_DELAY_MS)
      {
        switch1 = readSwitch(pcf1, 0);
        switch2 = readSwitch(pcf1, 1, false);
        switch3 = readSwitch(pcf1, 2);
        switch4 = readSwitch(pcf1, 3, true);
        potentiometer1Value = readPotentiometer(pcf2, 0, true);
        potentiometer2Value = readPotentiometer(pcf2, 1, true);
        potentiometer3Value = readPotentiometer(pcf2, 2, true);
        potentiometer4Value = readPotentiometer(pcf2, 3, false);
        if (enabled && !strip.isUpdating())
        {
          updateGlobalBrightness(potentiometer1Value);
          updatePalette(potentiometer3Value);
          updateVolume(potentiometer2Value);
          // upatePresets(potentiometer4Value);
          checkPhase1();
          checkPhase2();
          checkPhase3();
          checkPhase4();
        }
        lastTime = millis();
      }
    }

    if (!enabled || strip.isUpdating())
      return;

    reflectPixelsOverPWM();
  }

  uint16_t getOffTime(uint32_t pixelColor)
  {
    // int w = ((pixelColor >> 24) & 0xff) * bri / 255.0;
    // int r = ((pixelColor >> 16) & 0xff) * bri / 255.0;
    int g = ((pixelColor >> 8) & 0xff) * bri / 255.0;
    // int b = (pixelColor & 0xff) * bri / 255.0;
    uint16_t offTime = map(g, 0, 255, 0, 4095);
    return offTime;
  }

  uint16_t getOnTime(uint32_t pixelColor, uint16_t offTime)
  {
    uint16_t onTime = 4095 - offTime;
    return onTime;
  }

  // leds location on PWM boards
  // PWM1: 0-8 bottom turbine, virtual segment 3 (8 pixels), bottom turbine
  // PWM2: 0-8 top turbine, virtual segment 4 (8 pixels), top turbine
  // PWM1: 8-12 middle ring. virtual segment 5 (4 pixels)
  // PWM1: 12-16 bottom preheated ring, virtual segment 2 (8 pixels), 0-4 bottom preheated ring
  // PWM2: 12-16 top preheated ring, virtual segment 2 (8 pixels), 4-8 top preheated ring

  void reflectPixelsOverPWM()
  {
    if (strip.getSegmentsNum() >= 2)
    {
      Segment &seg2 = strip.getSegment(2);
      for (uint8_t pixelNum = 0; pixelNum < 4; pixelNum++)
      {
        uint32_t pixelColor = seg2.getPixelColor(pixelNum);
        uint16_t offTime = getOffTime(pixelColor);
        uint16_t onTime = getOnTime(pixelColor, offTime);
        pwm1.setPWM(pixelNum + 12, onTime, offTime);
      }
      for (uint8_t pixelNum = 4; pixelNum < 8; pixelNum++)
      {
        uint32_t pixelColor = seg2.getPixelColor(pixelNum);
        uint16_t offTime = getOffTime(pixelColor);
        uint16_t onTime = getOnTime(pixelColor, offTime);
        pwm2.setPWM(pixelNum + 12 - 4, onTime, offTime);
      }
    }
    if (strip.getSegmentsNum() >= 3)
    {
      Segment &seg3 = strip.getSegment(3);
      for (uint8_t pixelNum = 0; pixelNum < 8; pixelNum++)
      {
        uint32_t pixelColor = seg3.getPixelColor(pixelNum);
        uint16_t offTime = getOffTime(pixelColor);
        uint16_t onTime = getOnTime(pixelColor, offTime);
        pwm1.setPWM(pixelNum, onTime, offTime);
      }
    }

    if (strip.getSegmentsNum() >= 4)
    {
      Segment &seg4 = strip.getSegment(4);
      for (uint8_t pixelNum = 0; pixelNum < 8; pixelNum++)
      {
        uint32_t pixelColor = seg4.getPixelColor(pixelNum);
        uint16_t offTime = getOffTime(pixelColor);
        uint16_t onTime = getOnTime(pixelColor, offTime);
        pwm2.setPWM(pixelNum, onTime, offTime);
      }
    }

    if (strip.getSegmentsNum() >= 5)
    {
      Segment &seg5 = strip.getSegment(5);
      for (uint8_t pixelNum = 0; pixelNum < 4; pixelNum++)
      {
        uint32_t pixelColor = seg5.getPixelColor(pixelNum);
        uint16_t offTime = getOffTime(pixelColor);
        uint16_t onTime = getOnTime(pixelColor, offTime);
        pwm1.setPWM(pixelNum + 8, onTime, offTime);
      }
    }
  }

  void updateGlobalBrightness(uint8_t potentiometerValue = 0)
  {
    if (abs(bri - potentiometerValue) > potentiometerThreshold)
    {
      bri = potentiometerValue;
      lampUdated();
    }
  }

  void updatePalette(uint8_t potentiometerValue = 0)
  {
    if (!switch3) // switch3 is Off
    {
      return;
    }
    if (strip.getSegmentsNum() < 2)
    {
      return;
    }
    if (strip.getPaletteCount() == 0)
    {
      return;
    }
    if (potentiometerValue >= 240)
    {
      return; // no change to palette if potentiometerValue in the highest position
    }
    uint8_t value = map(potentiometerValue, 0, 240, 0, strip.getPaletteCount() + strip.customPalettes.size() - 1); // 0-240 to 0-strip.getPaletteCount()
    Segment &seg = strip.getSegment(1);
    if (seg.isActive() && seg.isSelected())
    {
      colorUpdated(CALL_MODE_FX_CHANGED);
      seg.setPalette(value);
    }
  }

  void updateSegmentBrightness(uint8_t segNum, uint8_t potentiometer)
  {
    if (segNum >= strip.getSegmentsNum())
    {
      return;
    }
    Segment &seg = strip.getSegment(segNum);
    if (abs(seg.currentBri() - potentiometer) > potentiometerThreshold)
    {
      seg.setOpacity(potentiometer);
      lampUdated();
    }
  }

  void updateVolume(uint8_t potentiometerValue = 0)
  {
    if (!isDFPlayer)
    {
      return;
    }
    int volume = map(potentiometerValue, 0, 255, 0, 30);
    if (lastVolume != volume)
    {
      lastVolume = volume;
      myDFPlayer.volume(volume);
    }
  }

  void updatePresets(uint8_t potentiometerValue = 0)
  {
    uint8_t value = map(potentiometerValue, 0, 255, 10, 20);
    applyPreset(value);
  }

  // segment 0 - optical fiber 8 addressable leds
  // segment 1 - addressable LED strip, 54 leds
  // segment 2 - 8x PWM LED (top and bottom preheated ring)
  // segment 3 - 8x PWM LED (top turbine)
  // segment 4 - 8x PWM LED (bottom turbine)
  // segment 5 - 4x PWM LED (middle ring)

  void checkPhase1()
  {
    if (switch1 && !phase1Started) // switch1 is On
    {
      strip.setTransition(phase1StartTransition);
      phase1Started = true;
      phase1Loop = false;
      phase1Time = millis();
      // myDFPlayer.disableLoop();
      myDFPlayer.playMp3Folder(11);
      // if (currentPreset != 1)
      applyPreset(1);
      strip.getSegment(0).setOption(SEG_OPTION_ON, true);
      strip.getSegment(2).setOption(SEG_OPTION_ON, true);
      stateChanged = true;
      lampUdated();
      return;
    }

    if (switch1 && phase1Started && !phase1Established) // switch1 is On and phase1Started
    {
      if (millis() - phase1Time > phase1StartTime) // phase1StartTime reached
      {
        phase1Established = true;
        strip.setTransition(defalutTransition);
        applyPreset(2);
        stateChanged = true;
        lampUdated();
        if (switch3 && phase3Started) // switch3 is On and phase3Started. Set segment 1 opacity to 255
        {
          if (strip.getSegmentsNum() < 2)
          {
            return;
          }
          Segment &seg = strip.getSegment(1);
          seg.setOpacity(255);
          stateChanged = true;
          lampUdated();
        }
        return;
      }
    }

    if (switch1 && !switch2 && phase1Established && !phase1Loop && millis() - phase1Time > phase1EstablishedTime) // phase1EstablishedTime reached
    {
      phase1Loop = true;
      myDFPlayer.loopFolder(01);
    }

    if (!switch1 && phase1Started) // switch1 is Off
    {
      phase1Started = false;
      phase1Established = false;
      phase1Established = false;
      phase1Loop = false;
      myDFPlayer.playMp3Folder(12);
      strip.setTransition(phase1FinishTransition); // fade out slowly
      strip.getSegment(0).setOption(SEG_OPTION_ON, false);
      strip.getSegment(2).setOption(SEG_OPTION_ON, false);
      stateChanged = true;
      lampUdated();
      return;
    }
  }

  void checkPhase2()
  {
    if (switch2 && !phase2Started) // switch2 is On
    {
      phase2Started = true;
      phase2Loop = false;
      phase2Time = millis();
      strip.setTransition(phase2StartTransition);
      applyPreset(3);
      myDFPlayer.playMp3Folder(21);
      return;
    }

    if (switch2 && phase2Started && !phase2Peak) // switch2 is On and phase2Started
    {
      if (millis() - phase2Time > phase2PeakTime)
      {
        phase2Peak = true;
        strip.setTransition(shortTransition);
        applyPreset(4);
        stateChanged = true;
        lampUdated();
        return;
      }
    }

    if (switch2 && phase2Started && phase2Peak && !phase2Flash) // switch2 is On and phase2Started and phase2Peak
    {
      if (millis() - phase2Time > phase2FlashTime)
      {
        phase2Flash = true;
        strip.setTransition(shortTransition);
        applyPreset(5);
        stateChanged = true;
        lampUdated();
        return;
      }
    }

    if (switch2 && phase2Started && phase2Peak && phase2Flash && !phase2Established) // switch2 is On and phase2Started and phase2Peak and phase2Flash
    {
      if (millis() - phase2Time > phase2StartTime)
      {
        phase2Established = true;
        strip.setTransition(shortTransition);
        applyPreset(6);
        if (switch3 && phase3Started) // switch3 is On and phase3Started. Set segment 1 opacity to 255
        {
          if (strip.getSegmentsNum() < 2)
          {
            return;
          }
          Segment &seg = strip.getSegment(1);
          seg.setOpacity(255);
        }
        stateChanged = true;
        lampUdated();
        return;
      }
    }

    if (switch2 && phase2Established && !phase2Loop && millis() - phase2Time > phase2EstablishedTime) // phase2EstablishedTime reached
    {
      phase2Loop = true;
      myDFPlayer.loopFolder(02);
    }

    if (!switch2 && phase2Started) // switch2 is Off
    {
      phase2Started = false;
      phase2Peak = false;
      phase2Established = false;
      phase2Loop = false;
      if (switch1) // switch2 is Off and switch1 is On
      {
        phase1Loop = true;
        myDFPlayer.loopFolder(01);
        strip.setTransition(phase2FinishTransition);
        applyPreset(2);
        if (switch3 && phase3Started) // switch3 is On and phase3Started. Set segment 1 opacity to 255
        {
          if (strip.getSegmentsNum() < 2)
          {
            return;
          }
          Segment &seg = strip.getSegment(1);
          seg.setOpacity(255);
        }
        stateChanged = true;
        lampUdated();
      }
      else // switch2 is Off and switch1 is Off
      {
        strip.setTransition(phase2FinishTransition);
        strip.getSegment(0).setOption(SEG_OPTION_ON, false);
        if (switch3 && phase3Started) // switch3 is On and phase3Started. Set segment 1 opacity to 255
        {
          if (strip.getSegmentsNum() < 2)
          {
            return;
          }
          Segment &seg = strip.getSegment(1);
          seg.setOpacity(255);
        }
        else
        {
          strip.getSegment(1).setOption(SEG_OPTION_ON, false);
        }
        strip.getSegment(2).setOption(SEG_OPTION_ON, false);
        strip.getSegment(3).setOption(SEG_OPTION_ON, false);
        strip.getSegment(4).setOption(SEG_OPTION_ON, false);
        strip.getSegment(5).setOption(SEG_OPTION_ON, false);
        // applyPreset(100);
        stateChanged = true;
        lampUdated();
        myDFPlayer.playMp3Folder(22);
        return;
      }
    }
  }

  void checkPhase3()
  {
    if (switch3 && !phase3Started) // switch3 is On
    {
      phase3Started = true;
      strip.setTransition(phase3MiddleTransition);
      if (strip.getSegmentsNum() < 2)
      {
        return;
      }
      Segment &seg = strip.getSegment(1);
      seg.setOpacity(255);
      lampUdated();
      if (!switch1 && !switch2)
      {
        myDFPlayer.loopFolder(03); // loop folder 03
        // myDFPlayer.loopFolder(02); // loop folder 03
      }
      return;
    }

    if (!switch3 && phase3Started) // switch3 is Off
    {
      phase3Started = false;
      strip.setTransition(phase3MiddleTransition);
      if (strip.getSegmentsNum() < 2)
      {
        return;
      }
      Segment &seg = strip.getSegment(1);
      seg.setOpacity(0);
      lampUdated();
      if (!switch1 && !switch2)
      {
        myDFPlayer.playMp3Folder(32); // play out sound
        // myDFPlayer.playMp3Folder(22); // play out sound
      }
      return;
    }
  }

  void checkPhase4()
  {
    if (switch4 && !phase4Started) // switch4 is On
    {
      phase4Started = true;

      return;
    }

    if (!switch4 && phase4Started) // switch4 is Off
    {
      phase4Started = false;

      return;
    }
  }

  void changePalette(bool increase)
  {
    if (increase)
    {
      effectPalette = (effectPalette + 1 >= strip.getPaletteCount()) ? 0 : (effectPalette + 1);
    }
    else
    {
      effectPalette = (effectPalette - 1 < 0) ? (strip.getPaletteCount() - 1) : (effectPalette - 1);
    }

    lampUdated();
  }

  void changeBrightness(bool increase)
  {
    if (increase)
    {
      bri = (bri + fadeAmount <= 255) ? (bri + fadeAmount) : 255;
    }
    else
    {
      bri = (bri - fadeAmount >= 0) ? (bri - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffect(bool increase)
  {
    if (increase)
    {
      effectCurrent = (effectCurrent + 1 >= strip.getModeCount()) ? 0 : (effectCurrent + 1);
    }
    else
    {
      effectCurrent = (effectCurrent - 1 < 0) ? (strip.getModeCount() - 1) : (effectCurrent - 1);
    }
    lampUdated();
  }

  void changePreset(bool increase)
  {

    if (increase)
    {
      presetCycCurr = (presetCycCurr + 1 >= 250) ? 0 : (presetCycCurr + 1);
    }
    else
    {
      presetCycCurr = (presetCycCurr - 1 < 0) ? 250 : (presetCycCurr - 1);
    }
    applyPreset(presetCycCurr);
  }

  void lampUdated()
  {
    colorUpdated(CALL_MODE_BUTTON);
    updateInterfaces(CALL_MODE_BUTTON);
  }

  /*
   * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
   * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
   * Below it is shown how this could be used for e.g. a light sensor
   */
  void addToJsonInfo(JsonObject &root)
  {
    // if "u" object does not exist yet wee need to create it

    if (!enabled)
      return; // prevent crash on boot applyPreset()
    JsonObject user = root["u"];
    if (user.isNull())
      user = root.createNestedObject("u");

    JsonArray infoI2C = user.createNestedArray(F("isI2C"));
    infoI2C.add(isI2C);

    JsonArray infoSerial2 = user.createNestedArray(F("isSerial2"));
    infoSerial2.add(isSerial2);

    JsonArray infoPCF1 = user.createNestedArray(F("isPCF1"));
    infoPCF1.add(isPCF1);

    JsonArray infoPCF2 = user.createNestedArray(F("isPCF2"));
    infoPCF2.add(isPCF2);

    JsonArray infoPWM1 = user.createNestedArray(F("isPWM1"));
    infoPWM1.add(isPWM1);

    JsonArray infoPWM2 = user.createNestedArray(F("isPWM2"));
    infoPWM2.add(isPWM2);

    JsonArray infoDFPlayer = user.createNestedArray(F("isDFPlayer"));
    infoDFPlayer.add(isDFPlayer);

    JsonArray _time = user.createNestedArray(F("lastTime"));
    _time.add(lastTime);

    JsonArray infoSwitch1 = user.createNestedArray(F("switch1"));
    infoSwitch1.add(switch1);

    JsonArray infoSwitch2 = user.createNestedArray(F("switch2"));
    infoSwitch2.add(switch2);

    JsonArray infoSwitch3 = user.createNestedArray(F("switch3"));
    infoSwitch3.add(switch3);

    JsonArray infoSwitch4 = user.createNestedArray(F("switch4"));
    infoSwitch4.add(switch4);

    JsonArray infoPotentiometer1 = user.createNestedArray(F("potentiometer1Value"));
    infoPotentiometer1.add(potentiometer1Value);

    JsonArray infoPotentiometer2 = user.createNestedArray(F("potentiometer2Value"));
    infoPotentiometer2.add(potentiometer2Value);

    JsonArray infoPotentiometer3 = user.createNestedArray(F("potentiometer3Value"));
    infoPotentiometer3.add(potentiometer3Value);

    JsonArray infoPotentiometer4 = user.createNestedArray(F("potentiometer4Value"));
    infoPotentiometer4.add(potentiometer4Value);
  }

  /*
   * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
   * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
   * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
   */
  //    void addToConfig(JsonObject& root)
  //    {
  //      JsonObject top = root.createNestedObject("VL53L0x");
  //      JsonArray pins = top.createNestedArray("pin");
  //      pins.add(i2c_scl);
  //      pins.add(i2c_sda);
  //    }

  /*
   * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
   * This could be used in the future for the system to determine whether your usermod is installed.
   */
  uint16_t getId()
  {
    return USERMOD_ID_EXAMPLE;
  }
};