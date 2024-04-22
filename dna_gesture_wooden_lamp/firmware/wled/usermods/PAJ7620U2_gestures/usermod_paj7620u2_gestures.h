/*
 * That usermod implements support of hand gestures with PAJ7620 sensor: on/off, brightness correction, color palette and presets change.
 *
 * Enabling this usermod:
 * 1. Attach PAJ7620 sensor to i2c pins according to default pins for your board.
 * 2. Add "acrandal/RevEng PAJ7620@^1.5.0" dependency below to `lib_deps` like this:
 * lib_deps = ${env.lib_deps}
 *     acrandal/RevEng PAJ7620@^1.5.0
 */

#pragma once

#include "wled.h"

// #include <Wire.h>
#include "RevEng_PAJ7620.h"

#ifndef GES_REACTION_TIME
#define GES_REACTION_TIME 800
#endif

#ifndef GES_QUIT_TIME
#define GES_QUIT_TIME 1000
#endif

#ifndef PAJ7620U2_DELAY_MS
#define PAJ7620U2_DELAY_MS 100 // how often to get data from sensor
#endif

class UsermodPAJ7620U2Gestures : public Usermod
{
private:
  RevEng_PAJ7620 sensor = RevEng_PAJ7620();
  bool enabled = false;
  bool initDone = true;
  unsigned long lastTime = 0;
  int gest_val = -4;

  int fadeAmount = 100;
  bool currentEffectAndPaletteInitialized = false;
  uint8_t effectCurrentIndex = 0;
  uint8_t effectPaletteIndex = 0;
  byte *modes_alpha_indexes = nullptr;
  byte *palettes_alpha_indexes = nullptr;

public:
  void setup()
  {
    if (i2c_scl < 0 || i2c_sda < 0) // ESP8266 GPIO4 - SDA GPIO5 - SCL
    {
      enabled = false;
      gest_val = -3;
      return;
    }

    if (!sensor.begin())
    {
      gest_val = -2;
      enabled = false;
      DEBUG_PRINTLN(F("PAJ7620 initialization failed"));
    }
    else
    {
      gest_val = -1;
      enabled = true;
    }
    initDone = true;
  }

  void loop()
  {
    if (!enabled || strip.isUpdating())
      return;

    if (millis() - lastTime > PAJ7620U2_DELAY_MS)
    {
      lastTime = millis();

      Gesture gesture;                // Gesture is an enum type from RevEng_PAJ7620.h
      gesture = sensor.readGesture(); // Read back current gesture (if any) of type Gesture

      switch (gesture)
      {
      case GES_FORWARD:
      {
        // changeBrightness(false);
        break;
      }

      case GES_BACKWARD:
      {

        // changeBrightness(true);
        break;
      }

      case GES_LEFT:
      {
        // changeEffect(false);
        changePreset(false);
        break;
      }

      case GES_RIGHT:
      {
        // changeEffect(true);
        changePreset(true);
        break;
      }

      case GES_UP:
      {
        changePalette(true);
        break;
      }

      case GES_DOWN:
      {
        changePalette(false);
        break;
      }

      case GES_CLOCKWISE:
      {
        changeBrightness(true);
        break;
      }

      case GES_ANTICLOCKWISE:
      {
        changeBrightness(false);
        break;
      }

      case GES_WAVE:
      {

        break;
      }

      case GES_NONE:
      {

        break;
      }
      }

      if (gesture != GES_NONE)
      {
        gest_val = gesture;
      }
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

    if (!initDone || !enabled)
      return; // prevent crash on boot applyPreset()
    JsonObject user = root["u"];
    if (user.isNull())
      user = root.createNestedObject("u");

    JsonArray info = user.createNestedArray(F("Gesture Sensor"));
    info.add(gest_val);

    JsonArray _time = user.createNestedArray(F("lastTime"));
    _time.add(lastTime);

    // JsonArray _mode = user.createNestedArray(F("Mode"));
    // _mode.add(strip.getModeCount());
    // _mode.add(effectCurrent);

    // JsonArray _pal = user.createNestedArray(F("Pal"));
    // _pal.add(strip.getPaletteCount());
    // _pal.add(strip.getSegment(0).palette);

    JsonArray _pres = user.createNestedArray(F("Pres"));
    _pres.add(presetCycCurr);
    _pres.add(F("-"));
    _pres.add(presetCycMin);
    _pres.add(F("-"));
    _pres.add(presetCycMax);
    // String name = "";
    // if (!getPresetName(10, name))
    //   _pres.add(10);
    // _pres.add(F("-"));
    // if (!getPresetName(1, name))
    //   _pres.add(1);
    // _pres.add(F("="));
    // _pres.add(getPresetName(1, name));

    // getPresetName()
    // applyPreset(1);
    // applyPresetWithFallback(1, 1);

    // JsonArray cur_bri = user.createNestedArray(F("Current Bri"));
    // cur_bri.add(bri);

    // float step = (maxValue - minValue) / 255;
    // float diff = hallSensorValue - minValue;
    // float value = diff / step;

    // if (value > 255)
    // {
    //   value = 255;
    // };
    // if (value < 0)
    // {
    //   value = 0;
    // };

    // JsonArray cur_min = user.createNestedArray(F("Current Min"));
    // cur_min.add(minValue);

    // JsonArray cur_max = user.createNestedArray(F("Current Max"));
    // cur_max.add(maxValue);

    // JsonArray cur_step = user.createNestedArray(F("Current Step"));
    // cur_step.add(step);

    // JsonArray cur_diff = user.createNestedArray(F("Current diff"));
    // cur_diff.add(diff);

    // JsonArray cur_val = user.createNestedArray(F("Current Val"));
    // cur_val.add(static_cast<int>(value));
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
    return USERMOD_ID_EXAMPLE; // USERMOD_ID_PAJ7620U2;
  }
};