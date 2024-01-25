
#ifndef USERMOD_HALL_SENSOR_PIN
  #ifdef ARDUINO_ARCH_ESP32
    #define USERMOD_HALL_SENSOR_PIN 35
  #else //ESP8266 boards
    #define USERMOD_HALL_SENSOR_PIN A0
  #endif
#endif


#ifndef USERMOD_HALL_SENSOR_MEASUREMENT_INTERVAL
  #define USERMOD_HALL_SENSOR_MEASUREMENT_INTERVAL 50
#endif

#ifndef USERMOD_HALL_MAX_VALUE
  #define USERMOD_HALL_MAX_VALUE 777
#endif

#ifndef USERMOD_HALL_MIN_VALUE
  #define USERMOD_HALL_MIN_VALUE 300
#endif

#ifndef USERMOD_HALL_TRESHOLD
  #define USERMOD_HALL_TRESHOLD 5
#endif

