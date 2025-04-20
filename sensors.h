#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "SparkFun_ENS160.h"

class ENS160
{
public:
    ENS160();
    bool init(uint8_t address);
    void update();
    uint8_t getAQI();
    uint16_t getTVOC();
    uint16_t getECO2();
    void getDebugInfo();

private:
    uint8_t _address;
    SparkFun_ENS160 sensor;
    bool _initialized;
    uint8_t _status;
    uint8_t _AQI;
    uint16_t _TVOC;
    uint16_t _ECO2;
    static String getStatusName(uint8_t ensStatus);
};

class MT6701
{
public:
    MT6701();
    bool init(uint8_t address);
    void update();
    uint16_t getAngle();
    String getDirection();
    void getDebugInfo();

private:
    bool _initialized;
    uint8_t _address;
    uint16_t _angle;
    String _direction;
    String getDirectionName(float angle);
};

class TEMT6000
{
public:
    TEMT6000();
    bool init(uint8_t pin);
    uint16_t getLight();
    void getDebugInfo();

private:
    bool _initialized;
};

class BME280
{
public:
    BME280();
    bool init(uint8_t address);
    void update();
    float getTemperature();
    float getHumidity();
    float getPressure();
    void getDebugInfo();

private:
    Adafruit_BME280 bme;
    float _temperature;
    float _humidity;
    float _pressure;
    bool _initialized;
};

class KY003
{
public:
    KY003();
    void init(uint8_t pin);
    float getSpeed() const;
    float getRotationFrequency() const;
    unsigned int getPulseCount() const;
    void resetPulseCount();
    void getDebugInfo();

private:
    uint8_t _pin;
    volatile unsigned long _lastPulseTime;
    volatile unsigned long _pulseInterval;
    volatile unsigned int _pulseCount;
    unsigned long _lastMeasurementTime;
    static const unsigned long DEBOUNCE_TIME;
    static KY003* _instance;
    void handleInterruptInstance();
    static void handleInterrupt();
};

class RainSensor
{
public:
    RainSensor(uint8_t pin);
    uint16_t getRainLevel();
    void getDebugInfo();

private:
    uint8_t _pin;
};

#endif // SENSORS_H