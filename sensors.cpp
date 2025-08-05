#include "SparkFun_ENS160.h"
#include <Adafruit_BME280.h>
#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include "config.h"
#include "sensors.h"

// ENS160 method implementations
ENS160::ENS160() : _initialized(false) {}

bool ENS160::init(uint8_t address)
{
    _address = address;

    if (!sensor.begin(_address))
    {
        if (DEBUG)
            Serial.println("Could not communicate with the ENS160, check wiring.");
        return false;
    }

    sensor.setOperatingMode(SFE_ENS160_RESET);
    delay(100);
    sensor.setOperatingMode(SFE_ENS160_STANDARD);
    _initialized = true;

    if (DEBUG)
    {
        _status = sensor.getFlags();
        Serial.print("ENS 160: ready, status - ");
        Serial.println(getStatusName(_status));
    }
    return true;
}

String ENS160::getStatusName(uint8_t ensStatus)
{
    switch (ensStatus)
    {
    case 0:
        return "Standard";
    case 1:
        return "Warm up";
    case 2:
        return "Initial Start Up";
    default:
        return "Unknown";
    }
}

void ENS160::update()
{
    if (!_initialized || !sensor.checkDataStatus())
        return;
    _AQI = sensor.getAQI();
    _TVOC = sensor.getTVOC();
    _ECO2 = sensor.getECO2();
}

uint8_t ENS160::getAQI() { return _AQI; }
String ENS160::getAQIName()
{
    switch (_AQI)
    {
    case 1:
        return "OK";
    default:
        return "BAD";
    }
}
uint16_t ENS160::getTVOC() { return _TVOC; }
uint16_t ENS160::getECO2() { return _ECO2; }

void ENS160::getDebugInfo()
{
    if (!_initialized)
        return;

    _status = sensor.getFlags();
    Serial.println("=== Качество воздуха ===");
    Serial.print("Индекс качества воздуха (1-5): ");
    Serial.println(_AQI);
    Serial.print("TVOC: ");
    Serial.print(_TVOC);
    Serial.println(" ppb");
    Serial.print("eCO2: ");
    Serial.print(_ECO2);
    Serial.println(" ppm");
    Serial.print("Статус: ");
    Serial.println(_status);
}

// MT6701 method implementations
MT6701::MT6701() : _initialized(false) {}

bool MT6701::init(uint8_t address)
{
    _address = address;
    _initialized = true;
    Wire.beginTransmission(_address);
    return Wire.endTransmission() == 0;
}

String MT6701::getDirectionName(float angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0)
        angle += 360.0;

    const char *directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = ((int)(angle + 22.5) / 45) % 8;
    return directions[index];
}

void MT6701::update()
{
    Wire.beginTransmission(MT6701_ADDRESS);
    Wire.write(ANGLE_MSB_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(MT6701_ADDRESS, 2);

    if (Wire.available() == 2)
    {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint16_t rawAngle = ((msb << 6) | (lsb & 0x3F));
        float angle = (float)rawAngle * 360.0 / 16384.0;
        angle += MT6701_OFFSET;

        if (angle < 0)
            angle += 360.0;
        if (angle >= 360.0)
            angle -= 360.0;
        _angle = 360.0 - angle;
        if (_angle >= 360.0)
            _angle -= 360.0;
        _direction = getDirectionName(_angle);
    }
    else if (DEBUG)
    {
        Serial.println("Ошибка чтения с MT6701");
    }
}

uint16_t MT6701::getAngle() { return _angle; }
String MT6701::getDirection() { return _direction; }

void MT6701::getDebugInfo()
{
    if (!_initialized)
        return;
    Serial.println("=== Направление ветра ===");
    Serial.print("Угол: ");
    Serial.print(_angle, 1);
    Serial.print("° | Направление: ");
    Serial.println(_direction);
}

// VEML7700 method implementations
VEML7700::VEML7700() : _initialized(false) {}

bool VEML7700::init()
{
    _initialized = sensor.begin();
    return _initialized;
}

uint16_t VEML7700::getLight()
{
    uint16_t lux = sensor.readLux();
    return lux;
}

void VEML7700::getDebugInfo()
{
    if (!_initialized)
        return;
    uint16_t lux = this->getLight();
    Serial.println("=== Датчик света ===");
    Serial.print("Освещенность: ");
    Serial.print(lux, 1);
    Serial.println(" лк");
}

// BME280 method implementations
BME280::BME280() : _initialized(false) {}

bool BME280::init(uint8_t address)
{
    if (!bme.begin(address))
    {
        if (DEBUG)
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return false;
    }

    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_0_5);
    _initialized = true;
    return true;
}

float BME280::calculatePressureAtAltitude(float seaLevelPressure, float altitude)
{
    const float temperatureLapseRate = 0.0065;  // Градиент температуры (K/m)
    const float standardTemp = 288.15;          // Стандартная температура на уровне моря (K)
    const float gravity = 9.80665;              // Ускорение свободного падения (m/s^2)
    const float molarMass = 0.0289644;          // Молярная масса сухого воздуха (kg/mol)
    const float universalGasConstant = 8.31447; // Универсальная газовая постоянная (J/(mol·K))

    // Рассчитываем давление по барометрической формуле
    float pressureAtAltitude = seaLevelPressure * (1 /
                                                   pow(1 - (temperatureLapseRate * altitude) / standardTemp,
                                                       (gravity * molarMass) / (universalGasConstant * temperatureLapseRate)));

    return pressureAtAltitude;
}

void BME280::update()
{
    _temperature = bme.readTemperature() + BME280_TEMP_OFFSET;
    _humidity = bme.readHumidity();
    _sea_level_pressure = bme.readPressure() / 100.0;
    _pressure = calculatePressureAtAltitude(_sea_level_pressure, SEA_LEVEL_ALT);
}

float BME280::getTemperature() { return _temperature; }
float BME280::getHumidity() { return _humidity; }
float BME280::getPressure() { return _pressure; }

void BME280::getDebugInfo()
{
    if (!_initialized)
        return;
    Serial.println("=== BME280 ===");
    Serial.print("Температура: ");
    Serial.print(_temperature, 1);
    Serial.println(" °C");
    Serial.print("Влажность: ");
    Serial.print(_humidity, 1);
    Serial.println(" %");
    Serial.print("Давление: ");
    Serial.print(_pressure, 1);
    Serial.println(" гПа");
}

// KY003 method implementations
KY003 *KY003::_instance = nullptr;
const unsigned long KY003::DEBOUNCE_TIME = 10000;

KY003::KY003()
{
    _instance = this;
}

void KY003::init(uint8_t pin)
{
    _pin = pin;
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), handleInterrupt, FALLING);
    _lastPulseTime = micros();
}

void KY003::handleInterruptInstance()
{
    unsigned long currentTime = micros();
    if (currentTime - _lastPulseTime > DEBOUNCE_TIME)
    {
        _pulseInterval = currentTime - _lastPulseTime;
        _lastPulseTime = currentTime;
        _pulseCount++;
    }
}

void KY003::handleInterrupt()
{
    if (_instance)
    {
        _instance->handleInterruptInstance();
    }
}

float KY003::getSpeed() const
{
    if (_pulseInterval == 0)
        return 0.0;
    float timeBetweenPulses_sec = _pulseInterval / 1000000.0;
    return KY003_CIRCUMFERENCE / timeBetweenPulses_sec;
}

float KY003::getRotationFrequency() const
{
    if (_pulseInterval == 0)
        return 0.0;
    return 1000000.0 / _pulseInterval;
}

unsigned int KY003::getPulseCount() const { return _pulseCount; }
void KY003::resetPulseCount() { _pulseCount = 0; }
void KY003::resetPulseInterval() { _pulseInterval = 0; }

void KY003::getDebugInfo()
{
    unsigned long currentTime = millis();
    if (currentTime - _lastMeasurementTime < 1000)
        return;

    _lastMeasurementTime = currentTime;
    float speed = getSpeed();
    float rpm = getRotationFrequency() * 60;

    Serial.println("=== Скорость ветра (KY-003) ===");
    Serial.print("Скорость: ");
    Serial.print(speed, 2);
    Serial.println(" м/с");
    Serial.print("Скорость вращения: ");
    Serial.print(rpm, 1);
    Serial.println(" RPM");
    Serial.print("Число срабатываний: ");
    Serial.println(_pulseCount);
    Serial.println();
}

// RainSensor method implementations
RainSensor::RainSensor(uint8_t pin) : _pin(pin)
{
    pinMode(_pin, INPUT);
}

uint16_t RainSensor::getRainLevel()
{
    int raw = analogRead(_pin);
    float level = 1000.0 - raw * (1000.0 / 4095.0);
    return (int)level;
}

String RainSensor::getRainLevelName()
{
    uint16_t level = this->getRainLevel();
    if (level >= RAIN_LEVEL)
        return "yes";
    else
        return "no";
}

void RainSensor::getDebugInfo()
{
    Serial.println("=== Уровень дождя ===");
    Serial.print("Уровень: ");
    Serial.println(this->getRainLevel());
}