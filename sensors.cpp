#include "SparkFun_ENS160.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "config.h"

class ENS160
{
private:
  uint8_t _address;
  SparkFun_ENS160 sensor;
  bool _initialized = false;
  uint8_t _status;
  uint8_t _AQI;
  uint16_t _TVOC;
  uint16_t _ECO2;

  static String getStatusName(uint8_t ensStatus)
  {
    switch (ensStatus)
    {
    case 0:
      return "Standard";
      break;
    case 1:
      return "Warm up";
      break;
    case 2:
      return "Initial Start Up";
      break;
    default:
      return "Unknown";
      break;
    }
  }

public:
  ENS160() {}

  bool init(uint8_t address)
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

  void update()
  {
    if (!_initialized || !sensor.checkDataStatus())
      return;
    _AQI = sensor.getAQI();
    _TVOC = sensor.getTVOC();
    _ECO2 = sensor.getECO2();
  }

  uint8_t getAQI()
  {
    return _AQI;
  }

  uint16_t getTVOC()
  {
    return _TVOC;
  }

  uint16_t getECO2()
  {
    return _ECO2;
  }

  void getDebugInfo()
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
};

class MT6701
{
private:
  bool _initialized = false;
  uint8_t _address;
  uint16_t _angle;
  String _direction;

  String getDirectionName(float angle)
  {
    angle = fmod(angle, 360.0);
    if (angle < 0)
      angle += 360.0;

    const char *directions[] = {
        "N", "NE", "E", "SE",
        "S", "SW", "W", "NW"};

    int index = ((int)(angle + 22.5) / 45) % 8;
    return directions[index];
  }

public:
  MT6701() {}

  bool init(uint8_t address)
  {
    _address = address;
    _initialized = true;
    Wire.beginTransmission(_address);
    return Wire.endTransmission() == 0;
  }

  void update()
  {
    Wire.beginTransmission(MT6701_ADDRESS);
    Wire.write(ANGLE_MSB_REG); // MSB регистр
    Wire.endTransmission(false);
    Wire.requestFrom(MT6701_ADDRESS, 2);

    if (Wire.available() == 2)
    {
      uint8_t msb = Wire.read();
      uint8_t lsb = Wire.read();
      uint16_t rawAngle = ((msb << 6) | (lsb & 0x3F)); // 14-битный угол

      float angle = (float)rawAngle * 360.0 / 16384.0; // Преобразуем в градусы
      angle += MT6701_OFFSET;

      // Нормализуем
      if (angle < 0)
        angle += 360.0;
      if (angle >= 360.0)
        angle -= 360.0;
      _angle = angle;
      _direction = getDirectionName(_angle);
    }
    else
    {
      if (DEBUG)
        Serial.println("Ошибка чтения с MT6701");
    }
  }

  uint16_t getAngle()
  {
    return _angle;
  }

  String getDirection()
  {
    return _direction;
  }

  void getDebugInfo()
  {
    if (!_initialized)
      return;

    Serial.println("=== Направление ветра ===");
    Serial.print("Угол: ");
    Serial.print(_angle, 1);
    Serial.print("° | Направление: ");
    Serial.println(_direction);
  }
};

class TEMT6000
{
private:
  bool _initialized = false;

public:
  TEMT6000() {}

  bool init(uint8_t pin)
  {
    pinMode(pin, INPUT);

    _initialized = true;
    return true;
  }
  uint16_t getLight()
  {
    int raw = analogRead(TEMT6000_PIN);        // Считываем аналоговое значение
    float lux = raw * (TEMT6000_LUX / 4095.0); // Пример: 0–1000 лк
    return lux;
  }
  void getDebugInfo()
  {
    if (!_initialized)
      return;

    float lux = this->getLight();

    Serial.println("=== Датчик света ===");
    Serial.print("Освещенность: ");
    Serial.print(lux, 1);
    Serial.println(" лк");
  }
};

class BME280
{
private:
  Adafruit_BME280 bme; // Экземпляр драйвера датчика
  float _temperature;
  float _humidity;
  float _pressure;
  bool _initialized = false;

public:
  BME280() {}

  bool init(uint8_t address)
  {

    if (!bme.begin(address))
    {
      if (DEBUG)
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
      return false;
    }

    // Настройка параметров датчика
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // Температура
                    Adafruit_BME280::SAMPLING_X16, // Давление
                    Adafruit_BME280::SAMPLING_X1,  // Влажность
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_0_5);

    _initialized = true;

    return true;
  }

  void update()
  {
    _temperature = bme.readTemperature();   // °C
    _humidity = bme.readHumidity();         // %
    _pressure = bme.readPressure() / 100.0; // гПа
  }

  float getTemperature()
  {
    return _temperature;
  }

  float getHumidity()
  {
    return _humidity;
  }

  float getPressure()
  {
    return _pressure;
  }

  void getDebugInfo()
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
};

class KY003
{
private:
  uint8_t _pin;                                     // Пин, к которому подключен датчик
  volatile unsigned long _lastPulseTime = 0;        // Время последнего срабатывания
  volatile unsigned long _pulseInterval = 0;        // Интервал между импульсами (мкс)
  volatile unsigned int _pulseCount = 0;            // Счетчик импульсов
  unsigned long _lastMeasurementTime = 0;           // Время последнего измерения
  static const unsigned long DEBOUNCE_TIME = 10000; // Время антидребезга (мкс)

  // Указатель на экземпляр класса для работы с прерываниями
  static KY003 *_instance;

  // Обработчик прерывания для конкретного экземпляра
  void handleInterruptInstance()
  {
    unsigned long currentTime = micros();
    if (currentTime - _lastPulseTime > DEBOUNCE_TIME)
    {
      _pulseInterval = currentTime - _lastPulseTime;
      _lastPulseTime = currentTime;
      _pulseCount++;
    }
  }

  // Статический метод-обработчик прерывания
  static void handleInterrupt()
  {
    if (_instance)
    {
      _instance->handleInterruptInstance();
    }
  }

public:
  KY003()
  {
    _instance = this; // Устанавливаем текущий экземпляр как активный
  }

  // Инициализация датчика
  void init(uint8_t pin)
  {
    _pin = pin;
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), handleInterrupt, FALLING);
    _lastPulseTime = micros();
  }

  // Получение текущей скорости (м/с)
  float getSpeed() const
  {
    if (_pulseInterval == 0)
      return 0.0; // Нет вращения

    // Преобразуем интервал между импульсами в скорость
    float timeBetweenPulses_sec = _pulseInterval / 1000000.0;
    return KY003_CIRCUMFERENCE / timeBetweenPulses_sec;
  }

  // Получение частоты вращения (оборотов в секунду)
  float getRotationFrequency() const
  {
    if (_pulseInterval == 0)
      return 0.0;
    return 1000000.0 / _pulseInterval;
  }

  // Получение количества импульсов с последнего сброса
  unsigned int getPulseCount() const
  {
    return _pulseCount;
  }

  // Сброс счетчика импульсов
  void resetPulseCount()
  {
    _pulseCount = 0;
  }

  // Обновление и вывод данных
  void getDebugInfo()
  {
    unsigned long currentTime = millis();
    if (currentTime - _lastMeasurementTime < 1000)
      return; // Обновляем не чаще 1 раза в секунду

    _lastMeasurementTime = currentTime;

    float speed = getSpeed();
    float rpm = getRotationFrequency() * 60; // Оборотов в минуту

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
};

class RainSensor
{
private:
  uint8_t _pin;

public:
  RainSensor(uint8_t pin)
  {
    _pin = pin;
    pinMode(_pin, INPUT);
  }
  uint16_t getRainLevel()
  {
    return analogRead(_pin);
  }
  void getDebugInfo()
  {
    Serial.println("=== Уровень дождя ===");
    Serial.print("Уровень (ADC): ");
    Serial.println(this->getRainLevel());
  }
};

// Инициализация статических членов класса
KY003 *KY003::_instance = nullptr;