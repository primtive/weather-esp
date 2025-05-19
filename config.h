#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG true
#define SERIAL_SPEED 115200

#define WIFI_SSID "Hello"                                  // Wifi SSID
#define WIFI_PASSWORD "12348765"                           // Wifi Пароль
#define SERVER_URL "http://weather.orenbio.ru/api/weather" // API URL

#define ENS160_ADDRESS 0x52      // Датчик качества воздуха
#define HOME_BME280_ADDRESS 0x77 // Датчик температуры, влажности, давления (дома)
#define EXT_BME280_ADDRESS 0x76  // Датчик температуры, влажности, давления (на улице)
#define MT6701_ADDRESS 0x06      // Датчик направления ветра
#define ANGLE_MSB_REG 0x03       // Датчик направления ветра (регистр)
#define ANGLE_LSB_REG 0x04       // Датчик направления ветра (регистр)
#define DISPLAY_ADDRESS 0x27     // Дисплей LCD2004

#define TEMT6000_PIN 34 // Датчик освещенности (аналоговый)
#define RAIN_PIN 32     // Датчик дождя (аналоговый)
#define KY003_PIN 35     // Датчик холла (ветер) (цифровой)

#define BUTTON_PIN 3             // Кнопка для управления
#define BUTTON_MODE INPUT_PULLUP // - режим работы (умолч. INPUT_PULLUP)
#define BUTTON_LEVEL LOW         // - уровень кнопки (умолч. LOW)

#define TEMT6000_LUX 1600.0     // Коофициент яркости
#define KY003_CIRCUMFERENCE 1.0 // Коофициент датчика скорости ветра
#define MT6701_OFFSET 0         // Смещение поворота энкодера (в градусах)
#define RAIN_LEVEL 500          // Уровень дождя для определения как ДА

#define START_MODE 0          // Режим отображения по умолчанию (0 - дом, 1 - улица)
#define UPDATE_INTERVAL 10000 // 10 сек
#define API_INTERVAL 60000    // 1 мин

#endif