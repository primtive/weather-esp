#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "sensors.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <EncButton.h>

ENS160 qualitySensor;
MT6701 windDirectionSensor;
TEMT6000 lightSensor;
BME280 homeEnvSensor;
BME280 extEnvSensor;
KY003 windSpeedSensor;
RainSensor rainSensor(RAIN_PIN);

LiquidCrystal_I2C lcd(DISPLAY_ADDRESS, 20, 4);

ButtonT<BUTTON_PIN> button(BUTTON_MODE, BUTTON_LEVEL);

unsigned long updateTimer = 0;
unsigned long apiTimer = 0;
bool mode = START_MODE;

void drawInfoOutside()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Outside:");
  lcd.setCursor(0, 1);
  lcd.print((int)round(extEnvSensor.getTemperature()));
  lcd.print((char)223);
  lcd.print("C ");
  lcd.print((int)round(extEnvSensor.getHumidity()));
  lcd.print("% ");
  lcd.print((int)round(extEnvSensor.getPressure()));
  lcd.print("hPa");
  lcd.setCursor(0, 2);
  lcd.print("Wind: ");
  lcd.print(windDirectionSensor.getDirection());
  lcd.print(" ");
  lcd.print((int)round(windSpeedSensor.getSpeed()));
  lcd.print("m/s");
  lcd.setCursor(0, 3);
  lcd.print("Rain: ");
  lcd.print(rainSensor.getRainLevelName());
  lcd.print(" | ");
  lcd.print(lightSensor.getLight());
  lcd.print(" lx");
}

void drawInfoHome()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Inside:");
  lcd.setCursor(0, 1);
  lcd.print((int)round(homeEnvSensor.getTemperature()));
  lcd.print((char)223);
  lcd.print("C ");
  lcd.print((int)round(homeEnvSensor.getHumidity()));
  lcd.print("% ");
  lcd.print((int)round(homeEnvSensor.getPressure()));
  lcd.print(" hPa");
  lcd.setCursor(0, 2);
  lcd.print("eCO2: ");
  lcd.print(qualitySensor.getECO2());
  lcd.print(" ppm");
  lcd.setCursor(0, 3);
  lcd.print("AQI: ");
  lcd.print(qualitySensor.getAQIName());
}

void drawInfo()
{
  if (mode)
  {
    drawInfoOutside();
  }
  else
  {
    drawInfoHome();
  }
}

void postData()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClient client;
    HTTPClient http;

    http.begin(client, SERVER_URL);

    http.addHeader("Content-Type", "application/json");
    JsonDocument doc;
    doc["temp_h"] = homeEnvSensor.getTemperature();
    doc["humd_h"] = homeEnvSensor.getHumidity();
    doc["pres_h"] = homeEnvSensor.getPressure();
    doc["temp_e"] = extEnvSensor.getTemperature();
    doc["humd_e"] = extEnvSensor.getHumidity();
    doc["pres_e"] = extEnvSensor.getPressure();
    doc["aqi"] = qualitySensor.getAQI();
    doc["tvoc"] = qualitySensor.getTVOC();
    doc["eco2"] = qualitySensor.getECO2();
    doc["light"] = lightSensor.getLight();
    doc["wind_dir"] = windDirectionSensor.getAngle();
    doc["wind_speed"] = windSpeedSensor.getSpeed();
    doc["rain"] = rainSensor.getRainLevel();
    char requestBody[256];
    serializeJson(doc, requestBody);
    int httpResponseCode = http.POST(requestBody);
    if (DEBUG)
    {
      serializeJson(doc, Serial);
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    if (DEBUG)
      Serial.println("WiFi Disconnected");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("wifi disconnected");
  }
}

void setup()
{
  analogReadResolution(12);
  Wire.begin();
  lcd.init();
  lcd.backlight();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (DEBUG)
  {
    Serial.begin(SERIAL_SPEED);

    Serial.println("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    if (!qualitySensor.init(ENS160_ADDRESS))
    {
      Serial.println("Failed to initialize ENS160 sensor!");
    }

    if (!windDirectionSensor.init(MT6701_ADDRESS))
    {
      Serial.println("Failed to initialize MT6701 sensor!");
    }

    if (!lightSensor.init(TEMT6000_PIN))
    {
      Serial.println("Failed to initialize TEMT6000 sensor!");
    }

    if (!homeEnvSensor.init(HOME_BME280_ADDRESS))
    {
      Serial.println("Failed to initialize home BME280 sensor!");
    }

    if (!extEnvSensor.init(EXT_BME280_ADDRESS))
    {
      Serial.println("Failed to initialize external BME280 sensor!");
    }
    windSpeedSensor.init(KY003_PIN);
    if (false)
    {
      Serial.println("Failed to initialize KY003 sensor!");
    }
  }
  else
  {
    qualitySensor.init(ENS160_ADDRESS);
    windDirectionSensor.init(MT6701_ADDRESS);
    lightSensor.init(TEMT6000_PIN);
    homeEnvSensor.init(HOME_BME280_ADDRESS);
    extEnvSensor.init(EXT_BME280_ADDRESS);
    windSpeedSensor.init(KY003_PIN);
  }
}

void loop()
{
  unsigned long currentMillis = millis();
  button.tick();
  if (button.click())
  {
    mode = !mode;
    drawInfo();
  }

  if (currentMillis - updateTimer >= UPDATE_INTERVAL)
  {
    updateTimer = currentMillis;
    mode = !mode;

    qualitySensor.update();
    windDirectionSensor.update();
    extEnvSensor.update();
    homeEnvSensor.update();

    if (DEBUG)
    {
      qualitySensor.getDebugInfo();
      windDirectionSensor.getDebugInfo();
      lightSensor.getDebugInfo();
      extEnvSensor.getDebugInfo();
      homeEnvSensor.getDebugInfo();
      windSpeedSensor.getDebugInfo();
      rainSensor.getDebugInfo();
    }

    drawInfo();
  }

  if (currentMillis - apiTimer >= API_INTERVAL)
  {
    apiTimer = currentMillis;
    if (DEBUG)
      Serial.println("Start posting...");
    postData();
  }
}