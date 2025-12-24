#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

#define LED 2 // Лед индикатор на ESP32

HardwareSerial mySerial(2); // UART2 для GPS
TinyGPSPlus gps;

// Переменные для хранения предыдущих координат
double prevLatitude = 0.0;
double prevLongitude = 0.0;
bool hasCoordinates = false; // Флаг, чтобы знать, есть ли сохранённые координаты
bool gpsStopped = false;
bool weatherRequested = false;

unsigned long lastRequestTime = 0;
const unsigned long requestInterval = 3600000; // 1 час = 3600 * 1000 мс

// const char *ssid = "DIR-615-B64B";
// const char *password = "91451989";
// const char *ssid2 = "";
// const char *password2 = "";

unsigned long lastSendTime = 0;          // Время последней отправки
const unsigned long sendInterval = 3000; // Интервал в миллисекундах (3 секунд)

void updateCoordinates()
{
  if (gps.satellites.isValid())
  {
    if (gps.location.isValid())
    {
      double newLatitude = gps.location.lat();
      double newLongitude = gps.location.lng();

      if (!hasCoordinates || newLatitude != prevLatitude || newLongitude != prevLongitude)
      {
        prevLatitude = newLatitude;
        prevLongitude = newLongitude;
        hasCoordinates = true;

        Serial.print("Updated Coordinates - Latitude: ");
        Serial.println(prevLatitude, 6);
        Serial.print("Longitude: ");
        Serial.println(prevLongitude, 6);
      }
    }
    else
    {
      Serial.println("Location: INVALID (No valid fix)");
    }
  }
  else
  {
    Serial.println("No satellites detected.");
  }
}

void sendWeatherRequest(double lat, double lon)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    return;
  }

  HTTPClient http;

  String url = "https://api.open-meteo.com/v1/forecast?";
  url += "latitude=" + String(lat, 2);
  url += "&longitude=" + String(lon, 2);
  url += "&current=temperature_2m";

  Serial.println("Request URL:");
  Serial.println(url);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0)
  {
    Serial.print("HTTP Code: ");
    Serial.println(httpCode);

    String payload = http.getString();
    Serial.println("Response:");
    Serial.println(payload);
  }
  else
  {
    Serial.print("HTTP Error: ");
    Serial.println(httpCode);
  }

  http.end();
}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 16, 17); // GPS: TX=16, RX=17, 9600 бод

  pinMode(LED, OUTPUT);

  Serial.println("GPS Test Starting...");

  // Задержка для инициализации
  delay(5000); // 5 секунд

  Serial.println("Connecting to wifi");

  bool connected = false;

  for (int i = 0; i < WIFI_NETWORKS_COUNT; i++)
  {
    WiFi.begin(WIFI_NETWORKS[i].ssid, WIFI_NETWORKS[i].pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 5000)
    {
      delay(200);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      connected = true;
      break;
    }

    Serial.println("\nFailed, trying next...");
    WiFi.disconnect(true);
    delay(300);
  }

  if (connected)
  {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void loop()
{
  // Лед индикатор загорается - подключен к wifi
  if (WiFi.status() == WL_CONNECTED)
    digitalWrite(LED, HIGH);

  // Читаем GPS только пока он нужен
  if (!hasCoordinates)
  {
    while (mySerial.available() > 0)
    {
      char c = mySerial.read();
      Serial.write(c);
      if (gps.encode(c))
      {
        updateCoordinates();
      }
    }
  }

  // Останавливаем GPS один раз
  if (hasCoordinates && !gpsStopped)
  {
    mySerial.end();
    Serial.println("GPS fixed, UART stopped");
    gpsStopped = true;
  }

  if (hasCoordinates && !weatherRequested)
  {
    sendWeatherRequest(prevLatitude, prevLongitude);
    lastRequestTime = millis();
    weatherRequested = true;
  }

  unsigned long currentTime = millis();

  // Раз в час отправляем запрос
  if (hasCoordinates && (currentTime - lastRequestTime >= requestInterval))
  {
    lastRequestTime = currentTime;
    sendWeatherRequest(prevLatitude, prevLongitude);
  }
}