
#include <Arduino.h>
#include <DHT.h>
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>

#define DHTPIN 14
#define DHTTYPE DHT11

const char* ssid = "no vives de ensalada";
const char* pass = "sisas420";
char* mqtt_server = "192.168.141.206";
int mqtt_port = 1883;
const char* mqtt_topic = "esp32/dht11";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
PubSubClient client(const char*, int);

void setup()
{
  Serial.begin(9600);
  PubSubClient client.setServer (mqtt_server. mqtt_port);
  client.setCallback(callback);  // Opcional: función para manejar respuestas del broker (ver paso 5)
  while (!client.connected());
  {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) 
    {
      Serial.println("conectado!");
    } else 
    {
      Serial.print("Falló conexión, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }    
    Serial.print(".");
  
    Serial.println("");
    Serial.println("WiFi conectado!");
    Serial.println("ip dinamica asignada");
    Serial.println(WiFi.localIP());
    lcd.init();
    lcd.backlight();
    dht.begin();
  }
  
}

void loop() 
{
  
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) 
  {
    Serial.println("Failed to read from DHT sensor!");
   
    return;   
  }

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Temp ");
  lcd.print(temperature);
  lcd.print("°C");

  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(humidity);
  lcd.print("%");

  delay(2000);
}

