
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
const char* mqtt_server = "40.85.182.68";
int mqtt_port = 1883;
const char* mqtt_topic = "esp32/dht11";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  // Aquí puedes manejar las respuestas recibidas del servidor MQTT
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("conectado!");
    } else {
      Serial.print("Falló conexión, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado!");
  Serial.println("IP dinámica asignada: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error al leer el sensor DHT!");
    return;
  }

  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humedad: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Publicar datos en el tema MQTT
  char tempString[8];
  char humString[8];
  dtostrf(temperature, 4, 2, tempString);
  dtostrf(humidity, 4, 2, humString);
  client.publish(mqtt_topic, tempString);
  client.publish(mqtt_topic, humString);

  delay(2000);
}