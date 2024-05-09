
//Define el ID y nombre de la plantilla Blynk
#define BLYNK_TEMPLATE_ID "TMPL2wcKOfKHp"
#define BLYNK_TEMPLATE_NAME "Dht11"
#define BLYNK_AUTH_TOKEN "-pCgcQcxYonLH7RexMsHtH16yCDS0n3x"

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
#include <BlynkSimpleEsp32.h>


#define DHTPIN 14
#define DHTTYPE DHT11
#define RELAY_PIN 26

const char* ssid = "Red_IoT";
const char* pass = "Rapsoda25";
const char* mqtt_server = "192.168.1.71";
int mqtt_port = 1883;
const char* mqtt_topic_temp = "esp32/dht11/temperature";
const char* mqtt_topic_hum = "esp32/dht11/humidity";
const char* mqtt_topic_relay = "esp32/relay";
char auth[] = "-pCgcQcxYonLH7RexMsHtH16yCDS0n3x";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);


bool relayState = false; // Variable para mantener el estado del relé

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(dht.readTemperature());
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("H: ");
  lcd.print(dht.readHumidity());
  lcd.print(" %");
  lcd.setCursor(10, 1);
  lcd.print("R: ");
  lcd.print(relayState ? "ON" : "OFF");
}

BLYNK_WRITE(V1) {
  int value = param.asInt(); // Lee el estado del botón (1 o 0)
  if (value == 1) {
    digitalWrite(RELAY_PIN, LOW); // Activa el relé
    relayState = true;
  } else {
    digitalWrite(RELAY_PIN, HIGH); // Desactiva el relé
    relayState = false;
  }
  updateLCD(); // Actualiza la pantalla LCD
}

void callback(char* topic, byte* payload, unsigned int length) {
  
  if (strcmp(topic, mqtt_topic_relay) == 0) {
    // Convertir payload en string
    String message;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    // Controlar el relé según el mensaje recibido
    if (message.equals("OFF")) {
      digitalWrite(RELAY_PIN, LOW);
      relayState = true;
      
    } else if (message.equals("ON")) {
      digitalWrite(RELAY_PIN, HIGH);
      relayState = false;
      updateLCD();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("conectado!");
      client.subscribe(mqtt_topic_relay);
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

  lcd.init();
  lcd.backlight();
  Blynk.connect();
  Blynk.config(auth);
  Blynk.begin(auth, ssid, pass);
  pinMode(RELAY_PIN, OUTPUT);
  updateLCD();
}

void loop() {

  Blynk.run();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error al leer el sensor DHT!");
    Blynk.virtualWrite(V0, String(temperature, 1));
    Blynk.virtualWrite(V2, String(humidity, 1));
    return;
  }else {
    Blynk.virtualWrite(V0, "NAN");
    Blynk.virtualWrite(V2, "NAN");
  }

  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" °C");
  delay(5000);
  Serial.print("Humedad: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Publicar datos en el tema MQTT
  char tempString[8];
  char humString[8];
  dtostrf(temperature, 4, 2, tempString);
  dtostrf(humidity, 4, 2, humString);
  client.publish(mqtt_topic_temp, tempString);
  client.publish(mqtt_topic_hum, humString);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("H: ");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(9, 1);
  lcd.print("R:");
  lcd.print(relayState ? "OFF" : "ON");

  // Controlar el relé
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);

  
  delay(2000);
}
