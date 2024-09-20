#include <WiFi.h>
#include <PubSubClient.h>
#include <max6675.h>

const char* ssid = "****"; // Your Wi-Fi SSID
const char* password = "****"; // Your Wi-Fi Password
const char* mqtt_server = "broker.hivemq.com"; // MQTT Broker
const int mqtt_port = 1883; // MQTT Port

WiFiClient espClient;
PubSubClient client(espClient);

int thermoDO = 19; // Adjust based on your ESP32 pinout
int thermoCS = 5;
int thermoCLK = 18;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

const int tdsPin = 34; // A1 equivalent on ESP32
const int phPin = 35;  // A2 equivalent on ESP32

const int relayA = 26;
const int relayB = 27;
const int relayC = 14;
const int relayD = 12;

const int samplingInterval = 20;
const int printInterval = 800; 
float Offset = 0.0; 

void setup() {
  Serial.begin(115200);

  // Initialize pin modes
  pinMode(relayA, OUTPUT);
  pinMode(relayB, OUTPUT);
  pinMode(relayC, OUTPUT);
  pinMode(relayD, OUTPUT);

  digitalWrite(relayA, LOW);
  digitalWrite(relayB, LOW);
  digitalWrite(relayC, LOW);
  digitalWrite(relayD, LOW);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  double temperature = thermocouple.readCelsius();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  int tdsValue = analogRead(tdsPin);
  Serial.print("TDS Value: ");
  Serial.println(tdsValue);

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;

  if (millis() - samplingTime > samplingInterval) {
    voltage = analogRead(phPin) * 3.3 / 4096; // Adjust to ESP32 ADC resolution
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }

  if (millis() - printTime > printInterval) {
    Serial.print("pH value: ");
    Serial.println(pHValue, 2);

    // Publish the sensor data to MQTT topics
    client.publish("Topic_temp", String(temperature).c_str());
    client.publish("Topic_tds", String(tdsValue).c_str());
    client.publish("Topic_ph", String(pHValue, 2).c_str());

    printTime = millis();
  }

  // The rest of the logic for controlling the relays remains unchanged
  if (pHValue < 6.5) {
    digitalWrite(relayA, LOW);
    digitalWrite(relayD, HIGH);
    digitalWrite(relayB, HIGH);
    digitalWrite(relayC, HIGH);
    delay(10000);
    digitalWrite(relayA, HIGH);
    delay(5000);
  } else {
    digitalWrite(relayA, HIGH);
    digitalWrite(relayD, HIGH);
  }

  if (pHValue > 8.5) {
    digitalWrite(relayB, LOW);
    digitalWrite(relayD, HIGH);
    digitalWrite(relayA, HIGH);
    digitalWrite(relayC, HIGH);
    delay(10000);
    digitalWrite(relayB, HIGH);
    delay(5000);
  } else {
    digitalWrite(relayB, HIGH);
    digitalWrite(relayD, HIGH);
  }

  if (tdsValue < 200) {
    digitalWrite(relayC, LOW);
    digitalWrite(relayD, HIGH);
    digitalWrite(relayA, HIGH);
    digitalWrite(relayB, HIGH);
    delay(10000);
    digitalWrite(relayC, HIGH);
    delay(5000);
  } else {
    digitalWrite(relayC, HIGH);
    digitalWrite(relayD, HIGH);
  }
}
